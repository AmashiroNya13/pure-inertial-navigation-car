import struct
import unittest
from unittest import mock

import app as pathcapture


def make_frame(sequence=7):
    values = (
        50000, 12, 18, 400.0, 1, 3862, 2,
        100.5, 200.5, 30.25, 90.0, 190.0, 29.5, 10.5, -4.0, 150.0, 250.0,
        31.0, 2.5, 0.75, 3300.0, 3200.0, 3400.0, 3150.0, 3350.0,
        200.0, 45.0, 42.0, 120.0, 180.0, 4000.0, 2800.0,
        1200, 1300, 1250, 1350, 3, 0, 3, 0,
    )
    payload = struct.pack(pathcapture.REPLAY_BINARY_SAMPLE_FORMAT, *values)
    header = struct.pack('<2sBBHH', pathcapture.REPLAY_BINARY_SYNC, 1, 1,
                         sequence, len(payload))
    body = header[2:] + payload
    return header + payload + struct.pack('<H', pathcapture._crc16_modbus(body))


def make_slip_frame(sequence=8):
    payload = struct.pack(pathcapture.REPLAY_BINARY_SLIP_FORMAT,
                          1, 1, 0, 1, 30.0, 20.0, 0.667, 8.0, 2.5, 2.0, 1.5,
                          15, 11, 4, 255, 150.0)
    header = struct.pack('<2sBBHH', pathcapture.REPLAY_BINARY_SYNC, 1, 2,
                         sequence, len(payload))
    body = header[2:] + payload
    return header + payload + struct.pack('<H', pathcapture._crc16_modbus(body))


class ReplayBinaryTelemetryTest(unittest.TestCase):
    def setUp(self):
        pathcapture.serial_buffer = ''
        pathcapture.serial_byte_buffer.clear()

    def test_frame_decodes_to_existing_vrpl_lines(self):
        lines = pathcapture.decode_replay_binary_frame(make_frame())
        self.assertEqual(len(lines), 4)
        self.assertTrue(lines[0].startswith('{vrpl0}50000,12,18,400.0,1,3862,2'))
        self.assertTrue(lines[1].startswith('{vrpl1}100.5,200.5,30.25'))
        self.assertIn(',3300,3200,3400,3150,3350,', lines[2])
        self.assertEqual(lines[3], '{vrpl3}1200,1300,1250,1350,3,0,3,0')

    def test_mixed_stream_handles_arbitrary_chunks_and_preserves_vspd(self):
        frame = make_frame()
        stream = b'{vspd}1,2,3\r\n' + frame + b'{vstop}ok\r\n'
        chunks = [stream[:1], stream[1:9], stream[9:17], stream[17:44],
                  stream[44:101], stream[101:139], stream[139:]]
        with mock.patch.object(pathcapture, 'process_line') as process_line:
            for chunk in chunks:
                pathcapture.process_serial_bytes(chunk)

        lines = [call.args[0].strip() for call in process_line.call_args_list]
        self.assertEqual(lines[0], '{vspd}1,2,3')
        self.assertEqual(lines[1][0:7], '{vrpl0}')
        self.assertEqual(lines[4][0:7], '{vrpl3}')
        self.assertEqual(lines[5], '{vstop}ok')

    def test_bad_crc_resynchronizes_to_next_frame(self):
        broken = bytearray(make_frame(1))
        broken[20] ^= 0x40
        good = make_frame(2)
        with mock.patch.object(pathcapture, 'process_line') as process_line:
            pathcapture.process_serial_bytes(bytes(broken) + good)

        lines = [call.args[0] for call in process_line.call_args_list]
        self.assertEqual(len(lines), 4)
        self.assertTrue(lines[0].startswith('{vrpl0}'))

    def test_slip_frame_restores_existing_vslip_packet(self):
        lines = pathcapture.decode_replay_binary_frame(make_slip_frame())
        self.assertEqual(len(lines), 1)
        parts = lines[0].removeprefix('{vslip}').split(',')
        self.assertEqual(parts[:3], ['1', '1', '0'])
        self.assertEqual(parts[10:14], ['1', '15', '11', '4'])
        self.assertEqual(parts[-1], '-1')


if __name__ == '__main__':
    unittest.main()
