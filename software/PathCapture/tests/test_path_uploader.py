import threading
import time
import unittest
import struct
import zlib

from path_uploader import (
    BINARY_MAGIC,
    PathUploadError,
    PathUploadManager,
)


class FakeTransport:
    def __init__(self, drop_first_point=False, reject_marker=False):
        self.manager = None
        self.commands = []
        self.drop_first_point = drop_first_point
        self.reject_marker = reject_marker
        self.dropped = False

    def send(self, command):
        self.commands.append(command)
        parts = command.split()
        if parts[2] == 'begin':
            line = f'{{pathupload}}begin,1,{parts[3]},32760'
        elif parts[2] == 'point':
            index = int(parts[3])
            if self.drop_first_point and index == 0 and not self.dropped:
                self.dropped = True
                return
            line = f'{{pathupload}}point,1,{index},{index + 1}'
        elif parts[2] == 'marker':
            line = ('{pathupload}usage:begin <count>|point <i>|marker <i> in|out'
                    if self.reject_marker
                    else f'{{pathupload}}marker,1,{parts[3]},{parts[4]}')
        elif parts[2] == 'ptzone':
            line = f'{{pathupload}}ptzone,1,{parts[3]},{parts[4]}'
        elif parts[2] == 'commit':
            count = len({item.split()[3] for item in self.commands if ' upload point ' in item})
            line = f'{{pathupload}}commit,1,{count},{count}'
        else:
            return
        self.manager.handle_line(line)
        if parts[2] == 'commit':
            self.manager.handle_line('{pathupload}done,2,1234,10.0')


class FakeBinaryTransport:
    def __init__(self, drop_first_block=False, failed_block_acks=0,
                 checksum_mismatch=False, support_binary=True):
        self.manager = None
        self.commands = []
        self.frames = []
        self.received = 0
        self.drop_first_block = drop_first_block
        self.failed_block_acks = failed_block_acks
        self.checksum_mismatch = checksum_mismatch
        self.support_binary = support_binary
        self.dropped = False
        self.session = 0
        self.count = 0
        self.checksum = 0

    def send_line(self, command):
        self.commands.append(command)
        parts = command.split()
        action = parts[2]
        if action == 'begin2':
            if not self.support_binary:
                self.manager.handle_line('{pathupload}usage:begin <count>|point <i>')
                return
            self.count = int(parts[3])
            self.session = int(parts[4])
            self.checksum = int(parts[5])
            self.manager.handle_line(
                f'{{pathupload}}begin2,1,{self.session},{self.count},32760'
            )
        elif action == 'marker':
            self.manager.handle_line(
                f'{{pathupload}}marker,1,{parts[3]},{parts[4]}'
            )
        elif action == 'ptzone':
            self.manager.handle_line(
                f'{{pathupload}}ptzone,1,{parts[3]},{parts[4]}'
            )
        elif action == 'commit2':
            actual = self.checksum + 1 if self.checksum_mismatch else self.checksum
            success = 0 if self.checksum_mismatch else 1
            self.manager.handle_line(
                f'{{pathupload}}commit2,{success},{self.session},'
                f'{self.received},{self.count},{actual}'
            )
            if success:
                self.manager.handle_line(
                    f'{{pathupload}}done,{self.received},{self.checksum},10.0'
                )
        elif action == 'begin':
            self.count = int(parts[3])
            self.manager.handle_line(f'{{pathupload}}begin,1,{self.count},32760')
        elif action == 'point':
            index = int(parts[3])
            self.received = max(self.received, index + 1)
            self.manager.handle_line(
                f'{{pathupload}}point,1,{index},{self.received}'
            )
        elif action == 'commit':
            self.manager.handle_line(
                f'{{pathupload}}commit,1,{self.received},{self.count}'
            )
            self.manager.handle_line(
                f'{{pathupload}}done,{self.received},1234,10.0'
            )

    def send_bytes(self, frame):
        self.frames.append(frame)
        self.assert_frame(frame)
        session, start, point_count = struct.unpack_from('<IIB', frame, 8)
        frame_crc = struct.unpack_from('<I', frame, len(frame) - 4)[0]
        if self.drop_first_block and not self.dropped:
            self.dropped = True
            return
        if self.failed_block_acks > 0:
            self.failed_block_acks -= 1
            self.manager.handle_line(
                f'{{pathupload}}block,0,{session},{start},{point_count},'
                f'{self.received},{frame_crc}'
            )
            return
        self.received = max(self.received, start + point_count)
        self.manager.handle_line(
            f'{{pathupload}}block,1,{session},{start},{point_count},'
            f'{self.received},{frame_crc}'
        )

    @staticmethod
    def assert_frame(frame):
        if frame[:4] != BINARY_MAGIC:
            raise AssertionError('bad magic')
        payload_length = struct.unpack_from('<H', frame, 6)[0]
        if len(frame) != 8 + payload_length + 4:
            raise AssertionError('bad length')
        expected_crc = struct.unpack_from('<I', frame, len(frame) - 4)[0]
        if expected_crc != zlib.crc32(frame[:-4]) & 0xFFFFFFFF:
            raise AssertionError('bad crc')


class PathUploadManagerTests(unittest.TestCase):
    def make_manager(self, drop=False, reject_marker=False):
        events = []
        transport = FakeTransport(drop, reject_marker)
        manager = PathUploadManager(transport.send, lambda event, data: events.append((event, data)),
                                    ack_timeout=0.02, retries=3, done_timeout=0.1)
        transport.manager = manager
        return manager, transport, events

    def wait(self, manager):
        deadline = time.monotonic() + 1
        while manager.active and time.monotonic() < deadline:
            time.sleep(0.005)
        self.assertFalse(manager.active)

    def test_success_and_done_arriving_with_commit_ack(self):
        manager, transport, events = self.make_manager()
        manager.start([[0, 0, 0, 500], [5, 0, 0, 0]], 'geometry')
        self.wait(manager)
        self.assertTrue(events[-1][1]['success'])
        self.assertEqual(events[-1][1]['checksum'], 1234)
        self.assertEqual(len([item for item in transport.commands if ' upload point ' in item]), 2)

    def test_dropped_ack_retries_same_index(self):
        manager, transport, events = self.make_manager(drop=True)
        manager.start([[0, 0, 0, 500], [5, 0, 0, 0]], 'external')
        self.wait(manager)
        self.assertTrue(events[-1][1]['success'])
        self.assertEqual(len([item for item in transport.commands if ' upload point 0 ' in item]), 2)

    def test_curve_markers_are_sent_before_commit(self):
        manager, transport, events = self.make_manager()
        manager.start(
            [[0, 0, 0, 500], [5, 0, 0, 500], [10, 5, 0.5, 0]],
            'external',
            [[1, 'in'], [2, 'out']]
        )
        self.wait(manager)
        self.assertTrue(events[-1][1]['success'])
        marker_commands = [item for item in transport.commands if ' upload marker ' in item]
        self.assertEqual(marker_commands, [
            'vpath upload marker 1 in',
            'vpath upload marker 2 out'
        ])
        self.assertLess(transport.commands.index(marker_commands[-1]),
                        transport.commands.index('vpath upload commit'))

    def test_usage_response_fails_without_retrying_marker(self):
        manager, transport, events = self.make_manager(reject_marker=True)
        manager.start(
            [[0, 0, 0, 500], [5, 0, 0, 0]],
            'external',
            [[1, 'in']]
        )
        self.wait(manager)
        self.assertFalse(events[-1][1]['success'])
        self.assertIn('参数格式错误', events[-1][1]['error'])
        self.assertEqual(len([item for item in transport.commands if ' upload marker ' in item]), 1)
        self.assertEqual(transport.commands[-1], 'vpath upload abort')

    def test_phototube_zones_are_sent_before_legacy_commit(self):
        manager, transport, events = self.make_manager()
        manager.start(
            self.sample_points(5), 'external', [], [[1, 2], [4, 4]]
        )
        self.wait(manager)
        self.assertTrue(events[-1][1]['success'])
        zone_commands = [item for item in transport.commands if ' upload ptzone ' in item]
        self.assertEqual(zone_commands, [
            'vpath upload ptzone 1 2',
            'vpath upload ptzone 4 4'
        ])
        self.assertLess(transport.commands.index(zone_commands[-1]),
                        transport.commands.index('vpath upload commit'))

    def test_phototube_zone_validation_rejects_overlap(self):
        manager, _, _ = self.make_manager()
        with self.assertRaises(PathUploadError):
            manager.start(self.sample_points(5), 'external', [], [[1, 3], [3, 4]])

    def test_validation_rejects_nonfinite(self):
        manager, _, _ = self.make_manager()
        with self.assertRaises(PathUploadError):
            manager.start([[0, 0, 0, 500], [float('nan'), 0, 0, 0]])

    def test_complete_ack_is_recovered_after_corrupted_prefix(self):
        manager, _, _ = self.make_manager()

        self.assertTrue(manager.handle_line(
            '{pathupload}poiH\ufffd{pathupload}point,1,15,16'))
        self.assertFalse(manager.handle_line('{pathupload}poiH\ufffd'))

        _, tag, fields = manager._messages[-1]
        self.assertEqual(tag, 'point')
        self.assertEqual(fields, ['1', '15', '16'])

    def test_erase_progress_extends_begin_wait(self):
        manager, _, _ = self.make_manager()
        erase_timer = threading.Timer(
            0.03, manager.handle_line, args=('{pathupload}erase,1,4,0xA0180000',)
        )
        begin_timer = threading.Timer(
            0.06, manager.handle_line, args=('{pathupload}begin2,1,7,2,32760',)
        )
        erase_timer.start()
        begin_timer.start()
        try:
            _, tag, fields = manager._wait_after(
                0,
                lambda message_tag, values: message_tag == 'begin2',
                0.04,
                keepalive=lambda message_tag, values: message_tag in ('erase', 'erase_done')
            )
        finally:
            erase_timer.cancel()
            begin_timer.cancel()

        self.assertEqual(tag, 'begin2')
        self.assertEqual(fields[1], '7')

    def make_binary_manager(self, **transport_options):
        events = []
        transport = FakeBinaryTransport(**transport_options)
        manager = PathUploadManager(
            transport.send_line,
            lambda event, data: events.append((event, data)),
            transport.send_bytes,
            ack_timeout=0.02,
            retries=3,
            done_timeout=0.1
        )
        transport.manager = manager
        return manager, transport, events

    @staticmethod
    def sample_points(count):
        return [[index * 5, index * 2, index * 0.01, 500 + index]
                for index in range(count)]

    def test_binary_upload_packs_32_points_per_block_with_crc(self):
        manager, transport, events = self.make_binary_manager()
        manager.start(self.sample_points(33), 'geometry')
        self.wait(manager)

        self.assertTrue(events[-1][1]['success'])
        self.assertEqual(len(transport.frames), 2)
        self.assertEqual(struct.unpack_from('<B', transport.frames[0], 16)[0], 32)
        self.assertEqual(struct.unpack_from('<B', transport.frames[1], 16)[0], 1)
        started = next(data for event, data in events if event == 'path_upload_started')
        self.assertEqual(started['protocol'], 'binary32')

    def test_binary_upload_sends_phototube_zones_before_commit(self):
        manager, transport, events = self.make_binary_manager()
        manager.start(self.sample_points(5), 'external', [], [[1, 3]])
        self.wait(manager)

        self.assertTrue(events[-1][1]['success'])
        command = 'vpath upload ptzone 1 3'
        self.assertIn(command, transport.commands)
        commit = next(item for item in transport.commands if ' upload commit2 ' in item)
        self.assertLess(transport.commands.index(command), transport.commands.index(commit))

    def test_binary_upload_retries_same_block_after_lost_ack(self):
        manager, transport, events = self.make_binary_manager(drop_first_block=True)
        manager.start(self.sample_points(2), 'external')
        self.wait(manager)

        self.assertTrue(events[-1][1]['success'])
        self.assertEqual(len(transport.frames), 2)
        self.assertEqual(transport.frames[0], transport.frames[1])

    def test_binary_upload_aborts_after_three_failed_block_acks(self):
        manager, transport, events = self.make_binary_manager(failed_block_acks=3)
        manager.start(self.sample_points(2), 'external')
        self.wait(manager)

        self.assertFalse(events[-1][1]['success'])
        self.assertEqual(len(transport.frames), 3)
        self.assertEqual(transport.commands[-1], 'vpath upload abort')

    def test_binary_upload_aborts_on_total_checksum_mismatch(self):
        manager, transport, events = self.make_binary_manager(checksum_mismatch=True)
        manager.start(self.sample_points(2), 'external')
        self.wait(manager)

        self.assertFalse(events[-1][1]['success'])
        self.assertIn('整条路径校验失败', events[-1][1]['error'])
        self.assertEqual(transport.commands[-1], 'vpath upload abort')

    def test_binary_unsupported_falls_back_to_legacy(self):
        manager, transport, events = self.make_binary_manager(support_binary=False)
        manager.start(self.sample_points(2), 'external')
        self.wait(manager)

        self.assertTrue(events[-1][1]['success'])
        self.assertEqual(transport.frames, [])
        self.assertIn('vpath upload begin 2', transport.commands)
        started = next(data for event, data in events if event == 'path_upload_started')
        self.assertEqual(started['protocol'], 'legacy')


if __name__ == '__main__':
    unittest.main()
