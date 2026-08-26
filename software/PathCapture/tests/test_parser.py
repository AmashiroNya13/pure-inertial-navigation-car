import io
import unittest
from unittest import mock

import app as pathcapture


class TelemetryParserTest(unittest.TestCase):
    def setUp(self):
        with pathcapture.browser_batch_lock:
            pathcapture.browser_event_batch.clear()
            pathcapture.browser_batching = False
        with pathcapture.raw_log_lock:
            pathcapture.raw_log_lines.clear()
        with pathcapture.data_lock:
            for points in pathcapture.parsed_data.values():
                points.clear()
            pathcapture.console_lines.clear()
        pathcapture.replay_point_seq = 0
        pathcapture.latest_replay_point_id = None
        pathcapture.pending_replay_meta = {}
        pathcapture.replay_sample_open = False
        pathcapture.replay_run_seq = 0
        pathcapture.active_replay_run = None
        pathcapture.last_slip_total_correction_mm = 0.0
        pathcapture.runtime_mode = {
            'mode': 'idle', 'reason': 'test', 'updated_at': 0.0
        }

    def test_replay_packets_build_one_compact_sample(self):
        self.assertTrue(pathcapture.process_line(
            '{vrpl0}20000,7,9,7.0,9.0,180,1,300,42,0,0,0'))
        self.assertTrue(pathcapture.process_line(
            '{vrpl1}100,200,30,90,190,28,12,-3,250,300'))
        self.assertTrue(pathcapture.process_line(
            '{vrpl2}35,34,5,2500,2300,2700,2250,2650,400,120,130,0,390,2600,2500'))
        self.assertTrue(pathcapture.process_line(
            '{vrpl3}10,11,1,2,3,6000,7000,6100,6900,2,0,2,0'))
        self.assertTrue(pathcapture.process_line(
            '{vrpl4}7,9,11,90,190,250,300,280,330,180,30'))
        self.assertTrue(pathcapture.process_line(
            '{vrplg}2,450,300,12,500,1,100'))

        self.assertEqual(len(pathcapture.parsed_data['realtime']), 1)
        self.assertEqual(len(pathcapture.parsed_data['lookahead']), 1)

        pose = pathcapture.parsed_data['realtime'][0]
        target = pathcapture.parsed_data['lookahead'][0]
        self.assertEqual(pose['sample_dt_ms'], 20.0)
        self.assertEqual(pose['target_index'], 9.0)
        self.assertEqual(pose['target_left_speed_mm_s'], 2300.0)
        self.assertEqual(pose['actual_right_speed_mm_s'], 2650.0)
        self.assertEqual(pose['actual_right_pwm'], 6900)
        self.assertEqual(pose['launch_phase'], 2)
        self.assertEqual(target, {
            'x': 250.0,
            'y': 300.0,
            't': target['t'],
            'source': 'vrpl1',
            'role': 'target',
            'id': 1,
            'target_index': 9,
            'lookahead_distance_mm': 180.0,
            'actual_target_distance_mm': (150.0 ** 2 + 100.0 ** 2) ** 0.5,
        })

    def test_slip_packet_marks_detection_and_actual_correction_delta(self):
        pathcapture.process_line('{pathinfo}replay_start,100')
        pathcapture.process_line('{pathinfo}replay_drive_start')
        pathcapture.process_line('{vrpl0}50000,7,9,180,0,100,1')
        pathcapture.process_line('{vrpl1}100,200,30,90,190,28,12,-3,250,300')
        self.assertTrue(pathcapture.process_line(
            '{vslip}1,1,0,30.0,20.0,0.667,8.0,0.0,0.0,0.0'))

        first = pathcapture.parsed_data['realtime'][0]
        self.assertEqual(first['slip_confirmed'], 1)
        self.assertEqual(first['slip_straight_ready'], 0)
        self.assertEqual(first['slip_pending_correction_mm'], 8.0)
        self.assertEqual(first['slip_correction_delta_mm'], 0.0)

        pathcapture.process_line('{vrpl0}50000,10,12,180,0,100,1')
        pathcapture.process_line('{vrpl1}120,220,30,110,210,28,10,-2,270,320')
        self.assertTrue(pathcapture.process_line(
            '{vslip}1,1,1,30.0,20.0,0.667,5.5,2.5,2.0,1.5'))

        second = pathcapture.parsed_data['realtime'][1]
        self.assertEqual(second['slip_straight_ready'], 1)
        self.assertEqual(second['slip_total_correction_mm'], 2.5)
        self.assertEqual(second['slip_correction_delta_mm'], 2.5)

    def test_slip_packet_parses_window_diagnostics(self):
        pathcapture.process_line('{pathinfo}replay_start,100')
        pathcapture.process_line('{vrpl0}50000,7,9,180,0,100,1')
        pathcapture.process_line('{vrpl1}100,200,30,90,190,28,12,-3,250,300')
        self.assertTrue(pathcapture.process_line(
            '{vslip}1,0,0,12.0,8.0,0.667,0.0,0.0,0.0,0.0,1,15,11,4,150,1'))
        point = pathcapture.parsed_data['realtime'][0]
        self.assertEqual(point['slip_window_ready'], 1)
        self.assertEqual(point['slip_window_bucket_count'], 15)
        self.assertEqual(point['slip_window_valid_count'], 11)
        self.assertEqual(point['slip_reject_reason'], 4)
        self.assertEqual(point['slip_window_ms'], 150.0)
        self.assertEqual(point['slip_turn_sign'], 1)

    def test_phototube_packet_distinguishes_monitor_gate_and_apply(self):
        cases = (
            ('monitor', '0,0,1,1,4.0,0.85,3200,105,205,22.5,100,200,7.1,-5,-5,0,0,1,0,0,0.2,0.5,3,8,1,35,90,190,1,1,0,1'),
            ('gated', '1,0,1,1,4.0,0.85,3200,105,205,22.5,100,200,7.1,-5,-5,-1,-1,1,0,3,0.2,0.5,3,8,1,35,90,190,0,1,1,1'),
            ('applied', '1,1,1,1,4.0,0.85,3200,105,205,22.5,100,200,7.1,-5,-5,-1,-1,1,1,8,0.2,0.5,3,8,1,35,90,190,0,1,1,1'),
        )
        for expected, payload in cases:
            with self.subTest(expected=expected):
                self.setUp()
                pathcapture.process_line('{vrpl0}50000,7,9,180,0,100,1')
                pathcapture.process_line('{vrpl1}100,200,30,90,190,28,12,-3,250,300')
                self.assertTrue(pathcapture.process_line(f'{{ptcorr}}{payload}'))
                point = pathcapture.parsed_data['realtime'][0]
                self.assertEqual(point['phototube_line_valid'], 1)
                self.assertEqual(point['phototube_projection_valid'], 1)
                self.assertEqual(point['phototube_sensor_powered'], 1)
                if expected == 'monitor':
                    self.assertEqual(point['phototube_monitor_enabled'], 1)
                    self.assertEqual(point['phototube_correction_enabled'], 0)
                elif expected == 'gated':
                    self.assertEqual(point['phototube_correction_enabled'], 1)
                    self.assertEqual(point['phototube_gate_pass'], 0)
                    self.assertEqual(point['phototube_correction_applied'], 0)
                else:
                    self.assertEqual(point['phototube_gate_pass'], 1)
                    self.assertEqual(point['phototube_correction_applied'], 1)

    def test_sparse_slip_and_phototube_events_attach_to_latest_point(self):
        pathcapture.process_line('{pathinfo}replay_start,100')
        pathcapture.process_line('{vrpl0}25000,7,9,180,0,100,1')
        pathcapture.process_line('{vrpl1}100,200,30,90,190,28,12,-3,250,300')

        self.assertTrue(pathcapture.process_line(
            '{vslipevt}3,77,111.0,222.0,0.650,30.0,19.5,8.0,2.0'))
        self.assertTrue(pathcapture.process_line(
            '{vslipacc}4,1,78,112.0,223.0,9000.0,3000.0,0.333'))
        self.assertTrue(pathcapture.process_line(
            '{ptcorrevt}79.5,113.0,224.0,0.850,-1.2,2.4'))

        point = pathcapture.parsed_data['realtime'][0]
        self.assertEqual(point['turn_slip_event'], 1)
        self.assertEqual(point['turn_slip_path_index'], 77)
        self.assertEqual(point['accel_slip_event_state'], 1)
        self.assertEqual(point['accel_slip_path_index'], 78)
        self.assertEqual(point['phototube_correction_event'], 1)
        self.assertEqual(point['phototube_event_path_index'], 79.5)
        self.assertEqual(point['phototube_correction_y_mm'], 2.4)

    def test_browser_batch_preserves_all_events_and_metadata_order(self):
        with mock.patch.object(pathcapture.socketio, 'emit') as emit_mock:
            with pathcapture.browser_batch_lock:
                pathcapture.browser_batching = True
            try:
                for index in range(1000):
                    pathcapture.process_line(
                        f'{{vrpl1}}{index},{index + 1},30,90,190,28,12,-3,250,300')
                    pathcapture.process_line(
                        f'{{vrpl2}}35,34,5,2500,2300,2700,{index},2650,400,120,130,0,390')

                self.assertEqual(pathcapture._flush_browser_events(), 2000)
            finally:
                with pathcapture.browser_batch_lock:
                    pathcapture.browser_batching = False

        batch_calls = [call for call in emit_mock.call_args_list if call.args[0] == 'serial_batch']
        self.assertEqual(len(batch_calls), 1)
        events = batch_calls[0].args[1]['events']
        self.assertEqual(len(events), 2000)
        self.assertEqual(events[0]['event'], 'serial_data')
        self.assertEqual(events[1]['event'], 'serial_point_update')
        self.assertEqual(events[-2]['data']['car']['actual_left_speed_mm_s'], 999.0)
        self.assertEqual(len(pathcapture.parsed_data['realtime']), 1000)

    def test_legacy_13_field_vrpl2_keeps_speed_data(self):
        pathcapture.process_line('{vrpl1}100,200,30,90,190,28,12,-3,250,300')
        self.assertTrue(pathcapture.process_line(
            '{vrpl2}35,34,5,2500,2300,2700,2250,2650,400,120,130,0,390'))

        pose = pathcapture.parsed_data['realtime'][0]
        self.assertEqual(pose['target_speed_mm_s'], 2500.0)
        self.assertEqual(pose['actual_left_speed_mm_s'], 2250.0)
        self.assertEqual(pose['actual_right_speed_mm_s'], 2650.0)
        self.assertNotIn('raw_plan_speed_mm_s', pose)

    def test_compact_replay_status_and_pwm_packets(self):
        self.assertTrue(pathcapture.process_line('{vrpl0}50000,7,9,180,1,3441'))
        self.assertTrue(pathcapture.process_line(
            '{vrpl1}100,200,30,90,190,28,12,-3,250,300'))
        self.assertTrue(pathcapture.process_line('{vrpl3}6000,7000,6100,6900,2,0,2,0'))

        pose = pathcapture.parsed_data['realtime'][0]
        self.assertEqual(pose['sample_dt_ms'], 50.0)
        self.assertEqual(pose['replay_cursor_index'], 7)
        self.assertEqual(pose['lookahead_distance_mm'], 180.0)
        self.assertEqual(pose['path_point_count'], 3441)
        self.assertEqual(pose['target_left_pwm'], 6000)
        self.assertEqual(pose['actual_right_pwm'], 6900)
        self.assertEqual(pose['right_esc_fault'], 0)
        self.assertNotIn('track_mode', pose)

    def test_compact_replay_track_mode_is_attached_to_pose_and_target(self):
        self.assertTrue(pathcapture.process_line('{vrpl0}50000,7,9,180,0,3441,1'))
        self.assertTrue(pathcapture.process_line(
            '{vrpl1}100,200,30,90,190,28,12,-3,250,300'))

        pose = pathcapture.parsed_data['realtime'][0]
        target = pathcapture.parsed_data['lookahead'][0]
        expected_distance = (150.0 ** 2 + 100.0 ** 2) ** 0.5
        self.assertEqual(pose['track_mode'], 1)
        self.assertAlmostEqual(pose['actual_target_distance_mm'], expected_distance)
        self.assertEqual(target['track_mode'], 1)
        self.assertEqual(target['target_index'], 9)
        self.assertEqual(target['lookahead_distance_mm'], 180.0)
        self.assertAlmostEqual(target['actual_target_distance_mm'], expected_distance)

    def test_partial_and_non_finite_packets_are_dropped(self):
        self.assertFalse(pathcapture.process_line('{vrpl1}100,200,30'))
        self.assertFalse(pathcapture.process_line(
            '{vrpl1}nan,200,30,90,190,28,12,-3,250,300'))
        self.assertEqual(pathcapture.parsed_data['realtime'], [])
        self.assertEqual(pathcapture.parsed_data['lookahead'], [])

    def test_missing_pose_does_not_attach_following_fields_to_previous_sample(self):
        pathcapture.process_line('{vrpl1}100,200,30,90,190,28,12,-3,250,300')
        pathcapture.process_line('{vrpl2}35,34,5,2500,2300,2700,2250,2650,400,120,130,0,390,2600,2500')
        previous = pathcapture.parsed_data['realtime'][0]
        self.assertEqual(previous['target_speed_mm_s'], 2500.0)

        pathcapture.process_line('{vrpl0}20000,8,10,8,10,180,1,300,43,0,0,0')
        self.assertFalse(pathcapture.process_line('{vrpl1}broken'))
        pathcapture.process_line('{vrpl2}35,34,5,9999,1,2,3,4,5,6,7,8,9,10,11')

        self.assertEqual(len(pathcapture.parsed_data['realtime']), 1)
        self.assertEqual(previous['target_speed_mm_s'], 2500.0)

    def test_unrelated_and_deep_diagnostic_packets_are_ignored(self):
        self.assertFalse(pathcapture.process_line('{startup}imu_ready'))
        self.assertTrue(pathcapture.process_line('{vrpl6}1,2,3,4'))

    def test_unhandled_braced_packet_is_shown_once(self):
        line = '{imustart}502,17000,0,0,1'

        self.assertFalse(pathcapture.process_line(line))

        self.assertEqual(
            [entry['text'] for entry in pathcapture.console_lines],
            [line]
        )
        self.assertTrue(pathcapture.process_line('{imu}1,2,3,4'))
        self.assertTrue(all(not points for points in pathcapture.parsed_data.values()))

    def test_upload_status_is_visible_but_point_ack_is_hidden(self):
        status = '{pathupload}status,0,3441,3441,32760,0'

        self.assertFalse(pathcapture.process_line(status))
        self.assertFalse(pathcapture.process_line('{pathupload}point,1,123,124'))

        self.assertEqual(
            [entry['text'] for entry in pathcapture.console_lines],
            [status]
        )

    def test_pathdump_accepts_points_and_rejects_partial_lines(self):
        self.assertTrue(pathcapture.process_line('{pathdump}begin,2'))
        self.assertTrue(pathcapture.process_line('{pathdump}0,100,200,1.57,500'))
        self.assertFalse(pathcapture.process_line('{pathdump}1,120'))
        self.assertEqual(len(pathcapture.parsed_data['predefined']), 1)
        self.assertEqual(pathcapture.parsed_data['predefined'][0]['speed_mm_s'], 500.0)

    def test_legacy_packets_require_a_complete_line(self):
        self.assertTrue(pathcapture.process_line('P(1.5,-2.3)'))
        self.assertFalse(pathcapture.process_line('P(2.0,3.0)truncated'))
        self.assertTrue(pathcapture.process_line('R(1,2),(3,4)'))
        self.assertEqual(len(pathcapture.parsed_data['predefined']), 1)
        self.assertEqual(len(pathcapture.parsed_data['realtime']), 1)

    def test_record_path_is_only_cleared_after_vehicle_confirmation(self):
        pathcapture.process_line('P(10,20)')
        pathcapture.process_line('{pathinfo}record_prepare')
        self.assertEqual(len(pathcapture.parsed_data['predefined']), 1)
        self.assertEqual(pathcapture.runtime_mode['mode'], 'record_prepare')

        pathcapture.process_line('{pathinfo}record_start')
        self.assertEqual(pathcapture.parsed_data['predefined'], [])
        self.assertEqual(pathcapture.runtime_mode['mode'], 'recording')

    def test_replay_confirmation_creates_timed_run_and_vstop_finishes_it(self):
        with mock.patch.object(pathcapture.time, 'time', side_effect=[10.0, 10.1, 10.4]):
            pathcapture.process_line('{pathinfo}replay_start')
            pathcapture.process_line('{vrpl1}100,200,30,90,190,28,12,-3,250,300')
            pathcapture.process_line('{vstop}ok')

        self.assertEqual(len(pathcapture.parsed_data['replay_runs']), 1)
        run = pathcapture.parsed_data['replay_runs'][0]
        self.assertEqual(run['status'], 'stopped')
        self.assertEqual(len(run['points']), 1)
        self.assertAlmostEqual(run['points'][0]['elapsed_s'], 0.1)
        self.assertIsNone(pathcapture.active_replay_run)

    def test_legacy_flat_data_becomes_one_replay_run(self):
        points = [{'x': 1, 'y': 2, 't': 3.0}, {'x': 2, 'y': 3, 't': 3.1}]
        targets = [{'x': 4, 'y': 5, 't': 3.0}]
        runs = pathcapture._normalize_loaded_runs([], points, targets)
        self.assertEqual(len(runs), 1)
        self.assertEqual(runs[0]['status'], 'imported')
        self.assertIs(runs[0]['points'], points)


class SerialCommandTest(unittest.TestCase):
    class FakeSerial:
        is_open = True

        def __init__(self):
            self.writes = []

        def write(self, payload):
            self.writes.append(payload)

        def flush(self):
            pass

    def setUp(self):
        self.client = pathcapture.socketio.test_client(pathcapture.app)
        self.fake_serial = self.FakeSerial()
        pathcapture.serial_conn = self.fake_serial
        pathcapture.serial_running = True
        with pathcapture.data_lock:
            pathcapture.console_lines.clear()
        self.client.get_received()

    def tearDown(self):
        pathcapture.serial_conn = None
        pathcapture.serial_running = False
        self.client.disconnect()

    def _results(self):
        return [item['args'][0] for item in self.client.get_received()
                if item['name'] == 'command_result']

    def test_command_is_sent_as_one_crlf_terminated_line(self):
        self.client.emit('send_serial_command', {'command': 'vstop'})
        self.assertEqual(self.fake_serial.writes, [b'vstop\r\n'])
        self.assertTrue(self._results()[-1]['success'])
        self.assertEqual(pathcapture.console_lines[-1]['direction'], 'tx')

    def test_embedded_newline_is_rejected(self):
        self.client.emit('send_serial_command', {'command': 'vstop\nvpath replay'})
        self.assertEqual(self.fake_serial.writes, [])
        self.assertFalse(self._results()[-1]['success'])

    def test_disconnected_serial_returns_clear_error(self):
        pathcapture.serial_running = False
        self.client.emit('send_serial_command', {'command': 'vstop'})
        result = self._results()[-1]
        self.assertFalse(result['success'])
        self.assertIn('未连接', result['error'])


class RawLogImportTest(unittest.TestCase):
    def setUp(self):
        self.client = pathcapture.app.test_client()

    def test_noisy_dat_is_filtered_and_timed(self):
        content = '\n'.join([
            'unrelated startup output',
            '{pathdump}begin,1',
            '{pathdump}0,10,20,0.0,500',
            '{pathinfo}replay_start',
            '{vrpl0}50000,0,1,0,0,120,0,1,0,0,0,0',
            '{vrpl1}10,20,0,10,20,0,0,0,30,40',
            '{vrpl0}52000,1,1,0,0,120,0,1,0,0,0,0',
            '{vrpl1}20,30,0,20,30,0,0,0,30,40',
            '{broken half packet',
            '{vstop}ok'
        ]).encode('utf-8')
        response = self.client.post('/api/import-log', data={
            'file': (io.BytesIO(content), 'mixed.dat')
        })

        self.assertEqual(response.status_code, 200)
        dataset = response.get_json()
        self.assertEqual(dataset['import_stats']['total_lines'], 10)
        self.assertEqual(dataset['import_stats']['filtered_lines'], 2)
        self.assertEqual(dataset['import_stats']['path_points'], 1)
        self.assertEqual(dataset['import_stats']['replay_runs'], 1)
        self.assertEqual(len(dataset['replay_runs'][0]['points']), 2)
        self.assertAlmostEqual(dataset['replay_runs'][0]['end_t'], 0.052)

    def test_raw_import_rejects_unsupported_extension(self):
        response = self.client.post('/api/import-log', data={
            'file': (io.BytesIO(b'{}'), 'dataset.json')
        })
        self.assertEqual(response.status_code, 400)


class RuntimeResetTest(unittest.TestCase):
    def setUp(self):
        self.client = pathcapture.app.test_client()

    def test_http_reset_clears_runtime_without_disconnecting_serial(self):
        sentinel_serial = object()
        pathcapture.serial_conn = sentinel_serial
        pathcapture.serial_running = True
        pathcapture.serial_buffer = 'partial packet'
        with pathcapture.data_lock:
            pathcapture.parsed_data['predefined'].append({'x': 1, 'y': 2})
            pathcapture.parsed_data['replay_runs'].append({'id': 1})
            pathcapture.console_lines.append({'text': 'old'})
        with pathcapture.raw_log_lock:
            pathcapture.raw_log_lines.append('{vrpl1}old')

        response = self.client.post('/api/reset-data')

        self.assertEqual(response.status_code, 200)
        self.assertTrue(response.get_json()['success'])
        self.assertEqual(pathcapture.serial_buffer, '')
        self.assertTrue(all(not values for values in pathcapture.parsed_data.values()))
        self.assertEqual(pathcapture.console_lines, [])
        self.assertEqual(list(pathcapture.raw_log_lines), [])
        self.assertIs(pathcapture.serial_conn, sentinel_serial)
        self.assertTrue(pathcapture.serial_running)
        self.assertEqual(pathcapture.runtime_mode['mode'], 'idle')

        pathcapture.serial_conn = None
        pathcapture.serial_running = False

    def test_raw_export_contains_unfiltered_rx_lines(self):
        with pathcapture.raw_log_lock:
            pathcapture.raw_log_lines.clear()
        pathcapture.process_line('{vrpl6}1,2,3,4')
        pathcapture.process_line('{vrpl1}10,20,0,10,20,0,0,0,30,40')

        response = self.client.get('/api/export-raw-log')

        self.assertEqual(response.status_code, 200)
        self.assertIn('attachment; filename="main-', response.headers['Content-Disposition'])
        self.assertEqual(
            response.get_data(as_text=True),
            '{vrpl6}1,2,3,4\r\n{vrpl1}10,20,0,10,20,0,0,0,30,40\r\n'
        )


if __name__ == '__main__':
    unittest.main()
