#!/usr/bin/env python3
"""Convert noisy Mad_Circuits serial logs into PathCapture datasets."""

import json
import math
import os
import sys
from datetime import datetime

PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_DIR not in sys.path:
    sys.path.insert(0, PROJECT_DIR)

import app as pathcapture  # noqa: E402

DEFAULT_SAMPLE_INTERVAL_S = 0.05
MIN_SAMPLE_INTERVAL_MS = 1.0
MAX_SAMPLE_INTERVAL_MS = 1000.0


def _reset_parser():
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
    pathcapture.runtime_mode = {'mode': 'idle', 'reason': 'log_import', 'updated_at': 0.0}


def _rebuild_run_timeline(run):
    points = run.get('points') or []
    if not points:
        run['start_t'] = 0.0
        run['end_t'] = 0.0
        return

    elapsed = 0.0
    elapsed_by_id = {}
    for index, point in enumerate(points):
        if index:
            interval_ms = point.get('sample_dt_ms')
            if not isinstance(interval_ms, (int, float)) or not math.isfinite(interval_ms) \
                    or not MIN_SAMPLE_INTERVAL_MS <= interval_ms <= MAX_SAMPLE_INTERVAL_MS:
                interval_ms = DEFAULT_SAMPLE_INTERVAL_S * 1000.0
            elapsed += interval_ms / 1000.0
        point['elapsed_s'] = elapsed
        point['t'] = elapsed
        if 'id' in point:
            elapsed_by_id[str(point['id'])] = elapsed

    for index, target in enumerate(run.get('lookahead') or []):
        target_elapsed = elapsed_by_id.get(str(target.get('id')))
        if target_elapsed is None:
            target_elapsed = points[min(index, len(points) - 1)]['elapsed_s']
        target['elapsed_s'] = target_elapsed
        target['t'] = target_elapsed

    run['start_t'] = 0.0
    run['end_t'] = elapsed
    if run.get('status') == 'running':
        run['status'] = 'imported'
        run['reason'] = 'log_ended'


def convert_log(input_path):
    _reset_parser()
    original_emit = pathcapture.socketio.emit
    pathcapture.socketio.emit = lambda *args, **kwargs: None
    total_lines = 0
    accepted_lines = 0

    try:
        with open(input_path, 'rb') as source:
            for raw_line in source:
                total_lines += 1
                line = raw_line.decode('utf-8', errors='replace').strip('\r\n')
                if pathcapture.process_line(line):
                    accepted_lines += 1

        with pathcapture.data_lock:
            predefined = list(pathcapture.parsed_data['predefined'])
            realtime = list(pathcapture.parsed_data['realtime'])
            lookahead = list(pathcapture.parsed_data['lookahead'])
            runs = list(pathcapture.parsed_data['replay_runs'])
    finally:
        pathcapture.socketio.emit = original_emit

    if not runs and realtime:
        runs = pathcapture._normalize_loaded_runs([], realtime, lookahead)
    for run in runs:
        _rebuild_run_timeline(run)
    if runs:
        realtime = runs[-1].get('points', [])
        lookahead = runs[-1].get('lookahead', [])

    return {
        'version': '3.0',
        'saved_at': datetime.now().isoformat(),
        'source': 'raw_log_import',
        'predefined': predefined,
        'realtime': realtime,
        'lookahead': lookahead,
        'replay_runs': runs,
        'import_stats': {
            'total_lines': total_lines,
            'accepted_lines': accepted_lines,
            'filtered_lines': total_lines - accepted_lines,
            'path_points': len(predefined),
            'replay_runs': len(runs),
            'replay_points': sum(len(run.get('points') or []) for run in runs)
        }
    }


def main():
    if len(sys.argv) != 3:
        raise SystemExit('usage: log_converter.py INPUT.dat OUTPUT.json')
    dataset = convert_log(sys.argv[1])
    with open(sys.argv[2], 'w', encoding='utf-8') as output:
        json.dump(dataset, output, ensure_ascii=False, separators=(',', ':'))


if __name__ == '__main__':
    main()
