#!/usr/bin/env python3
"""
PathCapture - High-Performance Real-Time Path Display Tool

Backend: Flask + Flask-SocketIO + pySerial
Receives serial data from embedded devices and streams to browser via WebSocket.
"""

import os
import sys
import re
import json
import math
import time
import threading
import logging
import subprocess
import tempfile
import struct
from collections import deque
from datetime import datetime

from flask import Flask, Response, request, jsonify, send_from_directory
from flask_socketio import SocketIO, emit
import serial
import serial.tools.list_ports
from path_uploader import PathUploadError, PathUploadManager

HEADLESS = sys.stdout is None
if HEADLESS:
    sys.stdout = open(os.devnull, 'w', encoding='utf-8')
if sys.stderr is None:
    sys.stderr = open(os.devnull, 'w', encoding='utf-8')

# ── Configuration ──────────────────────────────────────────────
HOST = os.environ.get('PATHCAPTURE_HOST', 'localhost')
PORT = int(os.environ.get('PATHCAPTURE_PORT', '5000'))
DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data')
MAX_LAYER_POINTS = 100000
MAX_SERIAL_BUFFER_CHARS = 8192
MAX_REPLAY_RUNS = 30
MAX_CONSOLE_LINES = 500
MAX_RAW_LOG_LINES = 500000
MAX_COMMAND_CHARS = 128
MAX_IMPORT_BYTES = 256 * 1024 * 1024
SERIAL_BROWSER_BATCH_INTERVAL_S = 0.020
RAW_LOG_EXTENSIONS = frozenset({'.dat', '.log', '.txt'})

# ── Logging ────────────────────────────────────────────────────
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(message)s',
    handlers=([logging.FileHandler(
        os.path.join(os.path.dirname(os.path.abspath(__file__)), 'pathcapture.log'),
        encoding='utf-8'
    )] if HEADLESS else None)
)
logger = logging.getLogger(__name__)

# ── App Setup ──────────────────────────────────────────────────
app = Flask(__name__)
app.config['SECRET_KEY'] = 'pathcapture-secret-key-2024'
socketio = SocketIO(
    app,
    cors_allowed_origins='*',
    async_mode='threading',
    ping_timeout=30,
    ping_interval=10,
    max_http_buffer_size=8 * 1024 * 1024
)

# ── Serial State ───────────────────────────────────────────────
serial_conn = None
serial_lock = threading.Lock()
serial_running = False
serial_buffer = ''
serial_byte_buffer = bytearray()
path_upload_manager = None

# ── Data Storage ───────────────────────────────────────────────
parsed_data = {
    'predefined': [],     # [{x, y, t}, ...]
    'realtime': [],       # [{x, y, t}, ...]
    'lookahead': [],      # [{x, y, t}, ...]
    'replay_runs': [],    # [{id, status, points, lookahead, ...}, ...]
}
console_lines = []
raw_log_lines = deque(maxlen=MAX_RAW_LOG_LINES)
data_lock = threading.Lock()
raw_log_lock = threading.Lock()
browser_batch_lock = threading.Lock()
browser_event_batch = []
browser_batching = False
replay_point_seq = 0
latest_replay_point_id = None
pending_replay_meta = {}
replay_sample_open = False
replay_run_seq = 0
active_replay_run = None
last_slip_total_correction_mm = 0.0
runtime_mode = {
    'mode': 'idle',
    'reason': 'startup',
    'updated_at': 0.0
}

# ── Regex Patterns ─────────────────────────────────────────────
# Legacy motorcycle protocol:
# R(car_x, car_y),(lookahead_x, lookahead_y)
RE_R = re.compile(r'^R\((-?\d+\.?\d*(?:[eE][+-]?\d+)?),(-?\d+\.?\d*(?:[eE][+-]?\d+)?)\),'
                  r'\((-?\d+\.?\d*(?:[eE][+-]?\d+)?),(-?\d+\.?\d*(?:[eE][+-]?\d+)?)\)$')
# P(x, y)
RE_P = re.compile(r'^P\((-?\d+\.?\d*(?:[eE][+-]?\d+)?),(-?\d+\.?\d*(?:[eE][+-]?\d+)?)\)$')
# Current Mad_Circuits protocol:
# {pathdump}index,x_mm,y_mm,theta_rad,speed_mm_s
# {path}x_mm,y_mm,theta_deg,index,dist_mm,...
# {vrpl1}x_mm,y_mm,theta_deg,base_x_mm,base_y_mm,base_theta_deg,cte_mm,along_mm,target_x_mm,target_y_mm
RE_BRACED = re.compile(r'^\{([^}]+)\}(.*)$')
IGNORED_BRACED_TAGS = frozenset({
    'vrpl5', 'vrpl6', 'vrpl7', 'imu', 'ptline', 'ptcal', 'ptmin', 'ptmax',
    'ptctr', 'ptblue', 'ptwhite', 'ptoff', 'ptgain'
})
HIGH_RATE_TAGS = frozenset({
    'path', 'pathdump', 'vrpl0', 'vrpl1', 'vrpl2', 'vrpl3', 'vrpl4',
    'vrpl5', 'vrpl6', 'vrpl7', 'vrplg', 'vrplm', 'vslip', 'vslipevt',
    'vslipacc', 'ptcorr', 'ptcorrevt', 'vwin', 'pathupload'
})
REPLAY_BINARY_SYNC = b'\xA5\x5A'
REPLAY_BINARY_VERSION = 1
REPLAY_BINARY_SAMPLE_TYPE = 1
REPLAY_BINARY_SLIP_TYPE = 2
REPLAY_BINARY_PAYLOAD_SIZE = 128
REPLAY_BINARY_FRAME_SIZE = 138
REPLAY_BINARY_SAMPLE_FORMAT = '<IHHfBHB10f15f4h4B'
REPLAY_BINARY_SLIP_PAYLOAD_SIZE = 40
REPLAY_BINARY_SLIP_FORMAT = '<4B7f4Bf'


def ensure_data_dir():
    """Create data directory if it doesn't exist."""
    os.makedirs(DATA_DIR, exist_ok=True)


# ── Serial Processing ──────────────────────────────────────────

def _csv_parts(payload: str):
    return [part.strip() for part in payload.strip().split(',')]


def _append_limited(items: list, item, limit: int):
    items.append(item)
    overflow = len(items) - limit
    if overflow > 0:
        del items[:overflow]


def _normalize_loaded_runs(replay_runs, realtime, lookahead):
    """Return replay runs for both v3 files and legacy v2 flat telemetry."""
    if isinstance(replay_runs, list) and replay_runs:
        return replay_runs[:MAX_REPLAY_RUNS]
    if not realtime:
        return []

    start_t = realtime[0].get('t', 0.0) if isinstance(realtime[0], dict) else 0.0
    end_t = realtime[-1].get('t', start_t) if isinstance(realtime[-1], dict) else start_t
    return [{
        'id': 1,
        'status': 'imported',
        'reason': 'legacy_import',
        'start_t': start_t,
        'end_t': end_t,
        'points': realtime,
        'lookahead': lookahead
    }]


def _emit_browser_event(event: str, data: dict):
    """Batch serial-originated browser events while the reader is active."""
    with browser_batch_lock:
        if browser_batching:
            browser_event_batch.append({'event': event, 'data': data})
            return
    socketio.emit(event, data)


def _flush_browser_events():
    with browser_batch_lock:
        if not browser_event_batch:
            return 0
        events = browser_event_batch[:]
        browser_event_batch.clear()
    socketio.emit('serial_batch', {'events': events})
    return len(events)


def _emit_console(direction: str, text: str, t: float):
    entry = {'direction': direction, 'text': text, 't': t}
    with data_lock:
        _append_limited(console_lines, entry, MAX_CONSOLE_LINES)
    _emit_browser_event('serial_console', entry)


def _capture_raw_rx_line(line: str):
    with raw_log_lock:
        raw_log_lines.append(line)


def _crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = ((crc >> 1) ^ 0xA001) if crc & 1 else crc >> 1
    return crc & 0xFFFF


def decode_replay_binary_frame(frame: bytes) -> list[str]:
    if len(frame) < 10 or frame[:2] != REPLAY_BINARY_SYNC:
        raise ValueError('invalid replay telemetry frame')
    version, frame_type, _sequence, payload_size = struct.unpack_from('<BBHH', frame, 2)
    if version != REPLAY_BINARY_VERSION or len(frame) != payload_size + 10:
        raise ValueError('unsupported replay telemetry frame')
    expected_crc = struct.unpack_from('<H', frame, len(frame) - 2)[0]
    actual_crc = _crc16_modbus(frame[2:-2])
    if actual_crc != expected_crc:
        raise ValueError('replay telemetry crc mismatch')

    if frame_type == REPLAY_BINARY_SAMPLE_TYPE and payload_size == REPLAY_BINARY_PAYLOAD_SIZE:
        values = struct.unpack_from(REPLAY_BINARY_SAMPLE_FORMAT, frame, 8)
        vrpl0 = values[:7]
        vrpl1 = values[7:17]
        vrpl2 = values[17:32]
        vrpl3 = values[32:40]
        return [
            '{vrpl0}' + ','.join(str(value) for value in vrpl0),
            '{vrpl1}' + ','.join(format(value, '.9g') for value in vrpl1),
            '{vrpl2}' + ','.join(format(value, '.9g') for value in vrpl2),
            '{vrpl3}' + ','.join(str(value) for value in vrpl3),
        ]
    if frame_type == REPLAY_BINARY_SLIP_TYPE and payload_size == REPLAY_BINARY_SLIP_PAYLOAD_SIZE:
        values = struct.unpack_from(REPLAY_BINARY_SLIP_FORMAT, frame, 8)
        active, confirmed, straight_ready, window_ready = values[:4]
        encoder_deg, imu_deg, ratio, pending_mm, total_mm, correction_x, correction_y = values[4:11]
        bucket_count, valid_count, reject_reason, turn_sign_raw = values[11:15]
        window_ms = values[15]
        turn_sign = turn_sign_raw - 256 if turn_sign_raw >= 128 else turn_sign_raw
        fields = (active, confirmed, straight_ready, encoder_deg, imu_deg, ratio,
                  pending_mm, total_mm, correction_x, correction_y, window_ready,
                  bucket_count, valid_count, reject_reason, window_ms, turn_sign)
        return ['{vslip}' + ','.join(format(value, '.9g') for value in fields)]
    raise ValueError('unsupported replay telemetry frame type')


def process_serial_bytes(data: bytes):
    global serial_buffer, serial_byte_buffer

    serial_byte_buffer.extend(data)
    while serial_byte_buffer:
        sync_index = serial_byte_buffer.find(REPLAY_BINARY_SYNC)
        newline_index = serial_byte_buffer.find(b'\n')

        if sync_index == 0:
            if len(serial_byte_buffer) < 8:
                return
            payload_size = struct.unpack_from('<H', serial_byte_buffer, 6)[0]
            frame_size = 10 + payload_size
            if payload_size > 1024:
                del serial_byte_buffer[0]
                continue
            if len(serial_byte_buffer) < frame_size:
                return
            frame = bytes(serial_byte_buffer[:frame_size])
            try:
                lines = decode_replay_binary_frame(frame)
            except ValueError:
                del serial_byte_buffer[:frame_size]
                continue
            del serial_byte_buffer[:frame_size]
            for line in lines:
                process_line(line)
            continue

        if newline_index >= 0 and (sync_index < 0 or newline_index < sync_index):
            line_bytes = bytes(serial_byte_buffer[:newline_index])
            del serial_byte_buffer[:newline_index + 1]
            line = serial_buffer + line_bytes.decode('utf-8', errors='replace')
            serial_buffer = ''
            process_line(line)
            continue

        if sync_index > 0:
            text_bytes = bytes(serial_byte_buffer[:sync_index])
            del serial_byte_buffer[:sync_index]
            serial_buffer += text_bytes.decode('utf-8', errors='replace')
            text_lines = serial_buffer.split('\n')
            serial_buffer = text_lines.pop()
            for line in text_lines:
                process_line(line)
            if len(serial_buffer) > MAX_SERIAL_BUFFER_CHARS:
                serial_buffer = serial_buffer[-MAX_SERIAL_BUFFER_CHARS:]
            continue

        if sync_index < 0:
            keep = 1 if serial_byte_buffer[-1:] == REPLAY_BINARY_SYNC[:1] else 0
            text_length = len(serial_byte_buffer) - keep
            if text_length > 0:
                serial_buffer += bytes(serial_byte_buffer[:text_length]).decode('utf-8', errors='replace')
                del serial_byte_buffer[:text_length]
            return


def _path_upload_serial_write(command: str):
    with serial_lock:
        if not serial_running or serial_conn is None or not serial_conn.is_open:
            raise PathUploadError('串口未连接')
        serial_conn.write((command + '\r\n').encode('utf-8'))
        serial_conn.flush()
    if not command.startswith('vpath upload point '):
        _emit_console('tx', command, time.time())


def _path_upload_serial_write_bytes(data: bytes):
    with serial_lock:
        if not serial_running or serial_conn is None or not serial_conn.is_open:
            raise PathUploadError('串口未连接')
        serial_conn.write(data)
        serial_conn.flush()


path_upload_manager = PathUploadManager(
    _path_upload_serial_write,
    socketio.emit,
    _path_upload_serial_write_bytes
)


def _mode_set(mode: str, reason: str, t: float):
    global runtime_mode
    runtime_mode = {'mode': mode, 'reason': reason, 'updated_at': t}
    socketio.emit('mode_status', runtime_mode)


def _reset_runtime_data(reason: str = 'data_cleared'):
    """Reset parser/session state without touching the serial connection or MCU Flash."""
    global serial_buffer, serial_byte_buffer, replay_point_seq, latest_replay_point_id
    global pending_replay_meta, replay_sample_open, replay_run_seq, active_replay_run
    global last_slip_total_correction_mm

    path_upload_manager.cancel()
    serial_buffer = ''
    serial_byte_buffer.clear()
    with browser_batch_lock:
        browser_event_batch.clear()
    with raw_log_lock:
        raw_log_lines.clear()
    with data_lock:
        parsed_data['predefined'].clear()
        parsed_data['realtime'].clear()
        parsed_data['lookahead'].clear()
        parsed_data['replay_runs'].clear()
        console_lines.clear()
        replay_point_seq = 0
        latest_replay_point_id = None
        pending_replay_meta = {}
        replay_sample_open = False
        replay_run_seq = 0
        active_replay_run = None
        last_slip_total_correction_mm = 0.0

    now = time.time()
    _mode_set('idle', reason, now)
    return {'success': True, 'reason': reason, 't': now}


def _replay_run_start(t: float, reason: str):
    global replay_run_seq, active_replay_run, latest_replay_point_id
    global last_slip_total_correction_mm
    if active_replay_run is not None:
        _replay_run_finish('interrupted', t, 'new_replay_started')

    replay_run_seq += 1
    run = {
        'id': replay_run_seq,
        'status': 'running',
        'reason': reason,
        'start_t': t,
        'end_t': None,
        'points': [],
        'lookahead': []
    }
    with data_lock:
        parsed_data['realtime'].clear()
        parsed_data['lookahead'].clear()
        _append_limited(parsed_data['replay_runs'], run, MAX_REPLAY_RUNS)
    latest_replay_point_id = None
    last_slip_total_correction_mm = 0.0
    active_replay_run = run
    socketio.emit('replay_run_started', {
        'run': {key: value for key, value in run.items() if key not in ('points', 'lookahead')}
    })
    _mode_set('replaying', reason, t)


def _replay_run_finish(status: str, t: float, reason: str):
    global active_replay_run
    run = active_replay_run
    if run is not None:
        run['status'] = status
        run['reason'] = reason
        run['end_t'] = t
        socketio.emit('replay_run_finished', {
            'id': run['id'],
            'status': status,
            'reason': reason,
            'end_t': t
        })
        active_replay_run = None


def _recording_start(t: float, reason: str):
    with data_lock:
        parsed_data['predefined'].clear()
    socketio.emit('recording_started', {'t': t, 'reason': reason})
    _mode_set('recording', reason, t)


def _process_mode_line(tag: str, parts: list, payload: str, t: float) -> bool:
    event = parts[0] if parts else ''

    if tag == 'pathinfo':
        if event == 'record_prepare':
            _mode_set('record_prepare', event, t)
        elif event == 'record_start':
            _recording_start(t, event)
        elif event in ('record_stop', 'record_saved'):
            _mode_set('recorded', event, t)
        elif event in ('replay_load_begin', 'replay_flash_load_begin', 'replay_load'):
            _mode_set('replay_prepare', event, t)
        elif event in ('replay_start', 'replay_drive_start'):
            if runtime_mode['mode'] != 'replaying':
                _replay_run_start(t, event)
        elif event == 'replay_finished':
            _replay_run_finish('completed', t, event)
            _mode_set('finished', event, t)
        elif event == 'replay_mode_lost':
            _replay_run_finish('error', t, event)
            _mode_set('error', event, t)
        return True

    if tag == 'pathcmd':
        if event == 'record_prepare' and len(parts) >= 2 and parts[1] == '1':
            _mode_set('record_prepare', event, t)
        elif event == 'record_stop':
            _mode_set('recorded', event, t)
        elif event == 'replay_stop' or event == 'stop':
            _replay_run_finish('stopped', t, event)
            _mode_set('stopped', event, t)
        return True

    if tag in ('vstop', 'safety'):
        status = 'error' if tag == 'safety' else 'stopped'
        _replay_run_finish(status, t, tag)
        _mode_set(status, payload.strip() or tag, t)
        return True

    return False


def _float_at(parts, index: int):
    value = float(parts[index])
    if not math.isfinite(value):
        raise ValueError(f'non-finite value at field {index}')
    return value


def _int_at(parts, index: int):
    return int(_float_at(parts, index))


def _next_replay_point_id():
    global replay_point_seq, latest_replay_point_id
    replay_point_seq += 1
    latest_replay_point_id = replay_point_seq
    return latest_replay_point_id


def _emit_path_point(x: float, y: float, t: float, source: str, **extra):
    point = {'x': x, 'y': y, 't': t, 'source': source}
    point.update(extra)
    with data_lock:
        _append_limited(parsed_data['predefined'], point, MAX_LAYER_POINTS)
    _emit_browser_event('serial_data', {
        'type': 'P',
        'point': point,
        't': t,
        'source': source
    })


def _emit_replay_point(car_x: float, car_y: float, lookahead_x: float, lookahead_y: float,
                       t: float, source: str, point_id=None, **extra):
    global pending_replay_meta, replay_sample_open
    car = {'x': car_x, 'y': car_y, 't': t, 'source': source, 'role': 'pose'}
    lookahead = {'x': lookahead_x, 'y': lookahead_y, 't': t, 'source': source, 'role': 'target'}
    if point_id is not None:
        car['id'] = point_id
        lookahead['id'] = point_id
    point_meta = {**pending_replay_meta, **extra}
    pending_replay_meta = {}
    replay_sample_open = True
    target_meta = {
        key: point_meta[key]
        for key in ('track_mode', 'target_index', 'lookahead_distance_mm')
        if key in point_meta
    }
    actual_target_distance_mm = math.hypot(lookahead_x - car_x, lookahead_y - car_y)
    point_meta['actual_target_distance_mm'] = actual_target_distance_mm
    target_meta['actual_target_distance_mm'] = actual_target_distance_mm
    if active_replay_run is not None:
        elapsed_s = max(0.0, t - active_replay_run['start_t'])
        car['run_id'] = active_replay_run['id']
        lookahead['run_id'] = active_replay_run['id']
        car['elapsed_s'] = elapsed_s
        lookahead['elapsed_s'] = elapsed_s
    car.update(point_meta)
    lookahead.update(target_meta)
    with data_lock:
        _append_limited(parsed_data['realtime'], car, MAX_LAYER_POINTS)
        _append_limited(parsed_data['lookahead'], lookahead, MAX_LAYER_POINTS)
        if active_replay_run is not None:
            _append_limited(active_replay_run['points'], car, MAX_LAYER_POINTS)
            _append_limited(active_replay_run['lookahead'], lookahead, MAX_LAYER_POINTS)
    _emit_browser_event('serial_data', {
        'type': 'R',
        'car': car,
        'lookahead': lookahead,
        't': t,
        'source': source
    })


def _emit_lookahead_point(x: float, y: float, t: float, source: str, **extra):
    point = {'x': x, 'y': y, 't': t, 'source': source, 'role': 'marker'}
    point.update(extra)
    with data_lock:
        _append_limited(parsed_data['lookahead'], point, MAX_LAYER_POINTS)
    _emit_browser_event('serial_data', {
        'type': 'L',
        'lookahead': point,
        't': t,
        'source': source
    })


def _clear_series(dtype: str):
    with data_lock:
        count = len(parsed_data[dtype])
        parsed_data[dtype].clear()
    socketio.emit('clear_type_result', {'success': True, 'type': dtype, 'count': count})


def _update_latest_replay_meta(meta: dict) -> bool:
    if latest_replay_point_id is None or replay_sample_open is False:
        return False

    updated = False
    with data_lock:
        for point in reversed(parsed_data['realtime']):
            if point.get('id') == latest_replay_point_id:
                point.update(meta)
                updated = True
                break

    if updated:
        _emit_browser_event('serial_point_update', {
            'id': latest_replay_point_id,
            'meta': meta
        })
    return updated


def _set_pending_replay_meta(meta: dict):
    """Hold vrpl0 metadata until the following vrpl1 creates its sample."""
    global pending_replay_meta, replay_sample_open
    pending_replay_meta = meta
    replay_sample_open = False


def _process_braced_line(tag: str, payload: str, t: float) -> bool:
    """Parse current Mad_Circuits brace-prefixed telemetry."""
    global pending_replay_meta, last_slip_total_correction_mm
    tag = tag.strip()
    parts = _csv_parts(payload)

    try:
        if _process_mode_line(tag, parts, payload, t):
            return True

        if tag == 'pathdump':
            if len(parts) >= 1 and parts[0] == 'begin':
                _clear_series('predefined')
                return True
            # {pathdump}index,x_mm,y_mm,theta_rad,speed_mm_s
            if len(parts) >= 5:
                _emit_path_point(
                    _float_at(parts, 1),
                    _float_at(parts, 2),
                    t,
                    tag,
                    index=_int_at(parts, 0),
                    theta_rad=_float_at(parts, 3),
                    speed_mm_s=_float_at(parts, 4)
                )
                return True

        if tag == 'vrpl0':
            # vrpl0 is printed immediately before vrpl1 for the same control sample.
            if len(parts) >= 12:
                _set_pending_replay_meta({
                    'sample_dt_ms': _float_at(parts, 0) / 1000.0,
                    'replay_cursor_index': _int_at(parts, 1),
                    'target_index': _int_at(parts, 2),
                    'lookahead_distance_mm': _float_at(parts, 5),
                    'curve_section_active': _int_at(parts, 6),
                    'path_point_count': _int_at(parts, 7)
                })
                return True
            # Compact firmware format: dt,cursor,target,lookahead,curve,total_points[,track_mode].
            if len(parts) >= 6:
                meta = {
                    'sample_dt_ms': _float_at(parts, 0) / 1000.0,
                    'replay_cursor_index': _int_at(parts, 1),
                    'target_index': _int_at(parts, 2),
                    'lookahead_distance_mm': _float_at(parts, 3),
                    'curve_section_active': _int_at(parts, 4),
                    'path_point_count': _int_at(parts, 5)
                }
                if len(parts) >= 7:
                    meta['track_mode'] = _int_at(parts, 6)
                _set_pending_replay_meta(meta)
                return True

        if tag == 'path':
            # {path}x_mm,y_mm,theta_deg,index,record_dist_mm,...
            if len(parts) >= 5:
                _emit_path_point(
                    _float_at(parts, 0),
                    _float_at(parts, 1),
                    t,
                    tag,
                    theta_deg=_float_at(parts, 2),
                    index=_int_at(parts, 3),
                    distance_mm=_float_at(parts, 4)
                )
                return True

        if tag == 'vrpl1':
            # {vrpl1}pose_x,pose_y,pose_theta,base_x,base_y,base_theta,cte,along,target_x,target_y
            if len(parts) >= 10:
                point_id = _next_replay_point_id()
                pose_x_mm = _float_at(parts, 0)
                pose_y_mm = _float_at(parts, 1)
                target_x_mm = _float_at(parts, 8)
                target_y_mm = _float_at(parts, 9)
                _emit_replay_point(
                    pose_x_mm,
                    pose_y_mm,
                    target_x_mm,
                    target_y_mm,
                    t,
                    tag,
                    point_id=point_id,
                    pose_x_mm=pose_x_mm,
                    pose_y_mm=pose_y_mm,
                    theta_deg=_float_at(parts, 2),
                    base_x_mm=_float_at(parts, 3),
                    base_y_mm=_float_at(parts, 4),
                    base_theta_deg=_float_at(parts, 5),
                    cross_track_error_mm=_float_at(parts, 6),
                    along_track_error_mm=_float_at(parts, 7),
                    target_x_mm=target_x_mm,
                    target_y_mm=target_y_mm
                )
                return True

        if tag == 'vrpl2':
            # {vrpl2}target_theta,feedforward_theta,angle_error,target_speed,
            #        target_left_speed,target_right_speed,actual_left_speed,actual_right_speed,...
            # Older logs end at effective_yaw_correction (13 fields). New firmware
            # appends raw_plan_speed and speed_preview_limit without reordering.
            if len(parts) >= 13:
                meta = {
                    'target_theta_deg': _float_at(parts, 0),
                    'feedforward_theta_deg': _float_at(parts, 1),
                    'angle_error_deg': _float_at(parts, 2),
                    'target_speed_mm_s': _float_at(parts, 3),
                    'target_left_speed_mm_s': _float_at(parts, 4),
                    'target_right_speed_mm_s': _float_at(parts, 5),
                    'actual_left_speed_mm_s': _float_at(parts, 6),
                    'actual_right_speed_mm_s': _float_at(parts, 7),
                    'yaw_speed_correction_mm_s': _float_at(parts, 8),
                    'yaw_rate_deg_s': _float_at(parts, 9),
                    'target_yaw_rate_deg_s': _float_at(parts, 10),
                    'yaw_ff_mm_s': _float_at(parts, 11),
                    'effective_yaw_correction_mm_s': _float_at(parts, 12)
                }
                if len(parts) >= 14:
                    meta['raw_plan_speed_mm_s'] = _float_at(parts, 13)
                if len(parts) >= 15:
                    meta['speed_preview_limit_mm_s'] = _float_at(parts, 14)
                _update_latest_replay_meta(meta)
                return True

        if tag == 'vrpl3':
            # {vrpl3}left_dist,right_dist,wheel_delta,enc_theta,enc_dist,
            #        target_left_pwm,target_right_pwm,actual_left_pwm,actual_right_pwm,...
            if len(parts) >= 9:
                _update_latest_replay_meta({
                    'target_left_pwm': _int_at(parts, 5),
                    'target_right_pwm': _int_at(parts, 6),
                    'actual_left_pwm': _int_at(parts, 7),
                    'actual_right_pwm': _int_at(parts, 8),
                    'left_esc_state': _int_at(parts, 9) if len(parts) >= 13 else None,
                    'left_esc_fault': _int_at(parts, 10) if len(parts) >= 13 else None,
                    'right_esc_state': _int_at(parts, 11) if len(parts) >= 13 else None,
                    'right_esc_fault': _int_at(parts, 12) if len(parts) >= 13 else None
                })
                return True
            # Compact firmware format: target PWM, actual PWM, then ESC state/fault.
            if len(parts) >= 8:
                _update_latest_replay_meta({
                    'target_left_pwm': _int_at(parts, 0),
                    'target_right_pwm': _int_at(parts, 1),
                    'actual_left_pwm': _int_at(parts, 2),
                    'actual_right_pwm': _int_at(parts, 3),
                    'left_esc_state': _int_at(parts, 4),
                    'left_esc_fault': _int_at(parts, 5),
                    'right_esc_state': _int_at(parts, 6),
                    'right_esc_fault': _int_at(parts, 7)
                })
                return True

        if tag == 'vrpl4':
            # {vrpl4}base_index,target_index,tangent_index,base_x,base_y,target_x,target_y,
            #        tangent_x,tangent_y,lookahead_distance,tangent_distance
            if len(parts) >= 11:
                _update_latest_replay_meta({
                    'base_index': _float_at(parts, 0),
                    'target_index': _float_at(parts, 1),
                    'tangent_index': _float_at(parts, 2),
                    'base_x_mm': _float_at(parts, 3),
                    'base_y_mm': _float_at(parts, 4),
                    'target_x_mm': _float_at(parts, 5),
                    'target_y_mm': _float_at(parts, 6),
                    'tangent_x_mm': _float_at(parts, 7),
                    'tangent_y_mm': _float_at(parts, 8),
                    'lookahead_distance_mm': _float_at(parts, 9),
                    'tangent_distance_mm': _float_at(parts, 10)
                })
                return True

        if tag == 'vslip':
            # {vslip}active,confirmed,straight_ready,encoder_deg,imu_deg,ratio,
            #         pending_mm,total_mm,correction_x_mm,correction_y_mm,
            #         window_ready,bucket_count,valid_count,reject_reason,window_ms,turn_sign
            if len(parts) >= 10:
                total_correction_mm = _float_at(parts, 7)
                correction_delta_mm = max(
                    0.0,
                    total_correction_mm - last_slip_total_correction_mm
                )
                last_slip_total_correction_mm = total_correction_mm
                _update_latest_replay_meta({
                    'slip_active': _int_at(parts, 0),
                    'slip_confirmed': _int_at(parts, 1),
                    'slip_straight_ready': _int_at(parts, 2),
                    'slip_encoder_turn_deg': _float_at(parts, 3),
                    'slip_imu_turn_deg': _float_at(parts, 4),
                    'slip_yaw_realization_ratio': _float_at(parts, 5),
                    'slip_pending_correction_mm': _float_at(parts, 6),
                    'slip_total_correction_mm': total_correction_mm,
                    'slip_correction_delta_mm': correction_delta_mm,
                    'slip_correction_x_mm': _float_at(parts, 8),
                    'slip_correction_y_mm': _float_at(parts, 9),
                    'slip_window_ready': _int_at(parts, 10) if len(parts) > 10 else 0,
                    'slip_window_bucket_count': _int_at(parts, 11) if len(parts) > 11 else 0,
                    'slip_window_valid_count': _int_at(parts, 12) if len(parts) > 12 else 0,
                    'slip_reject_reason': _int_at(parts, 13) if len(parts) > 13 else 0,
                    'slip_window_ms': _float_at(parts, 14) if len(parts) > 14 else 0.0,
                    'slip_turn_sign': _int_at(parts, 15) if len(parts) > 15 else 0
                })
                return True

        if tag == 'vslipevt' and len(parts) >= 9:
            _update_latest_replay_meta({
                'turn_slip_event': 1,
                'turn_slip_event_id': _int_at(parts, 0),
                'turn_slip_path_index': _int_at(parts, 1),
                'turn_slip_x_mm': _float_at(parts, 2),
                'turn_slip_y_mm': _float_at(parts, 3),
                'turn_slip_ratio': _float_at(parts, 4),
                'turn_slip_encoder_turn_deg': _float_at(parts, 5),
                'turn_slip_imu_turn_deg': _float_at(parts, 6),
                'turn_slip_pending_mm': _float_at(parts, 7),
                'turn_slip_total_mm': _float_at(parts, 8)
            })
            return True

        if tag == 'vslipacc' and len(parts) >= 8 and parts[1] in ('1', '2'):
            _update_latest_replay_meta({
                'accel_slip_event_state': _int_at(parts, 1),
                'accel_slip_event_id': _int_at(parts, 0),
                'accel_slip_path_index': _int_at(parts, 2),
                'accel_slip_x_mm': _float_at(parts, 3),
                'accel_slip_y_mm': _float_at(parts, 4),
                'accel_slip_encoder_accel_mm_s2': _float_at(parts, 5),
                'accel_slip_imu_accel_mm_s2': _float_at(parts, 6),
                'accel_slip_ratio': _float_at(parts, 7)
            })
            return True

        if tag == 'ptcorrevt' and len(parts) >= 6:
            _update_latest_replay_meta({
                'phototube_correction_event': 1,
                'phototube_correction_applied': 1,
                'phototube_event_path_index': _float_at(parts, 0),
                'phototube_event_x_mm': _float_at(parts, 1),
                'phototube_event_y_mm': _float_at(parts, 2),
                'phototube_confidence': _float_at(parts, 3),
                'phototube_correction_x_mm': _float_at(parts, 4),
                'phototube_correction_y_mm': _float_at(parts, 5)
            })
            return True

        if tag == 'ptcorr':
            # {ptcorr}enabled,applied,line_valid,projection_valid,line_body_y,confidence,
            # line_sum,line_world_x,line_world_y,projection_index,projection_x,projection_y,
            # projection_distance,error_x,error_y,correction_x,correction_y,gate_enabled,
            # gate_pass,gate_stable_count,target_yaw_rate,heading_delta,active_count,
            # active_width,shape_pass,line_body_x,pose_x,pose_y[,monitor,powered,allowed,straight]
            if len(parts) >= 28 and parts[0] in ('0', '1'):
                meta = {
                    'phototube_correction_enabled': _int_at(parts, 0),
                    'phototube_correction_applied': _int_at(parts, 1),
                    'phototube_line_valid': _int_at(parts, 2),
                    'phototube_projection_valid': _int_at(parts, 3),
                    'phototube_line_body_y_mm': _float_at(parts, 4),
                    'phototube_confidence': _float_at(parts, 5),
                    'phototube_line_sum': _int_at(parts, 6),
                    'phototube_line_world_x_mm': _float_at(parts, 7),
                    'phototube_line_world_y_mm': _float_at(parts, 8),
                    'phototube_projection_index': _float_at(parts, 9),
                    'phototube_projection_x_mm': _float_at(parts, 10),
                    'phototube_projection_y_mm': _float_at(parts, 11),
                    'phototube_projection_distance_mm': _float_at(parts, 12),
                    'phototube_error_x_mm': _float_at(parts, 13),
                    'phototube_error_y_mm': _float_at(parts, 14),
                    'phototube_correction_x_mm': _float_at(parts, 15),
                    'phototube_correction_y_mm': _float_at(parts, 16),
                    'phototube_gate_enabled': _int_at(parts, 17),
                    'phototube_gate_pass': _int_at(parts, 18),
                    'phototube_gate_stable_count': _int_at(parts, 19),
                    'phototube_target_yaw_rate_deg_s': _float_at(parts, 20),
                    'phototube_heading_delta_deg': _float_at(parts, 21),
                    'phototube_active_count': _int_at(parts, 22),
                    'phototube_active_width_mm': _float_at(parts, 23),
                    'phototube_shape_pass': _int_at(parts, 24),
                    'phototube_line_body_x_mm': _float_at(parts, 25),
                    'phototube_pose_x_mm': _float_at(parts, 26),
                    'phototube_pose_y_mm': _float_at(parts, 27),
                }
                if len(parts) >= 32:
                    meta.update({
                        'phototube_monitor_enabled': _int_at(parts, 28),
                        'phototube_sensor_powered': _int_at(parts, 29),
                        'phototube_runtime_allowed': _int_at(parts, 30),
                        'phototube_path_straight': _int_at(parts, 31),
                    })
                _update_latest_replay_meta(meta)
                return True

        if tag == 'vrplg':
            # Launch state is compact and directly explains skipped or prolonged startup phases.
            if len(parts) >= 7:
                _update_latest_replay_meta({
                    'launch_phase': _int_at(parts, 0),
                    'launch_gate_ms': _float_at(parts, 1),
                    'launch_stable_ms': _float_at(parts, 2),
                    'launch_wheel_ready': _int_at(parts, 5)
                })
                return True

        if tag == 'vmark':
            # Marker points are shown on the green layer so they can be toggled separately.
            # {vmark}add,kind,index,x_mm,y_mm,theta_deg
            # {vmark}row,kind,index,x_mm,y_mm,theta_deg
            if len(parts) >= 6 and parts[0] not in ('enable', 'clear'):
                _emit_lookahead_point(
                    _float_at(parts, 3),
                    _float_at(parts, 4),
                    t,
                    tag,
                    marker_kind=parts[1],
                    index=_int_at(parts, 2),
                    theta_deg=_float_at(parts, 5)
                )
                return True

        if tag in IGNORED_BRACED_TAGS:
            return True

    except (ValueError, IndexError) as exc:
        if tag == 'vrpl1':
            pending_replay_meta = {}
        logger.debug(f"Malformed {tag} line: {payload!r} ({exc})")
        return False

    if tag == 'vrpl1':
        pending_replay_meta = {}
    return False


def process_line(line: str):
    """Parse a single line of serial data and emit to clients."""
    line = line.strip()
    if not line:
        return False

    _capture_raw_rx_line(line)

    path_upload_manager.handle_line(line)

    t = time.time()

    # Only complete, recognized telemetry reaches the browser. All unrelated serial
    # chatter and partial packets are intentionally discarded here.
    m = RE_BRACED.match(line)
    if m:
        tag = m.group(1).strip()
        payload = m.group(2)
        is_upload_point_ack = tag == 'pathupload' and payload.lstrip().startswith('point,')
        if tag not in HIGH_RATE_TAGS or (tag == 'pathupload' and not is_upload_point_ack):
            _emit_console('rx', line, t)
        return _process_braced_line(tag, payload, t)

    # Try legacy R pattern: car position + lookahead point.
    m = RE_R.match(line)
    if m:
        _emit_replay_point(
            float(m.group(1)),
            float(m.group(2)),
            float(m.group(3)),
            float(m.group(4)),
            t,
            'legacy'
        )
        return True

    # Try legacy P pattern: predefined path point.
    m = RE_P.match(line)
    if m:
        _emit_path_point(float(m.group(1)), float(m.group(2)), t, 'legacy')
        return True

    logger.debug(f"Unknown format: {line}")
    if not m:
        _emit_console('rx', line, t)
    return False


def serial_reader_thread():
    """Background thread that reads from serial port and processes lines."""
    global serial_buffer, serial_running, serial_conn, browser_batching

    logger.info("Serial reader thread started")
    with browser_batch_lock:
        browser_batching = True
    last_browser_flush = time.monotonic()
    while serial_running:
        try:
            with serial_lock:
                if serial_conn is None or not serial_conn.is_open:
                    break
                waiting = serial_conn.in_waiting

            if waiting > 0:
                with serial_lock:
                    if serial_conn is None or not serial_conn.is_open:
                        break
                    raw = serial_conn.read(waiting)
                process_serial_bytes(raw)
                now = time.monotonic()
                if now - last_browser_flush >= SERIAL_BROWSER_BATCH_INTERVAL_S:
                    _flush_browser_events()
                    last_browser_flush = now
            else:
                time.sleep(0.001 if path_upload_manager.active else 0.005)
            now = time.monotonic()
            if now - last_browser_flush >= SERIAL_BROWSER_BATCH_INTERVAL_S:
                _flush_browser_events()
                last_browser_flush = now
        except serial.SerialException as e:
            logger.error(f"Serial exception: {e}")
            socketio.emit('serial_error', {'message': f'Serial error: {e}'})
            break
        except Exception as e:
            logger.error(f"Reader thread error: {e}")
            time.sleep(0.1)

    with browser_batch_lock:
        browser_batching = False
    _flush_browser_events()

    # Cleanup if thread exits unexpectedly
    with serial_lock:
        if serial_conn and serial_conn.is_open:
            try:
                serial_conn.close()
            except Exception:
                pass
            serial_conn = None
        serial_running = False

    socketio.emit('serial_status', {'connected': False, 'port': '', 'baudrate': 0})
    logger.info("Serial reader thread stopped")


# ── HTTP Routes ────────────────────────────────────────────────

@app.route('/')
def index():
    """Serve the main page."""
    return send_from_directory('static', 'index.html')


@app.after_request
def disable_local_cache(response):
    """Always expose the latest local UI while PathCapture is being tuned."""
    response.headers['Cache-Control'] = 'no-store, max-age=0'
    response.headers['Pragma'] = 'no-cache'
    response.headers['Expires'] = '0'
    return response


@app.route('/api/ports')
def api_ports():
    """List available serial ports."""
    ports = []
    for p in serial.tools.list_ports.comports():
        ports.append({
            'device': p.device,
            'name': p.name,
            'description': p.description,
            'hwid': p.hwid
        })
    return jsonify({'ports': ports})


@app.route('/api/export-raw-log')
def api_export_raw_log():
    """Download the complete line-oriented RX session in MCU-compatible .dat form."""
    with raw_log_lock:
        content = '\r\n'.join(raw_log_lines)
    if content:
        content += '\r\n'
    filename = f"main-{datetime.now().strftime('%Y%m%d-%H%M%S')}.dat"
    return Response(
        content,
        content_type='text/plain; charset=utf-8',
        headers={'Content-Disposition': f'attachment; filename="{filename}"'}
    )


@app.route('/api/reset-data', methods=['POST'])
def api_reset_data():
    """Emergency reset independent of the browser's Socket.IO session."""
    result = _reset_runtime_data('http_reset')
    logger.info("Runtime data reset through HTTP")
    socketio.emit('data_cleared', result)
    return jsonify(result)


@app.route('/api/import-log', methods=['POST'])
def api_import_log():
    """Convert a noisy line-oriented vehicle log without touching live parser state."""
    upload = request.files.get('file')
    if upload is None or not upload.filename:
        return jsonify({'error': '没有选择日志文件'}), 400

    extension = os.path.splitext(upload.filename)[1].lower()
    if extension not in RAW_LOG_EXTENSIONS:
        return jsonify({'error': '仅支持 .dat、.log 和 .txt 原始日志'}), 400
    if request.content_length and request.content_length > MAX_IMPORT_BYTES:
        return jsonify({'error': '日志文件超过 256 MB 限制'}), 413

    converter = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                             'tools', 'log_converter.py')
    input_path = None
    output_path = None
    try:
        with tempfile.NamedTemporaryFile(delete=False, suffix=extension) as input_file:
            input_path = input_file.name
            upload.save(input_file)
        if os.path.getsize(input_path) > MAX_IMPORT_BYTES:
            return jsonify({'error': '日志文件超过 256 MB 限制'}), 413

        with tempfile.NamedTemporaryFile(delete=False, suffix='.json') as output_file:
            output_path = output_file.name

        python_executable = sys.executable
        if os.path.basename(python_executable).lower() == 'pythonw.exe':
            console_python = os.path.join(os.path.dirname(python_executable), 'python.exe')
            if os.path.exists(console_python):
                python_executable = console_python
        result = subprocess.run(
            [python_executable, converter, input_path, output_path],
            capture_output=True,
            text=True,
            timeout=120,
            check=False
        )
        if result.returncode != 0:
            detail = (result.stderr or result.stdout or '未知解析错误').strip()
            logger.error('Raw log import failed: %s', detail)
            return jsonify({'error': f'日志解析失败：{detail[-500:]}'}), 422
        with open(output_path, 'r', encoding='utf-8') as converted_file:
            return jsonify(json.load(converted_file))
    except subprocess.TimeoutExpired:
        return jsonify({'error': '日志解析超过 120 秒，已终止'}), 408
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        logger.exception('Raw log import failed')
        return jsonify({'error': f'日志解析失败：{exc}'}), 422
    finally:
        for temporary_path in (input_path, output_path):
            if temporary_path and os.path.exists(temporary_path):
                try:
                    os.unlink(temporary_path)
                except OSError:
                    logger.warning('Could not remove temporary import file: %s', temporary_path)


# ── SocketIO Events ────────────────────────────────────────────

@socketio.on('connect')
def handle_connect():
    """Client connected."""
    logger.info(f"Client connected: {request.sid}")
    # Send current connection status
    with serial_lock:
        connected = serial_running and serial_conn is not None and serial_conn.is_open
        port = serial_conn.port if serial_conn else ''
        baudrate = serial_conn.baudrate if serial_conn else 0
    emit('serial_status', {
        'connected': connected,
        'port': port,
        'baudrate': baudrate
    })
    # Send all accumulated data for session sync
    with data_lock:
        emit('sync_data', {
            'predefined': parsed_data['predefined'],
            'realtime': parsed_data['realtime'],
            'lookahead': parsed_data['lookahead'],
            'replay_runs': parsed_data['replay_runs'],
            'mode': runtime_mode,
            'console': console_lines
        })


@socketio.on('disconnect')
def handle_disconnect():
    """Client disconnected."""
    logger.info(f"Client disconnected: {request.sid}")


@socketio.on('connect_serial')
def handle_connect_serial(data: dict):
    """Connect to a serial port."""
    global serial_conn, serial_running, serial_buffer, serial_byte_buffer

    port = data.get('port', '')
    baudrate = int(data.get('baudrate', 115200))
    serial_buffer = ''
    serial_byte_buffer.clear()

    if not port:
        emit('serial_error', {'message': 'No port selected'})
        return

    with serial_lock:
        # Close existing connection if any
        if serial_conn and serial_conn.is_open:
            serial_running = False
            try:
                serial_conn.close()
            except Exception:
                pass
            serial_conn = None

        try:
            serial_conn = serial.Serial(
                port=port,
                baudrate=baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1
            )
            serial_buffer = ''
            serial_running = True
            logger.info(f"Serial connected: {port} @ {baudrate}")

            # Start reader thread
            t = threading.Thread(target=serial_reader_thread, daemon=True)
            t.start()

            emit('serial_status', {
                'connected': True,
                'port': port,
                'baudrate': baudrate
            })
        except serial.SerialException as e:
            logger.error(f"Failed to connect serial: {e}")
            serial_conn = None
            serial_running = False
            emit('serial_error', {'message': f'Failed to connect: {e}'})
        except Exception as e:
            logger.error(f"Unexpected error: {e}")
            serial_conn = None
            serial_running = False
            emit('serial_error', {'message': f'Error: {e}'})


@socketio.on('disconnect_serial')
def handle_disconnect_serial():
    """Disconnect from serial port."""
    global serial_conn, serial_running, serial_buffer, serial_byte_buffer

    serial_running = False
    serial_buffer = ''
    serial_byte_buffer.clear()
    with serial_lock:
        if serial_conn and serial_conn.is_open:
            try:
                serial_conn.close()
            except Exception as e:
                logger.error(f"Error closing serial: {e}")
            serial_conn = None

    logger.info("Serial disconnected")
    emit('serial_status', {'connected': False, 'port': '', 'baudrate': 0})


@socketio.on('list_ports')
def handle_list_ports():
    """Return list of available serial ports."""
    ports = []
    for p in serial.tools.list_ports.comports():
        ports.append({
            'device': p.device,
            'name': p.name,
            'description': p.description,
            'hwid': p.hwid
        })
    emit('ports_list', {'ports': ports})


@socketio.on('send_serial_command')
def handle_send_serial_command(data: dict):
    """Send one line-oriented command without allowing embedded line breaks."""
    command = str((data or {}).get('command', '')).strip()
    if not command:
        emit('command_result', {'success': False, 'error': '命令不能为空'})
        return
    if '\r' in command or '\n' in command:
        emit('command_result', {'success': False, 'error': '命令中不能包含换行符'})
        return
    if len(command) > MAX_COMMAND_CHARS:
        emit('command_result', {
            'success': False,
            'error': f'命令不能超过 {MAX_COMMAND_CHARS} 个字符'
        })
        return

    command_lower = command.lower()
    upload_cancel_command = command_lower in ('vpath upload abort', 'vpath upload cancel')
    if path_upload_manager.active and command_lower != 'vstop' and not upload_cancel_command:
        emit('command_result', {'success': False, 'error': '路径正在写入 Flash，请等待完成或发送 VSTOP'})
        return
    if path_upload_manager.active and upload_cancel_command:
        path_upload_manager.cancel()
        emit('command_result', {'success': True, 'command': command, 'cancel_requested': True})
        return
    if path_upload_manager.active and command_lower == 'vstop':
        path_upload_manager.cancel()

    try:
        with serial_lock:
            if not serial_running or serial_conn is None or not serial_conn.is_open:
                emit('command_result', {'success': False, 'error': '串口未连接'})
                return
            serial_conn.write((command + '\r\n').encode('utf-8'))
            serial_conn.flush()
        now = time.time()
        _emit_console('tx', command, now)
        emit('command_result', {'success': True, 'command': command, 't': now})
    except (serial.SerialException, OSError) as exc:
        logger.error(f"Serial command failed: {exc}")
        emit('command_result', {'success': False, 'error': f'串口发送失败：{exc}'})


@socketio.on('upload_path_to_flash')
def handle_upload_path_to_flash(data: dict):
    points = (data or {}).get('points')
    markers = (data or {}).get('markers', [])
    phototube_zones = (data or {}).get('phototube_zones', [])
    source = str((data or {}).get('source', 'unknown'))[:40]
    try:
        count = path_upload_manager.start(points, source, markers, phototube_zones)
        emit('path_upload_accepted', {
            'success': True, 'source': source, 'count': count,
            'marker_count': len(markers), 'phototube_zone_count': len(phototube_zones)
        })
    except (PathUploadError, TypeError, ValueError) as exc:
        emit('path_upload_accepted', {'success': False, 'source': source, 'error': str(exc)})


@socketio.on('cancel_path_upload')
def handle_cancel_path_upload():
    emit('path_upload_cancelled', {'accepted': path_upload_manager.cancel()})


@socketio.on('save_data')
def handle_save_data(data: dict):
    """Save all accumulated data to a JSON file."""
    ensure_data_dir()
    filename = data.get('filename', f'pathcapture_{datetime.now().strftime("%Y%m%d_%H%M%S")}.json')
    filepath = os.path.join(DATA_DIR, filename)

    try:
        with data_lock:
            save_obj = {
                'version': '3.0',
                'saved_at': datetime.now().isoformat(),
                'predefined': parsed_data['predefined'],
                'realtime': parsed_data['realtime'],
                'lookahead': parsed_data['lookahead'],
                'replay_runs': parsed_data['replay_runs']
            }
        with open(filepath, 'w', encoding='utf-8') as f:
            json.dump(save_obj, f, indent=2, ensure_ascii=False)
        logger.info(f"Data saved to {filepath} (R={len(parsed_data['realtime'])})")
        emit('save_result', {'success': True, 'filepath': filepath, 'filename': filename})
    except Exception as e:
        logger.error(f"Save failed: {e}")
        emit('save_result', {'success': False, 'error': str(e)})


@socketio.on('load_data')
def handle_load_data(data: dict):
    """Load data from a JSON file."""
    filename = data.get('filename', '')
    filepath = os.path.join(DATA_DIR, filename) if filename else ''

    if not filename or not os.path.exists(filepath):
        emit('load_result', {'success': False, 'error': f'File not found: {filename}'})
        return

    try:
        with open(filepath, 'r', encoding='utf-8') as f:
            loaded = json.load(f)

        # Validate structure
        predefined = loaded.get('predefined', [])
        realtime = loaded.get('realtime', [])
        lookahead = loaded.get('lookahead', [])
        replay_runs = _normalize_loaded_runs(
            loaded.get('replay_runs', []), realtime, lookahead)
        # Replace server-side data
        with data_lock:
            parsed_data['predefined'] = predefined.copy()
            parsed_data['realtime'] = realtime.copy()
            parsed_data['lookahead'] = lookahead.copy()
            parsed_data['replay_runs'] = replay_runs.copy()

        logger.info(f"Data loaded from {filepath} "
                    f"(P={len(predefined)}, R={len(realtime)}, L={len(lookahead)})")

        emit('load_result', {
            'success': True,
            'filename': filename,
            'data': {
                'predefined': predefined,
                'realtime': realtime,
                'lookahead': lookahead,
                'replay_runs': replay_runs
            }
        })
    except json.JSONDecodeError as e:
        logger.error(f"Invalid JSON file: {e}")
        emit('load_result', {'success': False, 'error': f'Invalid JSON: {e}'})
    except Exception as e:
        logger.error(f"Load failed: {e}")
        emit('load_result', {'success': False, 'error': str(e)})


@socketio.on('clear_data')
def handle_clear_data():
    """Clear all accumulated data."""
    result = _reset_runtime_data('data_cleared')
    logger.info("Data cleared")
    emit('data_cleared', result)


@socketio.on('clear_data_type')
def handle_clear_data_type(data: dict):
    """Clear a specific data type only."""
    dtype = data.get('type', '')
    if dtype not in ('predefined', 'realtime', 'lookahead'):
        emit('clear_type_result', {'success': False, 'error': f'Unknown type: {dtype}'})
        return

    global active_replay_run, latest_replay_point_id
    with data_lock:
        count = len(parsed_data[dtype])
        parsed_data[dtype].clear()
        if dtype == 'realtime':
            parsed_data['replay_runs'].clear()
            parsed_data['lookahead'].clear()
            active_replay_run = None
            latest_replay_point_id = None

    logger.info(f"Cleared {dtype} data ({count} points)")
    emit('clear_type_result', {'success': True, 'type': dtype, 'count': count})


# ── Main ───────────────────────────────────────────────────────

if __name__ == '__main__':
    ensure_data_dir()
    logger.info(f"PathCapture starting on http://{HOST}:{PORT}")
    try:
        socketio.run(app, host=HOST, port=PORT, debug=False, allow_unsafe_werkzeug=True)
    except KeyboardInterrupt:
        logger.info("Shutting down...")
    finally:
        serial_running = False
        if serial_conn and serial_conn.is_open:
            serial_conn.close()
