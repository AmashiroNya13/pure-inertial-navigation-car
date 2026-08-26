import math
import re
import secrets
import struct
import threading
import time
import zlib


ACK_PATTERN = re.compile(r'\{pathupload\}([a-z0-9_]+)(?:[:,]([^\r\n]*))?$')
BINARY_MAGIC = b'\x00\xA5\x5A\xC3'
BINARY_VERSION = 1
BINARY_TYPE_BLOCK = 1
BINARY_BLOCK_POINT_COUNT = 32
PATH_CHECKSUM_SEED = 0xFFFFFFFF


class PathUploadError(RuntimeError):
    pass


class PathUploadManager:
    def __init__(self, send_line, emit_event, send_bytes=None, ack_timeout=0.8,
                 retries=3, done_timeout=30.0, begin_timeout=30.0):
        self._send_line = send_line
        self._send_bytes = send_bytes
        self._emit_event = emit_event
        self._ack_timeout = ack_timeout
        self._retries = retries
        self._done_timeout = done_timeout
        self._begin_timeout = begin_timeout
        self._condition = threading.Condition()
        self._messages = []
        self._sequence = 0
        self._active = False
        self._cancelled = False
        self._thread = None

    @property
    def active(self):
        with self._condition:
            return self._active

    def handle_line(self, line):
        match = ACK_PATTERN.search(str(line).strip())
        if not match:
            return False
        fields = [] if match.group(2) is None else [
            item.strip() for item in match.group(2).split(',')
        ]
        with self._condition:
            self._sequence += 1
            self._messages.append((self._sequence, match.group(1), fields))
            if len(self._messages) > 128:
                del self._messages[:-128]
            self._condition.notify_all()
        return True

    def start(self, points, source='unknown', markers=None, phototube_zones=None):
        normalized = self._validate_points(points)
        normalized_markers = self._validate_markers(markers, len(normalized))
        normalized_zones = self._validate_phototube_zones(
            phototube_zones, len(normalized)
        )
        with self._condition:
            if self._active:
                raise PathUploadError('已有路径正在写入')
            self._active = True
            self._cancelled = False
        self._thread = threading.Thread(
            target=self._run,
            args=(normalized, normalized_markers, normalized_zones, str(source)),
            daemon=True,
            name='path-flash-upload'
        )
        self._thread.start()
        return len(normalized)

    def cancel(self):
        with self._condition:
            if not self._active:
                return False
            self._cancelled = True
            self._condition.notify_all()
            return True

    @staticmethod
    def _validate_points(points):
        if not isinstance(points, list) or len(points) < 2:
            raise PathUploadError('路径至少需要两个点')
        normalized = []
        for index, value in enumerate(points):
            if not isinstance(value, (list, tuple)) or len(value) != 4:
                raise PathUploadError(f'第 {index} 个路径点格式错误')
            point = tuple(float(item) for item in value)
            if not all(math.isfinite(item) for item in point):
                raise PathUploadError(f'第 {index} 个路径点包含无效数值')
            x_mm, y_mm, theta_rad, speed_mm_s = point
            if abs(x_mm) > 1000000 or abs(y_mm) > 1000000 or abs(theta_rad) > 1000:
                raise PathUploadError(f'第 {index} 个路径点超出主控范围')
            if speed_mm_s < 0 or speed_mm_s > 65535:
                raise PathUploadError(f'第 {index} 个路径点速度超出主控范围')
            normalized.append(point)
        return normalized

    @staticmethod
    def _validate_markers(markers, point_count):
        if markers is None:
            return []
        if not isinstance(markers, list):
            raise PathUploadError('弯道标记必须是列表')
        normalized = []
        seen = set()
        for position, value in enumerate(markers):
            if not isinstance(value, (list, tuple)) or len(value) != 2:
                raise PathUploadError(f'第 {position} 个弯道标记格式错误')
            try:
                index = int(value[0])
            except (TypeError, ValueError) as exc:
                raise PathUploadError(f'第 {position} 个弯道标记索引无效') from exc
            kind = str(value[1]).lower()
            if index < 0 or index >= point_count or kind not in ('in', 'out'):
                raise PathUploadError(f'第 {position} 个弯道标记超出路径范围')
            key = (index, kind)
            if key not in seen:
                normalized.append(key)
                seen.add(key)
        if len(normalized) > 64:
            raise PathUploadError('弯道标记超过主控上限64个')
        normalized.sort(key=lambda marker: (marker[0], 0 if marker[1] == 'in' else 1))
        return normalized

    @staticmethod
    def _validate_phototube_zones(zones, point_count):
        if zones is None:
            return []
        if not isinstance(zones, list):
            raise PathUploadError('Phototube exclusion zones must be a list')
        normalized = []
        for position, value in enumerate(zones):
            if not isinstance(value, (list, tuple)) or len(value) != 2:
                raise PathUploadError(
                    f'Phototube exclusion zone {position} has invalid format'
                )
            try:
                start_index = int(value[0])
                end_index = int(value[1])
            except (TypeError, ValueError) as exc:
                raise PathUploadError(
                    f'Phototube exclusion zone {position} has invalid indexes'
                ) from exc
            if start_index < 0 or end_index < start_index or end_index >= point_count:
                raise PathUploadError(
                    f'Phototube exclusion zone {position} is outside the path'
                )
            if normalized and start_index <= normalized[-1][1]:
                raise PathUploadError('Phototube exclusion zones must not overlap')
            normalized.append((start_index, end_index))
        if len(normalized) > 128:
            raise PathUploadError('Phototube exclusion zones exceed controller limit 128')
        return normalized

    @staticmethod
    def _integer(fields, index, fallback=-1):
        try:
            return int(fields[index])
        except (IndexError, TypeError, ValueError):
            return fallback

    @staticmethod
    def _path_checksum(data):
        checksum = PATH_CHECKSUM_SEED
        for value in data:
            checksum ^= value
            checksum = ((checksum << 5) | (checksum >> 27)) & 0xFFFFFFFF
        return checksum

    @staticmethod
    def _pack_points(points):
        return b''.join(struct.pack('<ffff', *point) for point in points)

    @staticmethod
    def _build_block_frame(session_id, start_index, packed_points):
        if len(packed_points) == 0 or len(packed_points) % 16:
            raise PathUploadError('二进制路径块长度无效')
        point_count = len(packed_points) // 16
        if point_count > BINARY_BLOCK_POINT_COUNT:
            raise PathUploadError('二进制路径块超过32点')
        payload = struct.pack('<IIB3x', session_id, start_index, point_count) + packed_points
        frame = BINARY_MAGIC + struct.pack('<BBH', BINARY_VERSION, BINARY_TYPE_BLOCK,
                                           len(payload)) + payload
        return frame + struct.pack('<I', zlib.crc32(frame) & 0xFFFFFFFF)

    def _check_cancelled(self):
        with self._condition:
            if self._cancelled:
                raise PathUploadError('用户取消写入')

    def _wait_after(self, sequence, predicate, timeout, allow_usage=False,
                    keepalive=None):
        deadline = time.monotonic() + timeout
        keepalive_sequence = sequence
        with self._condition:
            while True:
                if self._cancelled:
                    raise PathUploadError('用户取消写入')
                for message_sequence, tag, fields in self._messages:
                    if message_sequence <= sequence:
                        continue
                    if tag == 'failed':
                        reason = fields[0] if fields else 'unknown'
                        raise PathUploadError(f'主控写入失败：{reason}')
                    if tag == 'usage':
                        if allow_usage:
                            return message_sequence, tag, fields
                        raise PathUploadError('主控不支持当前上传命令或参数格式错误')
                    if predicate(tag, fields):
                        return message_sequence, tag, fields
                    if (keepalive is not None
                            and message_sequence > keepalive_sequence
                            and keepalive(tag, fields)):
                        keepalive_sequence = message_sequence
                        deadline = time.monotonic() + timeout
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError('等待主控确认超时')
                self._condition.wait(remaining)

    def _exchange(self, command, predicate, timeout=None, allow_usage=False,
                  keepalive=None):
        last_error = None
        for _ in range(self._retries):
            self._check_cancelled()
            with self._condition:
                sequence = self._sequence
            self._send_line(command)
            try:
                return self._wait_after(
                    sequence, predicate, timeout or self._ack_timeout, allow_usage,
                    keepalive
                )
            except TimeoutError as exc:
                last_error = exc
        command_parts = command.split()
        action = command_parts[2]
        if action == 'point' and len(command_parts) > 3:
            action = f'{action} {command_parts[3]}'
        raise PathUploadError(f'{last_error}：{action}')

    def _send_block(self, frame, session_id, start_index, point_count):
        expected_received = start_index + point_count
        expected_crc = struct.unpack_from('<I', frame, len(frame) - 4)[0]
        last_error = None

        for _ in range(self._retries):
            self._check_cancelled()
            with self._condition:
                sequence = self._sequence
            self._send_bytes(frame)
            try:
                _, _, fields = self._wait_after(
                    sequence,
                    lambda tag, values: (
                        tag == 'block'
                        and self._integer(values, 1) == session_id
                        and self._integer(values, 2) == start_index
                    ),
                    self._ack_timeout
                )
            except TimeoutError as exc:
                last_error = exc
                continue

            success = self._integer(fields, 0) == 1
            received = self._integer(fields, 4, -1)
            acknowledged_crc = self._integer(fields, 5, -1)
            if success and received >= expected_received and acknowledged_crc == expected_crc:
                return received
            last_error = PathUploadError(
                f'块确认失败：起点 {start_index}，累计接收 {received}'
            )

        raise PathUploadError(f'{last_error}，连续 {self._retries} 次失败')

    def _emit_progress(self, source, completed, count):
        self._emit_event('path_upload_progress', {
            'source': source,
            'completed': completed,
            'count': count,
            'percent': completed * 100.0 / count
        })

    def _send_markers(self, markers):
        for marker_index, marker_kind in markers:
            command = f'vpath upload marker {marker_index} {marker_kind}'
            self._exchange(
                command,
                lambda tag, fields, expected_index=marker_index,
                expected_kind=marker_kind: (
                    tag == 'marker'
                    and self._integer(fields, 0) == 1
                    and self._integer(fields, 1) == expected_index
                    and len(fields) > 2 and fields[2] == expected_kind
                )
            )

    def _send_phototube_zones(self, zones):
        for start_index, end_index in zones:
            command = f'vpath upload ptzone {start_index} {end_index}'
            self._exchange(
                command,
                lambda tag, fields, expected_start=start_index,
                expected_end=end_index: (
                    tag == 'ptzone'
                    and self._integer(fields, 0) == 1
                    and self._integer(fields, 1) == expected_start
                    and self._integer(fields, 2) == expected_end
                )
            )

    def _run_binary(self, points, packed_path, checksum, markers, zones, source):
        count = len(points)
        session_id = secrets.randbits(32)
        begin_command = f'vpath upload begin2 {count} {session_id} {checksum}'

        try:
            _, begin_tag, begin = self._exchange(
                begin_command,
                lambda tag, fields: (
                    tag == 'begin2' and self._integer(fields, 1) == session_id
                ),
                timeout=self._begin_timeout,
                allow_usage=True,
                keepalive=lambda tag, fields: tag in ('preparing', 'erase', 'erase_done')
            )
        except PathUploadError as exc:
            if '等待主控确认超时' in str(exc):
                return False
            raise

        if begin_tag == 'usage':
            return False
        if self._integer(begin, 0) != 1:
            raise PathUploadError('主控拒绝二进制路径上传')
        accepted = self._integer(begin, 2, -1)
        capacity = self._integer(begin, 3, 0)
        if accepted != count or capacity < count:
            raise PathUploadError(f'主控容量不足：需要 {count} 点，可用 {capacity} 点')

        self._emit_event('path_upload_started', {
            'source': source,
            'count': count,
            'capacity': capacity,
            'marker_count': len(markers),
            'phototube_zone_count': len(zones),
            'protocol': 'binary32'
        })

        for start_index in range(0, count, BINARY_BLOCK_POINT_COUNT):
            point_count = min(BINARY_BLOCK_POINT_COUNT, count - start_index)
            byte_start = start_index * 16
            byte_end = byte_start + point_count * 16
            frame = self._build_block_frame(
                session_id, start_index, packed_path[byte_start:byte_end]
            )
            received = self._send_block(frame, session_id, start_index, point_count)
            self._emit_progress(source, min(received, count), count)

        self._send_markers(markers)
        self._send_phototube_zones(zones)
        commit_sequence, _, commit = self._exchange(
            f'vpath upload commit2 {session_id} {checksum}',
            lambda tag, fields: (
                tag == 'commit2' and self._integer(fields, 1) == session_id
            )
        )
        if (self._integer(commit, 0) != 1
                or self._integer(commit, 2) != count
                or self._integer(commit, 3) != count
                or self._integer(commit, 4) != checksum):
            raise PathUploadError('主控整条路径校验失败')
        return commit_sequence

    def _run_legacy(self, points, markers, zones, source):
        count = len(points)
        _, _, begin = self._exchange(
            f'vpath upload begin {count}',
            lambda tag, fields: tag == 'begin' and self._integer(fields, 0) == 1,
            timeout=self._begin_timeout,
            keepalive=lambda tag, fields: tag in ('preparing', 'erase', 'erase_done')
        )
        capacity = self._integer(begin, 2, 0)
        accepted = self._integer(begin, 1, -1)
        if accepted != count or capacity < count:
            raise PathUploadError(f'主控容量不足：需要 {count} 点，可用 {capacity} 点')
        self._emit_event('path_upload_started', {
            'source': source,
            'count': count,
            'capacity': capacity,
            'marker_count': len(markers),
            'phototube_zone_count': len(zones),
            'protocol': 'legacy'
        })

        progress_step = max(1, count // 200)
        for index, point in enumerate(points):
            command = 'vpath upload point {} {:.3f} {:.3f} {:.6f} {:.1f}'.format(
                index, *point
            )
            self._exchange(
                command,
                lambda tag, fields, expected=index: (
                    tag == 'point'
                    and self._integer(fields, 0) == 1
                    and self._integer(fields, 1) == expected
                    and self._integer(fields, 2) >= expected + 1
                )
            )
            completed = index + 1
            if completed == count or completed % progress_step == 0:
                self._emit_progress(source, completed, count)

        self._send_markers(markers)
        self._send_phototube_zones(zones)
        commit_sequence, _, _ = self._exchange(
            'vpath upload commit',
            lambda tag, fields: (
                tag == 'commit' and self._integer(fields, 0) == 1
                and self._integer(fields, 1) == count
                and self._integer(fields, 2) == count
            )
        )
        return commit_sequence

    def _run(self, points, markers, zones, source):
        count = len(points)
        started = time.monotonic()
        packed_path = self._pack_points(points)
        checksum = self._path_checksum(packed_path)
        try:
            commit_sequence = None
            if self._send_bytes is not None:
                binary_result = self._run_binary(
                    points, packed_path, checksum, markers, zones, source
                )
                if binary_result is not False:
                    commit_sequence = binary_result
            if commit_sequence is None:
                commit_sequence = self._run_legacy(points, markers, zones, source)

            _, _, done = self._wait_after(
                commit_sequence,
                lambda tag, fields: tag == 'done',
                self._done_timeout
            )
            self._emit_event('path_upload_finished', {
                'success': True,
                'source': source,
                'count': count,
                'marker_count': len(markers),
                'phototube_zone_count': len(zones),
                'checksum': self._integer(done, 1, 0),
                'length_mm': float(done[2]) if len(done) > 2 else 0,
                'elapsed_s': time.monotonic() - started
            })
        except Exception as exc:
            try:
                self._send_line('vpath upload abort')
            except Exception:
                pass
            self._emit_event('path_upload_finished', {
                'success': False,
                'source': source,
                'count': count,
                'error': str(exc),
                'marker_count': len(markers),
                'phototube_zone_count': len(zones),
                'elapsed_s': time.monotonic() - started
            })
        finally:
            with self._condition:
                self._active = False
                self._cancelled = False
                self._condition.notify_all()
