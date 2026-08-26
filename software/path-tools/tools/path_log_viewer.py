#!/usr/bin/env python3
"""Visualize vehicle path logs exported from the AMR UART stream.

Supported records:
  {path}x,y,theta_deg,index,dist,left_dist,right_dist,left_count,right_count,...
  {pathdump}index,x,y,theta_rad,speed_mm_s

By default the script prefers the last live recording segment ({path}). If the
file only contains pathdump data, it uses the last dump block.
"""

from __future__ import annotations

import argparse
import html
import math
import re
from pathlib import Path
from typing import Iterable


RAW_FIELDS = (
    "x_mm",
    "y_mm",
    "theta_deg",
    "index",
    "record_distance_mm",
    "left_total_distance_mm",
    "right_total_distance_mm",
    "left_total_count",
    "right_total_count",
    "lr_diff_mm",
    "encoder_theta_deg",
    "left_sample_count",
    "right_sample_count",
    "pose_update_count",
    "encoder_update_count",
)


def parse_prefixed_records(path: Path) -> list[tuple[str, str]]:
    records: list[tuple[str, str]] = []
    text = path.read_text(errors="ignore")
    matches = list(re.finditer(r"\{([^}]+)\}", text))

    for index, match in enumerate(matches):
        prefix = match.group(1)
        body_start = match.end()
        body_end = matches[index + 1].start() if (index + 1) < len(matches) else len(text)
        body = "".join(text[body_start:body_end].split())
        records.append((prefix, body))

    return records


def floats_from_csv(body: str) -> list[float] | None:
    try:
        return [float(part) for part in body.split(",")]
    except ValueError:
        return None


def parse_raw_segments(records: Iterable[tuple[str, str]]) -> list[list[dict[str, float]]]:
    segments: list[list[dict[str, float]]] = []
    current: list[dict[str, float]] = []
    last_index: int | None = None

    for prefix, body in records:
        if prefix != "path":
            continue
        values = floats_from_csv(body)
        if values is None or len(values) < len(RAW_FIELDS):
            continue

        point = {name: values[index] for index, name in enumerate(RAW_FIELDS)}
        point["index"] = int(point["index"])
        index_value = int(point["index"])

        if current and last_index is not None and index_value <= last_index:
            segments.append(current)
            current = []

        current.append(point)
        last_index = index_value

    if current:
        segments.append(current)

    return segments


def parse_dump_blocks(records: Iterable[tuple[str, str]]) -> list[dict[str, object]]:
    blocks: list[dict[str, object]] = []
    current: dict[str, object] | None = None
    current_points: list[dict[str, float]] | None = None
    current_slot = "unknown"

    for prefix, body in records:
        if prefix != "pathdump":
            continue

        if body.startswith("slot,"):
            current_slot = body.split(",", 1)[1]
            continue

        if body.startswith("begin,"):
            if current is not None and current_points:
                blocks.append(current)
            current_points = []
            current = {"slot": current_slot, "points": current_points, "declared_count": body.split(",", 1)[1]}
            continue

        if body.startswith("length,") and current is not None:
            try:
                current["declared_length_mm"] = float(body.split(",", 1)[1])
            except ValueError:
                pass
            continue

        if body.startswith("end,"):
            if current is not None:
                blocks.append(current)
            current = None
            current_points = None
            continue

        values = floats_from_csv(body)
        if values is None or len(values) < 5 or current_points is None:
            continue

        current_points.append(
            {
                "index": int(values[0]),
                "x_mm": values[1],
                "y_mm": values[2],
                "theta_deg": values[3] * 180.0 / math.pi,
                "theta_rad": values[3],
                "speed_mm_s": values[4],
            }
        )

    if current is not None and current_points:
        blocks.append(current)

    return blocks


def polyline_length(points: list[dict[str, float]]) -> float:
    total = 0.0
    for prev, cur in zip(points, points[1:]):
        total += math.hypot(cur["x_mm"] - prev["x_mm"], cur["y_mm"] - prev["y_mm"])
    return total


def bounds(points: list[dict[str, float]]) -> tuple[float, float, float, float]:
    xs = [point["x_mm"] for point in points]
    ys = [point["y_mm"] for point in points]
    return min(xs), max(xs), min(ys), max(ys)


def color_for_speed(speed: float, min_speed: float, max_speed: float) -> str:
    if max_speed <= min_speed:
        ratio = 0.0
    else:
        ratio = max(0.0, min(1.0, (speed - min_speed) / (max_speed - min_speed)))
    hue = 220.0 - 210.0 * ratio
    return f"hsl({hue:.1f} 82% 45%)"


def make_path_svg(points: list[dict[str, float]], title: str) -> str:
    if not points:
        return "<p>No points.</p>"

    min_x, max_x, min_y, max_y = bounds(points)
    width = max(1.0, max_x - min_x)
    height = max(1.0, max_y - min_y)
    pad = max(width, height) * 0.08 + 20.0
    view_min_x = min_x - pad
    view_min_y = min_y - pad
    view_w = width + pad * 2.0
    view_h = height + pad * 2.0
    stroke_w = max(1.0, max(width, height) / 220.0)

    speed_values = [point["speed_mm_s"] for point in points if "speed_mm_s" in point]
    min_speed = min(speed_values) if speed_values else 0.0
    max_speed = max(speed_values) if speed_values else 0.0

    elements: list[str] = []
    elements.append(
        f'<svg class="path-svg" viewBox="{view_min_x:.3f} {-view_min_y - view_h:.3f} {view_w:.3f} {view_h:.3f}" '
        f'preserveAspectRatio="xMidYMid meet" role="img" aria-label="{html.escape(title)}">'
    )
    elements.append(
        f'<rect x="{view_min_x:.3f}" y="{-view_min_y - view_h:.3f}" width="{view_w:.3f}" height="{view_h:.3f}" '
        'fill="#fbfbf8" />'
    )

    if speed_values:
        for prev, cur in zip(points, points[1:]):
            color = color_for_speed((prev["speed_mm_s"] + cur["speed_mm_s"]) * 0.5, min_speed, max_speed)
            elements.append(
                f'<line x1="{prev["x_mm"]:.3f}" y1="{-prev["y_mm"]:.3f}" '
                f'x2="{cur["x_mm"]:.3f}" y2="{-cur["y_mm"]:.3f}" '
                f'stroke="{color}" stroke-width="{stroke_w:.3f}" stroke-linecap="round" />'
            )
    else:
        path_data = " ".join(
            ("M" if index == 0 else "L") + f'{point["x_mm"]:.3f},{-point["y_mm"]:.3f}'
            for index, point in enumerate(points)
        )
        elements.append(
            f'<path d="{path_data}" fill="none" stroke="#1d5f99" stroke-width="{stroke_w:.3f}" '
            'stroke-linecap="round" stroke-linejoin="round" />'
        )

    marker_r = max(2.5, stroke_w * 2.0)
    start = points[0]
    end = points[-1]
    elements.append(f'<circle cx="{start["x_mm"]:.3f}" cy="{-start["y_mm"]:.3f}" r="{marker_r:.3f}" fill="#159447" />')
    elements.append(f'<circle cx="{end["x_mm"]:.3f}" cy="{-end["y_mm"]:.3f}" r="{marker_r:.3f}" fill="#d12d2d" />')

    arrow_every = max(1, len(points) // 24)
    arrow_len = max(10.0, min(35.0, max(width, height) / 18.0))
    for point in points[::arrow_every]:
        theta = point.get("theta_rad", point["theta_deg"] * math.pi / 180.0)
        x1 = point["x_mm"]
        y1 = -point["y_mm"]
        x2 = x1 + math.cos(theta) * arrow_len
        y2 = y1 - math.sin(theta) * arrow_len
        elements.append(
            f'<line x1="{x1:.3f}" y1="{y1:.3f}" x2="{x2:.3f}" y2="{y2:.3f}" '
            f'stroke="#222" stroke-opacity="0.45" stroke-width="{stroke_w * 0.45:.3f}" stroke-linecap="round" />'
        )

    elements.append("</svg>")
    return "\n".join(elements)


def make_speed_svg(points: list[dict[str, float]]) -> str:
    speeds = [point.get("speed_mm_s") for point in points]
    if not speeds or any(speed is None for speed in speeds):
        return ""

    values = [float(speed) for speed in speeds if speed is not None]
    min_speed = min(values)
    max_speed = max(values)
    span = max(1.0, max_speed - min_speed)
    width = 1000.0
    height = 220.0
    pad = 35.0
    step = (width - 2 * pad) / max(1, len(values) - 1)
    path_data = []
    for index, speed in enumerate(values):
        x = pad + index * step
        y = pad + (height - 2 * pad) * (1.0 - ((speed - min_speed) / span))
        path_data.append(("M" if index == 0 else "L") + f"{x:.2f},{y:.2f}")
    return (
        '<svg class="speed-svg" viewBox="0 0 1000 220" preserveAspectRatio="none" role="img" '
        'aria-label="speed profile">'
        '<rect x="0" y="0" width="1000" height="220" fill="#fbfbf8" />'
        f'<path d="{" ".join(path_data)}" fill="none" stroke="#b14b25" stroke-width="3" />'
        f'<text x="35" y="24">speed: {min_speed:.0f} - {max_speed:.0f} mm/s</text>'
        "</svg>"
    )


def summarize(points: list[dict[str, float]], source: str, block_info: dict[str, object] | None) -> list[tuple[str, str]]:
    length_xy = polyline_length(points)
    min_x, max_x, min_y, max_y = bounds(points)
    rows = [
        ("source", source),
        ("points", str(len(points))),
        ("xy length", f"{length_xy:.1f} mm"),
        ("x range", f"{min_x:.1f} .. {max_x:.1f} mm"),
        ("y range", f"{min_y:.1f} .. {max_y:.1f} mm"),
        ("start", f'{points[0]["x_mm"]:.1f}, {points[0]["y_mm"]:.1f}, {points[0]["theta_deg"]:.2f} deg'),
        ("end", f'{points[-1]["x_mm"]:.1f}, {points[-1]["y_mm"]:.1f}, {points[-1]["theta_deg"]:.2f} deg'),
    ]
    if "record_distance_mm" in points[-1]:
        rows.extend(
            [
                ("record distance", f'{points[-1]["record_distance_mm"]:.1f} mm'),
                ("left distance", f'{points[-1]["left_total_distance_mm"]:.1f} mm'),
                ("right distance", f'{points[-1]["right_total_distance_mm"]:.1f} mm'),
                ("left-right diff", f'{points[-1]["lr_diff_mm"]:.1f} mm'),
            ]
        )
    if "speed_mm_s" in points[0]:
        speeds = [point["speed_mm_s"] for point in points]
        rows.extend(
            [
                ("min speed", f"{min(speeds):.1f} mm/s"),
                ("max speed", f"{max(speeds):.1f} mm/s"),
                ("avg speed", f"{sum(speeds) / len(speeds):.1f} mm/s"),
            ]
        )
    if block_info is not None:
        for key in ("slot", "declared_count", "declared_length_mm"):
            if key in block_info:
                rows.append((key, str(block_info[key])))
    return rows


def write_html(output_path: Path, input_path: Path, source: str, points: list[dict[str, float]], block_info: dict[str, object] | None) -> None:
    rows = summarize(points, source, block_info)
    stats_html = "\n".join(
        f"<tr><th>{html.escape(key)}</th><td>{html.escape(value)}</td></tr>" for key, value in rows
    )
    title = f"{input_path.name} - {source}"
    svg = make_path_svg(points, title)
    speed_svg = make_speed_svg(points)
    html_text = f"""<!doctype html>
<html lang="zh-CN">
<head>
<meta charset="utf-8" />
<meta name="viewport" content="width=device-width, initial-scale=1" />
<title>{html.escape(title)}</title>
<style>
body {{ margin: 0; font-family: system-ui, -apple-system, Segoe UI, sans-serif; background: #f4f2ee; color: #202124; }}
main {{ max-width: 1200px; margin: 0 auto; padding: 24px; }}
h1 {{ font-size: 22px; margin: 0 0 14px; }}
.panel {{ background: white; border: 1px solid #ddd7ce; border-radius: 8px; padding: 16px; margin: 14px 0; }}
table {{ border-collapse: collapse; width: 100%; font-size: 14px; }}
th, td {{ border-bottom: 1px solid #eee8df; padding: 7px 9px; text-align: left; }}
th {{ width: 190px; color: #5f6368; font-weight: 600; }}
.path-svg {{ width: 100%; height: 72vh; min-height: 460px; border: 1px solid #e3ded5; background: #fbfbf8; }}
.speed-svg {{ width: 100%; height: 220px; border: 1px solid #e3ded5; background: #fbfbf8; }}
.note {{ color: #5f6368; font-size: 13px; }}
</style>
</head>
<body>
<main>
<h1>{html.escape(title)}</h1>
<div class="panel">
<table>{stats_html}</table>
</div>
<div class="panel">
{svg}
<p class="note">Green is start, red is end. Small black strokes show recorded heading.</p>
</div>
{('<div class="panel">' + speed_svg + '</div>') if speed_svg else ''}
</main>
</body>
</html>
"""
    output_path.write_text(html_text, encoding="utf-8")


def choose_points(
    records: list[tuple[str, str]],
    source: str,
    slot: str,
) -> tuple[str, list[dict[str, float]], dict[str, object] | None]:
    raw_segments = parse_raw_segments(records)
    dump_blocks = parse_dump_blocks(records)

    if source in ("auto", "raw") and raw_segments:
        return "last raw {path} segment", raw_segments[-1], None

    if source in ("auto", "dump") and dump_blocks:
        candidates = dump_blocks
        if slot != "any":
            candidates = [block for block in dump_blocks if block.get("slot") == slot]
        if not candidates:
            available = ", ".join(str(block.get("slot", "unknown")) for block in dump_blocks)
            raise SystemExit(f"No {{pathdump}} block for slot={slot}. Available slots: {available}")
        block = candidates[-1]
        return f"last {{pathdump}} block ({block.get('slot', 'unknown')})", block["points"], block

    raise SystemExit("No supported {path} or {pathdump} records found.")


def main() -> None:
    parser = argparse.ArgumentParser(description="Visualize AMR path UART logs.")
    parser.add_argument("log", type=Path, help="UART log .dat file")
    parser.add_argument("--source", choices=("auto", "raw", "dump"), default="auto")
    parser.add_argument("--slot", choices=("any", "planned", "raw"), default="any", help="Dump slot to use with --source dump")
    parser.add_argument("--out", type=Path, default=None, help="Output HTML path")
    args = parser.parse_args()

    records = parse_prefixed_records(args.log)
    source_name, points, block_info = choose_points(records, args.source, args.slot)
    output = args.out
    if output is None:
        output = args.log.with_name(args.log.stem + "_view.html")

    write_html(output, args.log, source_name, points, block_info)
    print(f"wrote: {output}")
    print(f"source: {source_name}")
    print(f"points: {len(points)}")
    print(f"xy_length_mm: {polyline_length(points):.1f}")
    if "record_distance_mm" in points[-1]:
        print(f"record_distance_mm: {points[-1]['record_distance_mm']:.1f}")
    if "speed_mm_s" in points[0]:
        speeds = [point["speed_mm_s"] for point in points]
        print(f"speed_mm_s: min={min(speeds):.1f}, max={max(speeds):.1f}, avg={sum(speeds) / len(speeds):.1f}")


if __name__ == "__main__":
    main()
