# PathCapture — High-Performance Real-Time Path Display

A high-performance, real-time 2D path visualization tool for robotics and embedded systems. Receives coordinate data via serial port and renders it on a Canvas-based web interface with smooth zooming, panning, and 20k+ point support.

## Features

- **Real-time rendering** — Canvas2D with `requestAnimationFrame` loop; handles 20,000+ points at 60 FPS
- **Serial data ingestion** — Reads `R(car_x,car_y),(lookahead_x,lookahead_y)` and `P(x,y)` messages via serial port
- **Three data layers** with independent visibility toggles and distinct colors:
  - 🟦 **Predefined Path** (persistent, blue)
  - 🟥 **Real-Time Track** (appends on arrival, red, with direction arrow)
  - 🟩 **Lookahead Point** (latest shown by default, toggleable history, green)
- **Zoom & Pan** — Mouse wheel zoom (centered on cursor), click-drag pan, pinch-to-zoom on touch devices
- **Auto-Fit** — Animated fit of visible data to the viewport
- **Light/Dark theme** — Default light theme with dark mode toggle; persists across sessions
- **Serial port picker** — Direct dropdown (no submenus); auto-refresh; baud rates up to 2,000,000
- **Replay sessions** — Each confirmed replay is stored as an independent run with a shared elapsed-time axis
- **Serial command console** — Send commands and inspect filtered RX/TX replies in the same window
- **Data persistence** — Save the baseline path and all replay runs as a version 3 JSON file
- **Status bar** — Live mouse world coordinates, zoom level, point counts, FPS
- **Keyboard shortcuts** — Arrow keys pan, `+`/`-` zoom, `F` or `0` auto-fit
- **Responsive** — Works on desktops and tablets; touch-friendly

## Quick Start

### Prerequisites

- Python 3.8 or later
- pip (Python package manager)

### Launch

**Linux / macOS:**

```bash
chmod +x launch.sh
./launch.sh
```

**Windows:**

```cmd
launch.bat
```

The script will:
1. Check for Python
2. Install dependencies (`flask`, `flask-socketio`, `pyserial`)
3. Start the web server on `http://localhost:5000`
4. Open your default browser automatically

### Manual Setup

```bash
cd PathCapture
pip install -r requirements.txt
python app.py
```

Then open **http://localhost:5000** in your browser.

## Usage

### Workspaces

The top segmented control separates tasks that use the same map:

- **实时监控** keeps serial recording, replay, emergency stop and run playback together.
- **规划验证** compares the untouched path, browser geometry result and firmware-equivalent speed plan. The path cursor shows the nearest/base point, steering lookahead, tangent sample and independent speed-preview point. A lateral-offset input makes cross-track behavior visible without altering the stored path.
- **外部路径** imports node-map JSON, previews the node polyline and exported cut-corner samples independently, then optionally promotes the selected route to the planning workspace.

The curvature and speed charts are linked by path distance. The speed chart exposes curvature, wheel-speed, forward-acceleration, backward-braking and final constraints instead of presenting only the final planned speed.

### Connecting to a Device

1. Connect your embedded device via USB serial
2. Select the **serial port** from the dropdown (click 🔄 to refresh)
3. Choose the **baud rate** (default: 115200; supports up to 2,000,000)
4. Click **Connect**

### Vehicle workflow

The mode badge follows vehicle replies; clicking a button does not change the mode optimistically.

| Action | Serial command | PathCapture behavior |
|--------|----------------|----------------------|
| Start recording | `vpath record` | Wait for `{pathinfo}record_start`, then clear the old blue baseline and display incoming path points |
| Stop and save recording | `vpath record stop` | Keep the completed blue path as the baseline |
| Start replay | `vpath replay` | Start a new replay run after the vehicle confirms replay start |
| Emergency stop | `vstop` | Send immediately; the current replay run is closed when stop/safety feedback arrives |

After replay finishes or stops, select a run at the bottom and use the time slider, play/pause button and speed selector. Imported files keep their recorded path as the blue baseline until the vehicle confirms a new recording.

### Geometry validation

Enter **规划验证** to configure browser-only geometry and speed planning:

- Blue points are the untouched recorded path.
- Orange points are the reconstructed geometric path.
- Every coordinate layer has an independent visibility switch. Recorded, geometry, firmware, external, replay and lookahead points can be compared and inspected in any workspace.
- **全局曲率样条（推荐）** fits one cubic parametric spline across the complete path. Detected straight sections receive strong TLS line constraints, while second- and third-control-difference penalties suppress curvature and curvature-rate discontinuities through compound and S bends.
- **分段回旋圆弧（旧）** keeps the earlier independent `line -> clothoid -> arc -> clothoid -> line` corner reconstruction for comparison.
- The curvature chart plots signed curvature against path distance. Hover an orange point to inspect its geometry type, curvature and radius.
- Enable **限制最小半径** and enter a radius in millimeters to search progressively smoother global splines. A result is marked **已满足** only after every planned point passes `abs(kappa) <= 1 / radius` and the complete curve remains inside the 120 mm offset guard. Dashed red lines show the selected curvature bound.
- Enable **横向偏移优化（实验）** to add coarse lateral-offset control points along the global spline. The browser performs projected coordinate searches inside the 120 mm corridor, directly penalizing curvature-limit violations, excessive displacement and abrupt offset changes. This can move individual bends instead of only increasing one global smoothing factor.
- The experimental optimizer is browser-only and intentionally disabled by default. It preserves path order but does not yet model measured left/right track boundaries, vehicle footprint or prove that the optimized path is globally optimal.
- **未满足** means the current search family did not find a valid curve inside the offset guard; it is not a mathematical proof that no possible path exists.
- A planned curve that still exceeds the 120 mm source-path offset guard after relaxation is rejected and displayed as the resampled source path.

The validator never overwrites the recorded path, sends planned points to the vehicle or changes replay control. The recommended method is a constrained parametric spline with curvature-related regularization; it is not yet a nonlinear direct optimization of `kappa(s)`.

### External node maps

Node-map JSON exported by `tools/node_coordinate_viewer.html` is accepted directly. Supported fields include `nodes[]`, `start_node_id`, `computed_shortest_route.full_path_node_ids` and `computed_shortest_route.speed_segments[].points_mm`.

The node viewer provides **下载 JSON** for a normal file workflow. Its **复制导出文本** output can also be pasted directly through PathCapture's **粘贴 JSON** dialog.

- The exported `start_node_id` is selected by default and remains at its exported `(x_mm, y_mm)` position.
- Editable start X/Y fields default to the exported start coordinates and translate the complete route to that absolute position.
- Selecting another origin restarts a closed route at that node and resets start X/Y to that node's exported coordinates.
- A shoulder layer is enabled by default on both sides of the selected external route. Its inner edge is 150 mm from the path and its width is 40 mm; both values remain editable.
- **首段对齐 +X** rotates the first route segment onto the inertial positive X axis.
- Additional `0°`, `+90°`, `-90°`, `180°`, Y mirroring and route reversal controls are applied around that origin.
- **节点折线路径** is the recommended raw input for browser geometry planning.
- **外部切弯路径** is a comparison layer by default, avoiding accidental double optimization.
- Exported coordinates do not carry the source tool's display-only `Y 向下` setting, so orientation is always explicit in the import controls.
- Imported X/Y controls are relative offsets and therefore start at `0, 0`; the exported origin coordinates remain the transform base. Shoulders are generated only from external node-track links, never from cut-corner samples. Each bounded region receives one thick line along its longer axis. The line starts from average Y or X, then follows the local maximum-clearance center as nearby node-road segments move inward. Every interpolated centerline section must retain the configured clearance plus half the shoulder width; impossible sections are omitted instead of crossing the road.
- The vehicle collision switch wraps the replay arrow with the configured body footprint. The replay coordinate and `theta_deg` define the motion-center pose; with the default 150 mm length and 50 mm rear-to-center mounting distance, the body extends 100 mm forward and 50 mm rearward from that point. Each safe-to-collision transition records replay time, traveled distance and coordinates.

The firmware speed planner uses the current defaults: 400-3200 mm/s, 25,000 mm/s² lateral and longitudinal limits, a 120 mm wheelbase, 200 mm at 500 mm/s after launch, and 300 mm terminal deceleration. These values can be adjusted in the planning workspace for comparison; browser planning does not write Flash or send commands to the vehicle.

### Data Format

The tool is adapted for the current Mad_Circuits serial replies. Each message must end with `\n` or `\r\n`.

| Format | Description | Layer |
|--------|-------------|-------|
| `{pathdump}index,x_mm,y_mm,theta_rad,speed_mm_s` | Dumped recorded/planned path point | Blue path |
| `{path}x_mm,y_mm,theta_deg,index,record_dist_mm,...` | Path point printed while recording | Blue path |
| `{vrpl1}pose_x_mm,pose_y_mm,pose_theta_deg,base_x_mm,base_y_mm,base_theta_deg,cte_mm,along_mm,target_x_mm,target_y_mm` | Replay pose + current tracking target | Red track + green target |
| `{vrpl2}...` | Target/actual wheel speed and compact angle diagnostics | Point details |
| `{vrpl3}...` | Target/final wheel PWM and ESC fault state | Point details |
| `{vrpl0}...`, `{vrpl4}...`, `{vrplg}...` | Point index, lookahead and replay startup phase | Point details |
| Binary replay frame `A5 5A` | Default compact equivalent of `{vrpl0}` through `{vrpl3}`; decoded by the backend | Red track + point details |
| `{vmark}add,kind,index,x_mm,y_mm,theta_deg` | Manual turn-in/turn-out marker | Green target layer |

Legacy `P(x,y)` and `R(car_x,car_y),(lookahead_x,lookahead_y)` lines are still accepted for compatibility.

Replay telemetry defaults to the compact binary frame. Use `vtelemetry text` for legacy text diagnostics,
`vtelemetry binary` to restore the compact mode, and `vtelemetry status` to query the current mode.
Speed-loop ground tests keep the original `{vspd}` text packet unchanged.

**Examples:**

```
{pathdump}0,100.000,200.000,1.570796,500.000
{path}110.000,210.000,90.000,1,12.000,0,0,0,0,0,0,0,0,0,0
{vrpl1}120.0,220.0,91.00,111.0,211.0,90.0,9.0,10.0,150.0,250.0
{vmark}add,in,3,130.000,230.000,92.000
```

- `{pathdump}begin,...` clears the blue path before a new dump, preventing repeated dumps from stacking.
- Firmware coordinates are millimeters; map axes and mouse coordinates are labeled in centimeters.
- Hover a point to inspect position, tracking target, cross-track error, planned/actual speed, wheel speed and PWM.
- Enable **Diagnostics** only when tuning the controller. It adds angle/yaw, tangent, base-point and ESC details.
- Deep diagnostics (`vrpl5/6/7`), IMU raw data, phototube data, Flash logs, startup messages, malformed lines and unknown prefixes are intentionally filtered out.
- Saved files contain structured visualization data only. Raw serial lines are no longer duplicated into the JSON file; version 1 files remain loadable.

### View Controls

| Action | Control |
|--------|---------|
| Zoom | Mouse wheel / `+` `-` keys / toolbar buttons |
| Pan | Click & drag / Arrow keys |
| Auto-fit | **Auto Fit** button / `F` key |
| Pinch-zoom | Two-finger gesture (touch devices) |

### Visibility Toggles

Use the checkboxes in the toolbar to show/hide:

- **Path** — Predefined trajectory points (blue)
- **Real-time** — Live car track (red)
- **Lookahead** — Preview point (green)
- **History** — When checked, all past lookahead points are displayed instead of only the latest

### Saving & Loading Data

- **Save** — Downloads a version 3 JSON file containing the baseline and every replay run
- **Load** — Click **Load**, select a `.json` file (previously saved or server-generated)

The JSON format:

```json
{
  "version": "3.0",
  "saved_at": "2026-07-14T14:00:00",
  "predefined": [{"x": 1.5, "y": -2.3, "t": 0.001}, ...],
  "realtime":  [{"x": 0.0, "y": 0.0, "t": 0.002}, ...],
  "lookahead": [{"x": 1.5, "y": -2.3, "t": 0.002}, ...],
  "replay_runs": [{
    "id": 1,
    "status": "completed",
    "points": [{"x": 0.0, "y": 0.0, "elapsed_s": 0.0}, ...],
    "lookahead": [{"x": 1.5, "y": -2.3, "elapsed_s": 0.0}, ...]
  }]
}
```

Version 2 flat files are accepted and are converted to one imported replay run in memory.

## Architecture

```
PathCapture/
├── app.py                  # Flask + Socket.IO backend; serial reader thread
├── static/
│   ├── index.html          # Single-page application shell
│   ├── style.css           # Light/dark theme, responsive layout
│   ├── app.js              # Canvas2D renderer, socket.io client, interaction
│   ├── geometry-planner.js # Browser geometry algorithms
│   ├── firmware-speed-planner.js # Firmware-equivalent speed constraints
│   └── external-path.js    # Node-map parsing and coordinate transforms
├── launch.sh               # Linux/macOS launcher
├── launch.bat              # Windows launcher
├── requirements.txt        # Python dependencies
└── README.md               # This file
```

### Data Flow

```
Embedded Device ──(serial)──> Serial Reader Thread ──> process_line()
                                    │
                              socketio.emit('serial_data')
                                    │
                              WebSocket ──> Browser ──> Canvas2D Renderer
```

## Performance Notes

- Uses **Canvas2D** with view-transform rendering — point coordinates never change, only the transform matrix
- Grid lines are clipped to prevent excessive draw calls at extreme zoom levels
- Dirty-flag optimization avoids re-rendering when nothing changes
- Device pixel ratio clamped to 2x for sharp rendering without excessive GPU load
- Tested rendering 20,000+ polyline points at 60 FPS on modern hardware

## Troubleshooting

| Issue | Solution |
|-------|----------|
| "No ports found" | Ensure device is connected and drivers are installed; click 🔄 to refresh |
| "Access denied" (Linux) | Add user to `dialout` group: `sudo usermod -a -G dialout $USER` (log out/in after) |
| Port in use | Close other terminal/serial monitor programs (Arduino IDE, PuTTY, screen, etc.) |
| Page shows "Unable to connect" | Ensure `python app.py` is running; check port 5000 isn't in use |

## License

MIT
