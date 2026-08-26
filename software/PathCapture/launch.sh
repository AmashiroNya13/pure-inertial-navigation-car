#!/usr/bin/env bash
# PathCapture - Launch Script (Linux / macOS)
# Starts the backend server and opens the browser automatically.

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

echo "========================================"
echo "  PathCapture - Real-Time Path Display"
echo "========================================"
echo ""

# Check Python availability
PYTHON=""
for cmd in python3 python; do
    if command -v "$cmd" &> /dev/null; then
        PYTHON="$cmd"
        break
    fi
done

if [ -z "$PYTHON" ]; then
    echo "ERROR: Python not found. Please install Python 3.8+."
    echo "       https://www.python.org/downloads/"
    exit 1
fi

echo "[1/3] Python found: $($PYTHON --version)"

# Create and activate virtual environment if not present
if [ ! -d "venv" ]; then
    echo "[2/3] Creating virtual environment..."
    $PYTHON -m venv venv
fi
source venv/bin/activate

# Install dependencies
echo "[3/3] Installing dependencies..."
pip install flask flask-socketio pyserial -q 2>&1 | tail -1

# Start server in background
echo "[4/4] Starting server..."
python app.py &
SERVER_PID=$!

# Wait for server to be ready
echo "Waiting for server to start..."
for i in $(seq 1 20); do
    if curl -s http://localhost:5000 > /dev/null 2>&1; then
        break
    fi
    sleep 0.5
done

# Open browser
echo "Opening browser at http://localhost:5000"
if command -v xdg-open &> /dev/null; then
    xdg-open http://localhost:5000
elif command -v open &> /dev/null; then
    open http://localhost:5000
elif command -v gnome-open &> /dev/null; then
    gnome-open http://localhost:5000
else
    echo "Please open http://localhost:5000 in your browser."
fi

echo ""
echo "Server running. Press Ctrl+C to stop."

# Wait for server process
wait $SERVER_PID
