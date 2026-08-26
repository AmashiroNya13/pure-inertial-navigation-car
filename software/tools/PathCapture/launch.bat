@echo off
REM PathCapture - Launch Script (Windows)
REM Starts the backend server and opens the browser automatically.

echo ========================================
echo   PathCapture - Real-Time Path Display
echo ========================================
echo.

REM Check Python availability
where python >nul 2>&1
if %ERRORLEVEL% NEQ 0 (
    where python3 >nul 2>&1
    if %ERRORLEVEL% NEQ 0 (
        echo ERROR: Python not found. Please install Python 3.8+.
        echo        https://www.python.org/downloads/
        pause
        exit /b 1
    )
    set PYTHON=python3
) else (
    set PYTHON=python
)

echo [1/4] Python found:
%PYTHON% --version

REM Create and activate virtual environment if not present
if not exist "venv\" (
    echo [2/4] Creating virtual environment...
    %PYTHON% -m venv venv
)
call venv\Scripts\activate.bat

REM Install dependencies
echo [3/4] Installing dependencies...
pip install flask flask-socketio pyserial -q

REM Start server
echo [4/4] Starting server...
echo.
start http://localhost:5000
python app.py

pause
