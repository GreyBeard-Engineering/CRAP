@echo off
rem Double-click to start the gearbox GUI. Uses the project venv when present.
cd /d "%~dp0"
if exist "venv\Scripts\python.exe" (
    "venv\Scripts\python.exe" launch_gui.py %*
) else (
    python launch_gui.py %*
)
if errorlevel 1 pause
