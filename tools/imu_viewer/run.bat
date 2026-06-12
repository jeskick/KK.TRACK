@echo off
setlocal
cd /d "%~dp0"

if not exist ".venv\Scripts\python.exe" (
  echo Creating venv...
  python -m venv .venv
  if errorlevel 1 exit /b 1
  .venv\Scripts\pip install -r requirements.txt
  if errorlevel 1 exit /b 1
)

.venv\Scripts\python imu_viewer.py %*
