@echo off
REM Launch the OpenArmX RobStride configurator.
cd /d "%~dp0"
if not exist ".venv\Scripts\python.exe" (
    echo Virtual environment missing. Create it with:
    echo    py -3.11 -m venv .venv
    echo    .venv\Scripts\python.exe -m pip install -r requirements.txt
    exit /b 1
)
".venv\Scripts\python.exe" app.py %*
