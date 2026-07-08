@echo off
setlocal

set "APP_DIR=%~dp0"
cd /d "%APP_DIR%"

where py.exe >nul 2>nul
if %errorlevel% neq 0 (
    echo Python launcher py.exe was not found.
    echo Install Python 3.12 for Windows and enable "Add python.exe to PATH".
    pause
    exit /b 1
)

py.exe -m pip install --upgrade pip
py.exe -m pip install -r "%APP_DIR%requirements-windows.txt"

echo.
echo Windows bridge dependencies installed.
echo You can now run "Launch Trimble Bridge.bat".
pause
