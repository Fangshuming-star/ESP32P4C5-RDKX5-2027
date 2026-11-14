@echo off
setlocal
cd /d "%~dp0"

rem Prefer a pythonw (no console window); fall back to python if needed.
set "PYW="
if exist "%LOCALAPPDATA%\Microsoft\WindowsApps\pythonw3.13.exe" set "PYW=%LOCALAPPDATA%\Microsoft\WindowsApps\pythonw3.13.exe"
if not defined PYW if exist "%LOCALAPPDATA%\Microsoft\WindowsApps\pythonw.exe" set "PYW=%LOCALAPPDATA%\Microsoft\WindowsApps\pythonw.exe"
if not defined PYW (
    for /f "delims=" %%i in ('where pythonw 2^>nul') do (
        set "PYW=%%i"
        goto :run
    )
)
:run
if defined PYW (
    start "" "%PYW%" "%~dp0p4_tool.py"
) else (
    python "%~dp0p4_tool.py"
    if errorlevel 1 pause
)
endlocal