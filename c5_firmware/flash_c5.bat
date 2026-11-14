@echo off
setlocal enabledelayedexpansion
rem ============================================================
rem  Flash the ESP32-C5 co-processor firmware (one time only).
rem    usage:  flash_c5.bat [COMx]        default: COM16
rem  Set the board switch to the C5 position before flashing.
rem ============================================================
set "PORT=%~1"
if "%PORT%"=="" set "PORT=COM16"

set "HERE=%~dp0"
set "PRE=%HERE%prebuilt"

rem ---- locate esptool.exe: IDF env vars first, then common install roots ----
set "ESPTOOL="
if defined IDF_PYTHON_ENV_PATH if exist "%IDF_PYTHON_ENV_PATH%\Scripts\esptool.exe" set "ESPTOOL=%IDF_PYTHON_ENV_PATH%\Scripts\esptool.exe"

if not defined ESPTOOL (
  for %%R in ("%IDF_TOOLS_PATH%" "%USERPROFILE%\.espressif" "%LOCALAPPDATA%\Espressif" "D:\ESP32-IDF\Espressif" "C:\Espressif") do (
    if not defined ESPTOOL if exist "%%~R\python_env" (
      for /d %%d in ("%%~R\python_env\*") do (
        if not defined ESPTOOL if exist "%%d\Scripts\esptool.exe" set "ESPTOOL=%%d\Scripts\esptool.exe"
      )
    )
  )
)
if not defined ESPTOOL set "ESPTOOL=esptool"

rem ---- verify prebuilt images ----
if not exist "%PRE%\bootloader.bin"         goto :missing
if not exist "%PRE%\partition-table.bin"    goto :missing
if not exist "%PRE%\ota_data_initial.bin"   goto :missing
if not exist "%PRE%\slave.bin"              goto :missing

echo [i] esptool : %ESPTOOL%
echo [i] port    : %PORT%
echo [i] flashing C5  bootloader@0x2000  table@0x8000  ota@0xd000  slave@0x10000

"%ESPTOOL%" --chip esp32c5 -p %PORT% -b 460800 ^
  --before default_reset --after hard_reset ^
  write_flash --flash_mode dio --flash_freq 80m --flash_size 4MB ^
  0x2000  "%PRE%\bootloader.bin" ^
  0x8000  "%PRE%\partition-table.bin" ^
  0xd000  "%PRE%\ota_data_initial.bin" ^
  0x10000 "%PRE%\slave.bin"

if errorlevel 1 (
  echo.
  echo [x] Flash failed. Check: board switch on C5, COM port and USB cable.
  echo     If needed: hold BOOT, tap RST, then run again.
  exit /b 1
)
echo.
echo [OK] C5 firmware flashed successfully.
exit /b 0

:missing
echo [x] Missing prebuilt images under: %PRE%
exit /b 1