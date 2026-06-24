@echo off
setlocal enableextensions
set DIR=%~dp0

rem Install the public certificate into Trusted Publishers and Root
"%SystemRoot%\System32\certutil.exe" -addstore -f TrustedPublisher "%DIR%OpenMotion_signing_cert.cer" || exit /b 1
"%SystemRoot%\System32\certutil.exe" -addstore -f Root             "%DIR%OpenMotion_signing_cert.cer" || exit /b 1

rem Install all four driver packages
"%SystemRoot%\System32\pnputil.exe" /add-driver "%DIR%COMMS_HISTO_IMU(HS)_(Interface_0).inf" /install || exit /b 1
"%SystemRoot%\System32\pnputil.exe" /add-driver "%DIR%COMMS_HISTO_IMU(HS)_(Interface_1).inf" /install || exit /b 1
"%SystemRoot%\System32\pnputil.exe" /add-driver "%DIR%COMMS_HISTO_IMU(HS)_(Interface_2).inf" /install || exit /b 1
"%SystemRoot%\System32\pnputil.exe" /add-driver "%DIR%DFU_in_FS_Mode.inf" /install || exit /b 1

exit /b 0
