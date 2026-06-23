# OpenMotion WinUSB driver MSI

Builds `OpenMotionDriver-x64.msi` (+ `cab1.cab`, zipped) which binds WinUSB to:

- the sensor run-mode device — `VID_0483&PID_5A5A` interfaces `MI_00/01/02`
  (`COMMS_HISTO_IMU(HS)_(Interface_*).inf`), and
- the STM32 DFU bootloader — `VID_0483&PID_DF11` (`openmotion-dfu.inf`),
  used by both the console and the sensor in firmware-update mode.

The console in run-mode is a CDC virtual COM port (`PID_A53E`) served by the
in-box Windows VCP driver, so it needs no INF here.

## Rebuild

```powershell
$env:OW_DRIVER_PFX_PASSWORD = "<pfx password>"
.\build_driver_msi.ps1 -AppResourcesZip <path to bloodflow-app resources\OpenMotionDriver-x64.zip>
```

The script mints `OpenMotion_signing_cert.pfx/.cer` on first run (self-signed,
20-yr), generates + signs all catalogs (timestamped), runs `wix build`, zips,
and copies the zip into the bloodflow-app `resources/`. The MSI installs the
public cert into TrustedPublisher + Root, then `pnputil /add-driver /install`s
each INF.

Signing uses in-box `New-FileCatalog` / `Set-AuthenticodeSignature` — no WDK
required. The PFX password is read from `$env:OW_DRIVER_PFX_PASSWORD` (or
prompted) and is never committed.
