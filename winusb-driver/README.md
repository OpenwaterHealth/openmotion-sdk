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
# first time / replacing the old cert: add -Fresh to mint a new signing cert
.\build_driver_msi.ps1 -Fresh -AppResourcesZip <path to bloodflow-app resources\OpenMotionDriver-x64.zip>
```

The script mints `OpenMotion_signing_cert.pfx/.cer` when no PFX is present
(self-signed, 20-yr). The repo currently ships an older cert (expires
2026-08-11) whose password is not on hand — to mint a **replacement**, run with
`-Fresh`, which deletes the existing `.pfx`/`.cer` first so a new one is minted
with the password you supply. It then generates + signs all catalogs
(timestamped), runs `wix build`, zips, and copies the zip into the bloodflow-app
`resources/`. The MSI installs the public cert into TrustedPublisher + Root,
then `pnputil /add-driver /install`s each INF. (`driver_install.cmd` performs
the same certutil + pnputil steps for a manual, no-MSI install.)

Signing uses in-box `New-FileCatalog` / `Set-AuthenticodeSignature` — no WDK
required. The PFX password is read from `$env:OW_DRIVER_PFX_PASSWORD` (or
prompted) and is never committed.
