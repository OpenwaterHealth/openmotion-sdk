# OpenMotion WinUSB driver MSI

Builds `OpenMotionDriver-x64.msi` (+ `cab1.cab`, zipped) which binds WinUSB to:

- the sensor run-mode device — `VID_0483&PID_5A5A` interfaces `MI_00/01/02`
  (`COMMS_HISTO_IMU(HS)_(Interface_*).inf`), and
- the STM32 DFU bootloader — `VID_0483&PID_DF11` (`DFU_in_FS_Mode.inf`),
  used by both the console and the sensor in firmware-update mode.

The console in run-mode is a CDC virtual COM port (`PID_A53E`) served by the
in-box Windows VCP driver, so it needs no INF here.

## Rebuild

```powershell
$env:OW_DRIVER_PFX_PASSWORD = "<pfx password>"
# first time / replacing the old cert: add -Fresh to mint a new signing cert
.\build_driver_msi.ps1 -Fresh -AppResourcesZip <path to bloodflow-app resources\OpenMotionDriver-x64.zip>
```

The script mints `OpenMotion_signing_cert.pfx` when no PFX is present
(self-signed, 20-yr). Use `-Fresh` to mint a **replacement** (deletes any
existing key first). It then derives the public `.cer` from the key and
(re)signs any catalog not already signed by that key (so a steady-state build
signs nothing; a key rotation re-signs all four). It then runs `wix build`,
zips, and copies the zip into the bloodflow-app `resources/`. The MSI installs
the public cert into TrustedPublisher + Root, removes the old retired cert
(`certutil -delstore`, by thumbprint), then `pnputil /add-driver /install`s each
INF. (`driver_install.cmd` performs the certutil + pnputil steps for a manual,
no-MSI install.)

Catalog signing uses in-box `Set-AuthenticodeSignature` (no WDK). The catalog
*content* is NOT generated here — the `.cat` files are libwdi/Zadig output,
committed as content (see below); the build only re-signs them with the current
key. **Do not** generate driver catalogs with `New-FileCatalog` — it produces
file-integrity catalogs, not driver catalogs, and `pnputil` rejects them
("hash for the file is not present in the specified catalog file").

## Source vs. build outputs

The **signing key is the single source of truth** for what ships; the private
key and the derived/built outputs are never committed:

- **Committed content:** the INFs, all four `.cat` files (libwdi/Zadig-generated
  driver catalogs — 3 sensor + `DFU_in_FS_Mode.cat`), `Product.wxs`,
  `build_driver_msi.ps1`, `driver_install.cmd`, this README, and the CI workflow.
  The build re-signs the catalogs only when the signing key changes; to add or
  change a device, regenerate its INF + `.cat` with libwdi/Zadig and commit them.
- **Not committed (gitignored), produced at build time:** `OpenMotion_signing_cert.pfx`
  (the private key — CI injects it from the `OW_DRIVER_PFX_BASE64` secret),
  `OpenMotion_signing_cert.cer` (derived from the key), `OpenMotionDriver-x64.msi`,
  `cab1.cab`, and `OpenMotionDriver-x64.zip`.

The PFX password is read from `$env:OW_DRIVER_PFX_PASSWORD` (or prompted) and is
never written to the repo or logs. CI (`.github/workflows/driver-msi.yml`)
decodes the PFX secret, runs this script, and uploads the zip as an artifact.
