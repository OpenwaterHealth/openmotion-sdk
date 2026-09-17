# OpenMotion WinUSB driver MSI

Builds `OpenMotionDriver-x64.msi` (+ `cab1.cab`, zipped) which binds WinUSB to:

- the sensor run-mode device — `VID_0483&PID_5A5A` interfaces `MI_00/01/02`
  (`COMMS_HISTO_IMU(HS)_(Interface_*).inf`), and
- the STM32 DFU bootloader — `VID_0483&PID_DF11` (`DFU_in_FS_Mode.inf`),
  used by both the console and the sensor in firmware-update mode.

The console in run-mode is a CDC virtual COM port (`PID_A53E`) served by the
in-box Windows VCP driver, so it needs no INF here.

## Rebuild

Two signing modes (sdk#216) — the script picks EV automatically when a
thumbprint is provided:

**EV (CI, the shipping variant):** `CODESIGN_THUMBPRINT` (or `-Thumbprint`)
selects a code-signing cert already in `Cert:\CurrentUser\My` — in CI that is
the SSL.com EV cert loaded by eSigner CKA (key held in SSL.com's cloud HSM).
Catalogs **and the MSI** are signtool-signed; the MSI is built with
`-d EvSigned=1` so it installs the cert into **TrustedPublisher only** (the EV
chain is publicly trusted — no Root install) and also removes the retired
self-signed `CN=Openwater WinUSB` cert from user machines.

**Legacy self-signed (local-dev fallback — never ship):**

```powershell
$env:OW_DRIVER_PFX_PASSWORD = "<pfx password>"
# first time / replacing the old cert: add -Fresh to mint a new signing cert
.\build_driver_msi.ps1 -Fresh -AppResourcesZip <path to bloodflow-app resources\OpenMotionDriver-x64.zip>
```

Mints `OpenMotion_signing_cert.pfx` when no PFX is present (self-signed,
20-yr; `-Fresh` deletes any existing key first) and signs with in-box
`Set-AuthenticodeSignature` (no WDK). This variant's MSI installs the cert
into TrustedPublisher **+ Root**, since nothing else trusts a self-signed
chain.

Both modes: the public `.cer` is derived from the active signing identity,
only catalogs not already signed by that identity are (re)signed (so a
steady-state build signs nothing; an identity change re-signs all four),
then `wix build`, zip, and optional copy into bloodflow-app `resources/`.
The MSI removes the old leaked cert (`certutil -delstore`, by thumbprint)
and `pnputil /add-driver /install`s each INF. (`driver_install.cmd` performs
the certutil + pnputil steps for a manual, no-MSI install.)

The catalog *content* is NOT generated here — the `.cat` files are
libwdi/Zadig output, committed as content (see below); the build only
re-signs them. **Do not** generate driver catalogs with `New-FileCatalog` —
it produces file-integrity catalogs, not driver catalogs, and `pnputil`
rejects them ("hash for the file is not present in the specified catalog
file").

## Source vs. build outputs

The **signing key is the single source of truth** for what ships; the private
key and the derived/built outputs are never committed:

- **Committed content:** the INFs, all four `.cat` files (libwdi/Zadig-generated
  driver catalogs — 3 sensor + `DFU_in_FS_Mode.cat`), `Product.wxs`,
  `build_driver_msi.ps1`, `driver_install.cmd`, this README, the CI workflow,
  **and `OpenMotionDriver-x64.zip` — the canonical EV-signed artifact**
  (signed once per driver change via a manual `driver-msi.yml` dispatch,
  then committed here and vendored into bloodflow-app `resources/`).
  A local or PR build overwrites the zip in your working tree with a
  legacy-signed one — revert it; **never commit a legacy-signed zip**.
  The build re-signs the catalogs only when the signing key changes; to add or
  change a device, regenerate its INF + `.cat` with libwdi/Zadig and commit them.
- **Not committed (gitignored), produced at build time:** `OpenMotion_signing_cert.pfx`
  (the legacy private key), `OpenMotion_signing_cert.cer` (derived from the
  active signing identity), `OpenMotionDriver-x64.msi`, and `cab1.cab`.

The legacy PFX password is read from `$env:OW_DRIVER_PFX_PASSWORD` (or
prompted) and is never written to the repo or logs. CI
(`.github/workflows/driver-msi.yml`) signs EV **only on manual
`workflow_dispatch`** — eSigner cloud signings are metered, so the shipping
driver is signed once per driver change (dispatch → download the artifact →
commit it here as `OpenMotionDriver-x64.zip` → vendor the same zip into
bloodflow-app `resources/`). PR-triggered runs always
use the legacy `OW_DRIVER_PFX_BASE64` secret (zero eSigner cost, build
validation only). The EV path sets up eSigner CKA from the
`ES_USERNAME`/`ES_PASSWORD`/`ES_TOTP_SECRET` secrets and exports
`CODESIGN_THUMBPRINT`. Either way it then runs this script and uploads the
zip as an artifact.
