# build_driver_msi.ps1 - regenerate OpenMotionDriver-x64.{msi,zip}.
#
# Steps:
#   1. Mint a fresh self-signed "CN=Openwater WinUSB" code-signing cert if the
#      PFX is absent (20-yr validity; the old cert expires 2026-08-11).
#   2. Load the signing key from the PFX and derive the public .cer from it (so
#      the shipped cert always matches the key). We do NOT trust the cert on the
#      build machine -- signing needs only the key; trust is the MSI's job.
#   3. (re)sign any catalog not already signed by the current key -- all
#      timestamped. The .cer/.msi/.cab/.zip are build outputs (gitignored); the
#      .cat files and INFs are committed, libwdi/Zadig-generated content. The key
#      is the single source of truth for what ships.
#   4. wix build the MSI (Util ext supplies the QuietExec custom actions).
#   5. Zip the MSI + external cab.
#   6. Refresh the bloodflow-app vendored zip.
#
# The PFX password is read from $env:OW_DRIVER_PFX_PASSWORD (or prompted) and is
# never written to the repo or logs.
param(
    [string]$AppResourcesZip = "C:\Users\ethan\Projects\openmotion-bloodflow-app\resources\OpenMotionDriver-x64.zip",
    [string]$TimeStampServer = "http://timestamp.digicert.com",
    [switch]$Fresh
)
$ErrorActionPreference = "Stop"
Set-Location (Split-Path -Parent $MyInvocation.MyCommand.Definition)

$pfx = "OpenMotion_signing_cert.pfx"
$cer = "OpenMotion_signing_cert.cer"

# -- 0. -Fresh: drop any existing cert so a brand-new one is minted below --
if ($Fresh) {
    Remove-Item $pfx,$cer -ErrorAction SilentlyContinue
    Write-Host "Fresh mode: removed any existing signing cert." -ForegroundColor Cyan
}

# -- password --
$pw = $env:OW_DRIVER_PFX_PASSWORD
if (-not $pw) {
    $sec = Read-Host "PFX password" -AsSecureString
    $bstr = [Runtime.InteropServices.Marshal]::SecureStringToBSTR($sec)
    try   { $pw = [Runtime.InteropServices.Marshal]::PtrToStringAuto($bstr) }
    finally { [Runtime.InteropServices.Marshal]::ZeroFreeBSTR($bstr) }
}

# -- 1. mint cert if absent --
if (-not (Test-Path $pfx)) {
    Write-Host "Minting fresh Openwater WinUSB signing cert (20 yr)..." -ForegroundColor Cyan
    $cert = New-SelfSignedCertificate -Type CodeSigningCert -Subject "CN=Openwater WinUSB" `
        -CertStoreLocation Cert:\CurrentUser\My -KeyExportPolicy Exportable `
        -KeyUsage DigitalSignature -KeyAlgorithm RSA -KeyLength 2048 `
        -NotAfter (Get-Date).AddYears(20) `
        -TextExtension @("2.5.29.37={text}1.3.6.1.5.5.7.3.3")
    $sp = ConvertTo-SecureString $pw -AsPlainText -Force
    Export-PfxCertificate -Cert $cert -FilePath $pfx -Password $sp | Out-Null
    Remove-Item ("Cert:\CurrentUser\My\" + $cert.Thumbprint) -Force
    Write-Host "  wrote $pfx" -ForegroundColor Green
}

# -- 2. load signing cert (with private key) + trust it on this machine --
try {
    $signCert = New-Object System.Security.Cryptography.X509Certificates.X509Certificate2(
        (Resolve-Path $pfx).Path, $pw,
        [System.Security.Cryptography.X509Certificates.X509KeyStorageFlags]::Exportable)
} catch {
    throw "Could not open $pfx with the supplied password. To mint a NEW signing cert, " +
          "re-run with -Fresh (deletes $pfx/$cer) and provide a new password."
}
# Always (re)derive the public .cer from the signing key, so the cert that ships
# can never disagree with the key that signs the catalogs. The .cer is a build
# output (gitignored), not committed source.
Export-Certificate -Cert $signCert -FilePath $cer | Out-Null
# NOTE: we deliberately do NOT import the cert into this machine's trust stores.
# The build only needs the private key to SIGN; trust is the end user's MSI's job
# (certutil -addstore at install). Skipping the import keeps the build/dev box's
# trust store clean and removes an interactive-prompt hang risk in CI.

# -- 3. (re)sign any catalog not already signed by this key --
# All four catalogs are committed, libwdi/Zadig-generated content -- the proven
# way driver catalogs are made for this project. (New-FileCatalog does NOT make
# valid driver catalogs: pnputil rejects them with "hash ... not present in the
# catalog".) The build only re-signs a catalog when the key changes; regenerating
# one (after an INF change) is a manual libwdi/Zadig step. Matching is by signer
# THUMBPRINT (not chain-validation Status), since the build box doesn't trust the
# cert -- so a steady-state build (key unchanged) signs nothing.
$cats = @(
    "DFU_in_FS_Mode.cat",
    "comms_histo_imu(hs)_(interface_0).cat",
    "comms_histo_imu(hs)_(interface_1).cat",
    "comms_histo_imu(hs)_(interface_2).cat"
)
foreach ($c in $cats) {
    $sig = Get-AuthenticodeSignature $c
    if ($sig.SignerCertificate -and $sig.SignerCertificate.Thumbprint -eq $signCert.Thumbprint) {
        Write-Host "  $c already signed by current key, skipping" -ForegroundColor DarkGray
        continue
    }
    $r = Set-AuthenticodeSignature -FilePath $c -Certificate $signCert `
            -HashAlgorithm SHA256 -TimeStampServer $TimeStampServer
    if (-not $r.SignerCertificate -or $r.SignerCertificate.Thumbprint -ne $signCert.Thumbprint) {
        throw "signing $c failed: $($r.Status) - $($r.StatusMessage)"
    }
    Write-Host "  signed $c -> $($r.SignerCertificate.Thumbprint)" -ForegroundColor Green
}

# -- 4. wix build (ensure Util ext present, pinned to the wix CLI version;
#       an unpinned 'add' grabs a newer major that is incompatible with v5) --
$wixVer = ((wix --version) -split '\+')[0].Trim()
if (-not ((wix extension list -g) -match 'WixToolset\.Util\.wixext')) {
    wix extension add -g "WixToolset.Util.wixext/$wixVer" | Out-Null
    if ($LASTEXITCODE -ne 0) { throw "wix extension add (WixToolset.Util.wixext/$wixVer) failed" }
}
$msi = "OpenMotionDriver-x64.msi"
Remove-Item $msi,"cab1.cab" -ErrorAction SilentlyContinue
wix build Product.wxs -arch x64 -ext WixToolset.Util.wixext -o $msi
if ($LASTEXITCODE -ne 0) { throw "wix build failed" }
if (-not (Test-Path $msi) -or (Get-Item $msi).Length -lt 100KB) { throw "MSI build looks empty/invalid" }

# -- 5. zip MSI + external cab --
$zip = "OpenMotionDriver-x64.zip"
Remove-Item $zip -ErrorAction SilentlyContinue
$payload = @($msi)
if (Test-Path "cab1.cab") { $payload += "cab1.cab" }
Compress-Archive -Path $payload -DestinationPath $zip -Force

# -- 6. refresh the bloodflow-app vendored zip --
if ($AppResourcesZip) {
    Copy-Item $zip $AppResourcesZip -Force
    Write-Host "  refreshed $AppResourcesZip" -ForegroundColor Green
}
Write-Host "Done." -ForegroundColor Green
