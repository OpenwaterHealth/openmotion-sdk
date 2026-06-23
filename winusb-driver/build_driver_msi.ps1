# build_driver_msi.ps1 - regenerate OpenMotionDriver-x64.{msi,zip}.
#
# Steps:
#   1. Mint a fresh self-signed "CN=Openwater WinUSB" code-signing cert if the
#      PFX is absent (20-yr validity; the old cert expires 2026-08-11).
#   2. Trust our cert on THIS machine (CurrentUser Root + TrustedPublisher) so
#      catalog signatures verify as Valid and the dev box can use the driver.
#   3. Generate the DFU catalog (in-box New-FileCatalog; no WDK needed) and
#      re-sign it + the three sensor catalogs with our cert, all timestamped.
#   4. wix build the MSI (Util ext supplies the QuietExec custom actions).
#   5. Zip the MSI + external cab.
#   6. Refresh the bloodflow-app vendored zip.
#
# The PFX password is read from $env:OW_DRIVER_PFX_PASSWORD (or prompted) and is
# never written to the repo or logs.
param(
    [string]$AppResourcesZip = "C:\Users\ethan\Projects\openmotion-bloodflow-app\resources\OpenMotionDriver-x64.zip",
    [string]$TimeStampServer = "http://timestamp.digicert.com"
)
$ErrorActionPreference = "Stop"
Set-Location (Split-Path -Parent $MyInvocation.MyCommand.Definition)

$pfx = "OpenMotion_signing_cert.pfx"
$cer = "OpenMotion_signing_cert.cer"

# -- password --
$pw = $env:OW_DRIVER_PFX_PASSWORD
if (-not $pw) {
    $sec = Read-Host "PFX password" -AsSecureString
    $bstr = [Runtime.InteropServices.Marshal]::SecureStringToBSTR($sec)
    $pw  = [Runtime.InteropServices.Marshal]::PtrToStringAuto($bstr)
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
    Export-Certificate    -Cert $cert -FilePath $cer | Out-Null
    Remove-Item ("Cert:\CurrentUser\My\" + $cert.Thumbprint) -Force
    Write-Host "  wrote $pfx and $cer" -ForegroundColor Green
}

# -- 2. load signing cert (with private key) + trust it on this machine --
$signCert = New-Object System.Security.Cryptography.X509Certificates.X509Certificate2(
    (Resolve-Path $pfx).Path, $pw,
    [System.Security.Cryptography.X509Certificates.X509KeyStorageFlags]::Exportable)
Import-Certificate -FilePath $cer -CertStoreLocation Cert:\CurrentUser\Root            | Out-Null
Import-Certificate -FilePath $cer -CertStoreLocation Cert:\CurrentUser\TrustedPublisher | Out-Null

# -- 3. generate DFU catalog, then sign all catalogs (timestamped) --
$dfuCat = "openmotion-dfu.cat"
Remove-Item $dfuCat -ErrorAction SilentlyContinue
New-FileCatalog -Path "openmotion-dfu.inf" -CatalogFilePath $dfuCat -CatalogVersion 2 | Out-Null

$cats = @(
    $dfuCat,
    "comms_histo_imu(hs)_(interface_0).cat",
    "comms_histo_imu(hs)_(interface_1).cat",
    "comms_histo_imu(hs)_(interface_2).cat"
)
foreach ($c in $cats) {
    $r = Set-AuthenticodeSignature -FilePath $c -Certificate $signCert `
            -HashAlgorithm SHA256 -TimeStampServer $TimeStampServer
    if ($r.Status -ne "Valid") { throw "signing $c failed: $($r.Status) - $($r.StatusMessage)" }
    Write-Host "  signed $c -> $($r.Status)" -ForegroundColor Green
}

# -- 4. wix build --
wix extension add -g WixToolset.Util.wixext | Out-Null
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
