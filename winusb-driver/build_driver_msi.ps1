# build_driver_msi.ps1 - regenerate OpenMotionDriver-x64.{msi,zip}.
#
# Two signing modes (sdk#216):
#
#   EV (CI, the shipping path): -Thumbprint (or $env:CODESIGN_THUMBPRINT)
#   selects a code-signing cert already in Cert:\CurrentUser\My — in CI that
#   is the SSL.com EV cert loaded by eSigner CKA (the private key stays in
#   SSL.com's cloud HSM; CKA exposes it through Windows CNG so signtool
#   works). Catalogs AND the MSI are signtool-signed and RFC3161-timestamped;
#   the shipped .cer is derived from the store cert; the MSI is built with
#   -d EvSigned=1 so it installs the cert into TrustedPublisher only (the EV
#   chain is publicly trusted — no Root install) and retires the old
#   self-signed cert from user machines.
#
#   Legacy self-signed (local-dev fallback): mints/loads the
#   "CN=Openwater WinUSB" PFX and signs with it; the MSI certutil-installs
#   that cert into Root + TrustedPublisher. Kept only so a dev box without
#   eSigner credentials can produce an installable MSI for bench testing —
#   never ship this variant.
#
# Steps:
#   1. Resolve the signing identity (store cert in EV mode; PFX in legacy
#      mode, minted fresh if absent — 20-yr validity).
#   2. Derive the public .cer from the signing identity (so the shipped cert
#      always matches the key). We do NOT trust the cert on the build
#      machine -- signing needs only the key; trust is the MSI's job.
#   3. (re)sign any catalog not already signed by the current key -- all
#      timestamped. The .cer/.msi/.cab/.zip are build outputs (gitignored);
#      the .cat files and INFs are committed, libwdi/Zadig-generated content.
#      The key is the single source of truth for what ships.
#   4. wix build the MSI (Util ext supplies the QuietExec custom actions).
#      EV mode also signs the MSI itself.
#   5. Zip the MSI + external cab.
#   6. Refresh the bloodflow-app vendored zip.
#
# The legacy PFX password is read from $env:OW_DRIVER_PFX_PASSWORD (or
# prompted) and is never written to the repo or logs.
param(
    [string]$AppResourcesZip = "C:\Users\ethan\Projects\openmotion-bloodflow-app\resources\OpenMotionDriver-x64.zip",
    [string]$TimeStampServer = "http://timestamp.digicert.com",  # legacy Set-AuthenticodeSignature timestamp
    [string]$Rfc3161Server   = "http://ts.ssl.com",              # EV signtool /tr timestamp
    [string]$Thumbprint      = $env:CODESIGN_THUMBPRINT,
    [switch]$Fresh
)
$ErrorActionPreference = "Stop"
Set-Location (Split-Path -Parent $MyInvocation.MyCommand.Definition)

$pfx = "OpenMotion_signing_cert.pfx"
$cer = "OpenMotion_signing_cert.cer"
$evMode = [bool]$Thumbprint

if ($evMode) {
    # -- EV: the cert must already be in the user store (eSigner CKA in CI) --
    $signCert = Get-Item "Cert:\CurrentUser\My\$Thumbprint" -ErrorAction SilentlyContinue
    if (-not $signCert) {
        throw "no certificate with thumbprint $Thumbprint in Cert:\CurrentUser\My (is eSigner CKA loaded?)"
    }
    Write-Host "EV mode: signing as $($signCert.Subject) [$Thumbprint]" -ForegroundColor Cyan

    # signtool: PATH first, else the newest Windows Kits copy
    $signtool = (Get-Command signtool.exe -ErrorAction SilentlyContinue).Source
    if (-not $signtool) {
        $signtool = Get-ChildItem "${env:ProgramFiles(x86)}\Windows Kits\10\bin\10.0.*\x64\signtool.exe" -ErrorAction SilentlyContinue |
            Sort-Object { [version]$_.Directory.Parent.Name } -Descending |
            Select-Object -First 1 -ExpandProperty FullName
    }
    if (-not $signtool) { throw "signtool.exe not found (PATH or Windows Kits)" }

    function Invoke-SignTool([string]$File) {
        # cloud signing and timestamp servers can both flake; retry before failing
        for ($i = 1; $i -le 3; $i++) {
            & $signtool sign /sha1 $Thumbprint /fd SHA256 /tr $Rfc3161Server /td SHA256 $File
            if ($LASTEXITCODE -eq 0) { return }
            if ($i -eq 3) { throw "signtool failed for $File after 3 attempts" }
            Write-Host "signtool exit $LASTEXITCODE -- retrying ($i/3)" -ForegroundColor Yellow
            Start-Sleep -Seconds 10
        }
    }
} else {
    # -- legacy self-signed fallback --
    # 0. -Fresh: drop any existing cert so a brand-new one is minted below
    if ($Fresh) {
        Remove-Item $pfx,$cer -ErrorAction SilentlyContinue
        Write-Host "Fresh mode: removed any existing signing cert." -ForegroundColor Cyan
    }

    # password
    $pw = $env:OW_DRIVER_PFX_PASSWORD
    if (-not $pw) {
        $sec = Read-Host "PFX password" -AsSecureString
        $bstr = [Runtime.InteropServices.Marshal]::SecureStringToBSTR($sec)
        try   { $pw = [Runtime.InteropServices.Marshal]::PtrToStringAuto($bstr) }
        finally { [Runtime.InteropServices.Marshal]::ZeroFreeBSTR($bstr) }
    }

    # 1. mint cert if absent
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

    # 2. load signing cert (with private key)
    try {
        $signCert = New-Object System.Security.Cryptography.X509Certificates.X509Certificate2(
            (Resolve-Path $pfx).Path, $pw,
            [System.Security.Cryptography.X509Certificates.X509KeyStorageFlags]::Exportable)
    } catch {
        throw "Could not open $pfx with the supplied password. To mint a NEW signing cert, " +
              "re-run with -Fresh (deletes $pfx/$cer) and provide a new password."
    }
}

# Always (re)derive the public .cer from the signing identity, so the cert
# that ships can never disagree with the key that signs the catalogs. The
# .cer is a build output (gitignored), not committed source.
Export-Certificate -Cert $signCert -FilePath $cer | Out-Null
# NOTE: we deliberately do NOT import the cert into this machine's trust stores.
# The build only needs the private key to SIGN; trust is the end user's MSI's job
# (certutil -addstore at install). Skipping the import keeps the build/dev box's
# trust store clean and removes an interactive-prompt hang risk in CI.

# -- 3. (re)sign any catalog not already signed by this key --
# All four catalogs are committed, libwdi/Zadig-generated content -- the proven
# way driver catalogs are made for this project. (New-FileCatalog does NOT make
# valid driver catalogs: pnputil rejects them with "hash ... not present in the
# catalog".) Regenerating a catalog (after an INF change) is a manual
# libwdi/Zadig step. Matching is by signer THUMBPRINT (not chain-validation
# Status), since the build box doesn't trust the cert. The committed cats carry
# the legacy self-signed signature, so an EV build re-signs all four; a
# steady-state legacy build signs nothing.
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
    if ($evMode) {
        Invoke-SignTool $c
        $r = Get-AuthenticodeSignature $c
    } else {
        $r = Set-AuthenticodeSignature -FilePath $c -Certificate $signCert `
                -HashAlgorithm SHA256 -TimeStampServer $TimeStampServer
    }
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
$wixDefines = @()
if ($evMode) { $wixDefines = @("-d", "EvSigned=1") }
wix build Product.wxs -arch x64 -ext WixToolset.Util.wixext @wixDefines -o $msi
if ($LASTEXITCODE -ne 0) { throw "wix build failed" }
if (-not (Test-Path $msi) -or (Get-Item $msi).Length -lt 100KB) { throw "MSI build looks empty/invalid" }

# EV mode also signs the MSI itself (pointless in legacy mode -- nothing
# trusts the self-signed cert until the MSI itself installs it).
if ($evMode) { Invoke-SignTool $msi }

# -- 5. zip MSI + external cab --
$zip = "OpenMotionDriver-x64.zip"
Remove-Item $zip -ErrorAction SilentlyContinue
$payload = @($msi)
if (Test-Path "cab1.cab") { $payload += "cab1.cab" }
Compress-Archive -Path $payload -DestinationPath $zip -Force
if (-not $evMode) {
    Write-Host "NOTE: $zip is COMMITTED content holding the EV-signed driver." -ForegroundColor Yellow
    Write-Host "      This legacy-signed rebuild overwrote it in your working tree -- revert" -ForegroundColor Yellow
    Write-Host "      before committing (git checkout -- winusb-driver/$zip)." -ForegroundColor Yellow
}

# -- 6. refresh the bloodflow-app vendored zip --
if ($AppResourcesZip) {
    Copy-Item $zip $AppResourcesZip -Force
    Write-Host "  refreshed $AppResourcesZip" -ForegroundColor Green
}
Write-Host "Done." -ForegroundColor Green
