# Flash a UF2 to the MANGO X4 (RP2350) over USB.
#   powershell -ExecutionPolicy Bypass -File deploy\firmware\flash_firmware.ps1 [-Uf2 <file.uf2>] [-Port COMn]
# Default UF2: newest outputs\robot_v2\deploy\firmware_build\<ts>\biped_runtime.uf2.
# Method copied from the teammate's mango_feetech/flash.ps1 + board-usb.ps1 (references/mango_x4_handoff_20261117,
# handoff 2026-11-17): 1200-baud touch on the native USB serial -> BOOTSEL drive (INFO_UF2.TXT mentions RP2350) -> copy.
# Flashing does not write servo EEPROM. Servo torque state is left as it was (the new firmware never enables torque
# on boot), so support the robot first.
param(
    [string]$Uf2,
    [ValidatePattern('^COM[0-9]+$')][string]$Port,
    [int]$TimeoutSeconds = 20
)
$ErrorActionPreference = 'Stop'
$root = Resolve-Path (Join-Path $PSScriptRoot '..\..')

function Get-BoardPorts {
    foreach ($d in @(Get-PnpDevice -Class Ports -PresentOnly -ErrorAction SilentlyContinue)) {
        if ($d.InstanceId -match '^USB\\VID_2E8A&' -and $d.FriendlyName -match '\((COM[0-9]+)\)') { $Matches[1] }
    }
}
function Get-BootDrives {
    foreach ($drive in [System.IO.DriveInfo]::GetDrives()) {
        if (-not $drive.IsReady -or $drive.DriveType -ne [System.IO.DriveType]::Removable) { continue }
        $info = Join-Path $drive.RootDirectory.FullName 'INFO_UF2.TXT'
        if ((Test-Path -LiteralPath $info) -and ((Get-Content -LiteralPath $info -Raw) -match '\bRP2350\b')) {
            $drive.RootDirectory.FullName
        }
    }
}

if (-not $Uf2) {
    $Uf2 = Get-ChildItem (Join-Path $root 'outputs\robot_v2\deploy\firmware_build\*\biped_runtime.uf2') |
        Sort-Object { $_.Directory.Name } | Select-Object -Last 1 -ExpandProperty FullName
}
if (-not $Uf2 -or -not (Test-Path -LiteralPath $Uf2)) { throw 'UF2 not found; build first (build_firmware.sh).' }
$Uf2 = (Resolve-Path -LiteralPath $Uf2).Path
Write-Host "UF2: $Uf2"
Write-Host ("SHA256: " + (Get-FileHash -LiteralPath $Uf2 -Algorithm SHA256).Hash.ToLower())

$drives = @(Get-BootDrives)
if ($drives.Count -gt 1) { throw 'Multiple RP2350 boot drives found; connect only one board.' }
if ($drives.Count -eq 0) {
    $ports = @(Get-BoardPorts)
    if ($Port) { $ports = @($ports | Where-Object { $_ -eq $Port }) }
    if ($ports.Count -ne 1) { throw "Expected one RP2350 USB serial port, found: [$($ports -join ', ')]. Use -Port." }
    Write-Host "1200-baud reset on $($ports[0]) -> BOOTSEL"
    $sp = [System.IO.Ports.SerialPort]::new($ports[0], 1200, [System.IO.Ports.Parity]::None, 8, [System.IO.Ports.StopBits]::One)
    $sp.DtrEnable = $true
    try { $sp.Open(); Start-Sleep -Milliseconds 300 } catch { Write-Host "serial reset returned: $($_.Exception.Message)" }
    finally { if ($sp.IsOpen) { try { $sp.Close() } catch {} }; $sp.Dispose() }
    $sw = [System.Diagnostics.Stopwatch]::StartNew()
    do { Start-Sleep -Milliseconds 250; $drives = @(Get-BootDrives) } while ($drives.Count -eq 0 -and $sw.Elapsed.TotalSeconds -lt $TimeoutSeconds)
}
if ($drives.Count -ne 1) { throw 'No RP2350 BOOTSEL drive appeared (press BOOT + RESET once). Nothing was copied.' }
Write-Host "Copying to $($drives[0])"
Copy-Item -LiteralPath $Uf2 -Destination (Join-Path $drives[0] 'firmware.uf2')
$sw = [System.Diagnostics.Stopwatch]::StartNew()
do {
    Start-Sleep -Milliseconds 500
    $ports = @(Get-BoardPorts); $booting = @(Get-BootDrives).Count -ne 0
} while (($ports.Count -ne 1 -or $booting) -and $sw.Elapsed.TotalSeconds -lt $TimeoutSeconds)
if ($ports.Count -ne 1 -or $booting) { throw 'UF2 copied but the USB serial port did not return; flash not confirmed.' }
Write-Host "Flashed; USB serial back on $($ports[0])"
