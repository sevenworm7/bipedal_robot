# Hanging demos of robot v2 on the MANGO X4 (v2 runtime firmware), the sequences tested on 2026-11-23.
#   play : INIT -> slow open-loop playback of the simulated student gait (PLAY) -> back to stance -> STOP
#   walk : INIT -> closed-loop policy WALK (real time) for N seconds -> STAND -> STOP
#
#   powershell -ExecutionPolicy Bypass -File deploy\host\demo_hanging.ps1 -Mode play [-Rate 0.5]
#   powershell -ExecutionPolicy Bypass -File deploy\host\demo_hanging.ps1 -Mode walk [-Vx 0.05] [-Freq 1.6] [-Seconds 6] [-VStop 5.5]
#   add -DryRun to only print the command sequence, -Yes to skip the confirmation question
#
# Safety: the robot must hang free (feet and tail touching nothing) and someone must hold it. STOP at the end switches
# torque off: legs and tail drop. -VStop below 6.5 V is for hanging tests only (2026-11-23: with the HW-133A 3 A buck
# the supply swings 6.1-7.5 V while walking; at 6.5 V the guard stops WALK after 0.34 s).
# Logs: outputs/robot_v2/deploy/sessions/<time>/ (console.log, telemetry.csv for walk).
param(
    [ValidateSet('play', 'walk')][string]$Mode = 'play',
    [ValidateRange(0.05, 1.0)][double]$Rate = 0.5,
    [ValidateRange(0.0, 0.15)][double]$Vx = 0.05,
    [ValidateRange(1.2, 2.0)][double]$Freq = 1.6,
    [ValidateRange(1, 30)][double]$Seconds = 6,
    [ValidateRange(5.5, 7.0)][double]$VStop = 6.5,
    [ValidatePattern('^COM[0-9]+$')][string]$Port = 'COM3',
    [switch]$Yes,
    [switch]$DryRun
)
$ErrorActionPreference = 'Stop'
$root = Resolve-Path (Join-Path $PSScriptRoot '..\..')
$python = Join-Path $root '.venv_fw\Scripts\python.exe'
$cli = Join-Path $root 'deploy\host\robot_cli.py'
$inv = [System.Globalization.CultureInfo]::InvariantCulture
$f = { param($v) $v.ToString('0.###', $inv) }

$run = @('ALIGN')
if ($Mode -eq 'play') {
    # PLAY: 1.5 s ramp in + 202 frames (4.04 s at 50 Hz) / rate + 1.5 s ramp out
    $playS = 3.0 + 202 / 50.0 / $Rate
    $run += @('INIT', 'WAIT 12', "PLAY $(& $f $Rate)")
    $exitAfter = [math]::Ceiling($playS) + 2
    $desc = "slow playback of the simulated gait at $(& $f $Rate)x (about $([math]::Round($playS)) s), then STOP"
} else {
    if ($VStop -ne 6.5) { $run += "VSTOP $(& $f $VStop)" }
    $run += @('TELEM 1', 'INIT', 'WAIT 10', "WALK $(& $f $Vx) $(& $f $Freq)", "WAIT $(& $f $Seconds)", 'STAND')
    $exitAfter = 3
    $desc = "closed-loop policy WALK vx=$(& $f $Vx) m/s freq=$(& $f $Freq) Hz for $(& $f $Seconds) s (voltage stop $(& $f $VStop) V), STAND, then STOP"
}

Write-Host "[demo] mode=$Mode port=$Port"
Write-Host "[demo] $desc"
Write-Host "[demo] commands: $($run -join ' | ')  (exit-after $exitAfter s -> STOP)"
if ($DryRun) { return }
if ($VStop -lt 6.5) { Write-Host "[demo] WARNING: voltage stop lowered to $(& $f $VStop) V - hanging tests only" }
if (-not $Yes) {
    $ans = Read-Host '[demo] Robot hanging free and held by someone? Type yes to start'
    if ($ans -ne 'yes') { Write-Host '[demo] cancelled'; return }
}
& $python $cli --port $Port --run @run --exit-after $exitAfter | Where-Object { $_ -notmatch '^(T|PL) ' }
