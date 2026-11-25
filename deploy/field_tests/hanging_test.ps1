# Staged hanging test with per-servo voltage / current tables (2026-11-25); wraps hanging_test.py in .venv_fw.
#   powershell -ExecutionPolicy Bypass -File deploy\field_tests\hanging_test.ps1 all                                # PLAY 0.5x, v2ft
#   powershell -ExecutionPolicy Bypass -File deploy\field_tests\hanging_test.ps1 all --mode walk --model v2slow     # WALK 0.05 1.0, 6 s
#   powershell -ExecutionPolicy Bypass -File deploy\field_tests\hanging_test.ps1 check | flash | run | report
# Options: --mode play|walk --model v2ft|v2slow --rate 0.5 --loops 1 --vx 0.05 --freq <Hz> --seconds 6 --vstop 6.5
#          --port COM3 --session <dir> --yes   (details: hanging_test.py --help)
$root = Resolve-Path (Join-Path $PSScriptRoot '..\..')
& (Join-Path $root '.venv_fw\Scripts\python.exe') (Join-Path $root 'deploy\field_tests\hanging_test.py') @args
exit $LASTEXITCODE
