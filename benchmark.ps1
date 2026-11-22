# Opens the training Benchmark page (benchmark/page.html) for this repo: one button runs ~10 s of the real Level 0
# training loop and records step time, machine and machine load in benchmark/history.json (commit it to compare laptops).
$ErrorActionPreference = 'Stop'
$py = Join-Path $PSScriptRoot '.venv/Scripts/python.exe'
if (-not (Test-Path $py)) {
    # README "Quick start" step 1: .venv with Python 3.11, mujoco, torch +cu128, the robotcontrol package.
    Write-Host 'Creating the training environment (.venv, Python 3.11, torch cu128, robotcontrol)...'
    Push-Location $PSScriptRoot
    uv venv .venv --python 3.11
    uv pip install --python $py -e RobotControl
    uv pip install --python $py 'torch==2.11.*' --index-url https://download.pytorch.org/whl/cu128
    Pop-Location
}
& $py (Join-Path $PSScriptRoot 'benchmark/serve.py') @args
