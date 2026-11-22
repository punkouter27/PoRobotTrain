<#
.SYNOPSIS
    Runs the browser tier (tests/PoRobotTrain.E2EUI) against the Release publish of the Blazor control UI.

.DESCRIPTION
    1. dotnet publish src/PoRobotTrain.Api -c Release into a temp folder: the trimmed Release WebAssembly build.
    2. Starts the behaviour service (python -m robotcontrol.behaviours.service --no-mirror, simulation only; it never
       moves a robot) unless something already answers on 8765, using .venv\Scripts\python.exe (or PYTHON).
    3. Starts the published app on http://127.0.0.1:5080 and waits until its /api/diag reaches the service.
    4. Runs tests/PoRobotTrain.E2EUI (Release, headless unless -Headed); installs the Playwright Chromium if missing.
    5. Stops the app and the service it started. Exit code = the test run's.

.EXAMPLE
    powershell -ExecutionPolicy Bypass -File scripts\run-e2eui.ps1
#>
[CmdletBinding()]
param(
    [switch]$Headed,
    [string]$Filter
)

$ErrorActionPreference = 'Stop'
$repoRoot = Split-Path -Parent $PSScriptRoot
$publishDir = Join-Path ([IO.Path]::GetTempPath()) 'porobottrain-e2eui-publish'
$baseUrl = 'http://127.0.0.1:5080'

function Test-Port([int]$port) {
    $client = [Net.Sockets.TcpClient]::new()
    try { $client.ConnectAsync('127.0.0.1', $port).Wait(1000) -and $client.Connected } catch { $false } finally { $client.Dispose() }
}

if (Test-Port 5080) { throw 'Something already listens on 5080. Stop it first: this script starts its own app there.' }

Write-Host "Publishing Release to $publishDir"
if (Test-Path $publishDir) { Remove-Item $publishDir -Recurse -Force }
dotnet publish (Join-Path $repoRoot 'src/PoRobotTrain.Api') -c Release -o $publishDir --nologo -v q
if ($LASTEXITCODE -ne 0) { throw 'dotnet publish failed' }

$service = $null
if (-not (Test-Port 8765)) {
    $python = if ($env:PYTHON) { $env:PYTHON } else { Join-Path $repoRoot '.venv\Scripts\python.exe' }
    if (-not (Test-Path $python)) { throw "No Python at ${python}: create .venv (README) or set PYTHON" }
    Write-Host "Starting the behaviour service with $python"
    $service = Start-Process $python -ArgumentList '-m', 'robotcontrol.behaviours.service', '--no-mirror' -WorkingDirectory $repoRoot `
        -PassThru -NoNewWindow -RedirectStandardOutput (Join-Path $publishDir 'service.log') -RedirectStandardError (Join-Path $publishDir 'service.err.log')
}

$env:ASPNETCORE_URLS = $baseUrl
$env:ASPNETCORE_ENVIRONMENT = 'Production'
$app = Start-Process (Join-Path $publishDir 'PoRobotTrain.Api.exe') -WorkingDirectory $publishDir -PassThru -NoNewWindow `
    -RedirectStandardOutput (Join-Path $publishDir 'app.log') -RedirectStandardError (Join-Path $publishDir 'app.err.log')
$exit = 1
try {
    $up = $false
    foreach ($i in 1..60) {
        if ($app.HasExited) { throw "The app exited during startup; see $publishDir\app.log" }
        # ready = the published app answers AND reaches the behaviour service (its /api/diag says so)
        try { if ((Invoke-WebRequest "$baseUrl/api/diag" -UseBasicParsing -TimeoutSec 10).Content -match '"upstreamReachable":true') { $up = $true; break } } catch { }
        Start-Sleep -Seconds 1
    }
    if (-not $up) { throw "The app or the behaviour service did not come up within 60 s; see $publishDir" }
    Write-Host "App (Release publish) is up at $baseUrl"

    $testProject = Join-Path $repoRoot 'tests/PoRobotTrain.E2EUI'
    dotnet build $testProject -c Release --nologo -v q
    if ($LASTEXITCODE -ne 0) { throw 'E2EUI build failed' }
    $playwright = Join-Path $testProject 'bin/Release/net10.0/playwright.ps1'
    powershell -ExecutionPolicy Bypass -File $playwright install chromium | Out-Null

    $env:BASE_URL = $baseUrl
    $env:HEADLESS = if ($Headed) { '0' } else { '1' }
    $testArgs = @('test', $testProject, '-c', 'Release', '--no-build')
    if ($Filter) { $testArgs += @('--filter', $Filter) }
    dotnet @testArgs
    $exit = $LASTEXITCODE
}
finally {
    if (-not $app.HasExited) { Stop-Process -Id $app.Id -Force }
    # the venv python.exe is a launcher with a child interpreter: stop the whole tree
    if ($service -and -not $service.HasExited) { taskkill /T /F /PID $service.Id | Out-Null }
}
exit $exit
