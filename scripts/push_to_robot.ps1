<#
.SYNOPSIS
  Copy the robot kit to the R1's Jetson and install it there (offline). Sends nothing to the motors.

.DESCRIPTION
  1. Builds a fresh kit unless -Kit is given (newest deploy bundle in dist\).
  2. scp the .tar.gz to the robot (password prompt: factory default 123).
  3. ssh: unpack into ~/porobot (an existing ~/porobot/.venv is kept) and run setup_robot.sh, which ends with a
     software-only preflight.

.EXAMPLE
  powershell -ExecutionPolicy Bypass -File scripts\push_to_robot.ps1
  powershell -ExecutionPolicy Bypass -File scripts\push_to_robot.ps1 -Kit dist\r1_robot_kit_....tar.gz
#>
param(
    [string]$Kit,
    [string]$RobotIP = "192.168.123.164",
    [string]$User = "unitree",
    [string]$RemoteDir = "porobot"
)
$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $PSScriptRoot
Set-Location $root

if (-not $Kit) {
    & "$root\.venv\Scripts\python.exe" -m robotcontrol.tools.robot_kit
    if ($LASTEXITCODE -ne 0) { throw "kit build failed" }
    $Kit = (Get-ChildItem "$root\dist\r1_robot_kit_*.tar.gz" | Sort-Object LastWriteTime | Select-Object -Last 1).FullName
}
$name = Split-Path -Leaf $Kit
Write-Host "Kit: $Kit"

if (-not (Get-Command scp -ErrorAction SilentlyContinue)) {
    throw "OpenSSH client not found. Settings > System > Optional features > add 'OpenSSH Client'."
}

Write-Host "`n[1/2] copying to ${User}@${RobotIP}:~/$name"
scp "$Kit" "${User}@${RobotIP}:~/$name"
if ($LASTEXITCODE -ne 0) { throw "scp failed (is the laptop on 192.168.123.x? run scripts\robot_network_setup.ps1)" }

Write-Host "`n[2/2] unpacking into ~/$RemoteDir and running setup_robot.sh"
ssh -t "${User}@${RobotIP}" "mkdir -p ~/$RemoteDir && tar -xzf ~/$name -C ~/$RemoteDir && cd ~/$RemoteDir && bash setup_robot.sh"
if ($LASTEXITCODE -ne 0) { throw "remote setup failed - see the output above" }

Write-Host "`nInstalled. Next, on the robot (robot on its stand, powered, in zero-torque):"
Write-Host "  ssh $User@$RobotIP"
Write-Host "  cd ~/$RemoteDir && ./run.sh robotcontrol.tools.robot_preflight --bundle bundle"
