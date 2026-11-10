<#
.SYNOPSIS
  Put this Windows laptop on the R1's wired network (192.168.123.x) and check the link.

.DESCRIPTION
  1. With no arguments: lists the laptop's wired network adapters so you can pick the one the robot cable is in.
  2. -Adapter "Ethernet 2" -Apply   (run PowerShell as Administrator): sets 192.168.123.222/24 on that adapter,
     no gateway, so Wi-Fi keeps your internet. Then pings the robot's Jetson (192.168.123.164) and tests SSH port 22.
  3. -Adapter "Ethernet 2" -Restore : puts the adapter back on DHCP.
  4. -Adapter "Ethernet 2"          : only runs the link checks.

.EXAMPLE
  powershell -ExecutionPolicy Bypass -File scripts\robot_network_setup.ps1
  powershell -ExecutionPolicy Bypass -File scripts\robot_network_setup.ps1 -Adapter "Ethernet 2" -Apply
#>
param(
    [string]$Adapter,
    [switch]$Apply,
    [switch]$Restore,
    [string]$LaptopIP = "192.168.123.222",
    [string]$RobotIP = "192.168.123.164"
)

function Test-Admin {
    $id = [Security.Principal.WindowsIdentity]::GetCurrent()
    return (New-Object Security.Principal.WindowsPrincipal $id).IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)
}

if (-not $Adapter) {
    Write-Host "Wired adapters on this laptop (plug the robot cable in, then pick the one whose Status is Up):`n"
    Get-NetAdapter -Physical | Where-Object { $_.Name -notmatch 'Wi-?Fi|Wireless|Bluetooth' -and $_.InterfaceDescription -notmatch 'Wi-?Fi|Wireless|Bluetooth' } |
        Select-Object Name, Status, LinkSpeed, InterfaceDescription | Format-Table -AutoSize
    Write-Host "Then run (as Administrator):  scripts\robot_network_setup.ps1 -Adapter `"<Name>`" -Apply"
    exit 0
}

$nic = Get-NetAdapter -Name $Adapter -ErrorAction Stop
if ($Apply -or $Restore) {
    if (-not (Test-Admin)) { Write-Error "Run PowerShell as Administrator to change the IP address."; exit 1 }
}

if ($Restore) {
    Get-NetIPAddress -InterfaceAlias $Adapter -AddressFamily IPv4 -ErrorAction SilentlyContinue | Remove-NetIPAddress -Confirm:$false
    Set-NetIPInterface -InterfaceAlias $Adapter -Dhcp Enabled
    Write-Host "$Adapter restored to DHCP."
    exit 0
}

if ($Apply) {
    Set-NetIPInterface -InterfaceAlias $Adapter -Dhcp Disabled
    Get-NetIPAddress -InterfaceAlias $Adapter -AddressFamily IPv4 -ErrorAction SilentlyContinue | Remove-NetIPAddress -Confirm:$false
    New-NetIPAddress -InterfaceAlias $Adapter -IPAddress $LaptopIP -PrefixLength 24 | Out-Null
    Write-Host "$Adapter set to $LaptopIP/24 (no gateway; internet stays on Wi-Fi)."
    Start-Sleep -Seconds 2
}

Write-Host "`nAdapter $Adapter : status $($nic.Status), speed $($nic.LinkSpeed)"
Get-NetIPAddress -InterfaceAlias $Adapter -AddressFamily IPv4 -ErrorAction SilentlyContinue | Select-Object IPAddress, PrefixLength | Format-Table -AutoSize
$ping = Test-Connection -ComputerName $RobotIP -Count 2 -Quiet
Write-Host ("ping $RobotIP (robot Jetson): " + $(if ($ping) { "OK" } else { "NO REPLY" }))
$ssh = Test-NetConnection -ComputerName $RobotIP -Port 22 -WarningAction SilentlyContinue
Write-Host ("SSH port 22 on $RobotIP     : " + $(if ($ssh.TcpTestSucceeded) { "OPEN" } else { "CLOSED" }))
if ($ping -and $ssh.TcpTestSucceeded) {
    Write-Host "`nLink is good. Log in with:  ssh unitree@$RobotIP   (factory password 123 - change it with 'passwd')"
} else {
    Write-Host "`nNo link. Check: robot powered on (wait ~1 min after boot), cable in the robot's RJ45 on the upper-body panel, correct -Adapter."
}
