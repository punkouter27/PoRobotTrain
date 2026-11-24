# Before/after screenshot report for a UI change (AGENTS.md: "document before/after screenshots and an HTML report
# under SCREENSHOTS/"). One command, from the repo root:
#
#   powershell -ExecutionPolicy Bypass -File scripts\screenshots.ps1 -Feature reach_tolerance
#   powershell -ExecutionPolicy Bypass -File scripts\screenshots.ps1 -Feature reach_tolerance -Before 4bfdf32~1
#   powershell -ExecutionPolicy Bypass -File scripts\screenshots.ps1 -Feature walk -Routes /walk -BeforeDir <PNGs>
#
# It captures the control UI LOCALLY (never the live site): "after" = the Release publish of this working tree,
# "before" = the Release publish of the -Before ref (a temporary git worktree), or, with -BeforeDir, PNGs named
# *_<route>_<W>x<H>.png (<route> = the route with non-word characters removed, "root" for /). The published app runs
# on 127.0.0.1:<Port> with the behaviour service in simulation (--no-mirror, as scripts\run-e2eui.ps1 does; it never
# moves a robot). Each route is shot at 1366x768 and 390x844 in Edge (headless, driven over CDP), then:
#   SCREENSHOTS/<feature>_{before,after}_<route>_<W>x<H>.png   (scaled down to stay small)
#   SCREENSHOTS/<feature>_report.html                           (before | after per screen, notes)
#   SCREENSHOTS/<feature>_sidebyside.png                        (one side-by-side annotated image)
# The Unreal window is not shot here: use the -ShowcaseCapture run (AGENTS.md part 3) and pass its PNGs via -BeforeDir /
# -AfterDir. One headless browser at a time on this PC (AGENTS.md part 2).

[CmdletBinding()]
param(
    [Parameter(Mandatory)] [string]$Feature,
    [string]$Before = "HEAD~1",
    [string]$BeforeDir = "",
    [string]$AfterDir = "",
    [string[]]$Routes = @("/", "/diag", "/skills"),
    [string]$Notes = "",
    [int]$Port = 5090,
    [int]$WaitSeconds = 3,
    # ready: Blazor rendered the page's h1 and /diag stopped saying RUNNING
    [string]$ReadyJs = "!!document.querySelector('h1') && !/RUNNING/.test((document.querySelector('.sec-hdr .po-pill') || {}).textContent || '')"
)

$ErrorActionPreference = "Stop"
Add-Type -AssemblyName System.Drawing
$Root = Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path)
$Py = if ($env:PYTHON) { $env:PYTHON } else { Join-Path $Root ".venv\Scripts\python.exe" }
$Sizes = @(@(1366, 768), @(390, 844))
$Stamp = (Get-Date).ToUniversalTime().ToString("yyMMdd")
$Out = Join-Path $Root "SCREENSHOTS"
New-Item -ItemType Directory -Force $Out | Out-Null
$Edge = @("${env:ProgramFiles(x86)}\Microsoft\Edge\Application\msedge.exe", "$env:ProgramFiles\Microsoft\Edge\Application\msedge.exe") |
    Where-Object { Test-Path $_ } | Select-Object -First 1
if (-not $Edge) { throw "Microsoft Edge not found" }

function Get-RouteKey([string]$Route) { $k = ($Route -replace '\W', ''); if ($k) { $k } else { "root" } }

function Send-Cdp($Ws, [int]$Id, [string]$Method, $Params) {
    $msg = @{ id = $Id; method = $Method; params = $Params } | ConvertTo-Json -Depth 6 -Compress
    $bytes = [Text.Encoding]::UTF8.GetBytes($msg)
    $Ws.SendAsync([ArraySegment[byte]]$bytes, "Text", $true, [Threading.CancellationToken]::None).Wait()
    $buf = New-Object byte[] (1MB)
    while ($true) {
        $ms = New-Object IO.MemoryStream
        do {
            $r = $Ws.ReceiveAsync([ArraySegment[byte]]$buf, [Threading.CancellationToken]::None).Result
            $ms.Write($buf, 0, $r.Count)
        } while (-not $r.EndOfMessage)
        $reply = [Text.Encoding]::UTF8.GetString($ms.ToArray()) | ConvertFrom-Json
        if ($reply.id -eq $Id) { return $reply }
    }
}

# Poll $ReadyJs (up to 120 s; the arena loads MuJoCo WASM, the brains and the GLBs), then let it render a few seconds.
function Wait-Ready($Ws, [ref]$Id) {
    Start-Sleep -Milliseconds 500
    $t0 = Get-Date
    while (((Get-Date) - $t0).TotalSeconds -lt 120) {
        $r = Send-Cdp $Ws (++$Id.Value) "Runtime.evaluate" @{ expression = "document.readyState === 'complete' && ($ReadyJs)"; returnByValue = $true }
        if ($r.result.result.value -eq $true) { break }
        Start-Sleep -Milliseconds 500
    }
    Start-Sleep -Seconds $WaitSeconds
}

# Shoot every route x size of the site served at $Base into $Dir as <prefix>_<route>_<W>x<H>.png (full size).
function Invoke-Capture([string]$Base, [string]$Dir, [string]$Prefix) {
    $profileDir = Join-Path $env:TEMP "porobottrain-shots-$PID"
    $dbg = $Port + 1
    $p = Start-Process $Edge -PassThru -ArgumentList @("--headless=new", "--remote-debugging-port=$dbg", "--user-data-dir=$profileDir",
        "--use-angle=d3d11", "--ignore-gpu-blocklist", "--hide-scrollbars", "--no-first-run", "about:blank")
    try {
        $target = $null
        foreach ($i in 1..50) {
            try { $target = (Invoke-RestMethod "http://127.0.0.1:$dbg/json" -TimeoutSec 2) | Where-Object type -eq "page" | Select-Object -First 1 } catch {}
            if ($target) { break }; Start-Sleep -Milliseconds 200
        }
        if (-not $target) { throw "Edge DevTools did not come up" }
        $ws = New-Object Net.WebSockets.ClientWebSocket
        $ws.ConnectAsync([Uri]$target.webSocketDebuggerUrl, [Threading.CancellationToken]::None).Wait()
        $id = 0
        # warm-up load: the first visit compiles the WASM and fills the cache; shooting it would catch the loading card
        $null = Send-Cdp $ws (++$id) "Page.navigate" @{ url = "$Base$($Routes[0])" }
        Wait-Ready $ws ([ref]$id)
        foreach ($route in $Routes) {
            foreach ($s in $Sizes) {
                $w, $h = $s
                $null = Send-Cdp $ws (++$id) "Emulation.setDeviceMetricsOverride" @{ width = $w; height = $h; deviceScaleFactor = 1; mobile = ($w -lt 600) }
                $null = Send-Cdp $ws (++$id) "Page.navigate" @{ url = "$Base$route" }
                Wait-Ready $ws ([ref]$id)
                $shot = Send-Cdp $ws (++$id) "Page.captureScreenshot" @{ format = "png" }
                $file = Join-Path $Dir ("{0}_{1}_{2}x{3}.png" -f $Prefix, (Get-RouteKey $route), $w, $h)
                [IO.File]::WriteAllBytes($file, [Convert]::FromBase64String($shot.result.data))
                Write-Host "  $file"
            }
        }
        $ws.Dispose()
    } finally {
        Stop-Process -Id $p.Id -Force -ErrorAction SilentlyContinue
        Get-CimInstance Win32_Process -Filter "Name='msedge.exe'" | Where-Object { $_.CommandLine -like "*$profileDir*" } |
            ForEach-Object { Stop-Process -Id $_.ProcessId -Force -ErrorAction SilentlyContinue }
        Start-Sleep -Milliseconds 500
        Remove-Item -Recurse -Force $profileDir -ErrorAction SilentlyContinue
    }
}

function Test-Port([int]$P) {
    $c = [Net.Sockets.TcpClient]::new()
    try { $c.ConnectAsync('127.0.0.1', $P).Wait(1000) -and $c.Connected } catch { $false } finally { $c.Dispose() }
}

# Release-publish the API (it hosts the WebAssembly client) from $Tree, run it with the behaviour service, shoot it, stop both.
function Invoke-BuildAndCapture([string]$Tree, [string]$Dir, [string]$Prefix) {
    if (Test-Port $Port) { throw "Something already listens on $Port; pass -Port" }
    $pub = Join-Path $env:TEMP "porobottrain-shots-pub-$PID-$Prefix"
    if (Test-Path $pub) { Remove-Item -Recurse -Force $pub }
    dotnet publish (Join-Path $Tree "src\PoRobotTrain.Api") -c Release -o $pub --nologo -v q | Out-Null
    if ($LASTEXITCODE) { throw "dotnet publish failed in $Tree" }
    $service = $null
    if (-not (Test-Port 8765)) {
        $service = Start-Process $Py -ArgumentList "-m", "robotcontrol.behaviours.service", "--no-mirror" -WorkingDirectory $Root -PassThru -WindowStyle Hidden `
            -RedirectStandardOutput (Join-Path $pub "service.log") -RedirectStandardError (Join-Path $pub "service.err.log")
    }
    $env:ASPNETCORE_URLS = "http://127.0.0.1:$Port"; $env:ASPNETCORE_ENVIRONMENT = "Production"
    $app = Start-Process (Join-Path $pub "PoRobotTrain.Api.exe") -WorkingDirectory $pub -PassThru -WindowStyle Hidden `
        -RedirectStandardOutput (Join-Path $pub "app.log") -RedirectStandardError (Join-Path $pub "app.err.log")
    try {
        foreach ($i in 1..60) {
            try { if ((Invoke-WebRequest "http://127.0.0.1:$Port/api/diag" -UseBasicParsing -TimeoutSec 5).Content -match '"upstreamReachable":true') { break } } catch {}
            Start-Sleep -Seconds 1
        }
        Invoke-Capture "http://127.0.0.1:$Port" $Dir $Prefix
    } finally {
        if (-not $app.HasExited) { Stop-Process -Id $app.Id -Force }
        if ($service -and -not $service.HasExited) { taskkill /T /F /PID $service.Id | Out-Null }   # venv launcher + child
        Start-Sleep -Milliseconds 500
        Remove-Item -Recurse -Force $pub -ErrorAction SilentlyContinue
    }
}

# Copy *_<route>_<W>x<H>.png from a folder to <prefix>_<route>_<W>x<H>.png.
function Copy-Shots([string]$From, [string]$Dir, [string]$Prefix) {
    foreach ($route in $Routes) { foreach ($s in $Sizes) {
        $name = "_{0}_{1}x{2}.png" -f (Get-RouteKey $route), $s[0], $s[1]
        $src = Get-ChildItem $From -Filter "*.png" | Where-Object { $_.Name -like "*$name" -or $_.Name -eq $name.TrimStart("_") } | Select-Object -First 1
        if ($src) { Copy-Item $src.FullName (Join-Path $Dir "$Prefix$name") -Force } else { Write-Warning "no $name in $From" }
    } }
}

$Raw = Join-Path $env:TEMP "porobottrain-shots-raw-$PID"
New-Item -ItemType Directory -Force $Raw | Out-Null
try {
    Write-Host "after (this working tree):"
    if ($AfterDir) { Copy-Shots $AfterDir $Raw "after" } else { Invoke-BuildAndCapture $Root $Raw "after" }

    Write-Host "before:"
    if ($BeforeDir) { Copy-Shots $BeforeDir $Raw "before" }
    else {
        $wt = Join-Path $env:TEMP "porobottrain-shots-wt-$PID"
        git -C $Root worktree add --detach $wt $Before | Out-Null
        try { Invoke-BuildAndCapture $wt $Raw "before" }
        finally { git -C $Root worktree remove --force $wt | Out-Null }
    }

    # Scale each shot down (the repo is git, not an image store), measure how much changed, build the page + one image.
    $rows = @()
    foreach ($route in $Routes) { foreach ($s in $Sizes) {
        $key = "{0}_{1}x{2}" -f (Get-RouteKey $route), $s[0], $s[1]
        $scale = if ($s[0] -gt 600) { 0.5 } else { 0.6 }
        $pair = @{}
        foreach ($side in "before", "after") {
            $src = Join-Path $Raw "${side}_$key.png"
            if (-not (Test-Path $src)) { continue }
            $img = [Drawing.Image]::FromFile($src)
            $bmp = [Drawing.Bitmap]::new([int]($img.Width * $scale), [int]($img.Height * $scale))
            $g = [Drawing.Graphics]::FromImage($bmp); $g.InterpolationMode = "HighQualityBicubic"
            $g.DrawImage($img, 0, 0, $bmp.Width, $bmp.Height); $g.Dispose(); $img.Dispose()
            $bmp.Save((Join-Path $Out "${Feature}_${side}_$key.png"), [Drawing.Imaging.ImageFormat]::Png)
            $pair[$side] = $bmp
        }
        $changed = "n/a"
        if ($pair.before -and $pair.after -and $pair.before.Width -eq $pair.after.Width -and $pair.before.Height -eq $pair.after.Height) {
            $diff = 0; $n = 0
            for ($y = 0; $y -lt $pair.after.Height; $y += 4) { for ($x = 0; $x -lt $pair.after.Width; $x += 4) {
                $a = $pair.before.GetPixel($x, $y); $b = $pair.after.GetPixel($x, $y); $n++
                if ([Math]::Abs($a.R - $b.R) + [Math]::Abs($a.G - $b.G) + [Math]::Abs($a.B - $b.B) -gt 48) { $diff++ }
            } }
            $changed = "{0:N1}% of pixels differ" -f (100.0 * $diff / [Math]::Max(1, $n))
        }
        $rows += [pscustomobject]@{ Route = $route; Key = $key; W = $s[0]; H = $s[1]; Changed = $changed; Before = $pair.before; After = $pair.after }
    } }

    # one side-by-side annotated PNG: a header, then per screen "label | before | after"
    $label = 220; $gap = 12; $head = 56
    $colW = ($rows | ForEach-Object { @($_.Before, $_.After) | Where-Object { $_ } | ForEach-Object { $_.Width } } | Measure-Object -Maximum).Maximum
    $height = $head + ($rows | ForEach-Object { (@($_.Before, $_.After) | Where-Object { $_ } | ForEach-Object { $_.Height } | Measure-Object -Maximum).Maximum + $gap } | Measure-Object -Sum).Sum
    $sheet = [Drawing.Bitmap]::new([int]($label + 2 * $colW + 3 * $gap), [int]$height)
    $g = [Drawing.Graphics]::FromImage($sheet); $g.Clear([Drawing.Color]::FromArgb(14, 14, 18)); $g.TextRenderingHint = "AntiAlias"
    $fh = New-Object Drawing.Font "Segoe UI", 14, ([Drawing.FontStyle]::Bold); $fb = New-Object Drawing.Font "Segoe UI", 10
    $white = [Drawing.Brushes]::White; $muted = New-Object Drawing.SolidBrush ([Drawing.Color]::FromArgb(170, 170, 185))
    $g.DrawString("PoRobotTrain - $Feature ($Stamp)", $fh, $white, $gap, 10)
    $g.DrawString("BEFORE", $fb, $muted, $label + $gap, 34); $g.DrawString("AFTER", $fb, $muted, $label + $colW + 2 * $gap, 34)
    $y = $head
    foreach ($r in $rows) {
        $g.DrawString("$($r.Route)`n$($r.W)x$($r.H)`n$($r.Changed)", $fb, $white, (New-Object Drawing.RectangleF $gap, $y, ($label - $gap), 80))
        if ($r.Before) { $g.DrawImage($r.Before, $label + $gap, $y, $r.Before.Width, $r.Before.Height) }
        if ($r.After) { $g.DrawImage($r.After, $label + $colW + 2 * $gap, $y, $r.After.Width, $r.After.Height) }
        $y += (@($r.Before, $r.After) | Where-Object { $_ } | ForEach-Object { $_.Height } | Measure-Object -Maximum).Maximum + $gap
    }
    $g.Dispose()
    $sheetPath = Join-Path $Out "${Feature}_sidebyside.png"
    $k = [Math]::Min(1.0, 800.0 / $sheet.Width)   # keep the one-image summary small (about 300 KB)
    $small = [Drawing.Bitmap]::new([int]($sheet.Width * $k), [int]($sheet.Height * $k))
    $g = [Drawing.Graphics]::FromImage($small); $g.InterpolationMode = "HighQualityBicubic"
    $g.DrawImage($sheet, 0, 0, $small.Width, $small.Height); $g.Dispose(); $sheet.Dispose()
    $small.Save($sheetPath, [Drawing.Imaging.ImageFormat]::Png); $small.Dispose()
    foreach ($r in $rows) { if ($r.Before) { $r.Before.Dispose() }; if ($r.After) { $r.After.Dispose() } }

    $beforeText = if ($BeforeDir) { "PNGs from $(Split-Path -Leaf $BeforeDir)" } else { "git $Before ($(git -C $Root rev-parse --short $Before))" }
    $afterText = if ($AfterDir) { "PNGs from $(Split-Path -Leaf $AfterDir)" } else { "working tree at $(git -C $Root rev-parse --short HEAD)" }
    $enc = { param($t) [Net.WebUtility]::HtmlEncode($t) }
    $html = @"
<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>PoRobotTrain - $(& $enc $Feature)</title>
<style>
:root { color-scheme: dark; } body { margin: 0; padding: 16px; background: #0e0e12; color: #e8e8ee; font: 14px/1.5 "Segoe UI", system-ui, sans-serif; }
h1 { font-size: 20px; margin: 0 0 4px; } p.meta { color: #aaaab9; margin: 0 0 16px; }
section { margin: 0 0 24px; } h2 { font-size: 15px; margin: 0 0 6px; } .pair { display: grid; grid-template-columns: 1fr 1fr; gap: 12px; }
figure { margin: 0; } figcaption { color: #aaaab9; font-size: 12px; } img { max-width: 100%; height: auto; border: 1px solid #2a2a33; }
@media (max-width: 700px) { .pair { grid-template-columns: 1fr; } }
</style></head><body>
<h1>PoRobotTrain - $(& $enc $Feature)</h1>
<p class="meta">Before: $(& $enc $beforeText). After: $(& $enc $afterText). Captured locally (Release publish + behaviour service in simulation), $((Get-Date).ToString("yyyy-MM-dd HH:mm")). $(& $enc $Notes)</p>
<p><img src="${Feature}_sidebyside.png" alt="All screens side by side, before and after"></p>
"@
    foreach ($r in $rows) {
        $html += "<section><h2>$(& $enc $r.Route) at $($r.W)x$($r.H) <small>($($r.Changed))</small></h2><div class=`"pair`">"
        foreach ($side in "before", "after") {
            $html += "<figure><img src=`"${Feature}_${side}_$($r.Key).png`" alt=`"$side, $(& $enc $r.Route) at $($r.W)x$($r.H)`"><figcaption>$side</figcaption></figure>"
        }
        $html += "</div></section>`n"
    }
    $html += "</body></html>`n"
    Set-Content -Path (Join-Path $Out "${Feature}_report.html") -Value $html -Encoding utf8
    Write-Host "wrote SCREENSHOTS\${Feature}_report.html and SCREENSHOTS\${Feature}_sidebyside.png"
} finally {
    Remove-Item -Recurse -Force $Raw -ErrorAction SilentlyContinue
}
