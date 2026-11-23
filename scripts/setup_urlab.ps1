# Recreate the URLab plugin in UnrealProject/Plugins/UnrealRoboticsLab: upstream clone at a pinned commit, MuJoCo
# moved to 4.0.0 (the version RobotControl pins), this repo's MuJoCo 4.0 / UE 5.8 compile fixes
# (scripts/patches/urlab_mujoco4_ue58.patch), then URLab's native dependencies built. The plugin folder is NOT tracked
# (its own git repo, ~2 GB with build output; ignored in .gitignore), so a fresh clone runs this once before building
# the R1TwinEditor target.
#
#   powershell -ExecutionPolicy Bypass -File scripts\setup_urlab.ps1
#
# Safe to re-run: the clone, checkout and patch are skipped when the plugin clone already has the local commit.
# Needs git, CMake >= 3.24 and the Visual Studio C++ toolchain.
$ErrorActionPreference = 'Stop'
$UrlabCommit  = 'efbc5e272a51dbe641c7c078406bd3c076b05823'   # URLab-Sim/UnrealRoboticsLab, branch alpha, 2026-08-08
$MujocoTag    = '4.0.0'
$root   = Split-Path -Parent $PSScriptRoot
$plugin = Join-Path $root 'UnrealProject/Plugins/UnrealRoboticsLab'
$patch  = Join-Path $root 'scripts/patches/urlab_mujoco4_ue58.patch'

function Invoke-Git { git @args; if ($LASTEXITCODE -ne 0) { throw "git $args failed ($LASTEXITCODE)" } }

if (-not (Test-Path (Join-Path $plugin '.git'))) {
    if (Test-Path $plugin) { Remove-Item $plugin -Recurse -Force }   # an empty placeholder folder only
    Invoke-Git clone https://github.com/URLab-Sim/UnrealRoboticsLab $plugin
}
# One local commit on top of the pinned upstream commit: the patch plus MuJoCo's submodule pointer at $MujocoTag.
# URLab.Build.cs compares the checked-out MuJoCo with the pointer in HEAD (drift check), so the pointer must move too.
$localSubject = "PoRobotTrain: MuJoCo $MujocoTag + UE 5.8 fixes"
if ((git -C $plugin log -1 --format=%s) -ne $localSubject) {
    Invoke-Git -C $plugin checkout --quiet --force $UrlabCommit
    Invoke-Git -C $plugin submodule update --init --recursive --force
    Invoke-Git -C (Join-Path $plugin 'third_party/MuJoCo/src') fetch --quiet --tags origin
    Invoke-Git -C (Join-Path $plugin 'third_party/MuJoCo/src') checkout --quiet $MujocoTag
    Invoke-Git -C $plugin apply $patch
    # imported meshes go next to the Blueprint (/Game/Project/Agents/MuJoCoImports), see scripts/ue_reimport_stand.py
    Invoke-Git -C $plugin apply (Join-Path $root 'scripts/patches/urlab_import_folder.patch')
    Invoke-Git -C $plugin add Source third_party/MuJoCo/src
    Invoke-Git -C $plugin -c user.name=PoRobotTrain -c user.email=porobottrain@localhost commit --quiet -m $localSubject
    Write-Host "URLab: $UrlabCommit + $patch + MuJoCo $MujocoTag committed locally"
} else {
    Write-Host "URLab: already at '$localSubject'"
}

# CoACD's build.ps1 names the "Visual Studio 17 2022" CMake generator; with only a newer Visual Studio installed,
# point it at the newest one (an edit inside the untracked plugin clone).
$vswhere = Join-Path ${env:ProgramFiles(x86)} 'Microsoft Visual Studio\Installer\vswhere.exe'
if (Test-Path $vswhere) {
    $vsMajor = (& $vswhere -latest -products * -property installationVersion).Split('.')[0]
    $gen = (cmake --help | Select-String "Visual Studio $vsMajor \d{4}" | Select-Object -First 1).Matches.Value
    $coacd = Join-Path $plugin 'third_party/CoACD/build.ps1'
    if ($gen -and $vsMajor -ne '17') {
        (Get-Content $coacd -Raw) -replace '"Visual Studio \d+ \d{4}"', "`"$gen`"" | Set-Content $coacd -NoNewline
    }
}

# build_all.ps1 uses relative paths, so it runs from its own folder. -NoSubmoduleSync keeps MuJoCo at $MujocoTag
# (each dep's build.ps1 would otherwise reset its submodule to URLab's own pin). It does not stop on a failed
# dependency, so check every install folder afterwards.
$deps = 'CoACD', 'MuJoCo', 'libzmq'
if ($deps | Where-Object { -not (Test-Path (Join-Path $plugin "third_party/install/$_")) }) {
    Push-Location (Join-Path $plugin 'third_party')
    try { powershell -ExecutionPolicy Bypass -File .\build_all.ps1 -NoSubmoduleSync } finally { Pop-Location }
} else {
    Write-Host 'URLab: third-party dependencies already built (delete third_party/install to rebuild)'
}
foreach ($dep in $deps) {
    if (-not (Test-Path (Join-Path $plugin "third_party/install/$dep"))) { throw "URLab third-party build failed: $dep not installed" }
}
Write-Host "URLab ready at $plugin (commit $UrlabCommit, MuJoCo $MujocoTag). Next: build R1TwinEditor."
