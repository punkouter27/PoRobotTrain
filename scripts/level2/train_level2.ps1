# Level 2 (walking) policy training in Docker on the local NVIDIA GPU.
#
#   pwsh scripts/level2/train_level2.ps1                    # build image + start training (detached)
#   pwsh scripts/level2/train_level2.ps1 -NumEnvs 2048 -MaxIterations 3000
#   pwsh scripts/level2/train_level2.ps1 -Resume            # continue the newest run
#   pwsh scripts/level2/train_level2.ps1 -Export            # export newest checkpoint -> logs/level2_training/candidate
#
# Trainer: Unitree unitree_rl_mjlab (mjlab 1.2.0 + MuJoCo Warp), task Unitree-R1-Flat, reference clone pinned below in
# external/unitree_rl_mjlab (gitignored). Checkpoints, TensorBoard logs and the exported candidate land in
# logs/level2_training/ (gitignored). Nothing here promotes a policy: RobotControl/training/validate_level2.py does.
param(
    [int]$NumEnvs = 4096,
    [int]$MaxIterations = 3000,
    [switch]$Resume,
    [switch]$Export,
    [string]$Checkpoint = "",
    [switch]$NoBuild
)
$ErrorActionPreference = "Stop"
$Repo = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
$Image = "porobot-l2:dev"
$Container = "porobot-l2-train"
$UpstreamUrl = "https://github.com/unitreerobotics/unitree_rl_mjlab.git"
$UpstreamCommit = "1425b15f73bd4095f0df53709d7c389c3eb9e790"   # main @ 2026-04-13
$Clone = Join-Path $Repo "external\unitree_rl_mjlab"
$Logs = Join-Path $Repo "logs\level2_training"

if (-not (Test-Path $Clone)) {
    git clone $UpstreamUrl $Clone
}
$head = (git -C $Clone rev-parse HEAD).Trim()
if ($head -ne $UpstreamCommit) {
    git -C $Clone fetch --depth 1 origin $UpstreamCommit
    git -C $Clone checkout --quiet $UpstreamCommit
}
New-Item -ItemType Directory -Force $Logs | Out-Null

if (-not $NoBuild) {
    $env:DOCKER_BUILDKIT = "1"
    docker build -f (Join-Path $Repo "scripts\level2\Dockerfile") -t $Image $Repo
    if ($LASTEXITCODE -ne 0) { throw "docker build failed" }
}

$mount = "${Logs}:/opt/unitree_rl_mjlab/logs"
$scripts = "$(Join-Path $Repo 'scripts\level2'):/opt/porobot:ro"
$cache = "porobot-l2-cache:/root/.cache"   # named volume: Warp kernel cache survives container restarts
docker volume create porobot-l2-cache | Out-Null

if ($Export) {
    $a = @()
    if ($Checkpoint) { $a += @("--checkpoint", $Checkpoint) }
    docker run --rm --gpus all --shm-size=2g -e "L2_SOURCE_COMMIT=$UpstreamCommit" -v $mount -v $scripts -v $cache `
        --entrypoint python $Image /opt/porobot/export_level2.py @a
    exit $LASTEXITCODE
}

if ((docker ps -a --filter "name=^/$Container$" --format "{{.Names}}") -eq $Container) {
    if ((docker inspect -f "{{.State.Running}}" $Container) -eq "true") { throw "$Container is already running (docker logs -f $Container)" }
    docker rm $Container | Out-Null
}

$trainArgs = @(
    "--env.scene.num-envs=$NumEnvs",
    "--agent.max-iterations=$MaxIterations",
    "--agent.logger=tensorboard",
    "--agent.upload-model=False"
)
if ($Resume) { $trainArgs += "--agent.resume=True" }

docker run -d --name $Container --gpus all --shm-size=2g -e "L2_SOURCE_COMMIT=$UpstreamCommit" -v $mount -v $cache $Image train @trainArgs
if ($LASTEXITCODE -ne 0) { throw "docker run failed" }

Write-Host ""
Write-Host "Training started in container '$Container' (task Unitree-R1-Flat, $NumEnvs envs, $MaxIterations iterations)."
Write-Host "  follow:      docker logs -f --tail 40 $Container"
Write-Host "  checkpoints: logs\level2_training\rsl_rl\r1_velocity\<run>\model_<it>.pt (every 100 iterations)"
Write-Host "  tensorboard: logs\level2_training\rsl_rl\r1_velocity"
Write-Host "  stop:        docker stop $Container        resume: -Resume -NoBuild"
Write-Host "  export:      pwsh scripts/level2/train_level2.ps1 -Export -NoBuild"
