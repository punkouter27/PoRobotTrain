using System.Text.Json.Serialization;

namespace PoRobotTrain.Shared.Contracts;

// Voice, robustness explorer and Level 2 walk sandbox (docs/service_api.md). Field names follow the Python
// behaviour service (snake_case); both the Blazor UI and the Unreal twin talk to the same endpoints.

// ---------------------------------------------------------------------------------------------- voice (#11)

public sealed record AssistantInterpretRequest(
    [property: JsonPropertyName("text")] string Text,
    [property: JsonPropertyName("arm_unlocked")] bool ArmUnlocked = false,
    [property: JsonPropertyName("walk_active")] bool WalkActive = false);

/// <summary>Walk intent: a velocity command (m/s, m/s, rad/s) or <c>stop</c>.</summary>
public sealed record WalkIntentDto(
    [property: JsonPropertyName("vx")] double? Vx = null,
    [property: JsonPropertyName("vy")] double? Vy = null,
    [property: JsonPropertyName("wz")] double? Wz = null,
    [property: JsonPropertyName("stop")] bool? Stop = null);

public sealed record AssistantInterpretResponse(
    [property: JsonPropertyName("ok")] bool Ok,
    [property: JsonPropertyName("kind")] string Kind,
    [property: JsonPropertyName("id")] string? Id = null,
    [property: JsonPropertyName("label")] string? Label = null,
    [property: JsonPropertyName("amplitude")] double? Amplitude = null,
    [property: JsonPropertyName("joint")] string? Joint = null,
    [property: JsonPropertyName("deg")] double? Deg = null,
    [property: JsonPropertyName("walk")] WalkIntentDto? Walk = null,
    [property: JsonPropertyName("blocked")] bool Blocked = false,
    [property: JsonPropertyName("score")] double? Score = null,
    [property: JsonPropertyName("note")] string? Note = null,
    [property: JsonPropertyName("say")] string? Say = null);

// ---------------------------------------------------------------------------------------- robustness (#19)

public sealed record RobustnessRunRequest(
    [property: JsonPropertyName("quick")] bool Quick = true);

public sealed record RobustnessCellDto(
    [property: JsonPropertyName("mean_err_deg")] double? MeanErrDeg,
    [property: JsonPropertyName("max_err_deg")] double? MaxErrDeg,
    [property: JsonPropertyName("max_acc")] double? MaxAcc,
    [property: JsonPropertyName("trips")] int Trips,
    [property: JsonPropertyName("verdict")] string Verdict,
    [property: JsonPropertyName("acc_joint")] string? AccJoint = null,
    [property: JsonPropertyName("jitter")] double? Jitter = null,
    [property: JsonPropertyName("finite")] bool Finite = true);

public sealed record RobustnessFactorDto(
    [property: JsonPropertyName("id")] string Id,
    [property: JsonPropertyName("label")] string Label,
    [property: JsonPropertyName("unit")] string Unit,
    [property: JsonPropertyName("nominal")] double? Nominal,
    [property: JsonPropertyName("levels")] List<double> Levels,
    [property: JsonPropertyName("cells")] List<RobustnessCellDto> Cells);

public sealed record RobustnessAxisDto(
    [property: JsonPropertyName("id")] string Id,
    [property: JsonPropertyName("label")] string Label,
    [property: JsonPropertyName("unit")] string Unit,
    [property: JsonPropertyName("levels")] List<double> Levels);

public sealed record RobustnessInteractionDto(
    [property: JsonPropertyName("x")] RobustnessAxisDto X,
    [property: JsonPropertyName("y")] RobustnessAxisDto Y,
    [property: JsonPropertyName("cells")] List<List<RobustnessCellDto>> Cells);

public sealed record RobustnessThresholdsDto(
    [property: JsonPropertyName("mean_err_deg")] double MeanErrDeg = 3.0,
    [property: JsonPropertyName("max_acc")] double MaxAcc = 80.0,
    [property: JsonPropertyName("warn_mean_err_deg")] double WarnMeanErrDeg = 2.0,
    [property: JsonPropertyName("warn_acc")] double WarnAcc = 60.0);

public sealed record RobustnessSummaryDto(
    [property: JsonPropertyName("pass")] int Pass,
    [property: JsonPropertyName("warn")] int Warn,
    [property: JsonPropertyName("fail")] int Fail,
    [property: JsonPropertyName("worst")] string? Worst);

public sealed record RobustnessReportDto(
    [property: JsonPropertyName("available")] bool Available,
    [property: JsonPropertyName("created_at")] string? CreatedAt = null,
    [property: JsonPropertyName("policy")] string? Policy = null,
    [property: JsonPropertyName("policy_sha256")] string? PolicySha256 = null,
    [property: JsonPropertyName("quick")] bool Quick = false,
    [property: JsonPropertyName("duration_s")] double? DurationS = null,
    [property: JsonPropertyName("rollout_s")] double? RolloutS = null,
    [property: JsonPropertyName("thresholds")] RobustnessThresholdsDto? Thresholds = null,
    [property: JsonPropertyName("nominal")] RobustnessCellDto? Nominal = null,
    [property: JsonPropertyName("sensitivity")] List<RobustnessFactorDto>? Sensitivity = null,
    [property: JsonPropertyName("interaction")] RobustnessInteractionDto? Interaction = null,
    [property: JsonPropertyName("summary")] RobustnessSummaryDto? Summary = null);

/// <summary>Background job progress in /status (e.g. the robustness sweep), null when idle.</summary>
public sealed record JobProgressDto(
    [property: JsonPropertyName("job")] string? Job,
    [property: JsonPropertyName("done")] int Done,
    [property: JsonPropertyName("total")] int Total);

// ------------------------------------------------------------------------------------- walk sandbox (#20)

/// <summary>course: flat | loop | procedural; mode: policy | hold | free_hold (no harness: topples). Seed and difficulty
/// (0..1) only matter for procedural and are left out of the body when null (docs/sim_features.md).</summary>
public sealed record WalkStartRequest(
    [property: JsonPropertyName("course")] string Course = "loop",
    [property: JsonPropertyName("mode")] string Mode = "policy",
    [property: JsonPropertyName("seed"), JsonIgnore(Condition = JsonIgnoreCondition.WhenWritingNull)] long? Seed = null,
    [property: JsonPropertyName("difficulty"), JsonIgnore(Condition = JsonIgnoreCondition.WhenWritingNull)] double? Difficulty = null);

public sealed record WalkCommandRequest(
    [property: JsonPropertyName("vx")] double Vx = 0,
    [property: JsonPropertyName("vy")] double Vy = 0,
    [property: JsonPropertyName("wz")] double Wz = 0);

public sealed record WalkBaseDto(
    [property: JsonPropertyName("pos")] double[] Pos,
    [property: JsonPropertyName("quat")] double[] Quat,
    [property: JsonPropertyName("lin_vel")] double[]? LinVel,
    [property: JsonPropertyName("yaw")] double Yaw);

public sealed record WalkCheckpointDto(
    [property: JsonPropertyName("index")] int Index,
    [property: JsonPropertyName("total")] int Total,
    [property: JsonPropertyName("next")] double[]? Next);

public sealed record WalkLapDto(
    [property: JsonPropertyName("current_s")] double? CurrentS,
    [property: JsonPropertyName("last_s")] double? LastS,
    [property: JsonPropertyName("best_s")] double? BestS,
    [property: JsonPropertyName("laps")] int Laps);

public sealed record WalkBoxDto(
    [property: JsonPropertyName("name")] string? Name,
    [property: JsonPropertyName("pos")] double[] Pos,
    [property: JsonPropertyName("size")] double[] Size,
    [property: JsonPropertyName("quat")] double[]? Quat,
    [property: JsonPropertyName("rgba")] double[]? Rgba);

public sealed record WalkGeometryDto(
    [property: JsonPropertyName("checkpoints")] List<double[]> Checkpoints,
    [property: JsonPropertyName("checkpoint_radius")] double CheckpointRadius,
    [property: JsonPropertyName("boxes")] List<WalkBoxDto> Boxes);

public sealed record WalkPuppetDto(
    [property: JsonPropertyName("connected")] bool Connected);

/// <summary>GET /walk/state. Seed .. Scenario are the simulation features (docs/sim_features.md): null when no sim
/// exists or the service predates them.</summary>
public sealed record WalkStateDto(
    [property: JsonPropertyName("ok")] bool Ok,
    [property: JsonPropertyName("policy_available")] bool PolicyAvailable,
    [property: JsonPropertyName("policy_reason")] string? PolicyReason,
    [property: JsonPropertyName("running")] bool Running,
    [property: JsonPropertyName("mode")] string? Mode = null,
    [property: JsonPropertyName("course")] string? Course = null,
    [property: JsonPropertyName("courses")] List<string>? Courses = null,
    [property: JsonPropertyName("t")] double? T = null,
    [property: JsonPropertyName("cmd")] double[]? Cmd = null,
    [property: JsonPropertyName("cmd_limits")] Dictionary<string, double[]>? CmdLimits = null,
    [property: JsonPropertyName("base")] WalkBaseDto? Base = null,
    [property: JsonPropertyName("q")] double[]? Q = null,
    [property: JsonPropertyName("joint_names")] List<string>? JointNames = null,
    [property: JsonPropertyName("fallen")] bool Fallen = false,
    [property: JsonPropertyName("falls")] int Falls = 0,
    [property: JsonPropertyName("checkpoint")] WalkCheckpointDto? Checkpoint = null,
    [property: JsonPropertyName("lap")] WalkLapDto? Lap = null,
    [property: JsonPropertyName("geometry")] WalkGeometryDto? Geometry = null,
    [property: JsonPropertyName("puppet")] WalkPuppetDto? Puppet = null,
    [property: JsonPropertyName("error")] string? Error = null,
    [property: JsonPropertyName("seed")] long? Seed = null,
    [property: JsonPropertyName("difficulty")] double? Difficulty = null,
    [property: JsonPropertyName("physics")] WalkPhysicsDto? Physics = null,
    [property: JsonPropertyName("motors")] WalkMotorsDto? Motors = null,
    [property: JsonPropertyName("thermal")] WalkThermalDto? Thermal = null,
    [property: JsonPropertyName("battery")] WalkBatteryDto? Battery = null,
    [property: JsonPropertyName("action_filter")] WalkActionFilterDto? ActionFilter = null,
    [property: JsonPropertyName("props")] List<WalkPropDto>? Props = null,
    [property: JsonPropertyName("fall_analysis")] WalkFallAnalysisDto? FallAnalysis = null,
    [property: JsonPropertyName("falls_log")] List<WalkFallLogDto>? FallsLog = null,
    [property: JsonPropertyName("replay")] WalkReplayStateDto? Replay = null,
    [property: JsonPropertyName("scenario")] WalkScenarioStateDto? Scenario = null);
