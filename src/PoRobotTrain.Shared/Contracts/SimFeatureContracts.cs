using System.Text.Json;
using System.Text.Json.Serialization;

namespace PoRobotTrain.Shared.Contracts;

// Simulation-only features (docs/sim_features.md): walk sandbox physics / motors / props / replay / action filter /
// scenarios / fall analysis, and the skill tools (duel, gap check, montage, taught poses). Field names follow the
// Python behaviour service (snake_case). Nothing here reaches the physical robot.

// ------------------------------------------------------------------------------------- walk: requests

/// <summary>#9: throw a ball or box at the robot. speed 1..8 m/s; from_deg = where it comes FROM (0 front, 90 left).</summary>
public sealed record WalkThrowRequest(
    [property: JsonPropertyName("kind")] string Kind = "ball",
    [property: JsonPropertyName("speed")] double Speed = 4.0,
    [property: JsonPropertyName("from_deg")] double FromDeg = 0.0);

/// <summary>#12: start a replay (seconds + speed), stop it, or arm the automatic replay after a fall. Null fields are
/// left out of the body, so each call sends only what it means.</summary>
public sealed record WalkReplayRequest(
    [property: JsonPropertyName("seconds"), JsonIgnore(Condition = JsonIgnoreCondition.WhenWritingNull)] double? Seconds = null,
    [property: JsonPropertyName("speed"), JsonIgnore(Condition = JsonIgnoreCondition.WhenWritingNull)] double? Speed = null,
    [property: JsonPropertyName("stop"), JsonIgnore(Condition = JsonIgnoreCondition.WhenWritingNull)] bool? Stop = null,
    [property: JsonPropertyName("auto_on_fall"), JsonIgnore(Condition = JsonIgnoreCondition.WhenWritingNull)] bool? AutoOnFall = null);

/// <summary>#15: low-pass (EMA) action filter; apply = actually feed the filtered action to the walk sim.</summary>
public sealed record WalkFilterRequest(
    [property: JsonPropertyName("alpha")] double Alpha = 0.5,
    [property: JsonPropertyName("apply")] bool Apply = false);

/// <summary>#18: play a walk scenario script by id, or stop the running script.</summary>
public sealed record WalkScenarioRequest(
    [property: JsonPropertyName("id"), JsonIgnore(Condition = JsonIgnoreCondition.WhenWritingNull)] string? Id = null,
    [property: JsonPropertyName("stop"), JsonIgnore(Condition = JsonIgnoreCondition.WhenWritingNull)] bool? Stop = null);

public sealed record ScenarioInfoDto(
    [property: JsonPropertyName("id")] string Id,
    [property: JsonPropertyName("name")] string? Name = null,
    [property: JsonPropertyName("target")] string? Target = null,
    [property: JsonPropertyName("description")] string? Description = null,
    [property: JsonPropertyName("duration_s")] double? DurationS = null);

public sealed record ScenariosDto(
    [property: JsonPropertyName("ok")] bool Ok,
    [property: JsonPropertyName("scenarios")] List<ScenarioInfoDto>? Scenarios = null,
    [property: JsonPropertyName("error")] string? Error = null);

// ------------------------------------------------------------------------------------- walk: state additions

/// <summary>A robot contact: world position (m), force on the robot (N, world), geom/body names.</summary>
public sealed record WalkContactDto(
    [property: JsonPropertyName("pos")] double[] Pos,
    [property: JsonPropertyName("force")] double[]? Force = null,
    [property: JsonPropertyName("a")] string? A = null,
    [property: JsonPropertyName("b")] string? B = null);

/// <summary>#2: centre of mass, capture point, support polygon (convex hull of foot contacts), contact forces.</summary>
public sealed record WalkPhysicsDto(
    [property: JsonPropertyName("com")] double[]? Com = null,
    [property: JsonPropertyName("com_vel")] double[]? ComVel = null,
    [property: JsonPropertyName("capture_point")] double[]? CapturePoint = null,
    [property: JsonPropertyName("support")] List<double[]>? Support = null,
    [property: JsonPropertyName("cp_margin_m")] double? CpMarginM = null,
    [property: JsonPropertyName("contacts")] List<WalkContactDto>? Contacts = null);

/// <summary>#5: per joint speed (rad/s), torque (Nm) and torque / effort limit.</summary>
public sealed record WalkMotorsDto(
    [property: JsonPropertyName("names")] List<string>? Names = null,
    [property: JsonPropertyName("speed")] double[]? Speed = null,
    [property: JsonPropertyName("torque")] double[]? Torque = null,
    [property: JsonPropertyName("torque_frac")] double[]? TorqueFrac = null);

/// <summary>#5: modelled winding temperatures (display only, never derates a motor).</summary>
public sealed record WalkThermalDto(
    [property: JsonPropertyName("temp_c")] double[]? TempC = null,
    [property: JsonPropertyName("hottest")] string? Hottest = null,
    [property: JsonPropertyName("hottest_c")] double? HottestC = null,
    [property: JsonPropertyName("warn")] List<string>? Warn = null,
    [property: JsonPropertyName("limit")] List<string>? Limit = null);

/// <summary>#5: battery model (the capacity is a placeholder assumption, not the R1 datasheet).</summary>
public sealed record WalkBatteryDto(
    [property: JsonPropertyName("soc_pct")] double? SocPct = null,
    [property: JsonPropertyName("power_w")] double? PowerW = null,
    [property: JsonPropertyName("minutes_left")] double? MinutesLeft = null);

/// <summary>#15: raw vs EMA-filtered policy action and the jitter score (RMS of the second difference).</summary>
public sealed record WalkActionFilterDto(
    [property: JsonPropertyName("alpha")] double Alpha = 0.5,
    [property: JsonPropertyName("apply")] bool Apply = false,
    [property: JsonPropertyName("raw")] double[]? Raw = null,
    [property: JsonPropertyName("filtered")] double[]? Filtered = null,
    [property: JsonPropertyName("jitter")] double? Jitter = null);

/// <summary>#9: a thrown object. size = [radius] for a ball, [hx, hy, hz] half sizes for a box; quat is wxyz.</summary>
public sealed record WalkPropDto(
    [property: JsonPropertyName("id")] int Id,
    [property: JsonPropertyName("kind")] string Kind,
    [property: JsonPropertyName("pos")] double[] Pos,
    [property: JsonPropertyName("quat")] double[]? Quat = null,
    [property: JsonPropertyName("size")] double[]? Size = null);

public sealed record WalkFallSampleDto(
    [property: JsonPropertyName("t")] double T,
    [property: JsonPropertyName("cp_margin_m")] double? CpMarginM = null,
    [property: JsonPropertyName("tilt_deg")] double? TiltDeg = null,
    [property: JsonPropertyName("height")] double? Height = null);

/// <summary>#19: the last fall. cause: cp_outside | tilt | height | prop_hit.</summary>
public sealed record WalkFallAnalysisDto(
    [property: JsonPropertyName("t_fall")] double TFall,
    [property: JsonPropertyName("t_unstable")] double? TUnstable = null,
    [property: JsonPropertyName("lead_s")] double? LeadS = null,
    [property: JsonPropertyName("reason")] string? Reason = null,
    [property: JsonPropertyName("cause")] string? Cause = null,
    [property: JsonPropertyName("history")] List<WalkFallSampleDto>? History = null);

public sealed record WalkFallLogDto(
    [property: JsonPropertyName("t_fall")] double TFall,
    [property: JsonPropertyName("t_unstable")] double? TUnstable = null,
    [property: JsonPropertyName("lead_s")] double? LeadS = null,
    [property: JsonPropertyName("cause")] string? Cause = null);

/// <summary>#12: while active, the pose fields of /walk/state are the replayed frame.</summary>
public sealed record WalkReplayStateDto(
    [property: JsonPropertyName("active")] bool Active = false,
    [property: JsonPropertyName("speed")] double? Speed = null,
    [property: JsonPropertyName("t")] double? T = null,
    [property: JsonPropertyName("duration_s")] double? DurationS = null,
    [property: JsonPropertyName("auto_on_fall")] bool AutoOnFall = false);

public sealed record WalkScenarioStateDto(
    [property: JsonPropertyName("id")] string Id,
    [property: JsonPropertyName("t")] double? T = null,
    [property: JsonPropertyName("step")] int Step = 0,
    [property: JsonPropertyName("steps")] int Steps = 0,
    [property: JsonPropertyName("done")] bool Done = false);

// ------------------------------------------------------------------------------------- skill tools

/// <summary>#10: a = / b = version number, null = the active version. Both run the same seeded lab scenario.</summary>
public sealed record DuelRequest(
    [property: JsonPropertyName("skill")] string Skill,
    [property: JsonPropertyName("a")] int? A,
    [property: JsonPropertyName("b")] int? B,
    [property: JsonPropertyName("scenario")] string Scenario);

/// <summary>One side of a duel; lower score wins. would_stop = why the runtime safety would stop it ("" = no).</summary>
public sealed record DuelScoreDto(
    [property: JsonPropertyName("version")] int? Version = null,
    [property: JsonPropertyName("err_mean_deg")] double? ErrMeanDeg = null,
    [property: JsonPropertyName("err_max_deg")] double? ErrMaxDeg = null,
    [property: JsonPropertyName("torque_rms")] double? TorqueRms = null,
    [property: JsonPropertyName("jitter")] double? Jitter = null,
    [property: JsonPropertyName("energy_j")] double? EnergyJ = null,
    [property: JsonPropertyName("would_stop")] string? WouldStop = null,
    [property: JsonPropertyName("score")] double? Score = null);

public sealed record DuelResultDto(
    [property: JsonPropertyName("skill")] string? Skill = null,
    [property: JsonPropertyName("scenario")] string? Scenario = null,
    [property: JsonPropertyName("a")] DuelScoreDto? A = null,
    [property: JsonPropertyName("b")] DuelScoreDto? B = null,
    [property: JsonPropertyName("winner")] string? Winner = null,
    [property: JsonPropertyName("why")] string? Why = null);

public sealed record DuelResponseDto(
    [property: JsonPropertyName("ok")] bool Ok,
    [property: JsonPropertyName("result")] DuelResultDto? Result = null,
    [property: JsonPropertyName("error")] string? Error = null);

/// <summary>#4: a logged run (logs/*.csv). target: sim | urlab | real.</summary>
public sealed record RunInfoDto(
    [property: JsonPropertyName("file")] string File,
    [property: JsonPropertyName("skill")] string? Skill = null,
    [property: JsonPropertyName("target")] string? Target = null,
    [property: JsonPropertyName("stamp")] string? Stamp = null);

public sealed record RunsDto(
    [property: JsonPropertyName("ok")] bool Ok,
    [property: JsonPropertyName("runs")] List<RunInfoDto>? Runs = null,
    [property: JsonPropertyName("error")] string? Error = null);

/// <summary>#4: compare two runs. a / b are file names inside logs/ (no paths).</summary>
public sealed record GapRequest(
    [property: JsonPropertyName("a")] string A,
    [property: JsonPropertyName("b")] string B,
    [property: JsonPropertyName("tol_deg")] double TolDeg = 5.0);

/// <summary>#4: a_deg / b_deg / err_deg are [time][joint]; max_deg / rms_deg are per joint.</summary>
public sealed record GapResultDto(
    [property: JsonPropertyName("ok")] bool Ok,
    [property: JsonPropertyName("joints")] List<string>? Joints = null,
    [property: JsonPropertyName("t")] double[]? T = null,
    [property: JsonPropertyName("a_deg")] List<double[]>? ADeg = null,
    [property: JsonPropertyName("b_deg")] List<double[]>? BDeg = null,
    [property: JsonPropertyName("err_deg")] List<double[]>? ErrDeg = null,
    [property: JsonPropertyName("max_deg")] double[]? MaxDeg = null,
    [property: JsonPropertyName("rms_deg")] double[]? RmsDeg = null,
    [property: JsonPropertyName("pass")] bool? Pass = null,
    [property: JsonPropertyName("error")] string? Error = null);

/// <summary>#11: training montage GIF; ok false comes with a reason (no checkpoints: train with --checkpoint-every).</summary>
public sealed record MontageDto(
    [property: JsonPropertyName("ok")] bool Ok,
    [property: JsonPropertyName("frames")] int? Frames = null,
    [property: JsonPropertyName("checkpoints")] List<int>? Checkpoints = null,
    [property: JsonPropertyName("gif_base64")] string? GifBase64 = null,
    [property: JsonPropertyName("reason")] string? Reason = null,
    [property: JsonPropertyName("error")] string? Error = null);

/// <summary>#14: a taught head pose; it trains as skill pose_&lt;id&gt;. The id is kept as raw JSON (number or string)
/// so a delete sends back exactly what the service gave.</summary>
public sealed record TaughtPoseDto(
    [property: JsonPropertyName("id")] JsonElement? Id = null,
    [property: JsonPropertyName("name")] string? Name = null,
    [property: JsonPropertyName("yaw_deg")] double? YawDeg = null,
    [property: JsonPropertyName("pitch_deg")] double? PitchDeg = null,
    [property: JsonPropertyName("skill")] string? Skill = null)
{
    /// <summary>The id as text (for display, keys and the skill name fallback).</summary>
    [JsonIgnore]
    public string IdText => Id is { } e ? e.ValueKind == JsonValueKind.String ? e.GetString() ?? string.Empty : e.GetRawText() : string.Empty;

    /// <summary>A taught pose has an id and both angles; GET /poses may also list the console's preset poses.</summary>
    [JsonIgnore]
    public bool IsTaught => Id is { ValueKind: JsonValueKind.String or JsonValueKind.Number } && YawDeg is not null && PitchDeg is not null;

    /// <summary>The head skill this pose trains as.</summary>
    [JsonIgnore]
    public string SkillId => string.IsNullOrEmpty(Skill) ? "pose_" + IdText : Skill;
}

public sealed record TaughtPosesDto(
    [property: JsonPropertyName("ok")] bool Ok,
    [property: JsonPropertyName("poses")] List<TaughtPoseDto>? Poses = null,
    [property: JsonPropertyName("error")] string? Error = null)
{
    [JsonIgnore]
    public IReadOnlyList<TaughtPoseDto> Taught => (Poses ?? []).Where(p => p.IsTaught).ToList();
}

/// <summary>#14: name 1..32 chars [A-Za-z0-9 _-], yaw -30..30, pitch -15..20 (TaughtPoseRequestValidator).</summary>
public sealed record TaughtPoseRequest(
    [property: JsonPropertyName("name")] string Name,
    [property: JsonPropertyName("yaw_deg")] double YawDeg,
    [property: JsonPropertyName("pitch_deg")] double PitchDeg);

public sealed record TaughtPoseResultDto(
    [property: JsonPropertyName("ok")] bool Ok,
    [property: JsonPropertyName("pose")] TaughtPoseDto? Pose = null,
    [property: JsonPropertyName("error")] string? Error = null);

public sealed record TaughtPoseDeleteRequest(
    [property: JsonPropertyName("id")] JsonElement Id);
