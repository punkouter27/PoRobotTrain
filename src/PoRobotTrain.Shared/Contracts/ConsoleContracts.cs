using System.Text.Json;
using System.Text.Json.Serialization;

namespace PoRobotTrain.Shared.Contracts;

// Operator console (docs/service_api.md sections 4-7). Field names follow the Python behaviour service
// (snake_case). The .NET API forwards these paths unchanged under /api/robot/...; the Blazor UI and the
// Unreal panel implement the same features against the same endpoints.

// ------------------------------------------------------------------------------------------------ status extras

public sealed record ConsolePlayerBriefDto(
    [property: JsonPropertyName("running")] bool? Running = null,
    [property: JsonPropertyName("kind")] string? Kind = null,
    [property: JsonPropertyName("name")] string? Name = null,
    [property: JsonPropertyName("t")] double? T = null);

public sealed record RecordingStatusDto(
    [property: JsonPropertyName("active")] bool Active = false,
    [property: JsonPropertyName("source")] string? Source = null,
    [property: JsonPropertyName("samples")] int Samples = 0,
    [property: JsonPropertyName("elapsed_s")] double ElapsedS = 0);

public sealed record PreflightBriefDto(
    [property: JsonPropertyName("complete")] bool Complete = false,
    [property: JsonPropertyName("open")] int Open = 0,
    [property: JsonPropertyName("reason")] string? Reason = null);

public sealed record ConsoleStatusDto(
    [property: JsonPropertyName("limits")] Dictionary<string, double>? Limits = null,
    [property: JsonPropertyName("limits_tightened")] List<string>? LimitsTightened = null,
    [property: JsonPropertyName("event_counts")] Dictionary<string, int>? EventCounts = null,
    [property: JsonPropertyName("recording")] RecordingStatusDto? Recording = null,
    [property: JsonPropertyName("player")] ConsolePlayerBriefDto? Player = null,
    [property: JsonPropertyName("preflight")] PreflightBriefDto? Preflight = null);

// ------------------------------------------------------------------------------------------------ 4. monitoring

public sealed record HealthRowDto(
    [property: JsonPropertyName("section")] string Section,
    [property: JsonPropertyName("name")] string Name,
    [property: JsonPropertyName("ok")] bool Ok,
    [property: JsonPropertyName("optional")] bool Optional,
    [property: JsonPropertyName("ms")] double Ms,
    [property: JsonPropertyName("detail")] string? Detail);

public sealed record HealthDto(
    [property: JsonPropertyName("ok")] bool Ok,
    [property: JsonPropertyName("overall")] string Overall,
    [property: JsonPropertyName("failing")] int Failing,
    [property: JsonPropertyName("rows")] List<HealthRowDto> Rows,
    [property: JsonPropertyName("checked_at")] double CheckedAt);

public sealed record EventDto(
    [property: JsonPropertyName("id")] int Id,
    [property: JsonPropertyName("at")] double At,
    [property: JsonPropertyName("level")] string Level,
    [property: JsonPropertyName("kind")] string Kind,
    [property: JsonPropertyName("message")] string Message,
    [property: JsonPropertyName("source")] string? Source = null,
    [property: JsonPropertyName("joint")] string? Joint = null);

public sealed record EventsDto(
    [property: JsonPropertyName("ok")] bool Ok,
    [property: JsonPropertyName("events")] List<EventDto> Events,
    [property: JsonPropertyName("counts")] Dictionary<string, int> Counts);

public sealed record HottestDto(
    [property: JsonPropertyName("joint")] string Joint,
    [property: JsonPropertyName("temp_c")] int TempC);

public sealed record TempHistoryDto(
    [property: JsonPropertyName("ok")] bool Ok,
    [property: JsonPropertyName("source")] string Source,
    [property: JsonPropertyName("warn_c")] int WarnC,
    [property: JsonPropertyName("crit_c")] int CritC,
    [property: JsonPropertyName("joint_names")] List<string> JointNames,
    [property: JsonPropertyName("t")] List<double> T,
    [property: JsonPropertyName("series")] List<List<int>> Series,
    [property: JsonPropertyName("latest")] List<int>? Latest = null,
    [property: JsonPropertyName("peak")] List<int>? Peak = null,
    [property: JsonPropertyName("rate_c_per_min")] List<double?>? RateCPerMin = null,
    [property: JsonPropertyName("hottest")] HottestDto? Hottest = null);

public sealed record PowerDto(
    [property: JsonPropertyName("ok")] bool Ok,
    [property: JsonPropertyName("source")] string Source,
    [property: JsonPropertyName("available")] bool Available,
    [property: JsonPropertyName("latest_v")] double? LatestV,
    [property: JsonPropertyName("soc_pct")] double? SocPct,
    [property: JsonPropertyName("minutes_left")] double? MinutesLeft,
    [property: JsonPropertyName("series_cells")] int SeriesCells,
    [property: JsonPropertyName("assumption")] string Assumption,
    [property: JsonPropertyName("current_a")] double? CurrentA,
    [property: JsonPropertyName("current_note")] string? CurrentNote,
    [property: JsonPropertyName("t")] List<double> T,
    [property: JsonPropertyName("bus_v")] List<double> BusV,
    [property: JsonPropertyName("cell_v")] double? CellV = null,
    [property: JsonPropertyName("trend_v_per_min")] double? TrendVPerMin = null,
    [property: JsonPropertyName("low")] bool Low = false);

public sealed record ImuCheckDto(
    [property: JsonPropertyName("ok")] bool Ok,
    [property: JsonPropertyName("source")] string Source,
    [property: JsonPropertyName("available")] bool Available,
    [property: JsonPropertyName("verdict")] string Verdict,
    [property: JsonPropertyName("samples")] int Samples,
    [property: JsonPropertyName("notes")] List<string> Notes,
    [property: JsonPropertyName("window_s")] double? WindowS = null,
    [property: JsonPropertyName("rpy_deg")] List<double>? RpyDeg = null,
    [property: JsonPropertyName("gravity_body")] List<double>? GravityBody = null,
    [property: JsonPropertyName("tilt_deg")] double? TiltDeg = null,
    [property: JsonPropertyName("roll_std_deg")] double? RollStdDeg = null,
    [property: JsonPropertyName("pitch_std_deg")] double? PitchStdDeg = null,
    [property: JsonPropertyName("yaw_drift_deg_per_min")] double? YawDriftDegPerMin = null,
    [property: JsonPropertyName("still")] bool? Still = null);

public sealed record PolicyVersionDto(
    [property: JsonPropertyName("name")] string Name,
    [property: JsonPropertyName("policy_sha256")] string PolicySha256,
    [property: JsonPropertyName("expected_sha256")] string? ExpectedSha256,
    [property: JsonPropertyName("gate")] string Gate,
    [property: JsonPropertyName("ok")] bool Ok,
    [property: JsonPropertyName("trained_at")] string? TrainedAt,
    [property: JsonPropertyName("num_obs")] int? NumObs,
    [property: JsonPropertyName("num_actions")] int? NumActions,
    [property: JsonPropertyName("joint_map_sha256")] string? JointMapSha256,
    [property: JsonPropertyName("joint_map_matches")] bool? JointMapMatches);

public sealed record RepoVersionDto(
    [property: JsonPropertyName("commit")] string? Commit,
    [property: JsonPropertyName("dirty_files")] int? DirtyFiles);

public sealed record RobotVersionDto(
    [property: JsonPropertyName("firmware")] string? Firmware,
    [property: JsonPropertyName("note")] string? Note);

public sealed record VersionsDto(
    [property: JsonPropertyName("ok")] bool Ok,
    [property: JsonPropertyName("service")] string Service,
    [property: JsonPropertyName("repo")] RepoVersionDto Repo,
    [property: JsonPropertyName("python")] string Python,
    [property: JsonPropertyName("packages")] Dictionary<string, string> Packages,
    [property: JsonPropertyName("robot")] RobotVersionDto Robot,
    [property: JsonPropertyName("policies")] List<PolicyVersionDto> Policies,
    [property: JsonPropertyName("warnings")] List<string> Warnings);

public sealed record EstopResultDto(
    [property: JsonPropertyName("ok")] bool Ok,
    [property: JsonPropertyName("stopped")] List<string>? Stopped = null,
    [property: JsonPropertyName("note")] string? Note = null,
    [property: JsonPropertyName("error")] string? Error = null);

// ------------------------------------------------------------------------------------------------ 5. control + motions

public sealed record LimitSpecDto(
    [property: JsonPropertyName("max")] double Max,
    [property: JsonPropertyName("min")] double Min,
    [property: JsonPropertyName("unit")] string Unit,
    [property: JsonPropertyName("label")] string Label);

public sealed record LimitsDto(
    [property: JsonPropertyName("ok")] bool Ok,
    [property: JsonPropertyName("values")] Dictionary<string, double>? Values = null,
    [property: JsonPropertyName("specs")] Dictionary<string, LimitSpecDto>? Specs = null,
    [property: JsonPropertyName("tightened")] List<string>? Tightened = null,
    [property: JsonPropertyName("changed_at")] double? ChangedAt = null,
    [property: JsonPropertyName("error")] string? Error = null);

public sealed record LimitsRequest(
    [property: JsonPropertyName("values")] Dictionary<string, double> Values);

public sealed record ModeRequest(
    [property: JsonPropertyName("mode")] string Mode,
    [property: JsonPropertyName("target")] string Target = "practice",
    [property: JsonPropertyName("joints")] List<string>? Joints = null,
    [property: JsonPropertyName("max_s")] double MaxS = 120);

public sealed record PoseDto(
    [property: JsonPropertyName("name")] string Name,
    [property: JsonPropertyName("label")] string Label,
    [property: JsonPropertyName("say")] string? Say,
    [property: JsonPropertyName("words")] List<string>? Words,
    [property: JsonPropertyName("q_deg")] Dictionary<string, double> QDeg);

public sealed record PosesDto(
    [property: JsonPropertyName("ok")] bool Ok,
    [property: JsonPropertyName("poses")] List<PoseDto> Poses);

public sealed record NameTargetRequest(
    [property: JsonPropertyName("name")] string Name,
    [property: JsonPropertyName("target")] string Target = "practice",
    [property: JsonPropertyName("loop")] bool Loop = false);

public sealed record KeyframeDto(
    [property: JsonPropertyName("t")] double T,
    [property: JsonPropertyName("q_deg")] Dictionary<string, double> QDeg,
    [property: JsonPropertyName("ease")] string? Ease = null);

public sealed record MotionClipDto(
    [property: JsonPropertyName("name")] string Name,
    [property: JsonPropertyName("keyframes")] List<KeyframeDto> Keyframes,
    [property: JsonPropertyName("title")] string? Title = null,
    [property: JsonPropertyName("description")] string? Description = null,
    [property: JsonPropertyName("author")] string? Author = null,
    [property: JsonPropertyName("tags")] List<string>? Tags = null,
    [property: JsonPropertyName("loop")] bool Loop = false,
    [property: JsonPropertyName("source")] string? Source = null);

public sealed record ValidationIssueDto(
    [property: JsonPropertyName("level")] string Level,
    [property: JsonPropertyName("message")] string Message,
    [property: JsonPropertyName("keyframe")] int? Keyframe = null,
    [property: JsonPropertyName("joint")] string? Joint = null);

public sealed record MotionValidationDto(
    [property: JsonPropertyName("ok")] bool Ok,
    [property: JsonPropertyName("issues")] List<ValidationIssueDto> Issues,
    [property: JsonPropertyName("duration_s")] double DurationS,
    [property: JsonPropertyName("joints")] List<string> Joints,
    [property: JsonPropertyName("max_vel_deg_s")] Dictionary<string, double>? MaxVelDegS = null,
    [property: JsonPropertyName("real_eligible")] bool RealEligible = false,
    [property: JsonPropertyName("real_reasons")] List<string>? RealReasons = null);

public sealed record MotionSummaryDto(
    [property: JsonPropertyName("name")] string Name,
    [property: JsonPropertyName("title")] string Title,
    [property: JsonPropertyName("description")] string? Description,
    [property: JsonPropertyName("author")] string? Author,
    [property: JsonPropertyName("tags")] List<string>? Tags,
    [property: JsonPropertyName("loop")] bool Loop,
    [property: JsonPropertyName("source")] string Source,
    [property: JsonPropertyName("keyframes")] int Keyframes,
    [property: JsonPropertyName("duration_s")] double DurationS,
    [property: JsonPropertyName("joints")] List<string> Joints,
    [property: JsonPropertyName("valid")] bool Valid,
    [property: JsonPropertyName("issues")] List<string> Issues,
    [property: JsonPropertyName("sha256")] string Sha256);

public sealed record MotionsDto(
    [property: JsonPropertyName("ok")] bool Ok,
    [property: JsonPropertyName("motions")] List<MotionSummaryDto> Motions);

public sealed record MotionDetailDto(
    [property: JsonPropertyName("ok")] bool Ok,
    [property: JsonPropertyName("clip")] MotionClipDto? Clip = null,
    [property: JsonPropertyName("validation")] MotionValidationDto? Validation = null,
    [property: JsonPropertyName("error")] string? Error = null);

public sealed record ClipRequest(
    [property: JsonPropertyName("clip")] MotionClipDto Clip,
    [property: JsonPropertyName("overwrite")] bool Overwrite = false,
    [property: JsonPropertyName("target")] string Target = "practice",
    [property: JsonPropertyName("loop")] bool Loop = false);

/// <summary>Reply of save / play / check / record-stop / import: ok + optional validation and clip.</summary>
public sealed record MotionResultDto(
    [property: JsonPropertyName("ok")] bool Ok,
    [property: JsonPropertyName("error")] string? Error = null,
    [property: JsonPropertyName("name")] string? Name = null,
    [property: JsonPropertyName("validation")] MotionValidationDto? Validation = null,
    [property: JsonPropertyName("clip")] MotionClipDto? Clip = null,
    [property: JsonPropertyName("duration_s")] double? DurationS = null,
    [property: JsonPropertyName("target")] string? Target = null,
    [property: JsonPropertyName("sha256")] string? Sha256 = null);

public sealed record RecordStartRequest(
    [property: JsonPropertyName("source")] string Source);

public sealed record RecordStopRequest(
    [property: JsonPropertyName("name")] string Name);

public sealed record NameRequest(
    [property: JsonPropertyName("name")] string Name);

public sealed record PlayerStateDto(
    [property: JsonPropertyName("ok")] bool Ok,
    [property: JsonPropertyName("running")] bool Running,
    [property: JsonPropertyName("kind")] string? Kind = null,
    [property: JsonPropertyName("job")] string? Job = null,
    [property: JsonPropertyName("name")] string? Name = null,
    [property: JsonPropertyName("t")] double? T = null,
    [property: JsonPropertyName("duration_s")] double? DurationS = null,
    [property: JsonPropertyName("loop")] bool? Loop = null,
    [property: JsonPropertyName("mode")] string? Mode = null,
    [property: JsonPropertyName("joints")] List<string>? Joints = null,
    [property: JsonPropertyName("reason")] string? Reason = null,
    [property: JsonPropertyName("recording")] RecordingStatusDto? Recording = null);

/// <summary>Store package (porobottrain.motion/1). Kept as raw JSON so the checksum survives a round trip.</summary>
public sealed record PackageImportRequest(
    [property: JsonPropertyName("package")] JsonElement Package,
    [property: JsonPropertyName("overwrite")] bool Overwrite = false);

// ------------------------------------------------------------------------------------------------ 6. setup

public sealed record OffsetRowDto(
    [property: JsonPropertyName("joint")] string Joint,
    [property: JsonPropertyName("measured_deg")] double MeasuredDeg,
    [property: JsonPropertyName("reference_deg")] double ReferenceDeg,
    [property: JsonPropertyName("offset_deg")] double OffsetDeg,
    [property: JsonPropertyName("verdict")] string Verdict,
    [property: JsonPropertyName("twin_offset_deg")] double? TwinOffsetDeg);

public sealed record CalibrationCaptureDto(
    [property: JsonPropertyName("ok")] bool Ok,
    [property: JsonPropertyName("error")] string? Error = null,
    [property: JsonPropertyName("source")] string? Source = null,
    [property: JsonPropertyName("reference")] string? Reference = null,
    [property: JsonPropertyName("captured_at")] double? CapturedAt = null,
    [property: JsonPropertyName("rows")] List<OffsetRowDto>? Rows = null);

public sealed record CalibrationSavedDto(
    [property: JsonPropertyName("ok")] bool Ok = true,
    [property: JsonPropertyName("error")] string? Error = null,
    [property: JsonPropertyName("saved_at")] string? SavedAt = null,
    [property: JsonPropertyName("source")] string? Source = null,
    [property: JsonPropertyName("reference")] string? Reference = null,
    [property: JsonPropertyName("offsets_deg")] Dictionary<string, double>? OffsetsDeg = null,
    [property: JsonPropertyName("applied")] bool Applied = false,
    [property: JsonPropertyName("note")] string? Note = null);

public sealed record CalibrationOffsetsDto(
    [property: JsonPropertyName("ok")] bool Ok,
    [property: JsonPropertyName("saved")] CalibrationSavedDto? Saved,
    [property: JsonPropertyName("pending")] CalibrationCaptureDto? Pending);

public sealed record CalibrationCaptureRequest(
    [property: JsonPropertyName("source")] string Source,
    [property: JsonPropertyName("reference")] string Reference = "zero",
    [property: JsonPropertyName("joints")] List<string>? Joints = null);

public sealed record CalibrationSaveRequest(
    [property: JsonPropertyName("confirm")] bool Confirm,
    [property: JsonPropertyName("offsets")] Dictionary<string, double>? Offsets = null);

public sealed record NetInterfaceDto(
    [property: JsonPropertyName("name")] string Name,
    [property: JsonPropertyName("ipv4")] List<string> Ipv4);

public sealed record ConnectionProfileDto(
    [property: JsonPropertyName("name")] string Name,
    [property: JsonPropertyName("target")] string Target,
    [property: JsonPropertyName("host")] string Host,
    [property: JsonPropertyName("port")] int Port,
    [property: JsonPropertyName("interface")] string? Interface = null,
    [property: JsonPropertyName("note")] string? Note = null);

public sealed record NetworkDto(
    [property: JsonPropertyName("ok")] bool Ok,
    [property: JsonPropertyName("profiles")] List<ConnectionProfileDto> Profiles,
    [property: JsonPropertyName("active")] string? Active,
    [property: JsonPropertyName("interfaces")] List<NetInterfaceDto>? Interfaces = null,
    [property: JsonPropertyName("error")] string? Error = null);

public sealed record NetworkTestRequest(
    [property: JsonPropertyName("host")] string Host,
    [property: JsonPropertyName("port")] int Port,
    [property: JsonPropertyName("icmp")] bool Icmp = true);

public sealed record NetworkTestDto(
    [property: JsonPropertyName("ok")] bool Ok,
    [property: JsonPropertyName("host")] string? Host = null,
    [property: JsonPropertyName("port")] int? Port = null,
    [property: JsonPropertyName("reachable")] bool Reachable = false,
    [property: JsonPropertyName("tcp_ok")] bool TcpOk = false,
    [property: JsonPropertyName("tcp_ms")] double? TcpMs = null,
    [property: JsonPropertyName("ping_ok")] bool? PingOk = null,
    [property: JsonPropertyName("ping_ms")] double? PingMs = null,
    [property: JsonPropertyName("detail")] string? Detail = null,
    [property: JsonPropertyName("error")] string? Error = null);

public sealed record ProfileRequest(
    [property: JsonPropertyName("profile")] ConnectionProfileDto Profile);

public sealed record PreflightItemDto(
    [property: JsonPropertyName("id")] string Id,
    [property: JsonPropertyName("label")] string Label,
    [property: JsonPropertyName("kind")] string Kind,
    [property: JsonPropertyName("ok")] bool Ok,
    [property: JsonPropertyName("detail")] string? Detail);

public sealed record PreflightDto(
    [property: JsonPropertyName("ok")] bool Ok,
    [property: JsonPropertyName("complete")] bool Complete,
    [property: JsonPropertyName("items")] List<PreflightItemDto> Items,
    [property: JsonPropertyName("missing")] List<string> Missing,
    [property: JsonPropertyName("valid_for_s")] double? ValidForS = null);

public sealed record PreflightTickRequest(
    [property: JsonPropertyName("item")] string Item,
    [property: JsonPropertyName("checked")] bool Checked);

// ------------------------------------------------------------------------------------------------ 7. help + chat

public sealed record DocInfoDto(
    [property: JsonPropertyName("name")] string Name,
    [property: JsonPropertyName("title")] string Title,
    [property: JsonPropertyName("bytes")] long Bytes);

public sealed record DocsDto(
    [property: JsonPropertyName("ok")] bool Ok,
    [property: JsonPropertyName("docs")] List<DocInfoDto> Docs);

public sealed record DocTextDto(
    [property: JsonPropertyName("ok")] bool Ok,
    [property: JsonPropertyName("name")] string? Name = null,
    [property: JsonPropertyName("title")] string? Title = null,
    [property: JsonPropertyName("markdown")] string? Markdown = null,
    [property: JsonPropertyName("error")] string? Error = null);

public sealed record ChatRequest(
    [property: JsonPropertyName("text")] string Text,
    [property: JsonPropertyName("session")] string Session = "blazor",
    [property: JsonPropertyName("arm_unlocked")] bool ArmUnlocked = false,
    [property: JsonPropertyName("walk_active")] bool WalkActive = false);

/// <summary>Superset of <see cref="AssistantInterpretResponse"/>: adds answer / pose / session memory.</summary>
public sealed record ChatResponseDto(
    [property: JsonPropertyName("ok")] bool Ok,
    [property: JsonPropertyName("kind")] string Kind,
    [property: JsonPropertyName("id")] string? Id = null,
    [property: JsonPropertyName("label")] string? Label = null,
    [property: JsonPropertyName("amplitude")] double? Amplitude = null,
    [property: JsonPropertyName("joint")] string? Joint = null,
    [property: JsonPropertyName("deg")] double? Deg = null,
    [property: JsonPropertyName("walk")] WalkIntentDto? Walk = null,
    [property: JsonPropertyName("blocked")] bool Blocked = false,
    [property: JsonPropertyName("note")] string? Note = null,
    [property: JsonPropertyName("say")] string? Say = null,
    [property: JsonPropertyName("answer")] string? Answer = null,
    [property: JsonPropertyName("pose")] string? Pose = null,
    [property: JsonPropertyName("session")] string? Session = null,
    [property: JsonPropertyName("turns")] int Turns = 0);

public sealed record SessionRequest(
    [property: JsonPropertyName("session")] string Session);
