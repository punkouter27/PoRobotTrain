using System.Text.Json.Serialization;

namespace PoRobotTrain.Shared.Contracts;

public sealed record BehaviourParamsDto(
    [property: JsonPropertyName("behaviour")] string Behaviour = "neck_turn",
    [property: JsonPropertyName("amplitude")] double Amplitude = 10.0,
    [property: JsonPropertyName("freq")] double Freq = 0.2,
    [property: JsonPropertyName("duration")] double Duration = 20.0,
    [property: JsonPropertyName("practice")] bool Practice = false);

public sealed record PreviewRequest(
    [property: JsonPropertyName("behaviour")] string Behaviour = "neck_turn",
    [property: JsonPropertyName("amplitude")] double Amplitude = 10.0,
    [property: JsonPropertyName("freq")] double Freq = 0.2,
    [property: JsonPropertyName("duration")] double Duration = 20.0,
    [property: JsonPropertyName("target")] string Target = "urlab",
    [property: JsonPropertyName("version")] int? Version = null);

public sealed record RunRequest(
    [property: JsonPropertyName("behaviour")] string Behaviour = "neck_turn",
    [property: JsonPropertyName("amplitude")] double Amplitude = 10.0,
    [property: JsonPropertyName("freq")] double Freq = 0.2,
    [property: JsonPropertyName("duration")] double Duration = 20.0,
    [property: JsonPropertyName("confirm")] bool Confirm = false,
    [property: JsonPropertyName("practice")] bool Practice = false);

public sealed record ActionModeRequest(
    [property: JsonPropertyName("practice")] bool Practice = false);

public sealed record MoveJointRequest(
    [property: JsonPropertyName("joint")] string Joint,
    [property: JsonPropertyName("target_deg")] double TargetDeg,
    [property: JsonPropertyName("target")] string Target = "real",
    [property: JsonPropertyName("confirm")] bool Confirm = false);

public sealed record CommandResultDto(
    [property: JsonPropertyName("ok")] bool Ok,
    [property: JsonPropertyName("error")] string? Error = null,
    [property: JsonPropertyName("note")] string? Note = null,
    [property: JsonPropertyName("policy_reason")] string? PolicyReason = null);

public sealed record JobResultDto(
    [property: JsonPropertyName("ok")] bool Ok,
    [property: JsonPropertyName("amplitude")] double Amplitude,
    [property: JsonPropertyName("at")] double At,
    [property: JsonPropertyName("reason")] string Reason);

public sealed record BehaviourInfoDto(
    [property: JsonPropertyName("id")] string Id,
    [property: JsonPropertyName("label")] string Label,
    [property: JsonPropertyName("desc")] string Desc,
    [property: JsonPropertyName("primary_label")] string PrimaryLabel,
    [property: JsonPropertyName("max_deg")] double MaxDeg,
    [property: JsonPropertyName("kind")] string Kind = "scripted",
    [property: JsonPropertyName("ready")] bool Ready = true,
    [property: JsonPropertyName("skill_state")] string? SkillState = null);

/// <summary>A trained head skill (RobotControl/behaviours/head_skills.py): state is untrained | unvalidated | failed | ready.</summary>
public sealed record SkillInfoDto(
    [property: JsonPropertyName("id")] string Id,
    [property: JsonPropertyName("label")] string Label,
    [property: JsonPropertyName("desc")] string Desc,
    [property: JsonPropertyName("powered")] List<string> Powered,
    [property: JsonPropertyName("max_deg")] double MaxDeg,
    [property: JsonPropertyName("state")] string State,
    [property: JsonPropertyName("reason")] string Reason,
    [property: JsonPropertyName("trained_at")] string? TrainedAt = null,
    [property: JsonPropertyName("validated_at")] string? ValidatedAt = null,
    [property: JsonPropertyName("track_err_deg")] double? TrackErrDeg = null,
    [property: JsonPropertyName("nominal_err_deg")] double? NominalErrDeg = null,
    [property: JsonPropertyName("randomised_err_deg")] double? RandomisedErrDeg = null,
    [property: JsonPropertyName("version")] int? Version = null,
    [property: JsonPropertyName("versions")] List<SkillVersionDto>? Versions = null,
    [property: JsonPropertyName("robot")] SkillRobotCopyDto? Robot = null);

/// <summary>One stored version of a trained skill (RobotControl/behaviours/skill_versions.py), newest first.</summary>
public sealed record SkillVersionDto(
    [property: JsonPropertyName("version")] int Version,
    [property: JsonPropertyName("active")] bool Active,
    [property: JsonPropertyName("passed")] bool Passed,
    [property: JsonPropertyName("trained_at")] string? TrainedAt = null,
    [property: JsonPropertyName("track_err_deg")] double? TrackErrDeg = null,
    [property: JsonPropertyName("iters")] int? Iters = null,
    [property: JsonPropertyName("num_envs")] int? NumEnvs = null,
    [property: JsonPropertyName("steps")] long? Steps = null,
    [property: JsonPropertyName("sim_hours")] double? SimHours = null,
    [property: JsonPropertyName("train_time_s")] double? TrainTimeS = null,
    [property: JsonPropertyName("device")] string? Device = null,
    [property: JsonPropertyName("onnx_bytes")] long? OnnxBytes = null);

/// <summary>Which version the robot has (recorded when a copy succeeds); Current = it is the active version.</summary>
public sealed record SkillRobotCopyDto(
    [property: JsonPropertyName("version")] int Version,
    [property: JsonPropertyName("sha256")] string Sha256,
    [property: JsonPropertyName("at")] string At,
    [property: JsonPropertyName("current")] bool Current);

public sealed record SkillRequest([property: JsonPropertyName("behaviour")] string Behaviour);

public sealed record ActivateSkillRequest(
    [property: JsonPropertyName("behaviour")] string Behaviour,
    [property: JsonPropertyName("version")] int Version);

public sealed record LabPlayRequest(
    [property: JsonPropertyName("behaviour")] string Behaviour,
    [property: JsonPropertyName("size")] double Size);

public sealed record LabPlayResultDto(
    [property: JsonPropertyName("ok")] bool Ok,
    [property: JsonPropertyName("error")] string? Error = null,
    [property: JsonPropertyName("seq")] int Seq = 0,
    [property: JsonPropertyName("unreal_running")] bool UnrealRunning = false);

public sealed record LabRequestDto(
    [property: JsonPropertyName("seq")] int Seq,
    [property: JsonPropertyName("behaviour")] string Behaviour,
    [property: JsonPropertyName("size")] double Size);

/// <summary>Web app -> Unreal scenario lab link: the latest play request, what the lab acknowledged, when it last polled.</summary>
public sealed record LabStatusDto(
    [property: JsonPropertyName("request")] LabRequestDto? Request,
    [property: JsonPropertyName("ack_seq")] int AckSeq,
    [property: JsonPropertyName("unreal_seen_s")] double? UnrealSeenS);

public sealed record SkillCurveDto(
    [property: JsonPropertyName("iter")] List<int> Iter,
    [property: JsonPropertyName("track_err_deg")] List<double> TrackErrDeg,
    [property: JsonPropertyName("reward")] List<double> Reward,
    [property: JsonPropertyName("time_s")] List<double> TimeS);

public sealed record GateRunDto(
    [property: JsonPropertyName("mean_err_deg")] double? MeanErrDeg = null,
    [property: JsonPropertyName("max_err_deg")] double? MaxErrDeg = null,
    [property: JsonPropertyName("other_joint_max_deg")] double? OtherJointMaxDeg = null,
    [property: JsonPropertyName("max_torque_nm")] double? MaxTorqueNm = null,
    [property: JsonPropertyName("exit_code")] int? ExitCode = null,
    [property: JsonPropertyName("abort")] string? Abort = null);

public sealed record GateMetricsDto(
    [property: JsonPropertyName("onnx_parity")] double? OnnxParity = null,
    [property: JsonPropertyName("nominal")] GateRunDto? Nominal = null,
    [property: JsonPropertyName("randomised")] GateRunDto? Randomised = null,
    [property: JsonPropertyName("runtime")] GateRunDto? Runtime = null);

public sealed record GateBarsDto(
    [property: JsonPropertyName("nominal")] double Nominal,
    [property: JsonPropertyName("randomised")] double Randomised,
    [property: JsonPropertyName("runtime")] double Runtime,
    [property: JsonPropertyName("other_joints")] double OtherJoints);

public sealed record SkillValidationDto(
    [property: JsonPropertyName("passed")] bool Passed,
    [property: JsonPropertyName("failures")] List<string>? Failures = null,
    [property: JsonPropertyName("validated_at")] string? ValidatedAt = null,
    [property: JsonPropertyName("bars_deg")] GateBarsDto? Bars = null,
    [property: JsonPropertyName("metrics")] GateMetricsDto? Metrics = null);

public sealed record SkillTrainingDto(
    [property: JsonPropertyName("trained_at")] string? TrainedAt = null,
    [property: JsonPropertyName("iters")] int? Iters = null,
    [property: JsonPropertyName("num_envs")] int? NumEnvs = null,
    [property: JsonPropertyName("train_time_s")] double? TrainTimeS = null,
    [property: JsonPropertyName("device")] string? Device = null);

/// <summary>GET /skills/detail: one version's training curve and validation report (the Library's results view).</summary>
public sealed record SkillDetailDto(
    [property: JsonPropertyName("ok")] bool Ok,
    [property: JsonPropertyName("skill")] string? Skill = null,
    [property: JsonPropertyName("version")] int? Version = null,
    [property: JsonPropertyName("curve")] SkillCurveDto? Curve = null,
    [property: JsonPropertyName("validation")] SkillValidationDto? Validation = null,
    [property: JsonPropertyName("training")] SkillTrainingDto? Training = null,
    [property: JsonPropertyName("error")] string? Error = null);

public sealed record TrainSkillRequest(
    [property: JsonPropertyName("behaviour")] string Behaviour,
    [property: JsonPropertyName("iters")] int? Iters = null);

public sealed record HeadReadoutDto(
    [property: JsonPropertyName("t")] double? T,
    [property: JsonPropertyName("phase")] string? Phase,
    [property: JsonPropertyName("yaw")] double? Yaw,
    [property: JsonPropertyName("pitch")] double? Pitch,
    [property: JsonPropertyName("target")] double? Target,
    [property: JsonPropertyName("other")] double? Other,
    [property: JsonPropertyName("source")] string? Source,
    [property: JsonPropertyName("age_s")] double AgeS);

public sealed record TelemetryReadoutDto(
    [property: JsonPropertyName("t")] double? T,
    [property: JsonPropertyName("phase")] string? Phase,
    [property: JsonPropertyName("behaviour")] string? Behaviour,
    [property: JsonPropertyName("primary_joint")] string? PrimaryJoint,
    [property: JsonPropertyName("primary_label")] string? PrimaryLabel,
    [property: JsonPropertyName("actual_deg")] double? ActualDeg,
    [property: JsonPropertyName("target_deg")] double? TargetDeg,
    [property: JsonPropertyName("other_max_deg")] double? OtherMaxDeg,
    [property: JsonPropertyName("q_deg")] double[]? QDeg,
    [property: JsonPropertyName("imu_rpy_deg")] double[]? ImuRpyDeg,
    [property: JsonPropertyName("temps_c")] int[]? TempsC,
    [property: JsonPropertyName("max_temp_c")] int? MaxTempC,
    [property: JsonPropertyName("hot_joint")] string? HotJoint,
    [property: JsonPropertyName("bus_v")] double? BusV,
    [property: JsonPropertyName("mode_machine")] int? ModeMachine,
    [property: JsonPropertyName("active_motors")] int? ActiveMotors,
    [property: JsonPropertyName("source")] string? Source,
    [property: JsonPropertyName("age_s")] double AgeS);

public sealed record SimCalibrationDto(
    [property: JsonPropertyName("control_delay_ms")] double ControlDelayMs,
    [property: JsonPropertyName("lowpass_tau_s")] double LowpassTauS,
    [property: JsonPropertyName("bus_voltage_v")] double BusVoltageV,
    [property: JsonPropertyName("bemf_nm_per_rad_s")] double BemfNmPerRadS,
    [property: JsonPropertyName("pos_noise_deg")] double PosNoiseDeg,
    [property: JsonPropertyName("compliant_stand")] bool CompliantStand,
    [property: JsonPropertyName("body_mass_scale")] double BodyMassScale,
    [property: JsonPropertyName("last_calibrated_at")] string? LastCalibratedAt,
    [property: JsonPropertyName("parity_rms_deg")] double? ParityRmsDeg,
    [property: JsonPropertyName("parity_max_deg")] double? ParityMaxDeg);

public sealed record RobotStatusDto(
    [property: JsonPropertyName("job")] string? Job,
    [property: JsonPropertyName("last_result")] string LastResult,
    [property: JsonPropertyName("previewed")] BehaviourParamsDto? Previewed,
    [property: JsonPropertyName("checked")] BehaviourParamsDto? Checked,
    [property: JsonPropertyName("check_age_s")] double? CheckAgeS,
    [property: JsonPropertyName("can_run_robot")] bool CanRunRobot,
    [property: JsonPropertyName("why_not_run")] string? WhyNotRun,
    [property: JsonPropertyName("mirror_samples")] int MirrorSamples,
    [property: JsonPropertyName("results")] Dictionary<string, JobResultDto> Results,
    [property: JsonPropertyName("head")] Dictionary<string, HeadReadoutDto> Head,
    [property: JsonPropertyName("telemetry")] Dictionary<string, TelemetryReadoutDto> Telemetry,
    [property: JsonPropertyName("behaviours")] List<BehaviourInfoDto> Behaviours,
    [property: JsonPropertyName("joint_names")] List<string> JointNames,
    [property: JsonPropertyName("joint_limits_deg")] List<double[]> JointLimitsDeg,
    [property: JsonPropertyName("joint_defaults_deg")] List<double> JointDefaultsDeg,
    [property: JsonPropertyName("lines")] List<string> Lines,
    [property: JsonPropertyName("calibration")] SimCalibrationDto? Calibration = null,
    [property: JsonPropertyName("real_connected")] bool RealConnected = false,
    [property: JsonPropertyName("progress")] JobProgressDto? Progress = null,
    [property: JsonPropertyName("console")] ConsoleStatusDto? Console = null,
    [property: JsonPropertyName("skills")] List<SkillInfoDto>? Skills = null,
    [property: JsonPropertyName("lab")] LabStatusDto? Lab = null);

// The API writes camelCase (ASP.NET web defaults) and RobotJsonContext reads with exact names, so the names are pinned
// here; without them the Blazor /diag page deserialised every field to its default ("Upstream Reachable: No").
public sealed record DiagSnapshotDto(
    [property: JsonPropertyName("appName")] string AppName,
    [property: JsonPropertyName("buildVersion")] string BuildVersion,
    [property: JsonPropertyName("environment")] string Environment,
    [property: JsonPropertyName("upstreamUrl")] string UpstreamUrl,
    [property: JsonPropertyName("upstreamReachable")] bool UpstreamReachable,
    [property: JsonPropertyName("activeJob")] string? ActiveJob,
    [property: JsonPropertyName("jointCount")] int JointCount);
