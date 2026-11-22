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
    [property: JsonPropertyName("target")] string Target = "urlab");

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
    [property: JsonPropertyName("max_deg")] double MaxDeg);

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
    [property: JsonPropertyName("console")] ConsoleStatusDto? Console = null);

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
