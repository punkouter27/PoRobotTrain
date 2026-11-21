using System.Text.Json.Serialization;

namespace PoRobotTrain.Shared.Contracts;

[JsonSerializable(typeof(RobotStatusDto))]
[JsonSerializable(typeof(DiagSnapshotDto))]
[JsonSerializable(typeof(BehaviourParamsDto))]
[JsonSerializable(typeof(PreviewRequest))]
[JsonSerializable(typeof(RunRequest))]
[JsonSerializable(typeof(MoveJointRequest))]
[JsonSerializable(typeof(ActionModeRequest))]
[JsonSerializable(typeof(CommandResultDto))]
[JsonSerializable(typeof(EmptyBody))]
public sealed partial class RobotJsonContext : JsonSerializerContext;

public sealed record EmptyBody;
