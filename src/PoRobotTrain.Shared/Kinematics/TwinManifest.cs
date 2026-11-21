using System.Text.Json.Serialization;

namespace PoRobotTrain.Shared.Kinematics;

/// <summary>
/// Subset of <c>wwwroot/twin/twin_manifest.json</c> written by <c>scripts/build_web_twin.py</c> from the compiled
/// MuJoCo stand model. Lengths in metres, joint angles in degrees, quaternions as [x, y, z, w], MuJoCo frame (z up).
/// </summary>
public sealed record TwinManifestDto(
    int Schema,
    IReadOnlyList<TwinJointDto> Joints,
    IReadOnlyList<TwinBodyDto> Bodies,
    IReadOnlyList<TwinColliderDto> Colliders,
    IReadOnlyList<TwinFkGoldenDto> FkGolden);

public sealed record TwinJointDto(
    string Name,
    int PolicyIndex,
    string Body,
    IReadOnlyList<double> Axis,
    double MinDeg,
    double MaxDeg,
    double DefaultDeg);

public sealed record TwinBodyDto(
    string Name,
    string? Parent,
    IReadOnlyList<double> Pos,
    IReadOnlyList<double> Quat,
    int? Joint);

public sealed record TwinColliderDto(
    string Body,
    string Type,
    IReadOnlyList<double> Size,
    IReadOnlyList<double> Pos,
    IReadOnlyList<double> Quat);

public sealed record TwinPoseDto(IReadOnlyList<double> Pos, IReadOnlyList<double> Quat);

public sealed record TwinFkGoldenDto(IReadOnlyList<double> QDeg, IReadOnlyDictionary<string, TwinPoseDto> Bodies);

[JsonSourceGenerationOptions(PropertyNamingPolicy = JsonKnownNamingPolicy.CamelCase)]
[JsonSerializable(typeof(TwinManifestDto))]
public sealed partial class TwinJsonContext : JsonSerializerContext;
