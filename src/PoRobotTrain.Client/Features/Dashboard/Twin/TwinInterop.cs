using System.Text.Json.Serialization;

namespace PoRobotTrain.Client.Features.Dashboard.Twin;

/// <summary>Pose + context pushed to wwwroot/js/twin/twin.js on every status poll and slider move.</summary>
public sealed record TwinState(
    double[]? Actual,
    double[]? Sim,
    int[]? Temps,
    string Mode,
    TwinPreview? Preview,
    string? Job,
    Dictionary<string, TwinJobResult>? Results,
    string? PrimaryJoint,
    bool RealConnected,
    TwinReach? Reach);

public sealed record TwinPreview(int Index, double Deg);

public sealed record TwinJobResult(bool Ok);

public sealed record TwinReach(bool On, string Behaviour, double Amplitude);

/// <summary>Rapier check of the proposed (dragged-slider) pose.</summary>
public sealed record TwinFeedback(
    IReadOnlyList<TwinHit> Self,
    IReadOnlyList<TwinHit> Stand,
    double Limit,
    bool Checked)
{
    public static TwinFeedback Empty { get; } = new([], [], 0, false);

    public bool HasHits => Self.Count > 0 || Stand.Count > 0;
}

public sealed record TwinHit(string A, string B, double DistanceM);

public sealed record TwinInfo(
    string Backend,
    bool FkOk,
    double? FkMaxPosMm,
    Dictionary<string, string>? Extras,
    long GlbBytes,
    bool Draco);

public sealed record TwinProbeRow(string Section, string Name, bool Ok, double Ms, string? Detail);

public sealed record AssistantSuggestion(
    string Kind,
    string? Id,
    string? Label,
    double? Amplitude,
    string? Joint,
    double? Deg,
    double? Score,
    bool Blocked,
    string? Note);

public sealed record AssistantBehaviour(string Id, string Label, string Desc, double MaxDeg);

public sealed record AssistantJoint(string Name, string Label, string Group, double MinDeg, double MaxDeg);

public sealed record AssistantRequest(
    string Text,
    IReadOnlyList<AssistantBehaviour> Behaviours,
    IReadOnlyList<AssistantJoint> Joints,
    IReadOnlyList<double> Sizes,
    bool ArmUnlocked);

[JsonSourceGenerationOptions(
    PropertyNamingPolicy = JsonKnownNamingPolicy.CamelCase,
    DefaultIgnoreCondition = JsonIgnoreCondition.WhenWritingNull,
    NumberHandling = JsonNumberHandling.AllowNamedFloatingPointLiterals)]
[JsonSerializable(typeof(TwinState))]
[JsonSerializable(typeof(TwinFeedback))]
[JsonSerializable(typeof(TwinInfo))]
[JsonSerializable(typeof(List<TwinProbeRow>))]
[JsonSerializable(typeof(TwinProbeRow))]
[JsonSerializable(typeof(AssistantSuggestion))]
[JsonSerializable(typeof(AssistantRequest))]
internal sealed partial class TwinInteropJsonContext : JsonSerializerContext;
