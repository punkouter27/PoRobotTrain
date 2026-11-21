using System.Text.Json.Serialization;

namespace PoRobotTrain.Client.Features.Operator;

/// <summary>Spec for console.js <c>lineChart</c>: one series per chart (small multiples), dashed reference lines.</summary>
public sealed record ChartSpec(
    [property: JsonPropertyName("xs")] List<double> Xs,
    [property: JsonPropertyName("ys")] List<double> Ys,
    [property: JsonPropertyName("unit")] string Unit,
    [property: JsonPropertyName("xUnit")] string XUnit,
    [property: JsonPropertyName("label")] string Label,
    [property: JsonPropertyName("refs")] List<ChartRef>? Refs = null);

public sealed record ChartRef(
    [property: JsonPropertyName("y")] double Y,
    [property: JsonPropertyName("label")] string Label);

[JsonSerializable(typeof(ChartSpec))]
public sealed partial class ConsoleUiJsonContext : JsonSerializerContext;
