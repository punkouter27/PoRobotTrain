using System.Text.Json;
using PoRobotTrain.Shared.Contracts;
using Xunit;

namespace PoRobotTrain.Integration;

/// <summary>
/// The Blazor client deserialises the behaviour service's console replies with the source-generated
/// <see cref="RobotJsonContext"/>. console_samples.json holds real replies captured from
/// robotcontrol.behaviours.service (local interface names scrubbed); a field type that does not match the
/// service (for example an int where the service sends 34.0) would make the page silently show nothing.
/// </summary>
public sealed class ConsoleContractTests
{
    private static readonly JsonElement Samples = JsonDocument.Parse(
        File.ReadAllText(Path.Combine(AppContext.BaseDirectory, "console_samples.json"))).RootElement;

    public static TheoryData<string> Names()
    {
        var data = new TheoryData<string>();
        foreach (var p in JsonDocument.Parse(File.ReadAllText(Path.Combine(AppContext.BaseDirectory, "console_samples.json"))).RootElement.EnumerateObject())
        {
            data.Add(p.Name);
        }
        return data;
    }

    [Theory]
    [MemberData(nameof(Names))]
    public void ServiceSampleDeserialisesWithTheClientContext(string typeName)
    {
        var type = typeof(RobotStatusDto).Assembly.GetType($"PoRobotTrain.Shared.Contracts.{typeName}")
            ?? throw new InvalidOperationException($"no contract type {typeName}");
        var info = RobotJsonContext.Default.GetTypeInfo(type) ?? throw new InvalidOperationException($"{typeName} is not in RobotJsonContext");
        var value = JsonSerializer.Deserialize(Samples.GetProperty(typeName).GetRawText(), info);
        Assert.NotNull(value);
    }

    [Fact]
    public void TemperatureHistoryKeepsItsSeries()
    {
        var t = JsonSerializer.Deserialize(Samples.GetProperty("TempHistoryDto").GetRawText(), RobotJsonContext.Default.TempHistoryDto)!;
        Assert.Equal(26, t.JointNames.Count);
        Assert.Equal(26, t.Series.Count);
        Assert.NotNull(t.Latest);
        Assert.NotNull(t.Hottest);
    }

    [Fact]
    public void StatusCarriesTheConsoleBlock()
    {
        var s = JsonSerializer.Deserialize(Samples.GetProperty("RobotStatusDto").GetRawText(), RobotJsonContext.Default.RobotStatusDto)!;
        Assert.NotNull(s.Console?.Limits);
        Assert.Contains("amplitude_cap_deg", s.Console!.Limits!.Keys);
    }
}
