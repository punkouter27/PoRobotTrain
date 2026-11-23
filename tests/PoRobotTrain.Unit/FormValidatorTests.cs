using System.Text.Json;
using PoRobotTrain.Shared.Contracts;
using PoRobotTrain.Shared.Validation;
using Xunit;

namespace PoRobotTrain.Unit;

public sealed class FormValidatorTests
{
    private static readonly JsonSerializerOptions WebJson = new(JsonSerializerDefaults.Web);

    private static readonly List<KeyframeDto> OneKeyframe = [new KeyframeDto(0, new Dictionary<string, double> { ["head_yaw_joint"] = 0 })];

    [Theory]
    [InlineData("wave_hello", true)]
    [InlineData("a", true)]
    [InlineData("Bad Name", false)]
    [InlineData("_lead", false)]
    [InlineData("", false)]
    [InlineData("x23456789012345678901234567890123456789012", false)]
    public void MotionNameFollowsTheServiceRule(string name, bool valid)
    {
        var result = new MotionClipValidator().Validate(new MotionClipDto(name, OneKeyframe));
        Assert.Equal(valid, result.IsValid);
    }

    [Fact]
    public void MotionNeedsAKeyframe()
    {
        var result = new MotionClipValidator().Validate(new MotionClipDto("ok", []));
        Assert.Contains(result.Errors, e => e.PropertyName == "Keyframes");
    }

    [Theory]
    [InlineData("lab", "real", "192.168.123.164", 22, true)]
    [InlineData("lab", "real", "bad host!", 22, false)]
    [InlineData("lab", "real", "10.0.0.2", 70000, false)]
    [InlineData("lab", "moon", "10.0.0.2", 22, false)]
    [InlineData("", "sim", "127.0.0.1", 8765, false)]
    public void ConnectionProfileFollowsTheServiceRule(string name, string target, string host, int port, bool valid)
    {
        var result = new ConnectionProfileValidator().Validate(new ConnectionProfileDto(name, target, host, port));
        Assert.Equal(valid, result.IsValid);
    }

    [Fact]
    public void DiagSnapshotWrittenByTheApiReadsBackInTheClient()
    {
        // the API serialises with ASP.NET web defaults (camelCase); the client reads with RobotJsonContext
        var dto = new DiagSnapshotDto("PoRobotTrain", "261122", "Production", "http://127.0.0.1:8765", true, "snapshot", 26);
        var json = JsonSerializer.Serialize(dto, WebJson);
        Assert.Equal(dto, JsonSerializer.Deserialize(json, RobotJsonContext.Default.DiagSnapshotDto));
    }
}
