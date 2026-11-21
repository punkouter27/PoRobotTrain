namespace PoRobotTrain.Api.Features.RobotControl;

public sealed class RobotServiceOptions
{
    public const string SectionName = "RobotControl";
    public string BaseUrl { get; set; } = "http://127.0.0.1:8765";
}
