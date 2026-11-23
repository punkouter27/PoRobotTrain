using PoRobotTrain.Shared.Contracts;

namespace PoRobotTrain.Api.Features.RobotControl;

public interface IRobotControlClient
{
    Task<RobotStatusDto> GetStatusAsync(CancellationToken ct = default);
    Task<CommandResultDto> PreviewAsync(PreviewRequest request, CancellationToken ct = default);
    Task<CommandResultDto> CheckAsync(BehaviourParamsDto request, CancellationToken ct = default);
    Task<CommandResultDto> MonitorAsync(ActionModeRequest? request = null, CancellationToken ct = default);
    Task<CommandResultDto> SnapshotAsync(ActionModeRequest? request = null, CancellationToken ct = default);
    Task<CommandResultDto> MoveJointAsync(MoveJointRequest request, CancellationToken ct = default);
    Task<CommandResultDto> RunAsync(RunRequest request, CancellationToken ct = default);
    Task<CommandResultDto> StopAsync(CancellationToken ct = default);
    Task<byte[]> GetViewPngAsync(string? mode = null, string? cam = null, int? j = null, double? deg = null, CancellationToken ct = default);

    // voice, robustness explorer, walk sandbox (docs/service_api.md)
    Task<AssistantInterpretResponse> InterpretAsync(AssistantInterpretRequest request, CancellationToken ct = default);
    Task<RobustnessReportDto> GetRobustnessAsync(CancellationToken ct = default);
    Task<CommandResultDto> RunRobustnessAsync(RobustnessRunRequest request, CancellationToken ct = default);
    Task<WalkStateDto> GetWalkStateAsync(CancellationToken ct = default);
    Task<CommandResultDto> WalkStartAsync(WalkStartRequest request, CancellationToken ct = default);
    Task<CommandResultDto> WalkCommandAsync(WalkCommandRequest request, CancellationToken ct = default);
    Task<CommandResultDto> WalkResetAsync(CancellationToken ct = default);
    Task<CommandResultDto> WalkStopAsync(CancellationToken ct = default);

    // simulation features in the walk sandbox (docs/sim_features.md)
    Task<CommandResultDto> WalkThrowAsync(WalkThrowRequest request, CancellationToken ct = default);
    Task<CommandResultDto> WalkReplayAsync(WalkReplayRequest request, CancellationToken ct = default);
    Task<CommandResultDto> WalkFilterAsync(WalkFilterRequest request, CancellationToken ct = default);
    Task<CommandResultDto> WalkScenarioAsync(WalkScenarioRequest request, CancellationToken ct = default);

    // operator console (docs/service_api.md sections 4-7): forwarded unchanged, status code and body included
    Task<ForwardResult> ForwardAsync(HttpMethod method, string pathAndQuery, byte[]? body, CancellationToken ct = default);
}

/// <summary>A forwarded behaviour-service reply: HTTP status, content type and raw body.</summary>
public sealed record ForwardResult(int StatusCode, string ContentType, byte[] Body);
