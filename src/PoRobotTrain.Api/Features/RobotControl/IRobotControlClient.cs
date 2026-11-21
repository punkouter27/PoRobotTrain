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
}
