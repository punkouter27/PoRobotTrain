using System.Net.Http.Json;
using System.Text.Json.Serialization.Metadata;
using PoRobotTrain.Shared.Contracts;

namespace PoRobotTrain.Client.Services;

public sealed class RobotApiClient(HttpClient http)
{
    public Task<RobotStatusDto?> GetStatusAsync(CancellationToken ct = default) =>
        http.GetFromJsonAsync("/api/robot/status", RobotJsonContext.Default.RobotStatusDto, ct);

    public Task<DiagSnapshotDto?> GetDiagAsync(CancellationToken ct = default) =>
        http.GetFromJsonAsync("/api/diag", RobotJsonContext.Default.DiagSnapshotDto, ct);

    public Task<CommandResultDto> PreviewAsync(PreviewRequest req, CancellationToken ct = default) =>
        PostAsync("/api/robot/preview", req, RobotJsonContext.Default.PreviewRequest, ct);

    public Task<CommandResultDto> CheckAsync(BehaviourParamsDto req, CancellationToken ct = default) =>
        PostAsync("/api/robot/check", req, RobotJsonContext.Default.BehaviourParamsDto, ct);

    public Task<CommandResultDto> MonitorAsync(bool practice = false, CancellationToken ct = default) =>
        PostAsync("/api/robot/monitor", new ActionModeRequest(practice), RobotJsonContext.Default.ActionModeRequest, ct);

    public Task<CommandResultDto> SnapshotAsync(bool practice = false, CancellationToken ct = default) =>
        PostAsync("/api/robot/snapshot", new ActionModeRequest(practice), RobotJsonContext.Default.ActionModeRequest, ct);

    public Task<CommandResultDto> MoveJointAsync(MoveJointRequest req, CancellationToken ct = default) =>
        PostAsync("/api/robot/move-joint", req, RobotJsonContext.Default.MoveJointRequest, ct);

    public Task<CommandResultDto> RunAsync(RunRequest req, CancellationToken ct = default) =>
        PostAsync("/api/robot/run", req, RobotJsonContext.Default.RunRequest, ct);

    public Task<CommandResultDto> StopAsync(CancellationToken ct = default) =>
        PostAsync("/api/robot/stop", new EmptyBody(), RobotJsonContext.Default.EmptyBody, ct);

    private async Task<CommandResultDto> PostAsync<T>(string url, T body, JsonTypeInfo<T> jsonType, CancellationToken ct)
    {
        using var resp = await http.PostAsJsonAsync(url, body, jsonType, ct);
        try
        {
            var res = await resp.Content.ReadFromJsonAsync(RobotJsonContext.Default.CommandResultDto, ct);
            if (res is not null)
            {
                return res;
            }
        }
        catch (System.Text.Json.JsonException)
        {
            // Fall back to HTTP status when upstream returned RFC 7807 problem+json
        }

        return new CommandResultDto(resp.IsSuccessStatusCode, resp.IsSuccessStatusCode ? null : $"HTTP {(int)resp.StatusCode} ({resp.ReasonPhrase})");
    }
}
