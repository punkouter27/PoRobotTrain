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

    // ---- voice intent, robustness explorer, walk sandbox (docs/service_api.md)
    public async Task<AssistantInterpretResponse?> InterpretAsync(AssistantInterpretRequest req, CancellationToken ct = default)
    {
        using var resp = await http.PostAsJsonAsync("/api/robot/assistant/interpret", req, RobotJsonContext.Default.AssistantInterpretRequest, ct);
        return resp.IsSuccessStatusCode
            ? await resp.Content.ReadFromJsonAsync(RobotJsonContext.Default.AssistantInterpretResponse, ct)
            : null;
    }

    /// <summary>Latest sweep report, or null when the API/service is unreachable (HTTP error).</summary>
    public async Task<(RobustnessReportDto? Report, string? Error)> GetRobustnessAsync(CancellationToken ct = default)
    {
        using var resp = await http.GetAsync("/api/robot/robustness", ct);
        if (!resp.IsSuccessStatusCode)
        {
            return (null, await ProblemTextAsync(resp, ct));
        }
        return (await resp.Content.ReadFromJsonAsync(RobotJsonContext.Default.RobustnessReportDto, ct), null);
    }

    public Task<CommandResultDto> RunRobustnessAsync(bool quick, CancellationToken ct = default) =>
        PostAsync("/api/robot/robustness/run", new RobustnessRunRequest(quick), RobotJsonContext.Default.RobustnessRunRequest, ct);

    public async Task<(WalkStateDto? State, string? Error)> GetWalkStateAsync(CancellationToken ct = default)
    {
        using var resp = await http.GetAsync("/api/robot/walk/state", ct);
        if (!resp.IsSuccessStatusCode)
        {
            return (null, await ProblemTextAsync(resp, ct));
        }
        return (await resp.Content.ReadFromJsonAsync(RobotJsonContext.Default.WalkStateDto, ct), null);
    }

    public Task<CommandResultDto> WalkStartAsync(WalkStartRequest req, CancellationToken ct = default) =>
        PostAsync("/api/robot/walk/start", req, RobotJsonContext.Default.WalkStartRequest, ct);

    public Task<CommandResultDto> WalkResetAsync(CancellationToken ct = default) =>
        PostAsync("/api/robot/walk/reset", new EmptyBody(), RobotJsonContext.Default.EmptyBody, ct);

    public Task<CommandResultDto> WalkStopAsync(CancellationToken ct = default) =>
        PostAsync("/api/robot/walk/stop", new EmptyBody(), RobotJsonContext.Default.EmptyBody, ct);

    private static async Task<string> ProblemTextAsync(HttpResponseMessage resp, CancellationToken ct)
    {
        try
        {
            using var doc = System.Text.Json.JsonDocument.Parse(await resp.Content.ReadAsStringAsync(ct));
            if (doc.RootElement.TryGetProperty("detail", out var d) && d.ValueKind == System.Text.Json.JsonValueKind.String)
            {
                return d.GetString()!;
            }
        }
        catch (System.Text.Json.JsonException)
        {
            // not problem+json
        }
        return $"HTTP {(int)resp.StatusCode} ({resp.ReasonPhrase})";
    }

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
