using System.Net.Http.Headers;
using System.Text.Json;
using System.Net.Http.Json;
using PoRobotTrain.Shared.Contracts;

namespace PoRobotTrain.Api.Features.RobotControl;

public sealed class RobotControlHttpClient(HttpClient http) : IRobotControlClient
{
    public async Task<RobotStatusDto> GetStatusAsync(CancellationToken ct = default)
    {
        var dto = await http.GetFromJsonAsync<RobotStatusDto>("/status", ct);
        return dto ?? throw new InvalidOperationException("Python behaviour service returned an empty status payload.");
    }

    public Task<CommandResultDto> PreviewAsync(PreviewRequest request, CancellationToken ct = default) =>
        PostAsync("/preview", request, ct);

    public Task<CommandResultDto> CheckAsync(BehaviourParamsDto request, CancellationToken ct = default) =>
        PostAsync("/check", request, ct);

    public Task<CommandResultDto> MonitorAsync(ActionModeRequest? request = null, CancellationToken ct = default) =>
        PostAsync("/monitor", request ?? new ActionModeRequest(), ct);

    public Task<CommandResultDto> SnapshotAsync(ActionModeRequest? request = null, CancellationToken ct = default) =>
        PostAsync("/snapshot", request ?? new ActionModeRequest(), ct);

    public Task<CommandResultDto> MoveJointAsync(MoveJointRequest request, CancellationToken ct = default) =>
        PostAsync("/move_joint", request, ct);

    public Task<CommandResultDto> RunAsync(RunRequest request, CancellationToken ct = default) =>
        PostAsync("/run", request, ct);

    public Task<CommandResultDto> StopAsync(CancellationToken ct = default) =>
        PostAsync("/stop", new { }, ct);

    public Task<byte[]> GetViewPngAsync(string? mode = null, string? cam = null, int? j = null, double? deg = null, CancellationToken ct = default)
    {
        var qs = $"?mode={Uri.EscapeDataString(mode ?? "real")}&cam={Uri.EscapeDataString(cam ?? "iso")}";
        if (j is int ji && deg is double d)
        {
            qs += $"&j={ji}&deg={d.ToString("0.0", System.Globalization.CultureInfo.InvariantCulture)}";
        }
        return http.GetByteArrayAsync($"/robot_view.png{qs}", ct);
    }

    public async Task<AssistantInterpretResponse> InterpretAsync(AssistantInterpretRequest request, CancellationToken ct = default)
    {
        using var resp = await http.PostAsync("/assistant/interpret", JsonBody(request), ct);
        resp.EnsureSuccessStatusCode(); // the contract always answers 200; anything else is a missing/old service
        var result = await resp.Content.ReadFromJsonAsync<AssistantInterpretResponse>(cancellationToken: ct);
        return result ?? new AssistantInterpretResponse(false, "unknown", Note: $"HTTP {(int)resp.StatusCode}");
    }

    public async Task<RobustnessReportDto> GetRobustnessAsync(CancellationToken ct = default) =>
        await http.GetFromJsonAsync<RobustnessReportDto>("/robustness", ct) ?? new RobustnessReportDto(false);

    public Task<CommandResultDto> RunRobustnessAsync(RobustnessRunRequest request, CancellationToken ct = default) =>
        PostAsync("/robustness/run", request, ct);

    public async Task<WalkStateDto> GetWalkStateAsync(CancellationToken ct = default) =>
        await http.GetFromJsonAsync<WalkStateDto>("/walk/state", ct)
        ?? throw new InvalidOperationException("Python behaviour service returned an empty walk state.");

    public Task<CommandResultDto> WalkStartAsync(WalkStartRequest request, CancellationToken ct = default) =>
        PostAsync("/walk/start", request, ct);

    public Task<CommandResultDto> WalkCommandAsync(WalkCommandRequest request, CancellationToken ct = default) =>
        PostAsync("/walk/command", request, ct);

    public Task<CommandResultDto> WalkResetAsync(CancellationToken ct = default) =>
        PostAsync("/walk/reset", new { }, ct);

    public Task<CommandResultDto> WalkStopAsync(CancellationToken ct = default) =>
        PostAsync("/walk/stop", new { }, ct);

    public Task<CommandResultDto> WalkThrowAsync(WalkThrowRequest request, CancellationToken ct = default) =>
        PostAsync("/walk/throw", request, ct);

    public Task<CommandResultDto> WalkReplayAsync(WalkReplayRequest request, CancellationToken ct = default) =>
        PostAsync("/walk/replay", request, ct);

    public Task<CommandResultDto> WalkFilterAsync(WalkFilterRequest request, CancellationToken ct = default) =>
        PostAsync("/walk/filter", request, ct);

    public Task<CommandResultDto> WalkScenarioAsync(WalkScenarioRequest request, CancellationToken ct = default) =>
        PostAsync("/walk/scenario", request, ct);

    public async Task<ForwardResult> ForwardAsync(HttpMethod method, string pathAndQuery, byte[]? body, CancellationToken ct = default)
    {
        using var req = new HttpRequestMessage(method, pathAndQuery);
        if (method != HttpMethod.Get)
        {
            // buffered, so Content-Length is always set (see JsonBody below)
            req.Content = new ByteArrayContent(body is { Length: > 0 } ? body : "{}"u8.ToArray());
            req.Content.Headers.ContentType = new MediaTypeHeaderValue("application/json") { CharSet = "utf-8" };
        }
        using var resp = await http.SendAsync(req, ct);
        var bytes = await resp.Content.ReadAsByteArrayAsync(ct);
        var type = resp.Content.Headers.ContentType?.ToString() ?? "application/json";
        return new ForwardResult((int)resp.StatusCode, type, bytes);
    }

    // The Python service (BaseHTTPRequestHandler) reads exactly Content-Length bytes and does not decode chunked
    // bodies. PostAsJsonAsync streams JSON with Transfer-Encoding: chunked and no Content-Length, so the service saw
    // an empty body ({}) for every POST: parameters fell back to defaults and "confirm" was dropped. Send a buffered
    // body so the length header is always set.
    internal static ByteArrayContent JsonBody<TBody>(TBody body)
    {
        var content = new ByteArrayContent(JsonSerializer.SerializeToUtf8Bytes(body, JsonOptions));
        content.Headers.ContentType = new MediaTypeHeaderValue("application/json") { CharSet = "utf-8" };
        return content;
    }

    private static readonly JsonSerializerOptions JsonOptions = new(JsonSerializerDefaults.Web);

    private async Task<CommandResultDto> PostAsync<TBody>(string path, TBody body, CancellationToken ct)
    {
        using var resp = await http.PostAsync(path, JsonBody(body), ct);
        var result = await resp.Content.ReadFromJsonAsync<CommandResultDto>(cancellationToken: ct);
        return result ?? new CommandResultDto(resp.IsSuccessStatusCode, resp.IsSuccessStatusCode ? null : $"HTTP {(int)resp.StatusCode}");
    }
}
