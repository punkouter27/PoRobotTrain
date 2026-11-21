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

    private async Task<CommandResultDto> PostAsync<TBody>(string path, TBody body, CancellationToken ct)
    {
        using var resp = await http.PostAsJsonAsync(path, body, ct);
        var result = await resp.Content.ReadFromJsonAsync<CommandResultDto>(cancellationToken: ct);
        return result ?? new CommandResultDto(resp.IsSuccessStatusCode, resp.IsSuccessStatusCode ? null : $"HTTP {(int)resp.StatusCode}");
    }
}
