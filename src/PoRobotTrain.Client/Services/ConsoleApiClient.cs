using System.Net.Http.Json;
using System.Text.Json;
using System.Text.Json.Serialization.Metadata;
using PoRobotTrain.Shared.Contracts;

namespace PoRobotTrain.Client.Services;

/// <summary>Operator console endpoints (docs/service_api.md sections 4-7) through the .NET API at /api/robot/...</summary>
public sealed class ConsoleApiClient(HttpClient http)
{
    private static RobotJsonContext Ctx => RobotJsonContext.Default;

    /// <summary>GET: the parsed body, or null and a plain error (service down, old service, HTTP error).</summary>
    public async Task<(T? Value, string? Error)> GetAsync<T>(string path, JsonTypeInfo<T> type, CancellationToken ct = default)
    {
        try
        {
            using var resp = await http.GetAsync("/api/robot/" + path, ct);
            return await ReadAsync(resp, type, ct);
        }
        catch (HttpRequestException ex)
        {
            return (default, ex.Message);
        }
    }

    /// <summary>POST: the body is parsed even on 4xx, so validation details and refusal reasons reach the page.</summary>
    public async Task<(TRes? Value, string? Error)> PostAsync<TReq, TRes>(string path, TReq body, JsonTypeInfo<TReq> reqType,
        JsonTypeInfo<TRes> resType, CancellationToken ct = default)
    {
        try
        {
            using var resp = await http.PostAsJsonAsync("/api/robot/" + path, body, reqType, ct);
            return await ReadAsync(resp, resType, ct);
        }
        catch (HttpRequestException ex)
        {
            return (default, ex.Message);
        }
    }

    private static async Task<(T? Value, string? Error)> ReadAsync<T>(HttpResponseMessage resp, JsonTypeInfo<T> type, CancellationToken ct)
    {
        var text = await resp.Content.ReadAsStringAsync(ct);
        string? error = null;
        try
        {
            using var doc = JsonDocument.Parse(text);
            var root = doc.RootElement;
            if (root.ValueKind == JsonValueKind.Object)
            {
                if (root.TryGetProperty("error", out var e) && e.ValueKind == JsonValueKind.String)
                {
                    error = e.GetString();
                }
                else if (!resp.IsSuccessStatusCode && root.TryGetProperty("detail", out var d) && d.ValueKind == JsonValueKind.String)
                {
                    error = d.GetString();
                }
                else if (!resp.IsSuccessStatusCode && root.TryGetProperty("title", out var t) && t.ValueKind == JsonValueKind.String)
                {
                    error = t.GetString();
                }
            }
        }
        catch (JsonException)
        {
            return (default, $"HTTP {(int)resp.StatusCode} ({resp.ReasonPhrase})");
        }
        if (!resp.IsSuccessStatusCode)
        {
            error ??= $"HTTP {(int)resp.StatusCode} ({resp.ReasonPhrase})";
        }
        T? value = default;
        try
        {
            value = JsonSerializer.Deserialize(text, type);
        }
        catch (JsonException)
        {
            // problem+json or an unexpected shape: the error string is what matters
        }
        return (value, error);
    }

    /// <summary>GET raw text (a store package keeps its exact bytes so the checksum survives).</summary>
    public async Task<(string? Text, string? Error)> GetTextAsync(string path, CancellationToken ct = default)
    {
        try
        {
            using var resp = await http.GetAsync("/api/robot/" + path, ct);
            var text = await resp.Content.ReadAsStringAsync(ct);
            return resp.IsSuccessStatusCode ? (text, null) : (null, $"HTTP {(int)resp.StatusCode} ({resp.ReasonPhrase})");
        }
        catch (HttpRequestException ex)
        {
            return (null, ex.Message);
        }
    }

    // ---- shortcuts used by several pages
    public Task<(EstopResultDto? Value, string? Error)> EstopAsync() =>
        PostAsync("estop", new EmptyBody(), Ctx.EmptyBody, Ctx.EstopResultDto);

    public Task<(ChatResponseDto? Value, string? Error)> ChatAsync(ChatRequest req) =>
        PostAsync("assistant/chat", req, Ctx.ChatRequest, Ctx.ChatResponseDto);

    public Task<(DocsDto? Value, string? Error)> DocsAsync() => GetAsync("docs", Ctx.DocsDto);

    public Task<(DocTextDto? Value, string? Error)> DocAsync(string name) =>
        GetAsync("docs/" + Uri.EscapeDataString(name), Ctx.DocTextDto);

    public Task<(PosesDto? Value, string? Error)> PosesAsync() => GetAsync("poses", Ctx.PosesDto);

    public Task<(MotionResultDto? Value, string? Error)> PreviewPoseAsync(string name, string target) =>
        PostAsync("poses/preview", new NameTargetRequest(name, target), Ctx.NameTargetRequest, Ctx.MotionResultDto);
}
