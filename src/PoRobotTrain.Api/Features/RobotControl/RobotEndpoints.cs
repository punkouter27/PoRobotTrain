using PoRobotTrain.Shared.Contracts;

namespace PoRobotTrain.Api.Features.RobotControl;

public static class RobotEndpoints
{
    public static IEndpointRouteBuilder MapRobotEndpoints(this IEndpointRouteBuilder app)
    {
        var group = app.MapGroup("/api/robot").WithTags("RobotControl");

        group.MapGet("/status", async (IRobotControlClient client, CancellationToken ct) =>
            await ExecuteAsync(() => client.GetStatusAsync(ct)));

        group.MapPost("/preview", async (PreviewRequest req, IRobotControlClient client, CancellationToken ct) =>
            await ExecuteCommandAsync(() => client.PreviewAsync(req, ct)));

        group.MapPost("/check", async (BehaviourParamsDto req, IRobotControlClient client, CancellationToken ct) =>
            await ExecuteCommandAsync(() => client.CheckAsync(req, ct)));

        group.MapPost("/monitor", async (ActionModeRequest? req, IRobotControlClient client, CancellationToken ct) =>
            await ExecuteCommandAsync(() => client.MonitorAsync(req, ct)));

        group.MapPost("/snapshot", async (ActionModeRequest? req, IRobotControlClient client, CancellationToken ct) =>
            await ExecuteCommandAsync(() => client.SnapshotAsync(req, ct)));

        group.MapPost("/move-joint", async (MoveJointRequest req, IRobotControlClient client, CancellationToken ct) =>
            await ExecuteCommandAsync(() => client.MoveJointAsync(req, ct)));

        group.MapPost("/run", async (RunRequest req, IRobotControlClient client, CancellationToken ct) =>
            await ExecuteCommandAsync(() => client.RunAsync(req, ct)));

        group.MapPost("/stop", async (IRobotControlClient client, CancellationToken ct) =>
            await ExecuteCommandAsync(() => client.StopAsync(ct)));

        // Voice intent (suggest-only), robustness explorer and the simulation-only walk sandbox (docs/service_api.md)
        group.MapPost("/assistant/interpret", async (AssistantInterpretRequest req, IRobotControlClient client, CancellationToken ct) =>
            await ExecuteAsync(() => client.InterpretAsync(req, ct)));

        group.MapGet("/robustness", async (IRobotControlClient client, CancellationToken ct) =>
            await ExecuteAsync(() => client.GetRobustnessAsync(ct)));

        group.MapPost("/robustness/run", async (RobustnessRunRequest? req, IRobotControlClient client, CancellationToken ct) =>
            await ExecuteCommandAsync(() => client.RunRobustnessAsync(req ?? new RobustnessRunRequest(), ct)));

        group.MapGet("/walk/state", async (IRobotControlClient client, CancellationToken ct) =>
            await ExecuteAsync(() => client.GetWalkStateAsync(ct)));

        group.MapPost("/walk/start", async (WalkStartRequest? req, IRobotControlClient client, CancellationToken ct) =>
            await ExecuteCommandAsync(() => client.WalkStartAsync(req ?? new WalkStartRequest(), ct)));

        group.MapPost("/walk/command", async (WalkCommandRequest req, IRobotControlClient client, CancellationToken ct) =>
            await ExecuteCommandAsync(() => client.WalkCommandAsync(req, ct)));

        group.MapPost("/walk/reset", async (IRobotControlClient client, CancellationToken ct) =>
            await ExecuteCommandAsync(() => client.WalkResetAsync(ct)));

        group.MapPost("/walk/stop", async (IRobotControlClient client, CancellationToken ct) =>
            await ExecuteCommandAsync(() => client.WalkStopAsync(ct)));

        MapConsole(group);

        group.MapGet("/view.png", async (string? mode, string? cam, int? j, double? deg, IRobotControlClient client, CancellationToken ct) =>
        {
            try
            {
                var bytes = await client.GetViewPngAsync(mode, cam, j, deg, ct);
                return Results.File(bytes, "image/png");
            }
            catch (HttpRequestException ex)
            {
                return Results.Problem(
                    title: "RobotControl view unavailable",
                    detail: $"Could not reach robotcontrol.behaviours.service: {ex.Message}",
                    statusCode: StatusCodes.Status503ServiceUnavailable);
            }
        });

        return app;
    }

    // Operator console (docs/service_api.md sections 4-7). Each route is forwarded to the same path on the behaviour
    // service; {name} is re-encoded from the route value so only a single path segment reaches the service.
    internal static readonly string[] ConsoleGets =
    [
        "health", "versions", "events", "history/temps", "power", "imu", "camera.png", "limits", "poses", "motions",
        "motions/record", "motions/{name}", "motions/{name}/export", "player", "calibration/offsets", "network",
        "preflight", "docs", "docs/{name}", "assistant/chat/history",
        "skills", "skills/detail",
    ];

    internal static readonly string[] ConsolePosts =
    [
        "estop", "limits", "limits/reset", "mode", "poses/preview", "motions/check", "motions/save", "motions/delete",
        "motions/import", "motions/play", "motions/record/start", "motions/record/stop", "player/stop",
        "calibration/capture", "calibration/save", "network/test", "network/profiles", "network/profiles/delete",
        "network/active", "preflight", "preflight/reset", "assistant/chat", "assistant/chat/reset",
        "skills/train", "skills/activate", "skills/push", "lab/play",
    ];

    private static void MapConsole(RouteGroupBuilder group)
    {
        foreach (var route in ConsoleGets)
        {
            group.MapGet("/" + route, (HttpContext ctx, IRobotControlClient client, CancellationToken ct) =>
                ForwardAsync(ctx, route, HttpMethod.Get, client, ct)).WithTags("RobotConsole");
        }
        foreach (var route in ConsolePosts)
        {
            group.MapPost("/" + route, (HttpContext ctx, IRobotControlClient client, CancellationToken ct) =>
                ForwardAsync(ctx, route, HttpMethod.Post, client, ct)).WithTags("RobotConsole");
        }
    }

    private static async Task<IResult> ForwardAsync(HttpContext ctx, string route, HttpMethod method, IRobotControlClient client, CancellationToken ct)
    {
        var path = "/" + route;
        if (route.Contains("{name}", StringComparison.Ordinal))
        {
            var name = ctx.Request.RouteValues["name"]?.ToString() ?? string.Empty;
            path = path.Replace("{name}", Uri.EscapeDataString(name), StringComparison.Ordinal);
        }
        byte[]? body = null;
        if (method != HttpMethod.Get)
        {
            using var ms = new MemoryStream();
            await ctx.Request.Body.CopyToAsync(ms, ct);
            body = ms.ToArray();
        }
        try
        {
            var res = await client.ForwardAsync(method, path + ctx.Request.QueryString.Value, body, ct);
            if (res.StatusCode == StatusCodes.Status404NotFound && IsOldServiceNotFound(res.Body))
            {
                return Unavailable(new HttpRequestException("not found", null, System.Net.HttpStatusCode.NotFound));
            }
            return new ForwardedResult(res);
        }
        catch (HttpRequestException ex)
        {
            return Unavailable(ex);
        }
        catch (TaskCanceledException) when (!ct.IsCancellationRequested)
        {
            return Results.Problem(title: "RobotControl service timed out", statusCode: StatusCodes.Status504GatewayTimeout);
        }
    }

    /// <summary>Writes the service's status code, content type and body unchanged.</summary>
    private sealed class ForwardedResult(ForwardResult res) : IResult
    {
        public async Task ExecuteAsync(HttpContext httpContext)
        {
            httpContext.Response.StatusCode = res.StatusCode;
            httpContext.Response.ContentType = res.ContentType;
            httpContext.Response.Headers.CacheControl = "no-store";
            httpContext.Response.ContentLength = res.Body.Length;
            await httpContext.Response.Body.WriteAsync(res.Body);
        }
    }

    // An older service answers an unknown path with exactly {"ok": false, "error": "not found"}.
    private static bool IsOldServiceNotFound(byte[] body)
    {
        try
        {
            using var doc = System.Text.Json.JsonDocument.Parse(body);
            return doc.RootElement.TryGetProperty("error", out var e) && e.ValueKind == System.Text.Json.JsonValueKind.String
                && e.GetString() == "not found";
        }
        catch (System.Text.Json.JsonException)
        {
            return false;
        }
    }

    private static async Task<IResult> ExecuteAsync<T>(Func<Task<T>> action)
    {
        try
        {
            return Results.Ok(await action());
        }
        catch (HttpRequestException ex)
        {
            return Unavailable(ex);
        }
    }

    private static async Task<IResult> ExecuteCommandAsync(Func<Task<CommandResultDto>> action)
    {
        try
        {
            var res = await action();
            return res.Ok ? Results.Ok(res) : Results.BadRequest(res);
        }
        catch (HttpRequestException ex)
        {
            return Unavailable(ex);
        }
    }

    // A 404 from the service means it is running but predates the endpoint (restart it from the current RobotControl).
    private static IResult Unavailable(HttpRequestException ex) => ex.StatusCode == System.Net.HttpStatusCode.NotFound
        ? Results.Problem(
            title: "RobotControl service has no such endpoint",
            detail: "robotcontrol.behaviours.service is running but does not provide this endpoint; restart it from the current RobotControl.",
            statusCode: StatusCodes.Status501NotImplemented)
        : Results.Problem(
            title: "RobotControl service unavailable",
            detail: $"Could not reach robotcontrol.behaviours.service: {ex.Message}",
            statusCode: StatusCodes.Status503ServiceUnavailable);
}
