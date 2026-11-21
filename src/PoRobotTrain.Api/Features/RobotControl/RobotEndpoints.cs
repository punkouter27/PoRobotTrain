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

    private static async Task<IResult> ExecuteAsync<T>(Func<Task<T>> action)
    {
        try
        {
            return Results.Ok(await action());
        }
        catch (HttpRequestException ex)
        {
            return Results.Problem(
                title: "RobotControl service unavailable",
                detail: $"Could not reach robotcontrol.behaviours.service: {ex.Message}",
                statusCode: StatusCodes.Status503ServiceUnavailable);
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
            return Results.Problem(
                title: "RobotControl service unavailable",
                detail: $"Could not reach robotcontrol.behaviours.service: {ex.Message}",
                statusCode: StatusCodes.Status503ServiceUnavailable);
        }
    }
}
