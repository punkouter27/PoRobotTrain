using System.Reflection;
using Microsoft.Extensions.Options;
using PoRobotTrain.Api.Features.RobotControl;
using PoRobotTrain.Shared.Contracts;

namespace PoRobotTrain.Api.Features.Diagnostics;

public static class DiagEndpoints
{
    public static IEndpointRouteBuilder MapDiagEndpoints(this IEndpointRouteBuilder app)
    {
        app.MapGet("/health", () => Results.Ok(new { status = "ok" }))
            .WithTags("Diagnostics");

        app.MapGet("/api/diag", async (
            IWebHostEnvironment env,
            IOptions<RobotServiceOptions> opts,
            IRobotControlClient client,
            CancellationToken ct) =>
        {
            var buildVersion = typeof(DiagEndpoints).Assembly
                .GetCustomAttributes<AssemblyMetadataAttribute>()
                .FirstOrDefault(a => a.Key == "BuildVersion")?.Value ?? "dev";

            bool reachable = false;
            string? activeJob = null;
            try
            {
                var status = await client.GetStatusAsync(ct);
                reachable = true;
                activeJob = status.Job;
            }
            catch (Exception ex) when (ex is HttpRequestException or TaskCanceledException or InvalidOperationException)
            {
                reachable = false;
            }

            return Results.Ok(new DiagSnapshotDto(
                AppName: "PoRobotTrain",
                BuildVersion: buildVersion,
                Environment: env.EnvironmentName,
                UpstreamUrl: opts.Value.BaseUrl,
                UpstreamReachable: reachable,
                ActiveJob: activeJob,
                JointCount: RobotJointCatalog.All.Count));
        }).WithTags("Diagnostics");

        return app;
    }
}
