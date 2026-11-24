using System.Reflection;
using System.Text.Json;
using Microsoft.AspNetCore.Diagnostics.HealthChecks;
using Microsoft.Extensions.Diagnostics.HealthChecks;
using Microsoft.Extensions.Options;
using PoRobotTrain.Api.Features.RobotControl;
using PoRobotTrain.Shared.Contracts;

namespace PoRobotTrain.Api.Features.Diagnostics;

public static class DiagEndpoints
{
    public static IEndpointRouteBuilder MapDiagEndpoints(this IEndpointRouteBuilder app)
    {
        // ASP.NET health checks; the behaviour-service check reads the same cached report as /api/diag.
        app.MapHealthChecks("/health", new HealthCheckOptions { ResponseWriter = WriteHealthAsync })
            .WithTags("Diagnostics");

        app.MapGet("/api/diag", async (
            IWebHostEnvironment env,
            IOptions<RobotServiceOptions> opts,
            IRobotControlClient client,
            ServiceHealthReport report,
            CancellationToken ct) =>
        {
            var buildVersion = typeof(DiagEndpoints).Assembly
                .GetCustomAttributes<AssemblyMetadataAttribute>()
                .FirstOrDefault(a => a.Key == "BuildVersion")?.Value ?? "dev";

            var probe = await report.GetAsync(client, ct);

            return Results.Ok(new DiagSnapshotDto(
                AppName: "PoRobotTrain",
                BuildVersion: buildVersion,
                Environment: env.EnvironmentName,
                UpstreamUrl: opts.Value.BaseUrl,
                UpstreamReachable: probe.Reachable,
                ActiveJob: probe.ActiveJob,
                JointCount: RobotJointCatalog.All.Count));
        }).WithTags("Diagnostics");

        return app;
    }

    private static Task WriteHealthAsync(HttpContext ctx, HealthReport report)
    {
        ctx.Response.ContentType = "application/json";
        using var stream = new MemoryStream();
        using (var w = new Utf8JsonWriter(stream))
        {
            w.WriteStartObject();
            w.WriteString("status", report.Status.ToString());
            w.WriteStartArray("checks");
            foreach (var (name, entry) in report.Entries)
            {
                w.WriteStartObject();
                w.WriteString("name", name);
                w.WriteString("status", entry.Status.ToString());
                w.WriteString("description", entry.Description);
                w.WriteEndObject();
            }
            w.WriteEndArray();
            w.WriteEndObject();
        }
        return ctx.Response.Body.WriteAsync(stream.ToArray()).AsTask();
    }
}
