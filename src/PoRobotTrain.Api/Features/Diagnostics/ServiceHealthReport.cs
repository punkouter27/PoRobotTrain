using Microsoft.Extensions.Diagnostics.HealthChecks;
using PoRobotTrain.Api.Features.RobotControl;

namespace PoRobotTrain.Api.Features.Diagnostics;

/// <summary>One probe result of the behaviour service (GET /status: local, free, moves nothing).</summary>
public sealed record ServiceProbe(bool Reachable, string? ActiveJob, DateTimeOffset CheckedAt, double LatencyMs);

/// <summary>
/// The one health report that /health and /api/diag share. The behaviour service is probed at most once per
/// <see cref="ReachableTtl"/> while it answers; an unreachable result is kept only <see cref="UnreachableTtl"/>, so a
/// service that has just been started shows up quickly. Concurrent callers wait for the same probe.
/// </summary>
public sealed class ServiceHealthReport(TimeProvider time) : IDisposable
{
    public static readonly TimeSpan ReachableTtl = TimeSpan.FromSeconds(60);
    public static readonly TimeSpan UnreachableTtl = TimeSpan.FromSeconds(5);

    private readonly SemaphoreSlim _gate = new(1, 1);
    private ServiceProbe? _last;

    public async Task<ServiceProbe> GetAsync(IRobotControlClient client, CancellationToken ct = default)
    {
        if (Fresh(_last) is { } cached)
        {
            return cached;
        }
        await _gate.WaitAsync(ct);
        try
        {
            if (Fresh(_last) is { } again)
            {
                return again;
            }
            var start = time.GetTimestamp();
            ServiceProbe probe;
            try
            {
                var status = await client.GetStatusAsync(ct);
                probe = new ServiceProbe(true, status.Job, time.GetUtcNow(), time.GetElapsedTime(start).TotalMilliseconds);
            }
            catch (Exception ex) when (ex is HttpRequestException or TaskCanceledException or InvalidOperationException)
            {
                probe = new ServiceProbe(false, null, time.GetUtcNow(), time.GetElapsedTime(start).TotalMilliseconds);
            }
            _last = probe;
            return probe;
        }
        finally
        {
            _gate.Release();
        }
    }

    public void Dispose() => _gate.Dispose();

    private ServiceProbe? Fresh(ServiceProbe? p) =>
        p is not null && time.GetUtcNow() - p.CheckedAt < (p.Reachable ? ReachableTtl : UnreachableTtl) ? p : null;
}

/// <summary>/health: the console itself is up; an unreachable behaviour service makes it Degraded (still HTTP 200),
/// because the console must keep serving the software E-STOP and the offline pages.</summary>
public sealed class BehaviourServiceHealthCheck(ServiceHealthReport report, IRobotControlClient client) : IHealthCheck
{
    public async Task<HealthCheckResult> CheckHealthAsync(HealthCheckContext context, CancellationToken cancellationToken = default)
    {
        var p = await report.GetAsync(client, cancellationToken);
        var data = new Dictionary<string, object> { ["checkedAt"] = p.CheckedAt, ["latencyMs"] = Math.Round(p.LatencyMs, 1) };
        return p.Reachable
            ? HealthCheckResult.Healthy("behaviour service reachable", data)
            : HealthCheckResult.Degraded("behaviour service unreachable", data: data);
    }
}
