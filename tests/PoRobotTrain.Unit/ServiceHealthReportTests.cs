using System.Reflection;
using PoRobotTrain.Api.Features.Diagnostics;
using PoRobotTrain.Api.Features.RobotControl;
using PoRobotTrain.Shared.Contracts;
using Xunit;

namespace PoRobotTrain.Unit;

public sealed class ServiceHealthReportTests
{
    private sealed class ManualTime : TimeProvider
    {
        public DateTimeOffset Now { get; set; } = new(2026, 11, 23, 12, 0, 0, TimeSpan.Zero);
        public override DateTimeOffset GetUtcNow() => Now;
    }

    /// <summary>Counts GetStatusAsync calls; the report never calls any other IRobotControlClient member.</summary>
    private sealed class CountingClient
    {
        public int Calls;
        public bool Fail;

        public Task<RobotStatusDto> GetStatusAsync()
        {
            Interlocked.Increment(ref Calls);
            return Fail
                ? Task.FromException<RobotStatusDto>(new HttpRequestException("offline"))
                : Task.FromResult(new RobotStatusDto(Job: "train", LastResult: "idle", Previewed: null, Checked: null,
                    CheckAgeS: null, CanRunRobot: false, WhyNotRun: null, MirrorSamples: 0, Results: new(), Head: new(),
                    Telemetry: new(), Behaviours: [], JointNames: [], JointLimitsDeg: [], JointDefaultsDeg: [], Lines: []));
        }

        public IRobotControlClient AsClient()
        {
            var proxy = DispatchProxy.Create<IRobotControlClient, StatusOnlyProxy>();
            ((StatusOnlyProxy)(object)proxy).Owner = this;
            return proxy;
        }
    }

    [Fact]
    public async Task ReachableResultIsCachedForSixtySeconds()
    {
        var time = new ManualTime();
        var client = new CountingClient();
        var report = new ServiceHealthReport(time);

        var first = await report.GetAsync(client.AsClient());
        time.Now += TimeSpan.FromSeconds(59);
        var second = await report.GetAsync(client.AsClient());
        Assert.True(first.Reachable);
        Assert.Equal("train", second.ActiveJob);
        Assert.Equal(1, client.Calls);

        time.Now += TimeSpan.FromSeconds(2);
        await report.GetAsync(client.AsClient());
        Assert.Equal(2, client.Calls);
    }

    [Fact]
    public async Task UnreachableResultIsRetriedAfterFiveSeconds()
    {
        var time = new ManualTime();
        var client = new CountingClient { Fail = true };
        var report = new ServiceHealthReport(time);

        Assert.False((await report.GetAsync(client.AsClient())).Reachable);
        time.Now += TimeSpan.FromSeconds(4);
        await report.GetAsync(client.AsClient());
        Assert.Equal(1, client.Calls);

        client.Fail = false;
        time.Now += TimeSpan.FromSeconds(2);
        Assert.True((await report.GetAsync(client.AsClient())).Reachable);
        Assert.Equal(2, client.Calls);
    }

    [Fact]
    public async Task ConcurrentCallersShareOneProbe()
    {
        var client = new CountingClient();
        var report = new ServiceHealthReport(new ManualTime());
        await Task.WhenAll(Enumerable.Range(0, 8).Select(_ => report.GetAsync(client.AsClient())));
        Assert.Equal(1, client.Calls);
    }
}

/// <summary>An IRobotControlClient whose only working member is GetStatusAsync (DispatchProxy needs a public type).</summary>
public class StatusOnlyProxy : DispatchProxy
{
    internal object? Owner { get; set; }

    protected override object? Invoke(MethodInfo? targetMethod, object?[]? args) =>
        targetMethod?.Name == nameof(IRobotControlClient.GetStatusAsync)
            ? Owner!.GetType().GetMethod("GetStatusAsync")!.Invoke(Owner, null)
            : throw new NotSupportedException(targetMethod?.Name);
}
