namespace PoRobotTrain.Api.Features.RobotControl;

/// <summary>
/// Per-request timeout for calls to the behaviour service. Most endpoints answer at once (5 s, as before); the skill
/// tools of docs/sim_features.md run headless MuJoCo or read logs first (policy duel, gap check, training montage), so
/// they get longer. The HttpClient's own timeout is the ceiling (Program.cs).
/// </summary>
public sealed class RobotServiceTimeoutHandler : DelegatingHandler
{
    public static readonly TimeSpan Default = TimeSpan.FromSeconds(5);

    /// <summary>The ceiling set on the HttpClient; stays under the browser HttpClient's 100 s default.</summary>
    public static readonly TimeSpan Ceiling = TimeSpan.FromSeconds(95);

    private static readonly Dictionary<string, TimeSpan> Slow = new(StringComparer.Ordinal)
    {
        ["/skills/duel"] = TimeSpan.FromSeconds(90),
        ["/gap"] = TimeSpan.FromSeconds(60),
        ["/skills/montage"] = TimeSpan.FromSeconds(60),
    };

    public static TimeSpan TimeoutFor(string? path) =>
        path is not null && Slow.TryGetValue(path, out var t) ? t : Default;

    protected override async Task<HttpResponseMessage> SendAsync(HttpRequestMessage request, CancellationToken cancellationToken)
    {
        using var cts = CancellationTokenSource.CreateLinkedTokenSource(cancellationToken);
        cts.CancelAfter(TimeoutFor(request.RequestUri?.AbsolutePath));
        try
        {
            return await base.SendAsync(request, cts.Token);
        }
        catch (OperationCanceledException ex) when (!cancellationToken.IsCancellationRequested)
        {
            throw new TaskCanceledException($"behaviour service did not answer {request.RequestUri?.AbsolutePath} in time", new TimeoutException(ex.Message, ex));
        }
    }
}
