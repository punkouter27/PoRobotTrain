using Microsoft.JSInterop;
using PoRobotTrain.Shared.Contracts;

namespace PoRobotTrain.Client.Services;

/// <summary>
/// Shared UI state for the simplified layout: the Advanced toggle, the chosen home task, the twin engine, the
/// first-run tour flag and a slow status poll for the header's TWIN ONLY / REAL ROBOT badge. Preferences are kept
/// in localStorage (per browser); nothing here talks to the robot except the read-only status poll.
/// </summary>
public sealed class UiState(IJSRuntime js, RobotApiClient api) : IAsyncDisposable
{
    public event Action? Changed;

    public bool Loaded { get; private set; }
    public bool Advanced { get; private set; }
    public string? Task { get; private set; }

    /// <summary>"builtin" (headless MuJoCo stand twin) or "unreal" (the UE5 twin through URLab).</summary>
    public string Engine { get; private set; } = "builtin";

    public bool TourDone { get; private set; } = true;
    public bool TourOpen { get; set; }
    public RobotStatusDto? Status { get; private set; }
    public bool Online { get; private set; }
    public string ChatSession { get; } = "blazor-" + Guid.NewGuid().ToString("N")[..8];

    public bool RealConnected => Status?.RealConnected ?? false;

    /// <summary>Preview target for the Control page's "Run on twin".</summary>
    public string PreviewTarget => Engine == "unreal" ? "urlab" : "sim";

    /// <summary>Play target for poses, motions and modes.</summary>
    public string PlayTarget => Engine == "unreal" ? "urlab" : "practice";

    private CancellationTokenSource? _cts;

    public async Task InitAsync()
    {
        if (Loaded)
        {
            return;
        }
        Advanced = await GetAsync("po.advanced") == "1";
        Task = await GetAsync("po.task") is { Length: > 0 } t ? t : null;
        Engine = await GetAsync("po.engine") == "unreal" ? "unreal" : "builtin";
        TourDone = await GetAsync("po.tour") == "done";
        TourOpen = !TourDone;
        Loaded = true;
        _cts = new CancellationTokenSource();
        _ = PollAsync(_cts.Token);
        Changed?.Invoke();
    }

    public async Task SetAdvancedAsync(bool on)
    {
        Advanced = on;
        await SetAsync("po.advanced", on ? "1" : "0");
        Changed?.Invoke();
    }

    public async Task SetTaskAsync(string? task)
    {
        Task = task;
        await SetAsync("po.task", task ?? string.Empty);
        Changed?.Invoke();
    }

    public async Task SetEngineAsync(string engine)
    {
        Engine = engine == "unreal" ? "unreal" : "builtin";
        await SetAsync("po.engine", Engine);
        Changed?.Invoke();
    }

    public async Task FinishTourAsync()
    {
        TourDone = true;
        TourOpen = false;
        await SetAsync("po.tour", "done");
        Changed?.Invoke();
    }

    public void OpenTour()
    {
        TourOpen = true;
        Changed?.Invoke();
    }

    private async Task PollAsync(CancellationToken ct)
    {
        using var timer = new PeriodicTimer(TimeSpan.FromSeconds(2));
        do
        {
            try
            {
                Status = await api.GetStatusAsync(ct);
                Online = Status is not null;
            }
            catch (Exception ex) when (ex is HttpRequestException or TaskCanceledException or System.Text.Json.JsonException)
            {
                Online = false;
            }
            Changed?.Invoke();
        }
        while (await timer.WaitForNextTickAsync(ct).ConfigureAwait(false));
    }

    private async Task<string?> GetAsync(string key)
    {
        try
        {
            return await js.InvokeAsync<string?>("localStorage.getItem", key);
        }
        catch (JSException)
        {
            return null; // private window / storage blocked: defaults apply
        }
    }

    private async Task SetAsync(string key, string value)
    {
        try
        {
            await js.InvokeVoidAsync("localStorage.setItem", key, value);
        }
        catch (JSException)
        {
            // not persisted; the in-memory value still applies for this visit
        }
    }

    public ValueTask DisposeAsync()
    {
        _cts?.Cancel();
        return ValueTask.CompletedTask;
    }
}
