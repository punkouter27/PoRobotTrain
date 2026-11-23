using Microsoft.JSInterop;
using PoRobotTrain.Shared.Contracts;

namespace PoRobotTrain.Client.Services;

/// <summary>
/// Shared UI state for the simplified layout: the Advanced toggle, the last skill and size (so the next visit starts
/// where you left off), the twin engine, the first-run tour flag and a slow status poll for the header's robot dot.
/// Preferences are kept in localStorage (per browser); nothing here talks to the robot except the read-only poll.
/// </summary>
public sealed class UiState(IJSRuntime js, RobotApiClient api) : IAsyncDisposable
{
    public event Action? Changed;

    public bool Loaded { get; private set; }
    public bool Advanced { get; private set; }

    /// <summary>The five-step flow (Layout/StepNav.razor): 1 Check, 2 Coded moves, 3 Train, 4 Virtual robot, 5 Real robot.
    /// A step unlocks when the one before it is done. Done steps are remembered per browser.</summary>
    public static readonly (int N, string Title, string Route)[] Steps =
    [
        (1, "Check", "/step/1"),
        (2, "Coded moves", "/step/2"),
        (3, "Train", "/step/3"),
        (4, "Virtual robot", "/step/4"),
        (5, "Real robot", "/step/5"),
    ];

    private readonly HashSet<int> _stepsDone = [];

    public bool StepDone(int n) => _stepsDone.Contains(n);

    public bool StepUnlocked(int n) => n <= 1 || Enumerable.Range(1, n - 1).All(StepDone);

    /// <summary>The first step that is not done yet (5 when everything is).</summary>
    public int CurrentStep => Enumerable.Range(1, 5).FirstOrDefault(n => !StepDone(n), 5);

    public async Task MarkStepAsync(int n)
    {
        if (_stepsDone.Add(n))
        {
            await SetAsync("po.steps", string.Join(',', _stepsDone.Order()));
            Changed?.Invoke();
        }
    }

    public async Task ResetStepsAsync()
    {
        _stepsDone.Clear();
        await SetAsync("po.steps", string.Empty);
        Changed?.Invoke();
    }

    /// <summary>Last skill used, and its size (deg).</summary>
    public string LastSkill { get; private set; } = "hold_centre";
    public double LastSize { get; private set; } = 10.0;

    /// <summary>"builtin" (headless MuJoCo stand twin) or "unreal" (the UE5 twin through URLab).</summary>
    public string Engine { get; private set; } = "builtin";

    public bool TourDone { get; private set; } = true;
    public bool TourOpen { get; set; }
    public RobotStatusDto? Status { get; private set; }
    public bool Online { get; private set; }
    public string ChatSession { get; } = "blazor-" + Guid.NewGuid().ToString("N")[..8];

    public bool RealConnected => Status?.RealConnected ?? false;

    /// <summary>Preview target for "Play on twin".</summary>
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
        LastSkill = await GetAsync("po.skill") is { Length: > 0 } sk ? sk : LastSkill;
        LastSize = double.TryParse(await GetAsync("po.size"), System.Globalization.NumberStyles.Float,
            System.Globalization.CultureInfo.InvariantCulture, out var sz) && sz is >= 1 and <= 30 ? sz : LastSize;
        Engine = await GetAsync("po.engine") == "unreal" ? "unreal" : "builtin";
        TourDone = await GetAsync("po.tour") == "done";
        foreach (var part in (await GetAsync("po.steps") ?? string.Empty).Split(',', StringSplitOptions.RemoveEmptyEntries))
        {
            if (int.TryParse(part, out var n) && n is >= 1 and <= 5)
            {
                _stepsDone.Add(n);
            }
        }
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

    public async Task SetSkillAsync(string skill, double size)
    {
        LastSkill = skill;
        LastSize = size;
        await SetAsync("po.skill", skill);
        await SetAsync("po.size", size.ToString(System.Globalization.CultureInfo.InvariantCulture));
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
            await AutoCompleteStepsAsync();
            Changed?.Invoke();
        }
        while (await timer.WaitForNextTickAsync(ct).ConfigureAwait(false));
    }

    /// <summary>Whether the physical robot's connection is known: it streams state, or the service's look for it
    /// (the start-up or a later snapshot) has finished.</summary>
    public bool ConnectionKnown => Online && (RealConnected || (Status?.Results.ContainsKey("snapshot") == true
        && !string.Equals(Status?.Job, "snapshot", StringComparison.OrdinalIgnoreCase)));

    /// <summary>Steps that follow from facts: 1 once the connection is known, 3 once a trained skill is ready, and 2 / 4 / 5
    /// when the service reports a clean virtual preview of a coded move / trained skill, or a clean run of a skill.</summary>
    private async Task AutoCompleteStepsAsync()
    {
        if (Status is null)
        {
            return;
        }
        if (ConnectionKnown)
        {
            await MarkStepAsync(1);
        }
        if (Status.Skills?.Any(s => s.State == "ready") == true)
        {
            await MarkStepAsync(3);
        }
        var kind = (string? id) => Status.Behaviours.FirstOrDefault(b => b.Id == id)?.Kind;
        if (Status.Previewed is { } p && Status.Results.GetValueOrDefault("preview") is { Ok: true })
        {
            await MarkStepAsync(kind(p.Behaviour) == "trained" ? 4 : 2);
        }
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
