using System.Net;
using System.Net.Http.Json;
using Microsoft.AspNetCore.Hosting;
using Microsoft.AspNetCore.Mvc.Testing;
using Microsoft.Extensions.DependencyInjection;
using Microsoft.Extensions.DependencyInjection.Extensions;
using PoRobotTrain.Api.Features.RobotControl;
using PoRobotTrain.Shared.Contracts;
using Xunit;

namespace PoRobotTrain.Integration;

public sealed class RobotApiIntegrationTests
{
    private sealed class FakeRobotControlClient : IRobotControlClient
    {
        public bool FailUpstream { get; set; }
        public string? LastPreviewTarget { get; private set; }

        public int StatusCalls { get; private set; }

        public Task<RobotStatusDto> GetStatusAsync(CancellationToken ct = default)
        {
            StatusCalls++;
            if (FailUpstream)
            {
                throw new HttpRequestException("upstream offline");
            }
            return Task.FromResult(new RobotStatusDto(
                Job: null,
                LastResult: "idle",
                Previewed: null,
                Checked: null,
                CheckAgeS: null,
                CanRunRobot: false,
                WhyNotRun: "preview first",
                MirrorSamples: 0,
                Results: new(),
                Head: new(),
                Telemetry: new(),
                Behaviours: [new("neck_turn", "Neck Turn", "desc", "HEAD YAW", 30.0)],
                JointNames: [.. RobotJointCatalog.All.Select(j => j.Name)],
                JointLimitsDeg: [.. RobotJointCatalog.All.Select(j => new[] { j.MinDeg, j.MaxDeg })],
                JointDefaultsDeg: [.. RobotJointCatalog.All.Select(j => j.DefaultDeg)],
                Lines: ["service ready"]));
        }

        public Task<CommandResultDto> PreviewAsync(PreviewRequest request, CancellationToken ct = default)
        {
            LastPreviewTarget = request.Target;
            return Task.FromResult(new CommandResultDto(true));
        }

        public Task<CommandResultDto> CheckAsync(BehaviourParamsDto request, CancellationToken ct = default) =>
            Task.FromResult(new CommandResultDto(true));

        public Task<CommandResultDto> MonitorAsync(ActionModeRequest? request = null, CancellationToken ct = default) =>
            Task.FromResult(new CommandResultDto(true));

        public Task<CommandResultDto> SnapshotAsync(ActionModeRequest? request = null, CancellationToken ct = default) =>
            Task.FromResult(new CommandResultDto(true));

        public Task<CommandResultDto> MoveJointAsync(MoveJointRequest request, CancellationToken ct = default) =>
            Task.FromResult(request.Target == "real" && !request.Confirm
                ? new CommandResultDto(false, "confirmation required")
                : new CommandResultDto(true));

        public Task<CommandResultDto> RunAsync(RunRequest request, CancellationToken ct = default) =>
            Task.FromResult(request.Confirm
                ? new CommandResultDto(true)
                : new CommandResultDto(false, "confirmation required"));

        public Task<CommandResultDto> StopAsync(CancellationToken ct = default) =>
            Task.FromResult(new CommandResultDto(true));

        public Task<byte[]> GetViewPngAsync(string? mode = null, string? cam = null, int? j = null, double? deg = null, CancellationToken ct = default)
        {
            if (FailUpstream)
            {
                throw new HttpRequestException("upstream offline");
            }
            return Task.FromResult(new byte[] { 0x89, 0x50, 0x4E, 0x47, 0x0D, 0x0A, 0x1A, 0x0A });
        }

        public string? LastInterpretText { get; private set; }
        public WalkCommandRequest? LastWalkCommand { get; private set; }

        public Task<AssistantInterpretResponse> InterpretAsync(AssistantInterpretRequest request, CancellationToken ct = default)
        {
            LastInterpretText = request.Text;
            return Task.FromResult(request.WalkActive
                ? new AssistantInterpretResponse(true, "walk", Walk: new WalkIntentDto(0.5, 0, 0), Say: "Walking forward.")
                : new AssistantInterpretResponse(true, "behaviour", "neck_nod", "Neck Nod", 20.0, Say: "Neck Nod, twenty degrees, selected."));
        }

        public Task<RobustnessReportDto> GetRobustnessAsync(CancellationToken ct = default) =>
            FailUpstream ? throw new HttpRequestException("upstream offline") : Task.FromResult(new RobustnessReportDto(false));

        public Task<CommandResultDto> RunRobustnessAsync(RobustnessRunRequest request, CancellationToken ct = default) =>
            Task.FromResult(new CommandResultDto(true));

        public Task<WalkStateDto> GetWalkStateAsync(CancellationToken ct = default) =>
            FailUpstream
                ? throw new HttpRequestException("not found", null, HttpStatusCode.NotFound)
                : Task.FromResult(new WalkStateDto(true, false, "no validated Level 2 policy", false, Courses: ["flat", "loop"]));

        public Task<CommandResultDto> WalkStartAsync(WalkStartRequest request, CancellationToken ct = default) =>
            Task.FromResult(request.Mode == "hold"
                ? new CommandResultDto(true)
                : new CommandResultDto(false, "no validated Level 2 policy", PolicyReason: "train it, then validate"));

        public Task<CommandResultDto> WalkCommandAsync(WalkCommandRequest request, CancellationToken ct = default)
        {
            LastWalkCommand = request;
            return Task.FromResult(new CommandResultDto(true));
        }

        public Task<CommandResultDto> WalkResetAsync(CancellationToken ct = default) => Task.FromResult(new CommandResultDto(true));

        public Task<CommandResultDto> WalkStopAsync(CancellationToken ct = default) => Task.FromResult(new CommandResultDto(true));

        public object? LastSimRequest { get; private set; }

        public Task<CommandResultDto> WalkThrowAsync(WalkThrowRequest request, CancellationToken ct = default) => Sim(request);

        public Task<CommandResultDto> WalkReplayAsync(WalkReplayRequest request, CancellationToken ct = default) => Sim(request);

        public Task<CommandResultDto> WalkFilterAsync(WalkFilterRequest request, CancellationToken ct = default) => Sim(request);

        public Task<CommandResultDto> WalkScenarioAsync(WalkScenarioRequest request, CancellationToken ct = default) =>
            request.Id == "lab_stress" ? Task.FromResult(new CommandResultDto(false, "scenario lab_stress targets lab, not walk")) : Sim(request);

        private Task<CommandResultDto> Sim(object request)
        {
            if (FailUpstream)
            {
                throw new HttpRequestException("upstream offline");
            }
            LastSimRequest = request;
            return Task.FromResult(new CommandResultDto(true));
        }

        public List<(HttpMethod Method, string Path, string Body)> Forwarded { get; } = [];
        public int ForwardStatus { get; set; } = 200;
        public string ForwardBody { get; set; } = """{"ok": true}""";

        public Task<ForwardResult> ForwardAsync(HttpMethod method, string pathAndQuery, byte[]? body, CancellationToken ct = default)
        {
            if (FailUpstream)
            {
                throw new HttpRequestException("upstream offline");
            }
            Forwarded.Add((method, pathAndQuery, body is null ? string.Empty : System.Text.Encoding.UTF8.GetString(body)));
            return Task.FromResult(new ForwardResult(ForwardStatus, "application/json", System.Text.Encoding.UTF8.GetBytes(ForwardBody)));
        }
    }

    private static WebApplicationFactory<Program> CreateFactory(FakeRobotControlClient fake) =>
        new WebApplicationFactory<Program>().WithWebHostBuilder(b =>
        {
            b.UseEnvironment("Test");
            b.ConfigureServices(services =>
            {
                services.RemoveAll<IRobotControlClient>();
                services.AddSingleton<IRobotControlClient>(fake);
            });
        });

    [Fact]
    public async Task ConsoleRoutesForwardPathQueryBodyAndStatus()
    {
        var fake = new FakeRobotControlClient { ForwardBody = """{"ok": true, "overall": "ok", "failing": 0, "rows": [], "checked_at": 1.0}""" };
        using var factory = CreateFactory(fake);
        var client = factory.CreateClient();

        var health = await client.GetFromJsonAsync("/api/robot/health", RobotJsonContext.Default.HealthDto);
        Assert.Equal("ok", health!.Overall);
        Assert.Equal((HttpMethod.Get, "/health"), (fake.Forwarded[^1].Method, fake.Forwarded[^1].Path));

        await client.GetAsync("/api/robot/events?level=warn&limit=5");
        Assert.Equal("/events?level=warn&limit=5", fake.Forwarded[^1].Path);

        var resp = await client.PostAsJsonAsync("/api/robot/limits", new LimitsRequest(new() { ["walk_speed_scale"] = 0.5 }),
            RobotJsonContext.Default.LimitsRequest);
        Assert.Equal(HttpStatusCode.OK, resp.StatusCode);
        Assert.Equal(HttpMethod.Post, fake.Forwarded[^1].Method);
        Assert.Contains("walk_speed_scale", fake.Forwarded[^1].Body);

        // route values are re-encoded: a name can never smuggle extra path segments to the service
        await client.GetAsync("/api/robot/motions/wave%2F..%2Fx/export");
        Assert.Equal("/motions/wave%252F..%252Fx/export", fake.Forwarded[^1].Path); // still one (rejected) segment

        // the service's own status codes pass through (409 busy, 400 refused)
        fake.ForwardStatus = 409;
        fake.ForwardBody = """{"ok": false, "error": "walk is running"}""";
        var busy = await client.PostAsync("/api/robot/motions/play", new StringContent("{}"));
        Assert.Equal(HttpStatusCode.Conflict, busy.StatusCode);

        // an older service without the console endpoint -> 501 with a restart hint
        fake.ForwardStatus = 404;
        fake.ForwardBody = """{"ok": false, "error": "not found"}""";
        var old = await client.GetAsync("/api/robot/preflight");
        Assert.Equal(HttpStatusCode.NotImplemented, old.StatusCode);

        fake.FailUpstream = true;
        var down = await client.PostAsync("/api/robot/estop", new StringContent("{}"));
        Assert.Equal(HttpStatusCode.ServiceUnavailable, down.StatusCode);
    }

    [Fact]
    public async Task HealthDiagAndBlazorRootServeSuccessfully()
    {
        var fake = new FakeRobotControlClient();
        using var factory = CreateFactory(fake);
        using var client = factory.CreateClient();

        var health = await client.GetAsync("/health");
        Assert.Equal(HttpStatusCode.OK, health.StatusCode);

        var diag = await client.GetFromJsonAsync<DiagSnapshotDto>("/api/diag");
        Assert.NotNull(diag);
        Assert.True(diag.UpstreamReachable);
        Assert.Equal(26, diag.JointCount);

        var root = await client.GetAsync("/");
        Assert.Equal(HttpStatusCode.OK, root.StatusCode);
        var html = await root.Content.ReadAsStringAsync();
        Assert.Contains("blazor.webassembly.js", html, StringComparison.Ordinal);
    }

    [Fact]
    public async Task HealthAndDiagShareOneCachedServiceProbe()
    {
        var fake = new FakeRobotControlClient();
        using var factory = CreateFactory(fake);
        using var client = factory.CreateClient();

        var health = await client.GetStringAsync("/health");
        Assert.Contains("\"status\":\"Healthy\"", health, StringComparison.Ordinal);
        Assert.Contains("behaviour-service", health, StringComparison.Ordinal);
        Assert.True((await client.GetFromJsonAsync<DiagSnapshotDto>("/api/diag"))!.UpstreamReachable);
        await client.GetStringAsync("/health");
        Assert.Equal(1, fake.StatusCalls); // one probe serves both endpoints for 60 s
    }

    [Fact]
    public async Task HealthIsDegradedButServedWhenTheServiceIsDown()
    {
        var fake = new FakeRobotControlClient { FailUpstream = true };
        using var factory = CreateFactory(fake);
        using var client = factory.CreateClient();

        var resp = await client.GetAsync("/health");
        Assert.Equal(HttpStatusCode.OK, resp.StatusCode);
        Assert.Contains("\"status\":\"Degraded\"", await resp.Content.ReadAsStringAsync(), StringComparison.Ordinal);
        Assert.False((await client.GetFromJsonAsync<DiagSnapshotDto>("/api/diag"))!.UpstreamReachable);
    }

    [Fact]
    public async Task RobotEndpointsProxyAndReport503WhenUpstreamOffline()
    {
        var fake = new FakeRobotControlClient();
        using var factory = CreateFactory(fake);
        using var client = factory.CreateClient();

        var preview = await client.PostAsJsonAsync("/api/robot/preview", new PreviewRequest("hand_wave", 20.0, 0.2, 20.0, "sim"));
        Assert.Equal(HttpStatusCode.OK, preview.StatusCode);
        Assert.Equal("sim", fake.LastPreviewTarget);

        var snapshot = await client.PostAsJsonAsync("/api/robot/snapshot", new ActionModeRequest(Practice: true));
        Assert.Equal(HttpStatusCode.OK, snapshot.StatusCode);

        var moveUnconfirmed = await client.PostAsJsonAsync("/api/robot/move-joint", new MoveJointRequest("head_yaw_joint", 10.0, "real", false));
        Assert.Equal(HttpStatusCode.BadRequest, moveUnconfirmed.StatusCode);
        var moveVirtual = await client.PostAsJsonAsync("/api/robot/move-joint", new MoveJointRequest("head_yaw_joint", 10.0, "sim", false));
        Assert.Equal(HttpStatusCode.OK, moveVirtual.StatusCode);

        var view = await client.GetAsync("/api/robot/view.png?mode=real&cam=iso");
        Assert.Equal(HttpStatusCode.OK, view.StatusCode);
        Assert.Equal("image/png", view.Content.Headers.ContentType?.MediaType);

        var unconfirmed = await client.PostAsJsonAsync("/api/robot/run", new RunRequest("neck_turn", 10.0, 0.2, 20.0, false));
        Assert.Equal(HttpStatusCode.BadRequest, unconfirmed.StatusCode);

        fake.FailUpstream = true;
        var down = await client.GetAsync("/api/robot/status");
        Assert.Equal(HttpStatusCode.ServiceUnavailable, down.StatusCode);
    }

    [Fact]
    public async Task WalkSimFeatureEndpointsValidateAndProxyTheService()
    {
        var fake = new FakeRobotControlClient();
        using var factory = CreateFactory(fake);
        using var client = factory.CreateClient();

        // #3 procedural start: seed + difficulty reach the service; out of range is refused here
        var start = await client.PostAsJsonAsync("/api/robot/walk/start", new WalkStartRequest("procedural", "hold", 7, 0.3));
        Assert.Equal(HttpStatusCode.OK, start.StatusCode);
        var badStart = await client.PostAsJsonAsync("/api/robot/walk/start", new WalkStartRequest("procedural", "free_hold", 7, 1.5));
        Assert.Equal(HttpStatusCode.BadRequest, badStart.StatusCode);
        Assert.Equal("Difficulty must be 0-1.", (await badStart.Content.ReadFromJsonAsync<CommandResultDto>())!.Error);

        // #9 throw
        var thrown = await client.PostAsJsonAsync("/api/robot/walk/throw", new WalkThrowRequest("box", 5, 90));
        Assert.Equal(HttpStatusCode.OK, thrown.StatusCode);
        Assert.Equal(new WalkThrowRequest("box", 5, 90), fake.LastSimRequest);
        var tooFast = await client.PostAsJsonAsync("/api/robot/walk/throw", new WalkThrowRequest("ball", 9, 0));
        Assert.Equal(HttpStatusCode.BadRequest, tooFast.StatusCode);
        Assert.Equal("Speed must be 1-8 m/s.", (await tooFast.Content.ReadFromJsonAsync<CommandResultDto>())!.Error);

        // #12 replay: start, stop, auto on fall
        Assert.Equal(HttpStatusCode.OK, (await client.PostAsJsonAsync("/api/robot/walk/replay", new WalkReplayRequest(3, 0.25))).StatusCode);
        Assert.Equal(new WalkReplayRequest(3, 0.25), fake.LastSimRequest);
        Assert.Equal(HttpStatusCode.OK, (await client.PostAsJsonAsync("/api/robot/walk/replay", new WalkReplayRequest(Stop: true))).StatusCode);
        Assert.Equal(new WalkReplayRequest(Stop: true), fake.LastSimRequest);
        Assert.Equal(HttpStatusCode.OK, (await client.PostAsJsonAsync("/api/robot/walk/replay", new WalkReplayRequest(AutoOnFall: true))).StatusCode);
        Assert.Equal(HttpStatusCode.BadRequest, (await client.PostAsJsonAsync("/api/robot/walk/replay", new WalkReplayRequest(12, 0.25))).StatusCode);
        Assert.Equal(HttpStatusCode.BadRequest, (await client.PostAsJsonAsync("/api/robot/walk/replay", new WalkReplayRequest(3, 0.05))).StatusCode);

        // #15 action filter
        Assert.Equal(HttpStatusCode.OK, (await client.PostAsJsonAsync("/api/robot/walk/filter", new WalkFilterRequest(0.3, true))).StatusCode);
        Assert.Equal(new WalkFilterRequest(0.3, true), fake.LastSimRequest);
        Assert.Equal(HttpStatusCode.BadRequest, (await client.PostAsJsonAsync("/api/robot/walk/filter", new WalkFilterRequest(1.2, false))).StatusCode);

        // #18 scenario: play, stop, a service refusal passes through as 400, an empty id is refused here
        Assert.Equal(HttpStatusCode.OK, (await client.PostAsJsonAsync("/api/robot/walk/scenario", new WalkScenarioRequest("push_test"))).StatusCode);
        Assert.Equal(HttpStatusCode.OK, (await client.PostAsJsonAsync("/api/robot/walk/scenario", new WalkScenarioRequest(Stop: true))).StatusCode);
        var lab = await client.PostAsJsonAsync("/api/robot/walk/scenario", new WalkScenarioRequest("lab_stress"));
        Assert.Equal(HttpStatusCode.BadRequest, lab.StatusCode);
        Assert.Contains("targets lab", (await lab.Content.ReadFromJsonAsync<CommandResultDto>())!.Error, StringComparison.Ordinal);
        Assert.Equal(HttpStatusCode.BadRequest, (await client.PostAsJsonAsync("/api/robot/walk/scenario", new WalkScenarioRequest())).StatusCode);

        fake.FailUpstream = true;
        Assert.Equal(HttpStatusCode.ServiceUnavailable, (await client.PostAsJsonAsync("/api/robot/walk/throw", new WalkThrowRequest())).StatusCode);
    }

    [Fact]
    public async Task SkillToolAndPoseRoutesForwardToTheService()
    {
        var fake = new FakeRobotControlClient();
        using var factory = CreateFactory(fake);
        using var client = factory.CreateClient();

        fake.ForwardBody = """{"ok": true, "scenarios": [{"id": "push_test", "name": "Ball from the left", "target": "walk", "description": "d", "duration_s": 8}]}""";
        var scenarios = await client.GetFromJsonAsync("/api/robot/scenarios", RobotJsonContext.Default.ScenariosDto);
        Assert.Equal("push_test", scenarios!.Scenarios![0].Id);
        Assert.Equal((HttpMethod.Get, "/scenarios"), (fake.Forwarded[^1].Method, fake.Forwarded[^1].Path));

        fake.ForwardBody = """{"ok": true, "runs": [{"file": "look_at_sim_20261122_191054.csv", "skill": "look_at", "target": "sim", "stamp": "20261122_191054"}]}""";
        var runs = await client.GetFromJsonAsync("/api/robot/runs", RobotJsonContext.Default.RunsDto);
        Assert.Equal("look_at", runs!.Runs![0].Skill);
        Assert.Equal("/runs", fake.Forwarded[^1].Path);

        fake.ForwardBody = """{"ok": false, "reason": "no checkpoints: train with --checkpoint-every"}""";
        var montage = await client.GetFromJsonAsync("/api/robot/skills/montage?skill=look_at", RobotJsonContext.Default.MontageDto);
        Assert.False(montage!.Ok);
        Assert.Equal("/skills/montage?skill=look_at", fake.Forwarded[^1].Path);

        fake.ForwardBody = """{"ok": true, "result": {"skill": "look_at", "scenario": "lab_stress", "winner": "b", "why": "lower error"}}""";
        var duel = await client.PostAsJsonAsync("/api/robot/skills/duel", new DuelRequest("look_at", null, 2, "lab_stress"), RobotJsonContext.Default.DuelRequest);
        Assert.Equal(HttpStatusCode.OK, duel.StatusCode);
        Assert.Equal((HttpMethod.Post, "/skills/duel"), (fake.Forwarded[^1].Method, fake.Forwarded[^1].Path));
        Assert.Contains("\"a\":null", fake.Forwarded[^1].Body, StringComparison.Ordinal);
        Assert.Contains("\"b\":2", fake.Forwarded[^1].Body, StringComparison.Ordinal);

        fake.ForwardBody = """{"ok": true, "joints": ["head_yaw_joint"], "t": [0, 0.02], "a_deg": [[0], [1]], "b_deg": [[0], [2]], "err_deg": [[0], [1]], "max_deg": [1], "rms_deg": [0.7], "pass": true}""";
        var gap = await client.PostAsJsonAsync("/api/robot/gap", new GapRequest("a.csv", "b.csv", 5), RobotJsonContext.Default.GapRequest);
        Assert.Equal(HttpStatusCode.OK, gap.StatusCode);
        Assert.Contains("\"tol_deg\":5", fake.Forwarded[^1].Body, StringComparison.Ordinal);

        // taught poses: GET is the same /poses route the console uses; POST /poses and /poses/delete are new
        fake.ForwardBody = """{"ok": true, "pose": {"id": 3, "name": "Look up", "yaw_deg": 0, "pitch_deg": -10, "skill": "pose_3"}}""";
        var saved = await client.PostAsJsonAsync("/api/robot/poses", new TaughtPoseRequest("Look up", 0, -10), RobotJsonContext.Default.TaughtPoseRequest);
        Assert.Equal(HttpStatusCode.OK, saved.StatusCode);
        Assert.Equal((HttpMethod.Post, "/poses"), (fake.Forwarded[^1].Method, fake.Forwarded[^1].Path));
        Assert.Contains("\"pitch_deg\":-10", fake.Forwarded[^1].Body, StringComparison.Ordinal);
        fake.ForwardBody = """{"ok": true}""";
        Assert.Equal(HttpStatusCode.OK, (await client.PostAsync("/api/robot/poses/delete", new StringContent("""{"id": 3}"""))).StatusCode);
        Assert.Equal("/poses/delete", fake.Forwarded[^1].Path);

        // an older service without these endpoints -> 501
        fake.ForwardStatus = 404;
        fake.ForwardBody = """{"ok": false, "error": "not found"}""";
        Assert.Equal(HttpStatusCode.NotImplemented, (await client.GetAsync("/api/robot/runs")).StatusCode);
    }

    [Fact]
    public async Task VoiceRobustnessAndWalkEndpointsProxyTheService()
    {
        var fake = new FakeRobotControlClient();
        using var factory = CreateFactory(fake);
        using var client = factory.CreateClient();

        var interpret = await client.PostAsJsonAsync("/api/robot/assistant/interpret", new AssistantInterpretRequest("nod the head twenty degrees"));
        Assert.Equal(HttpStatusCode.OK, interpret.StatusCode);
        var reply = await interpret.Content.ReadFromJsonAsync<AssistantInterpretResponse>();
        Assert.Equal("neck_nod", reply!.Id);
        Assert.Equal("nod the head twenty degrees", fake.LastInterpretText);

        var robustness = await client.GetFromJsonAsync<RobustnessReportDto>("/api/robot/robustness");
        Assert.False(robustness!.Available);
        var run = await client.PostAsJsonAsync("/api/robot/robustness/run", new RobustnessRunRequest(true));
        Assert.Equal(HttpStatusCode.OK, run.StatusCode);

        var state = await client.GetFromJsonAsync<WalkStateDto>("/api/robot/walk/state");
        Assert.False(state!.PolicyAvailable);
        Assert.Equal(["flat", "loop"], state.Courses);

        var refused = await client.PostAsJsonAsync("/api/robot/walk/start", new WalkStartRequest("loop", "policy"));
        Assert.Equal(HttpStatusCode.BadRequest, refused.StatusCode);
        var why = await refused.Content.ReadFromJsonAsync<CommandResultDto>();
        Assert.Equal("train it, then validate", why!.PolicyReason);
        var hold = await client.PostAsJsonAsync("/api/robot/walk/start", new WalkStartRequest("loop", "hold"));
        Assert.Equal(HttpStatusCode.OK, hold.StatusCode);

        var cmd = await client.PostAsJsonAsync("/api/robot/walk/command", new WalkCommandRequest(0.4, -0.1, 0.2));
        Assert.Equal(HttpStatusCode.OK, cmd.StatusCode);
        Assert.Equal(new WalkCommandRequest(0.4, -0.1, 0.2), fake.LastWalkCommand);
        Assert.Equal(HttpStatusCode.OK, (await client.PostAsync("/api/robot/walk/reset", null)).StatusCode);
        Assert.Equal(HttpStatusCode.OK, (await client.PostAsync("/api/robot/walk/stop", null)).StatusCode);

        // service running but without the endpoint (older RobotControl) -> 501, unreachable -> 503
        fake.FailUpstream = true;
        Assert.Equal(HttpStatusCode.NotImplemented, (await client.GetAsync("/api/robot/walk/state")).StatusCode);
        Assert.Equal(HttpStatusCode.ServiceUnavailable, (await client.GetAsync("/api/robot/robustness")).StatusCode);
    }
}
