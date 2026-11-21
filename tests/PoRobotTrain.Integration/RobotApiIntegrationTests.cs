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

        public Task<RobotStatusDto> GetStatusAsync(CancellationToken ct = default)
        {
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
