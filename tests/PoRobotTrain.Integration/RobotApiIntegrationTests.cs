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
}
