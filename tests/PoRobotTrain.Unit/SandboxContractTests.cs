using System.Net;
using System.Text;
using System.Text.Json;
using PoRobotTrain.Api.Features.RobotControl;
using PoRobotTrain.Shared.Contracts;
using Xunit;

namespace PoRobotTrain.Unit;

/// <summary>Voice, robustness and walk-sandbox payloads (docs/service_api.md) through the source-generated context.</summary>
public sealed class SandboxContractTests
{
    [Fact]
    public void InterpretResponseDeserializesServicePayload()
    {
        const string json = """
        {"ok": true, "kind": "behaviour", "id": "neck_nod", "label": "Neck Nod", "amplitude": 20.0, "joint": null,
         "deg": null, "walk": null, "blocked": false, "score": 0.6,
         "note": "Neck Nod at 20°.", "say": "Neck Nod, twenty degrees, selected. Run it on the virtual robot first."}
        """;
        var r = JsonSerializer.Deserialize(json, RobotJsonContext.Default.AssistantInterpretResponse);
        Assert.NotNull(r);
        Assert.Equal("behaviour", r.Kind);
        Assert.Equal("neck_nod", r.Id);
        Assert.Equal(20.0, r.Amplitude);
        Assert.StartsWith("Neck Nod, twenty degrees", r.Say, StringComparison.Ordinal);

        var walk = JsonSerializer.Deserialize("""{"ok":true,"kind":"walk","walk":{"vx":0.5,"vy":0,"wz":0,"stop":null},"say":"Walking forward."}""",
            RobotJsonContext.Default.AssistantInterpretResponse);
        Assert.Equal(0.5, walk!.Walk!.Vx);
        Assert.Null(walk.Walk.Stop);
    }

    [Fact]
    public void InterpretRequestUsesServiceFieldNames()
    {
        var json = JsonSerializer.Serialize(new AssistantInterpretRequest("walk forward", ArmUnlocked: true, WalkActive: true),
            RobotJsonContext.Default.AssistantInterpretRequest);
        Assert.Contains("\"text\":\"walk forward\"", json, StringComparison.Ordinal);
        Assert.Contains("\"arm_unlocked\":true", json, StringComparison.Ordinal);
        Assert.Contains("\"walk_active\":true", json, StringComparison.Ordinal);
    }

    [Fact]
    public void RobustnessReportDeserializesWithGridsAndFailureDetails()
    {
        const string json = """
        {"available": true, "created_at": "2026-11-21T14:31:18Z", "policy": "RobotControl/models/r1_level0_upperbody/policy.onnx",
         "policy_sha256": "e6e2", "quick": true, "duration_s": 2.4, "rollout_s": 5.0,
         "thresholds": {"mean_err_deg": 3.0, "max_acc": 80.0, "warn_mean_err_deg": 2.0, "warn_acc": 60.0},
         "nominal": {"mean_err_deg": 2.45, "max_err_deg": 5.99, "max_acc": 288.9, "trips": 0, "verdict": "fail",
                     "acc_joint": "head_yaw_joint", "jitter": 0.031, "finite": true},
         "sensitivity": [{"id": "mass", "label": "Body mass", "unit": "×", "nominal": 1.0, "levels": [0.7, 1.0],
                          "cells": [{"mean_err_deg": 3.2, "max_err_deg": 6.0, "max_acc": 368.0, "trips": 0, "verdict": "fail", "acc_joint": "head_yaw_joint", "jitter": 0.04, "finite": true},
                                    {"mean_err_deg": null, "max_err_deg": null, "max_acc": null, "trips": 0, "verdict": "fail", "acc_joint": null, "jitter": null, "finite": false}]}],
         "interaction": {"x": {"id": "gain", "label": "Motor gains kp/kd", "unit": "×", "levels": [0.6, 1.0]},
                         "y": {"id": "latency_ms", "label": "Action latency", "unit": "ms", "levels": [0, 10]},
                         "cells": [[{"mean_err_deg": 1.9, "max_err_deg": 4, "max_acc": 287, "trips": 0, "verdict": "fail"},
                                    {"mean_err_deg": 2.5, "max_err_deg": 6, "max_acc": 289, "trips": 0, "verdict": "fail"}],
                                   [{"mean_err_deg": 1.2, "max_err_deg": 3, "max_acc": 40, "trips": 0, "verdict": "pass"},
                                    {"mean_err_deg": 2.1, "max_err_deg": 5, "max_acc": 70, "trips": 0, "verdict": "warn"}]]},
         "summary": {"pass": 1, "warn": 1, "fail": 4, "worst": "latency_ms=80 ms"}}
        """;
        var r = JsonSerializer.Deserialize(json, RobotJsonContext.Default.RobustnessReportDto);
        Assert.NotNull(r);
        Assert.True(r.Available);
        Assert.Equal("head_yaw_joint", r.Nominal!.AccJoint);
        var mass = Assert.Single(r.Sensitivity!);
        Assert.Equal(2, mass.Cells.Count);
        Assert.False(mass.Cells[1].Finite);
        Assert.Null(mass.Cells[1].MeanErrDeg);
        Assert.Equal(2, r.Interaction!.Cells.Count);
        Assert.Equal("pass", r.Interaction.Cells[1][0].Verdict);
        Assert.True(r.Interaction.Cells[1][0].Finite); // absent "finite" defaults to true
        Assert.Equal(4, r.Summary!.Fail);

        var empty = JsonSerializer.Deserialize("""{"available": false}""", RobotJsonContext.Default.RobustnessReportDto);
        Assert.False(empty!.Available);
        Assert.Null(empty.Sensitivity);
    }

    [Fact]
    public void WalkStateDeserializesIdleAndRunningPayloads()
    {
        const string idle = """
        {"ok": true, "policy_available": false, "policy_reason": "no validated Level 2 policy", "running": false, "mode": null,
         "course": "loop", "courses": ["flat", "loop"], "t": null, "cmd": null,
         "cmd_limits": {"vx": [-0.5, 1], "vy": [-0.5, 0.5], "wz": [-1, 1]}, "base": null, "q": null, "joint_names": null,
         "fallen": false, "falls": 0, "checkpoint": null, "lap": null,
         "geometry": {"checkpoints": [[3, 0, 0], [5, 2.5, 0]], "checkpoint_radius": 0.5,
                      "boxes": [{"name": "plank", "pos": [1.5, 0, 0.01], "size": [0.12, 0.6, 0.01], "quat": [1, 0, 0, 0], "rgba": [0.85, 0.62, 0.25, 1]}]}}
        """;
        var s = JsonSerializer.Deserialize(idle, RobotJsonContext.Default.WalkStateDto);
        Assert.NotNull(s);
        Assert.False(s.Running);
        Assert.False(s.PolicyAvailable);
        Assert.Equal(["flat", "loop"], s.Courses);
        Assert.Equal(1.0, s.CmdLimits!["vx"][1]);
        Assert.Equal(2, s.Geometry!.Checkpoints.Count);
        Assert.Equal("plank", s.Geometry.Boxes[0].Name);

        var q = string.Join(",", Enumerable.Repeat("0.1", 26));
        var running = $$$"""
        {"ok": true, "policy_available": false, "policy_reason": null, "running": true, "mode": "hold", "course": "loop",
         "t": 6.3, "cmd": [0, 0, 0], "cmd_limits": {"vx": [0, 0], "vy": [0, 0], "wz": [0, 0]},
         "base": {"pos": [0.02, -0.005, 0.726], "quat": [0.99996, 0.003, 0.004, 0.006], "lin_vel": [0, 0, 0], "yaw": 0.013},
         "q": [{{{q}}}], "joint_names": ["left_hip_pitch_joint"], "fallen": false, "falls": 0,
         "checkpoint": {"index": 0, "total": 6, "next": [3, 0, 0]},
         "lap": {"current_s": 7.12, "last_s": null, "best_s": null, "laps": 0}, "puppet": {"connected": false}}
        """;
        var r = JsonSerializer.Deserialize(running, RobotJsonContext.Default.WalkStateDto);
        Assert.NotNull(r);
        Assert.Equal("hold", r.Mode);
        Assert.Equal(26, r.Q!.Length);
        Assert.Equal(0.726, r.Base!.Pos[2], 3);
        Assert.Equal(6, r.Checkpoint!.Total);
        Assert.Null(r.Lap!.BestS);
        Assert.False(r.Puppet!.Connected);
    }

    [Fact]
    public void StatusCarriesJobProgressAndCommandResultCarriesPolicyReason()
    {
        const string json = """
        {"job": "robustness", "last_result": "", "previewed": null, "checked": null, "check_age_s": null, "can_run_robot": false,
         "why_not_run": null, "mirror_samples": 0, "results": {}, "head": {}, "telemetry": {}, "behaviours": [], "joint_names": [],
         "joint_limits_deg": [], "joint_defaults_deg": [], "lines": [], "progress": {"job": "robustness", "done": 12, "total": 55}}
        """;
        var s = JsonSerializer.Deserialize(json, RobotJsonContext.Default.RobotStatusDto);
        Assert.Equal(12, s!.Progress!.Done);
        Assert.Equal(55, s.Progress.Total);

        var refused = JsonSerializer.Deserialize("""{"ok": false, "error": "no policy", "policy_reason": "train it first"}""",
            RobotJsonContext.Default.CommandResultDto);
        Assert.False(refused!.Ok);
        Assert.Equal("train it first", refused.PolicyReason);
    }

    private sealed class RecordingHandler : HttpMessageHandler
    {
        public List<(string Path, long? Length, bool Chunked, string Body)> Requests { get; } = [];

        protected override async Task<HttpResponseMessage> SendAsync(HttpRequestMessage request, CancellationToken ct)
        {
            // read the length BEFORE the body: reading buffers the content, after which any content reports a length
            var length = request.Content?.Headers.ContentLength;
            var body = request.Content is null ? string.Empty : await request.Content.ReadAsStringAsync(ct);
            Requests.Add((request.RequestUri!.AbsolutePath, length, request.Headers.TransferEncodingChunked == true, body));
            var reply = request.RequestUri.AbsolutePath == "/assistant/interpret"
                ? """{"ok": true, "kind": "unknown"}"""
                : """{"ok": true}""";
            return new HttpResponseMessage(HttpStatusCode.OK) { Content = new StringContent(reply, Encoding.UTF8, "application/json") };
        }
    }

    // The Python service reads exactly Content-Length bytes (BaseHTTPRequestHandler) and does not decode chunked bodies.
    // PostAsJsonAsync used to stream every POST chunked, so the service saw "{}": parameters fell back to defaults and
    // "confirm" was dropped. Every POST must carry a Content-Length that matches its JSON body.
    [Fact]
    public async Task EveryServicePostCarriesContentLengthAndTheFullBody()
    {
        var handler = new RecordingHandler();
        var client = new RobotControlHttpClient(new HttpClient(handler) { BaseAddress = new Uri("http://127.0.0.1:8765") });

        await client.RunAsync(new RunRequest("neck_turn", 20.0, 0.2, 20.0, Confirm: true));
        await client.PreviewAsync(new PreviewRequest("hand_wave", 20.0, 0.2, 20.0, "sim"));
        await client.MoveJointAsync(new MoveJointRequest("head_yaw_joint", 12.5, "sim"));
        await client.InterpretAsync(new AssistantInterpretRequest("nod the head twenty degrees"));
        await client.WalkCommandAsync(new WalkCommandRequest(0.5, 0, 0.3));
        await client.WalkStartAsync(new WalkStartRequest("flat", "hold"));
        await client.RunRobustnessAsync(new RobustnessRunRequest(false));
        await client.WalkStopAsync();

        Assert.Equal(8, handler.Requests.Count);
        foreach (var (path, length, chunked, body) in handler.Requests)
        {
            Assert.False(chunked, $"{path} was sent chunked");
            Assert.Equal(Encoding.UTF8.GetByteCount(body), length);
        }
        Assert.Contains("\"confirm\":true", handler.Requests[0].Body, StringComparison.Ordinal);
        Assert.Contains("\"amplitude\":20", handler.Requests[0].Body, StringComparison.Ordinal);
        Assert.Contains("\"target\":\"sim\"", handler.Requests[1].Body, StringComparison.Ordinal);
        Assert.Contains("\"target_deg\":12.5", handler.Requests[2].Body, StringComparison.Ordinal);
        Assert.Contains("\"text\":\"nod the head twenty degrees\"", handler.Requests[3].Body, StringComparison.Ordinal);
        Assert.Contains("\"wz\":0.3", handler.Requests[4].Body, StringComparison.Ordinal);
        Assert.Contains("\"mode\":\"hold\"", handler.Requests[5].Body, StringComparison.Ordinal);
        Assert.Contains("\"quick\":false", handler.Requests[6].Body, StringComparison.Ordinal);
    }
}
