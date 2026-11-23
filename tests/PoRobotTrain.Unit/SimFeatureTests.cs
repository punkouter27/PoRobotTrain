using System.Text.Json;
using PoRobotTrain.Shared.Contracts;
using PoRobotTrain.Shared.Validation;
using Xunit;

namespace PoRobotTrain.Unit;

/// <summary>Simulation features (docs/sim_features.md): form rules, payloads through the source-generated context, helpers.</summary>
public sealed class SimFeatureTests
{
    // ------------------------------------------------------------------ validators

    [Theory]
    [InlineData("Look up", 0, 0, true)]
    [InlineData("a", -30, -15, true)]
    [InlineData("left_low-2", 30, 20, true)]
    [InlineData("x2345678901234567890123456789012", 0, 0, true)]
    [InlineData("x23456789012345678901234567890123", 0, 0, false)]
    [InlineData("", 0, 0, false)]
    [InlineData("   ", 0, 0, false)]
    [InlineData("bad/name", 0, 0, false)]
    [InlineData("ok", 31, 0, false)]
    [InlineData("ok", -31, 0, false)]
    [InlineData("ok", 0, 21, false)]
    [InlineData("ok", 0, -16, false)]
    public void TaughtPoseFollowsTheServiceRule(string name, double yaw, double pitch, bool valid)
    {
        Assert.Equal(valid, new TaughtPoseRequestValidator().Validate(new TaughtPoseRequest(name, yaw, pitch)).IsValid);
    }

    [Theory]
    [InlineData("ball", 1, 0, true)]
    [InlineData("box", 8, 90, true)]
    [InlineData("ball", 0.5, 0, false)]
    [InlineData("ball", 8.5, 0, false)]
    [InlineData("anvil", 4, 0, false)]
    public void ThrowSpeedIsOneToEight(string kind, double speed, double fromDeg, bool valid)
    {
        Assert.Equal(valid, new WalkThrowRequestValidator().Validate(new WalkThrowRequest(kind, speed, fromDeg)).IsValid);
    }

    [Theory]
    [InlineData(1.0, 0.1, true)]
    [InlineData(10.0, 1.0, true)]
    [InlineData(0.5, 0.25, false)]
    [InlineData(11.0, 0.25, false)]
    [InlineData(3.0, 0.05, false)]
    [InlineData(3.0, 1.5, false)]
    public void ReplayLengthAndSpeedAreBounded(double seconds, double speed, bool valid)
    {
        Assert.Equal(valid, new WalkReplayRequestValidator().Validate(new WalkReplayRequest(seconds, speed)).IsValid);
    }

    [Fact]
    public void ReplayStopAndAutoNeedNoLength()
    {
        Assert.True(new WalkReplayRequestValidator().Validate(new WalkReplayRequest(Stop: true)).IsValid);
        Assert.True(new WalkReplayRequestValidator().Validate(new WalkReplayRequest(AutoOnFall: false)).IsValid);
    }

    [Theory]
    [InlineData(0.0, true)]
    [InlineData(1.0, true)]
    [InlineData(-0.1, false)]
    [InlineData(1.1, false)]
    public void FilterAlphaIsZeroToOne(double alpha, bool valid)
    {
        Assert.Equal(valid, new WalkFilterRequestValidator().Validate(new WalkFilterRequest(alpha)).IsValid);
    }

    [Theory]
    [InlineData("procedural", "free_hold", 0L, 0.0, true)]
    [InlineData("procedural", "policy", 123456L, 1.0, true)]
    [InlineData("procedural", "hold", -1L, 0.5, false)]
    [InlineData("procedural", "hold", 1L, 1.5, false)]
    [InlineData("procedural", "hold", 1L, -0.1, false)]
    [InlineData("moon", "hold", 1L, 0.5, false)]
    [InlineData("flat", "dance", 1L, 0.5, false)]
    public void WalkStartChecksCourseModeSeedAndDifficulty(string course, string mode, long seed, double difficulty, bool valid)
    {
        Assert.Equal(valid, new WalkStartRequestValidator().Validate(new WalkStartRequest(course, mode, seed, difficulty)).IsValid);
    }

    [Fact]
    public void WalkStartWithoutProceduralFieldsIsValid()
    {
        Assert.True(new WalkStartRequestValidator().Validate(new WalkStartRequest("loop", "policy")).IsValid);
    }

    [Theory]
    [InlineData("a.csv", "b.csv", 5, true)]
    [InlineData("a.csv", "a.csv", 5, false)]
    [InlineData("", "b.csv", 5, false)]
    [InlineData("../a.csv", "b.csv", 5, false)]
    [InlineData("a.csv", "b.csv", 0, false)]
    public void GapNeedsTwoDifferentLogFileNames(string a, string b, double tol, bool valid)
    {
        Assert.Equal(valid, new GapRequestValidator().Validate(new GapRequest(a, b, tol)).IsValid);
    }

    [Theory]
    [InlineData("look_at", null, 2, "lab_stress", true)]
    [InlineData("look_at", 1, 2, "lab_calm", true)]
    [InlineData("look_at", null, null, "lab_stress", false)]
    [InlineData("look_at", 2, 2, "lab_stress", false)]
    [InlineData("", 1, 2, "lab_stress", false)]
    [InlineData("look_at", 1, 2, "", false)]
    public void DuelNeedsSkillScenarioAndTwoVersions(string skill, int? a, int? b, string scenario, bool valid)
    {
        Assert.Equal(valid, new DuelRequestValidator().Validate(new DuelRequest(skill, a, b, scenario)).IsValid);
    }

    // ------------------------------------------------------------------ payloads

    [Fact]
    public void RequestsUseServiceFieldNamesAndLeaveOutNulls()
    {
        var ctx = RobotJsonContext.Default;
        Assert.Equal("""{"course":"procedural","mode":"free_hold","seed":7,"difficulty":0.5}""",
            JsonSerializer.Serialize(new WalkStartRequest("procedural", "free_hold", 7, 0.5), ctx.WalkStartRequest));
        Assert.Equal("""{"course":"loop","mode":"policy"}""", JsonSerializer.Serialize(new WalkStartRequest(), ctx.WalkStartRequest));
        Assert.Equal("""{"kind":"box","speed":5,"from_deg":90}""", JsonSerializer.Serialize(new WalkThrowRequest("box", 5, 90), ctx.WalkThrowRequest));
        Assert.Equal("""{"seconds":3,"speed":0.25}""", JsonSerializer.Serialize(new WalkReplayRequest(3, 0.25), ctx.WalkReplayRequest));
        Assert.Equal("""{"stop":true}""", JsonSerializer.Serialize(new WalkReplayRequest(Stop: true), ctx.WalkReplayRequest));
        Assert.Equal("""{"auto_on_fall":true}""", JsonSerializer.Serialize(new WalkReplayRequest(AutoOnFall: true), ctx.WalkReplayRequest));
        Assert.Equal("""{"alpha":0.5,"apply":false}""", JsonSerializer.Serialize(new WalkFilterRequest(), ctx.WalkFilterRequest));
        Assert.Equal("""{"id":"push_test"}""", JsonSerializer.Serialize(new WalkScenarioRequest("push_test"), ctx.WalkScenarioRequest));
        Assert.Equal("""{"skill":"look_at","a":null,"b":2,"scenario":"lab_stress"}""",
            JsonSerializer.Serialize(new DuelRequest("look_at", null, 2, "lab_stress"), ctx.DuelRequest));
        Assert.Equal("""{"a":"x.csv","b":"y.csv","tol_deg":5}""", JsonSerializer.Serialize(new GapRequest("x.csv", "y.csv"), ctx.GapRequest));
        Assert.Equal("""{"name":"Look up","yaw_deg":0,"pitch_deg":-10}""", JsonSerializer.Serialize(new TaughtPoseRequest("Look up", 0, -10), ctx.TaughtPoseRequest));
    }

    [Fact]
    public void WalkStateWithEverySimFeatureDeserializes()
    {
        const string json = """
        {"ok": true, "policy_available": false, "policy_reason": null, "running": true, "mode": "free_hold", "course": "procedural",
         "t": 12.5, "seed": 42, "difficulty": 0.5,
         "physics": {"com": [0.01, 0.0, 0.62], "com_vel": [0.1, 0.0, -0.2], "capture_point": [0.04, 0.0],
                     "support": [[0.1, 0.1], [0.1, -0.1], [-0.1, -0.1]], "cp_margin_m": 0.03,
                     "contacts": [{"pos": [0.1, 0.1, 0.0], "force": [1.0, 2.0, 120.0], "a": "left_foot", "b": "floor"}]},
         "motors": {"names": ["a", "b"], "speed": [0.1, 0.2], "torque": [1.0, 2.0], "torque_frac": [0.1, 0.9]},
         "thermal": {"temp_c": [25.0, 72.0], "hottest": "b", "hottest_c": 72.0, "warn": ["b"], "limit": []},
         "battery": {"soc_pct": 99.7, "power_w": 312.0, "minutes_left": 92.0},
         "action_filter": {"alpha": 0.5, "apply": false, "raw": [0.1, -0.2], "filtered": [0.05, -0.1], "jitter": 0.012},
         "props": [{"id": 0, "kind": "ball", "pos": [1, 0, 0.6], "quat": [1, 0, 0, 0], "size": [0.1]},
                   {"id": 1, "kind": "box", "pos": [0, 1, 0.6], "quat": [1, 0, 0, 0], "size": [0.125, 0.125, 0.125]}],
         "fall_analysis": {"t_fall": 12.34, "t_unstable": 11.92, "lead_s": 0.42, "reason": "capture point left the feet (6 cm outside)",
                           "cause": "cp_outside", "history": [{"t": 11.0, "cp_margin_m": 0.02, "tilt_deg": 3.0, "height": 0.7},
                                                              {"t": 11.1, "cp_margin_m": null, "tilt_deg": 5.0, "height": 0.69}]},
         "falls_log": [{"t_fall": 12.34, "t_unstable": 11.92, "lead_s": 0.42, "cause": "cp_outside"}],
         "replay": {"active": true, "speed": 0.25, "t": 11.0, "duration_s": 3.0, "auto_on_fall": true},
         "scenario": {"id": "push_test", "t": 4.2, "step": 2, "steps": 5, "done": false}}
        """;
        var s = JsonSerializer.Deserialize(json, RobotJsonContext.Default.WalkStateDto)!;
        Assert.Equal(42, s.Seed);
        Assert.Equal(0.03, s.Physics!.CpMarginM);
        Assert.Equal(3, s.Physics.Support!.Count);
        Assert.Equal("floor", s.Physics.Contacts![0].B);
        Assert.Equal(0.9, s.Motors!.TorqueFrac![1]);
        Assert.Equal(["b"], s.Thermal!.Warn);
        Assert.Equal(92.0, s.Battery!.MinutesLeft);
        Assert.Equal(0.012, s.ActionFilter!.Jitter);
        Assert.Equal("box", s.Props![1].Kind);
        Assert.Equal(3, s.Props[1].Size!.Length);
        Assert.Equal("cp_outside", s.FallAnalysis!.Cause);
        Assert.Null(s.FallAnalysis.History![1].CpMarginM);
        Assert.Single(s.FallsLog!);
        Assert.True(s.Replay!.Active);
        Assert.Equal(2, s.Scenario!.Step);
    }

    [Fact]
    public void WalkStateWithoutASimHasNullSimFeatures()
    {
        const string json = """
        {"ok": true, "policy_available": true, "policy_reason": null, "running": false, "physics": null, "motors": null,
         "thermal": null, "battery": null, "action_filter": null, "props": null, "fall_analysis": null, "falls_log": null,
         "replay": null, "scenario": null, "seed": null, "difficulty": null}
        """;
        var s = JsonSerializer.Deserialize(json, RobotJsonContext.Default.WalkStateDto)!;
        Assert.Null(s.Physics);
        Assert.Null(s.Replay);
        Assert.Null(s.Seed);
    }

    [Fact]
    public void SkillToolRepliesDeserialize()
    {
        var ctx = RobotJsonContext.Default;
        var duel = JsonSerializer.Deserialize("""
            {"ok": true, "result": {"skill": "look_at", "scenario": "lab_stress",
             "a": {"version": 3, "err_mean_deg": 1.2, "err_max_deg": 4.5, "torque_rms": 0.8, "jitter": 0.01, "energy_j": 3.2, "would_stop": "", "score": 2.1},
             "b": {"version": 2, "err_mean_deg": 1.9, "err_max_deg": 6.0, "torque_rms": 0.9, "jitter": 0.02, "energy_j": 3.9, "would_stop": "torque", "score": 3.4},
             "winner": "a", "why": "lower tracking error"}}
            """, ctx.DuelResponseDto)!;
        Assert.Equal("a", duel.Result!.Winner);
        Assert.Equal(3, duel.Result.A!.Version);
        Assert.Equal("torque", duel.Result.B!.WouldStop);

        var gap = JsonSerializer.Deserialize("""
            {"ok": true, "joints": ["head_pitch_joint", "head_yaw_joint"], "t": [0.0, 0.02],
             "a_deg": [[0, 0], [1, 2]], "b_deg": [[0, 0], [1.5, 1]], "err_deg": [[0, 0], [0.5, -1]],
             "max_deg": [0.5, 1.0], "rms_deg": [0.35, 0.7], "pass": true}
            """, ctx.GapResultDto)!;
        Assert.True(gap.Pass);
        Assert.Equal(2, gap.ErrDeg!.Count);
        Assert.Equal(-1, gap.ErrDeg[1][1]);

        var montage = JsonSerializer.Deserialize("""{"ok": true, "frames": 4, "checkpoints": [50, 100, 150, 200], "gif_base64": "R0lGODlh"}""", ctx.MontageDto)!;
        Assert.Equal([50, 100, 150, 200], montage.Checkpoints);
        var missing = JsonSerializer.Deserialize("""{"ok": false, "reason": "no checkpoints: train with --checkpoint-every"}""", ctx.MontageDto)!;
        Assert.StartsWith("no checkpoints", missing.Reason, StringComparison.Ordinal);

        var runs = JsonSerializer.Deserialize("""{"ok": true, "runs": [{"file": "look_at_urlab_20261122_191054.csv", "skill": "look_at", "target": "urlab", "stamp": "20261122_191054"}]}""", ctx.RunsDto)!;
        Assert.Equal("urlab", runs.Runs![0].Target);

        var sc = JsonSerializer.Deserialize("""{"ok": true, "scenarios": [{"id": "lab_calm", "name": "Calm", "target": "lab", "description": "", "duration_s": 12}]}""", ctx.ScenariosDto)!;
        Assert.Equal(12, sc.Scenarios![0].DurationS);
    }

    [Fact]
    public void TaughtPosesKeepTheirIdAndSkipConsolePresets()
    {
        // GET /poses also serves the console's preset poses (name/label/q_deg); only taught ones have id + angles
        const string json = """
        {"ok": true, "poses": [
          {"name": "look_left", "label": "Look left", "say": "", "words": [], "q_deg": {"head_yaw_joint": 30.0}},
          {"id": 3, "name": "Look up", "yaw_deg": 0.0, "pitch_deg": -10.0, "skill": "pose_3"},
          {"id": "a1b2", "name": "Side", "yaw_deg": 20.0, "pitch_deg": 5.0}]}
        """;
        var poses = JsonSerializer.Deserialize(json, RobotJsonContext.Default.TaughtPosesDto)!;
        Assert.Equal(2, poses.Taught.Count);
        Assert.Equal("3", poses.Taught[0].IdText);
        Assert.Equal("pose_3", poses.Taught[0].SkillId);
        Assert.Equal("a1b2", poses.Taught[1].IdText);
        Assert.Equal("pose_a1b2", poses.Taught[1].SkillId);

        // a delete sends the id back exactly as the service gave it (number stays a number)
        var del = JsonSerializer.Serialize(new TaughtPoseDeleteRequest(poses.Taught[0].Id!.Value), RobotJsonContext.Default.TaughtPoseDeleteRequest);
        Assert.Equal("""{"id":3}""", del);
        var delText = JsonSerializer.Serialize(new TaughtPoseDeleteRequest(poses.Taught[1].Id!.Value), RobotJsonContext.Default.TaughtPoseDeleteRequest);
        Assert.Equal("""{"id":"a1b2"}""", delText);
    }

    // ------------------------------------------------------------------ helpers

    [Fact]
    public void PoseFromJointsFillsDefaultsAndIgnoresUnknownNames()
    {
        var q = SimFeatureMath.PoseFromJoints(["head_yaw_joint", "no_such_joint", "head_pitch_joint"], [12.0, 99.0, double.NaN]);
        Assert.Equal(26, q.Length);
        var yaw = RobotJointCatalog.All.Single(j => j.Name == "head_yaw_joint").PolicyIndex;
        var pitch = RobotJointCatalog.All.Single(j => j.Name == "head_pitch_joint");
        Assert.Equal(12.0, q[yaw]);
        Assert.Equal(pitch.DefaultDeg, q[pitch.PolicyIndex]); // NaN keeps the default
        Assert.Equal(RobotJointCatalog.All[0].DefaultDeg, q[0]);
        Assert.Equal(26, SimFeatureMath.PoseFromJoints(["head_yaw_joint"], null).Length);
    }

    [Fact]
    public void HeadPoseSetsOnlyTheHead()
    {
        var q = SimFeatureMath.HeadPose(20, -5);
        var defaults = RobotJointCatalog.All.Select(j => j.DefaultDeg).ToArray();
        var changed = Enumerable.Range(0, 26).Where(i => q[i] != defaults[i]).Select(i => RobotJointCatalog.All[i].Name).ToList();
        Assert.Equal(["head_pitch_joint", "head_yaw_joint"], changed);
    }

    [Theory]
    [InlineData("cp_outside", "capture point left the feet")]
    [InlineData("prop_hit", "hit by a thrown object")]
    [InlineData(null, "unknown")]
    [InlineData("new_cause", "new cause")]
    public void FallCausesReadInPlainWords(string? cause, string text)
    {
        Assert.Equal(text, SimFeatureMath.FallCauseText(cause));
    }

    [Fact]
    public void NearestIndexAndErrorColour()
    {
        Assert.Equal(-1, SimFeatureMath.NearestIndex([], 1));
        Assert.Equal(2, SimFeatureMath.NearestIndex([0.0, 0.5, 1.0, 1.5], 0.9));
        Assert.Equal(3, SimFeatureMath.NearestIndex([0.0, 0.5, 1.0, 1.5], 9));
        Assert.Equal((0x2e, 0x7d, 0x4f, 255), ToInts(SimFeatureMath.ErrorColour(0, 5)));
        Assert.Equal((0xff, 0xbd, 0x42, 255), ToInts(SimFeatureMath.ErrorColour(5, 5)));
        Assert.Equal((0xff, 0x5c, 0x57, 255), ToInts(SimFeatureMath.ErrorColour(-50, 5)));
    }

    private static (int, int, int, int) ToInts((byte R, byte G, byte B, byte A) c) => (c.R, c.G, c.B, c.A);
}
