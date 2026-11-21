using System.Text.Json;
using PoRobotTrain.Shared.Contracts;
using Xunit;

namespace PoRobotTrain.Unit;

public sealed class RobotJointCatalogTests
{
    [Fact]
    public void CatalogHasAll26UniqueR1JointsWithinValidLimits()
    {
        var joints = RobotJointCatalog.All;
        Assert.Equal(26, joints.Count);
        Assert.Equal(Enumerable.Range(0, 26), joints.Select(j => j.PolicyIndex));
        Assert.Equal(26, joints.Select(j => j.Name).Distinct().Count());
        Assert.Equal(26, joints.Select(j => j.MotorSlot).Distinct().Count());

        foreach (var j in joints)
        {
            Assert.True(j.MinDeg < j.MaxDeg, $"{j.Name} min >= max");
            Assert.InRange(j.DefaultDeg, j.MinDeg, j.MaxDeg);
        }
    }

    [Fact]
    public void GroupOrderCoversAll26Joints()
    {
        var total = RobotJointCatalog.GroupOrder
            .Sum(g => RobotJointCatalog.All.Count(j => j.Group == g.Group));
        Assert.Equal(26, total);
    }

    [Fact]
    public void StatusDtoDeserializesPythonServicePayload()
    {
        const string json = """
        {
          "job": null,
          "last_result": "preview OK",
          "previewed": {"behaviour": "hand_wave", "amplitude": 20.0, "freq": 0.2, "duration": 20.0},
          "checked": null,
          "check_age_s": null,
          "can_run_robot": false,
          "why_not_run": "run check",
          "mirror_samples": 0,
          "results": {},
          "head": {},
          "telemetry": {},
          "behaviours": [{"id": "hand_wave", "label": "Hand Wave", "desc": "wave", "primary_label": "R.ARM WAVE", "max_deg": 20.0}],
          "joint_names": ["left_hip_pitch"],
          "joint_limits_deg": [[-168.0, 146.0]],
          "joint_defaults_deg": [-5.7],
          "lines": ["ready"]
        }
        """;
        var dto = JsonSerializer.Deserialize<RobotStatusDto>(json);
        Assert.NotNull(dto);
        Assert.Equal("hand_wave", dto.Previewed?.Behaviour);
        Assert.Single(dto.Behaviours);
        Assert.Equal(20.0, dto.Behaviours[0].MaxDeg);
    }
}
