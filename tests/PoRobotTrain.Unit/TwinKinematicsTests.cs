using System.Text.Json;
using PoRobotTrain.Shared.Contracts;
using PoRobotTrain.Shared.Kinematics;
using Silk.NET.Maths;
using Xunit;

namespace PoRobotTrain.Unit;

public sealed class TwinKinematicsTests
{
    private static readonly Lazy<TwinManifestDto> Manifest = new(LoadManifest);

    private static TwinManifestDto LoadManifest() => LoadManifestFrom();

    // Anchored on this source file (not the test binary) so custom --artifacts-path / OutDir builds still find the repo.
    private static TwinManifestDto LoadManifestFrom([System.Runtime.CompilerServices.CallerFilePath] string sourceFile = "")
    {
        var dir = new DirectoryInfo(Path.GetDirectoryName(sourceFile) ?? AppContext.BaseDirectory);
        while (dir is not null && !File.Exists(Path.Combine(dir.FullName, "PoRobotTrain.slnx")))
        {
            dir = dir.Parent;
        }

        Assert.NotNull(dir);
        var path = Path.Combine(dir.FullName, "src", "PoRobotTrain.Client", "wwwroot", "twin", "twin_manifest.json");
        Assert.True(File.Exists(path), $"run scripts/build_web_twin.py first ({path} missing)");
        var manifest = JsonSerializer.Deserialize(File.ReadAllText(path), TwinJsonContext.Default.TwinManifestDto);
        Assert.NotNull(manifest);
        return manifest;
    }

    [Fact]
    public void SilkForwardKinematicsMatchesMuJoCoGoldenPoses()
    {
        var fk = new TwinKinematics(Manifest.Value);
        Assert.NotEmpty(Manifest.Value.FkGolden);

        foreach (var golden in Manifest.Value.FkGolden)
        {
            var poses = fk.Forward(golden.QDeg);
            foreach (var (body, reference) in golden.Bodies)
            {
                var got = poses[body];
                var dp = Vector3D.Distance(got.Position, TwinKinematics.ToVec(reference.Pos));
                Assert.True(dp < 1e-4, $"{body}: position off by {dp * 1000:0.000} mm");

                var dot = Math.Abs(Quaternion<double>.Dot(got.Rotation, TwinKinematics.ToQuat(reference.Quat)));
                Assert.True(dot > 1 - 1e-6, $"{body}: rotation differs (|dot| = {dot})");
            }
        }
    }

    [Fact]
    public void TwinJointOrderAndLimitsMatchTheJointCatalog()
    {
        var joints = Manifest.Value.Joints;
        Assert.Equal(RobotJointCatalog.All.Count, joints.Count);
        foreach (var spec in RobotJointCatalog.All)
        {
            var j = joints[spec.PolicyIndex];
            Assert.Equal(spec.Name, j.Name);
            Assert.Equal(spec.PolicyIndex, j.PolicyIndex);
            Assert.InRange(j.MinDeg, spec.MinDeg - 0.6, spec.MinDeg + 0.6);
            Assert.InRange(j.MaxDeg, spec.MaxDeg - 0.6, spec.MaxDeg + 0.6);
        }
    }

    [Fact]
    public void EndEffectorsMoveWithTheirJoint()
    {
        var fk = new TwinKinematics(Manifest.Value);
        var home = fk.EndEffectors(null).ToDictionary(p => p.Label);
        var q = Manifest.Value.Joints.Select(j => j.DefaultDeg).ToArray();
        q[RobotJointCatalog.All.Single(j => j.Name == "right_shoulder_pitch_joint").PolicyIndex] -= 30;
        var raised = fk.EndEffectors(q).ToDictionary(p => p.Label);

        Assert.True(raised["R hand"].Position.Z > home["R hand"].Position.Z + 0.05, "raising the right shoulder lifts the right hand");
        Assert.Equal(home["L hand"].Position, raised["L hand"].Position);
        Assert.Equal(home["Face"].Position, raised["Face"].Position);
    }
}
