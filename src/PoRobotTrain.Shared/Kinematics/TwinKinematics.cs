using Silk.NET.Maths;

namespace PoRobotTrain.Shared.Kinematics;

/// <summary>World pose of one body in the MuJoCo frame (z up, metres).</summary>
public readonly record struct BodyPose(Vector3D<double> Position, Quaternion<double> Rotation);

/// <summary>A named point of interest (hand / face) for a pose.</summary>
public sealed record EndEffectorPoint(string Label, string Body, Vector3D<double> Position);

/// <summary>
/// Forward kinematics of the R1 stand twin with Silk.NET.Maths, from the same manifest the browser view uses.
/// Pure geometry for read-outs and tests; MuJoCo owns physics and nothing here commands the robot.
/// </summary>
public sealed class TwinKinematics
{
    private readonly TwinManifestDto _manifest;
    private readonly Dictionary<string, Vector3D<double>> _tips;

    public TwinKinematics(TwinManifestDto manifest)
    {
        ArgumentNullException.ThrowIfNull(manifest);
        _manifest = manifest;
        _tips = manifest.Bodies.ToDictionary(b => b.Name, b => TipOffset(b.Name), StringComparer.Ordinal);
    }

    public IReadOnlyList<TwinJointDto> Joints => _manifest.Joints;

    public static Quaternion<double> ToQuat(IReadOnlyList<double> xyzw)
    {
        ArgumentNullException.ThrowIfNull(xyzw);
        return new Quaternion<double>(xyzw[0], xyzw[1], xyzw[2], xyzw[3]);
    }

    public static Vector3D<double> ToVec(IReadOnlyList<double> xyz)
    {
        ArgumentNullException.ThrowIfNull(xyz);
        return new Vector3D<double>(xyz[0], xyz[1], xyz[2]);
    }

    /// <summary>Body poses for joint angles in policy order (degrees); null or non-finite entries use defaults.</summary>
    public IReadOnlyDictionary<string, BodyPose> Forward(IReadOnlyList<double>? qDeg)
    {
        var poses = new Dictionary<string, BodyPose>(_manifest.Bodies.Count, StringComparer.Ordinal);
        foreach (var b in _manifest.Bodies)
        {
            var local = ToQuat(b.Quat);
            if (b.Joint is int ji)
            {
                var j = _manifest.Joints[ji];
                var deg = qDeg is not null && ji < qDeg.Count && double.IsFinite(qDeg[ji]) ? qDeg[ji] : j.DefaultDeg;
                var axis = Vector3D.Normalize(ToVec(j.Axis));
                local = Quaternion<double>.Multiply(local, Quaternion<double>.CreateFromAxisAngle(axis, deg * Math.PI / 180.0));
            }

            var pos = ToVec(b.Pos);
            if (b.Parent is null)
            {
                poses[b.Name] = new BodyPose(pos, local);
                continue;
            }

            var parent = poses[b.Parent];
            poses[b.Name] = new BodyPose(
                parent.Position + Vector3D.Transform(pos, parent.Rotation),
                Quaternion<double>.Multiply(parent.Rotation, local));
        }

        return poses;
    }

    /// <summary>Right hand, left hand and face points (farthest collider end on the wrist / head bodies).</summary>
    public IReadOnlyList<EndEffectorPoint> EndEffectors(IReadOnlyList<double>? qDeg)
    {
        var fk = Forward(qDeg);
        return
        [
            Point(fk, "R hand", "right_wrist_roll_link"),
            Point(fk, "L hand", "left_wrist_roll_link"),
            Point(fk, "Face", "head_yaw_link"),
        ];
    }

    private EndEffectorPoint Point(IReadOnlyDictionary<string, BodyPose> fk, string label, string body)
    {
        var pose = fk[body];
        return new EndEffectorPoint(label, body, pose.Position + Vector3D.Transform(_tips[body], pose.Rotation));
    }

    private Vector3D<double> TipOffset(string body)
    {
        var best = Vector3D<double>.Zero;
        var bestLen = -1.0;
        foreach (var c in _manifest.Colliders.Where(c => c.Body == body))
        {
            var center = ToVec(c.Pos);
            var candidates = new List<Vector3D<double>> { center };
            if (c.Type is "capsule" or "cylinder")
            {
                var ext = c.Size[1] + (c.Type == "capsule" ? c.Size[0] : 0.0);
                var along = Vector3D.Transform(new Vector3D<double>(0, 0, ext), ToQuat(c.Quat));
                candidates = [center + along, center - along];
            }

            foreach (var p in candidates)
            {
                var len = p.Length;
                if (len > bestLen)
                {
                    bestLen = len;
                    best = p;
                }
            }
        }

        return best;
    }
}
