namespace PoRobotTrain.Shared.Contracts;

/// <summary>Small pure helpers for the simulation feature views (docs/sim_features.md), shared so they are unit-tested.</summary>
public static class SimFeatureMath
{
    /// <summary>The 26-joint pose (degrees, joint-map order) with the default angle everywhere except the named joints.
    /// Unknown names are ignored, missing values keep the default.</summary>
    public static double[] PoseFromJoints(IReadOnlyList<string> joints, IReadOnlyList<double>? valuesDeg)
    {
        var q = RobotJointCatalog.All.Select(j => j.DefaultDeg).ToArray();
        if (valuesDeg is null)
        {
            return q;
        }
        for (var i = 0; i < joints.Count && i < valuesDeg.Count; i++)
        {
            var spec = RobotJointCatalog.All.FirstOrDefault(j => j.Name == joints[i]);
            if (spec is not null && double.IsFinite(valuesDeg[i]))
            {
                q[spec.PolicyIndex] = valuesDeg[i];
            }
        }
        return q;
    }

    /// <summary>The default pose with the head turned to a taught pose (yaw = head_yaw_joint, pitch = head_pitch_joint).</summary>
    public static double[] HeadPose(double yawDeg, double pitchDeg) =>
        PoseFromJoints(["head_yaw_joint", "head_pitch_joint"], [yawDeg, pitchDeg]);

    /// <summary>Plain words for a fall cause (cp_outside | tilt | height | prop_hit).</summary>
    public static string FallCauseText(string? cause) => cause switch
    {
        "cp_outside" => "capture point left the feet",
        "tilt" => "tilted too far",
        "height" => "pelvis dropped too low",
        "prop_hit" => "hit by a thrown object",
        null or "" => "unknown",
        _ => cause.Replace('_', ' '),
    };

    /// <summary>Index of the sample whose time is closest to t (times ascending); -1 when there are none.</summary>
    public static int NearestIndex(IReadOnlyList<double>? times, double t)
    {
        if (times is null || times.Count == 0)
        {
            return -1;
        }
        var best = 0;
        for (var i = 1; i < times.Count; i++)
        {
            if (Math.Abs(times[i] - t) < Math.Abs(times[best] - t))
            {
                best = i;
            }
        }
        return best;
    }

    /// <summary>Heatmap colour for an error against a tolerance: green (0) through amber (tol) to red (2 x tol), RGBA.</summary>
    public static (byte R, byte G, byte B, byte A) ErrorColour(double errDeg, double tolDeg)
    {
        var u = tolDeg > 0 ? Math.Clamp(Math.Abs(errDeg) / (2 * tolDeg), 0, 1) : 1;
        // 0 -> #2e7d4f, 0.5 -> #ffbd42, 1 -> #ff5c57
        static byte Lerp(double a, double b, double k) => (byte)Math.Round(a + (b - a) * k);
        return u < 0.5
            ? (Lerp(0x2e, 0xff, u * 2), Lerp(0x7d, 0xbd, u * 2), Lerp(0x4f, 0x42, u * 2), 255)
            : (Lerp(0xff, 0xff, (u - 0.5) * 2), Lerp(0xbd, 0x5c, (u - 0.5) * 2), Lerp(0x42, 0x57, (u - 0.5) * 2), 255);
    }
}
