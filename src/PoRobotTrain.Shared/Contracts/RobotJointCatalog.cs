namespace PoRobotTrain.Shared.Contracts;

public enum JointGroup
{
    Head,
    Waist,
    LeftArm,
    RightArm,
    LeftLeg,
    RightLeg
}

public sealed record JointSpec(
    int PolicyIndex,
    string Name,
    string DisplayLabel,
    int MotorSlot,
    JointGroup Group,
    double MinDeg,
    double MaxDeg,
    double DefaultDeg);

/// <summary>
/// C# mirror of <c>RobotControl/configs/r1_joint_map.yaml</c> (26-DOF Unitree R1 EDU A5).
/// </summary>
public static class RobotJointCatalog
{
    public static IReadOnlyList<JointSpec> All { get; } =
    [
        new(0,  "left_hip_pitch_joint",       "L Hip Pitch",    0,  JointGroup.LeftLeg,  -168.0, 146.0,  -5.7),
        new(1,  "left_hip_roll_joint",        "L Hip Roll",     1,  JointGroup.LeftLeg,   -60.0, 100.0,   0.0),
        new(2,  "left_hip_yaw_joint",         "L Hip Yaw",      2,  JointGroup.LeftLeg,  -157.0, 157.0,   0.0),
        new(3,  "left_knee_joint",            "L Knee",         3,  JointGroup.LeftLeg,   -10.0, 139.0,  17.2),
        new(4,  "left_ankle_pitch_joint",     "L Ankle Pitch",  4,  JointGroup.LeftLeg,   -50.0,  33.0, -11.5),
        new(5,  "left_ankle_roll_joint",      "L Ankle Roll",   5,  JointGroup.LeftLeg,   -15.0,  15.0,   0.0),
        new(6,  "right_hip_pitch_joint",      "R Hip Pitch",    6,  JointGroup.RightLeg, -168.0, 146.0,  -5.7),
        new(7,  "right_hip_roll_joint",       "R Hip Roll",     7,  JointGroup.RightLeg, -100.0,  60.0,   0.0),
        new(8,  "right_hip_yaw_joint",        "R Hip Yaw",      8,  JointGroup.RightLeg, -157.0, 157.0,   0.0),
        new(9,  "right_knee_joint",           "R Knee",         9,  JointGroup.RightLeg,  -10.0, 139.0,  17.2),
        new(10, "right_ankle_pitch_joint",    "R Ankle Pitch", 10,  JointGroup.RightLeg,  -50.0,  33.0, -11.5),
        new(11, "right_ankle_roll_joint",     "R Ankle Roll",  11,  JointGroup.RightLeg,  -15.0,  15.0,   0.0),
        new(12, "waist_roll_joint",           "Waist Roll",    12,  JointGroup.Waist,     -30.0,  30.0,   0.0),
        new(13, "waist_yaw_joint",            "Waist Yaw",     13,  JointGroup.Waist,    -150.0, 150.0,   0.0),
        new(14, "left_shoulder_pitch_joint",  "L Shld Pitch",  15,  JointGroup.LeftArm,  -180.0, 120.0,  20.1),
        new(15, "left_shoulder_roll_joint",   "L Shld Roll",   16,  JointGroup.LeftArm,   -13.0, 142.0,  10.3),
        new(16, "left_shoulder_yaw_joint",    "L Shld Yaw",    17,  JointGroup.LeftArm,  -110.0, 110.0,   0.0),
        new(17, "left_elbow_joint",           "L Elbow",       18,  JointGroup.LeftArm,   -55.9, 125.2,  49.8),
        new(18, "left_wrist_roll_joint",      "L Wrist Roll",  19,  JointGroup.LeftArm,  -110.0, 110.0,   0.0),
        new(19, "right_shoulder_pitch_joint", "R Shld Pitch",  22,  JointGroup.RightArm, -180.0, 120.0,  20.1),
        new(20, "right_shoulder_roll_joint",  "R Shld Roll",   23,  JointGroup.RightArm, -142.0,  13.0, -10.3),
        new(21, "right_shoulder_yaw_joint",   "R Shld Yaw",    24,  JointGroup.RightArm, -110.0, 110.0,   0.0),
        new(22, "right_elbow_joint",          "R Elbow",       25,  JointGroup.RightArm,  -55.9, 125.2,  49.8),
        new(23, "right_wrist_roll_joint",     "R Wrist Roll",  26,  JointGroup.RightArm, -110.0, 110.0,   0.0),
        new(24, "head_pitch_joint",           "Head Pitch",    29,  JointGroup.Head,      -36.0,  36.0,   0.0),
        new(25, "head_yaw_joint",             "Head Yaw",      30,  JointGroup.Head,     -115.0, 115.0,   0.0),
    ];

    public static IReadOnlyList<(JointGroup Group, string Title, string ColorHex)> GroupOrder { get; } =
    [
        (JointGroup.Head,     "HEAD",      "#3dc7f2"),
        (JointGroup.Waist,    "WAIST",     "#a68cf2"),
        (JointGroup.LeftArm,  "LEFT ARM",  "#4e92ff"),
        (JointGroup.RightArm, "RIGHT ARM", "#52d984"),
        (JointGroup.LeftLeg,  "LEFT LEG",  "#ffbd42"),
        (JointGroup.RightLeg, "RIGHT LEG", "#ff8c3a"),
    ];
}
