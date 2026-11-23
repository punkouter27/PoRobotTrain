using FluentValidation;
using PoRobotTrain.Shared.Contracts;

namespace PoRobotTrain.Shared.Validation;

// Simulation feature rules (docs/sim_features.md), the same ranges the behaviour service enforces, so the operator
// sees them inline before a request is refused. Registered by type in the Client Program.cs; the API also checks
// the walk requests before forwarding them.

/// <summary>#3: course, mode, seed and difficulty of a walk start.</summary>
public sealed class WalkStartRequestValidator : AbstractValidator<WalkStartRequest>
{
    public WalkStartRequestValidator()
    {
        RuleFor(r => r.Course)
            .Must(c => c is "flat" or "loop" or "procedural").WithMessage("Course must be flat, loop or procedural.");
        RuleFor(r => r.Mode)
            .Must(m => m is "policy" or "hold" or "free_hold").WithMessage("Mode must be policy, hold or free_hold.");
        RuleFor(r => r.Seed)
            .GreaterThanOrEqualTo(0).When(r => r.Seed is not null).WithMessage("Seed must be 0 or more.");
        RuleFor(r => r.Difficulty)
            .InclusiveBetween(0.0, 1.0).When(r => r.Difficulty is not null).WithMessage("Difficulty must be 0-1.");
    }
}

/// <summary>#9: ball or box, 1-8 m/s.</summary>
public sealed class WalkThrowRequestValidator : AbstractValidator<WalkThrowRequest>
{
    public WalkThrowRequestValidator()
    {
        RuleFor(r => r.Kind)
            .Must(k => k is "ball" or "box").WithMessage("Throw a ball or a box.");
        RuleFor(r => r.Speed)
            .InclusiveBetween(1.0, 8.0).WithMessage("Speed must be 1-8 m/s.");
        RuleFor(r => r.FromDeg)
            .InclusiveBetween(-360.0, 360.0).WithMessage("Direction must be -360 to 360°.");
    }
}

/// <summary>#12: replay the last 1-10 s at 0.1-1x.</summary>
public sealed class WalkReplayRequestValidator : AbstractValidator<WalkReplayRequest>
{
    public WalkReplayRequestValidator()
    {
        RuleFor(r => r.Seconds)
            .InclusiveBetween(1.0, 10.0).When(r => r.Seconds is not null).WithMessage("Replay length must be 1-10 s.");
        RuleFor(r => r.Speed)
            .InclusiveBetween(0.1, 1.0).When(r => r.Speed is not null).WithMessage("Replay speed must be 0.1-1x.");
    }
}

/// <summary>#15: filter alpha 0-1.</summary>
public sealed class WalkFilterRequestValidator : AbstractValidator<WalkFilterRequest>
{
    public WalkFilterRequestValidator()
    {
        RuleFor(r => r.Alpha)
            .InclusiveBetween(0.0, 1.0).WithMessage("Filter alpha must be 0-1.");
    }
}

/// <summary>#14: same rule as behaviours/taught_poses.py.</summary>
public sealed class TaughtPoseRequestValidator : AbstractValidator<TaughtPoseRequest>
{
    public TaughtPoseRequestValidator()
    {
        RuleFor(r => r.Name)
            .Matches("^[A-Za-z0-9 _-]{1,32}$").WithMessage("Name: 1-32 letters, digits, spaces, - or _.");
        RuleFor(r => r.Name)
            .Must(n => !string.IsNullOrWhiteSpace(n)).WithMessage("Give the pose a name.");
        RuleFor(r => r.YawDeg)
            .InclusiveBetween(-30.0, 30.0).WithMessage("Head yaw must be -30 to 30°.");
        RuleFor(r => r.PitchDeg)
            .InclusiveBetween(-15.0, 20.0).WithMessage("Head pitch must be -15 to 20°.");
    }
}

/// <summary>#4: two different log files (names only, no paths) and a tolerance.</summary>
public sealed class GapRequestValidator : AbstractValidator<GapRequest>
{
    private const string FileRule = "^[A-Za-z0-9_.-]{1,200}$";

    public GapRequestValidator()
    {
        RuleFor(r => r.A)
            .Matches(FileRule).WithMessage("Pick run A.");
        RuleFor(r => r.B)
            .Matches(FileRule).WithMessage("Pick run B.");
        RuleFor(r => r.B)
            .NotEqual(r => r.A).When(r => !string.IsNullOrEmpty(r.A)).WithMessage("Pick two different runs.");
        RuleFor(r => r.TolDeg)
            .InclusiveBetween(0.1, 90.0).WithMessage("Tolerance must be 0.1-90°.");
    }
}

/// <summary>#10: a skill, a lab scenario and two different versions.</summary>
public sealed class DuelRequestValidator : AbstractValidator<DuelRequest>
{
    public DuelRequestValidator()
    {
        RuleFor(r => r.Skill)
            .NotEmpty().WithMessage("Pick a skill.");
        RuleFor(r => r.Scenario)
            .NotEmpty().WithMessage("Pick a lab scenario.");
        RuleFor(r => r.B)
            .NotEqual(r => r.A).WithMessage("Pick two different versions.");
    }
}
