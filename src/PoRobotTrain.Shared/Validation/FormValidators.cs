using FluentValidation;
using PoRobotTrain.Shared.Contracts;

namespace PoRobotTrain.Shared.Validation;

/// <summary>
/// Motion editor rules, the same ones the behaviour service enforces (RobotControl/behaviours/motions.py NAME_RE), so
/// the operator sees them inline before a save is refused. Registered by type in the Client Program.cs (no assembly scanning).
/// </summary>
public sealed class MotionClipValidator : AbstractValidator<MotionClipDto>
{
    public MotionClipValidator()
    {
        RuleFor(c => c.Name)
            .NotEmpty().WithMessage("Give the motion a name.")
            .Matches("^[a-z0-9][a-z0-9_]{0,39}$")
            .WithMessage("Name: 1-40 lower-case letters, digits or _, starting with a letter or digit.");
        RuleFor(c => c.Title).MaximumLength(80).WithMessage("Title: 80 characters at most.");
        RuleFor(c => c.Description).MaximumLength(500).WithMessage("Description: 500 characters at most.");
        RuleFor(c => c.Keyframes).NotEmpty().WithMessage("Add at least one keyframe.");
    }
}

/// <summary>Connection profile rules, mirroring RobotControl/behaviours/ops.py ProfileStore.upsert.</summary>
public sealed class ConnectionProfileValidator : AbstractValidator<ConnectionProfileDto>
{
    public ConnectionProfileValidator()
    {
        RuleFor(p => p.Name)
            .Matches("^[A-Za-z0-9_-]{1,32}$").WithMessage("Name: 1-32 letters, digits, - or _.");
        RuleFor(p => p.Target)
            .Must(t => t is "real" or "urlab" or "sim").WithMessage("Target must be real, urlab or sim.");
        RuleFor(p => p.Host)
            .Matches("^[A-Za-z0-9.:-]{1,253}$").WithMessage("Host: a hostname or IP address.");
        RuleFor(p => p.Port)
            .InclusiveBetween(1, 65535).WithMessage("Port must be 1-65535.");
    }
}
