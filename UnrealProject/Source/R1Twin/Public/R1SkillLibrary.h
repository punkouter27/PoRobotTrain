// Trained head skills for the scenario lab: the validated ONNX policies the web app trains
// (RobotControl/behaviours/head_skills.py is the contract; this file mirrors it for the engine).
//
// A skill loads only when <dir>/deploy.json says validation.passed and validation.policy_sha256 equals the SHA-256 of
// <dir>/policy.onnx - the same gate the PC runner and the Jetson apply. Unreal never trains and never talks to the
// real robot; it only runs these brains on the virtual R1.
#pragma once

#include "CoreMinimal.h"

/** One trained head skill, as exported by robotcontrol.training.validate_head_skill (deploy.json). */
struct R1TWIN_API FR1SkillDef
{
	FString Id;
	FString Label;
	FString Desc;
	FString Dir;
	FString OnnxPath;

	/** Powered joints (policy order) and their URLab actuator names. */
	TArray<FName> Joints;
	TArray<FName> Actuators;
	TArray<float> Default;
	TArray<float> ActionScale;
	TArray<float> SoftLo;
	TArray<float> SoftHi;
	TArray<float> Kp;
	TArray<float> Kd;
	TArray<float> Effort;

	/** Every other upper-body joint, held at its default with joint-map gains while a skill runs. */
	TArray<FName> HoldJoints;
	TArray<FName> HoldActuators;
	TArray<float> HoldDefault;
	TArray<float> HoldKp;
	TArray<float> HoldKd;

	int32 NumObs = 0;
	float PeriodS = 0.f;      // periodic skills: one cycle; 0 = command driven
	float MaxDeg = 30.f;
	float YawRange = 0.f;     // look_at training range (rad)
	float PitchRange = 0.f;
	float GlanceMove = 0.4f;
	TArray<FVector2D> GlanceKeys;   // (yaw, pitch) fractions of the size

	/** Taught pose (docs/sim_features.md #14, skill id pose_<id>): hold the head at a fixed angle.
	 *  Angles from deploy.json pose_yaw_deg / pose_pitch_deg (top level; metadata.* and a "pose" object are accepted
	 *  too). Command = deploy.json pose_command [3] if present, else [0, 0, 0] (held like Hold Centre). */
	bool bPose = false;
	float PoseYawRad = 0.f;     // yaw left +
	float PosePitchRad = 0.f;   // pitch down +
	float PoseCommand[3] = {0.f, 0.f, 0.f};

	/** Validation headline (mean error of the Jetson code path on the realistic MuJoCo twin), deg. */
	float TrackErrDeg = -1.f;
	FString ValidatedAt;

	/** True only when the validation gate passed for this exact ONNX file. */
	bool bReady = false;
	FString Reason;

	int32 NumJoints() const { return Joints.Num(); }
	int32 JointIndex(FName Joint) const { return Joints.IndexOfByKey(Joint); }
};

namespace R1Skills
{
	inline const FName HeadPitch(TEXT("head_pitch_joint"));
	inline const FName HeadYaw(TEXT("head_yaw_joint"));
	constexpr float PolicyDt = 0.02f;
	constexpr float ActionClip = 5.f;
	constexpr int32 NumCmd = 3;

	/** RobotControl/models/head_skills next to UnrealProject/, or -R1SkillsDir=<path>. */
	R1TWIN_API FString DefaultDir();

	/** Every skill folder under Dir (fixed order: hold_centre, look_at, nod_yes, shake_no, glance_around, then others). */
	R1TWIN_API TArray<FR1SkillDef> LoadAll(const FString& Dir);

	/** Lower-case hex SHA-256 of a file ("" if unreadable). */
	R1TWIN_API FString Sha256File(const FString& Path);
	R1TWIN_API FString Sha256Hex(const uint8* Data, int64 Len);

	/** Periodic skills: the [sin, cos, size] command at time T (s). */
	R1TWIN_API void PhaseCommand(const FR1SkillDef& Skill, float T, float Size, float OutCmd[NumCmd]);

	/** Reference motion (rad, per powered joint) for a command; Phase in [0,1) for periodic skills (<0 = from sin/cos). */
	R1TWIN_API void Reference(const FR1SkillDef& Skill, const float Cmd[NumCmd], float Phase, TArray<float>& Out);
}
