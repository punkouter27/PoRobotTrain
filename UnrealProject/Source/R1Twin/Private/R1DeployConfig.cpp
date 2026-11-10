// Written for UE 5.7; NOT YET COMPILED (see docs/ue5_setup.md).
#include "R1DeployConfig.h"
#include "R1Twin.h"

#if WITH_EDITOR
#include "Misc/DataValidation.h"
#endif

namespace
{
	const FString JointSuffix(TEXT("_joint"));
}

FName FR1DeployConfig::ToJointName(FName ShortOrFull)
{
	const FString S = ShortOrFull.ToString();
	return S.EndsWith(JointSuffix) ? ShortOrFull : FName(*(S + JointSuffix));
}

FName FR1DeployConfig::ToActuatorName(FName ShortOrFull)
{
	const FString S = ShortOrFull.ToString();
	return S.EndsWith(JointSuffix) ? FName(*S.LeftChop(JointSuffix.Len())) : ShortOrFull;
}

FName FR1DeployConfig::GetJointName(int32 Index) const
{
	return JointNames.IsValidIndex(Index) ? ToJointName(JointNames[Index]) : NAME_None;
}

FName FR1DeployConfig::GetActuatorName(int32 Index) const
{
	return JointNames.IsValidIndex(Index) ? ToActuatorName(JointNames[Index]) : NAME_None;
}

bool FR1DeployConfig::Validate(FString& OutError) const
{
	const int32 N = JointNames.Num();
	if (N == 0)
	{
		OutError = TEXT("JointNames is empty");
		return false;
	}
	if (NumActions != N)
	{
		OutError = FString::Printf(TEXT("NumActions (%d) != JointNames.Num() (%d)"), NumActions, N);
		return false;
	}
	if (DefaultJointPos.Num() != N) { OutError = FString::Printf(TEXT("DefaultJointPos has %d entries, expected %d"), DefaultJointPos.Num(), N); return false; }
	if (Kp.Num() != N)             { OutError = FString::Printf(TEXT("Kp has %d entries, expected %d"), Kp.Num(), N); return false; }
	if (Kd.Num() != N)             { OutError = FString::Printf(TEXT("Kd has %d entries, expected %d"), Kd.Num(), N); return false; }
	if (ActionScale.Num() != N)    { OutError = FString::Printf(TEXT("ActionScale has %d entries, expected %d (broadcast scalars at import time)"), ActionScale.Num(), N); return false; }

	const int32 ExpectedObs = Level0ObsSize(NumActions, NumCommands);
	if (NumObs != ExpectedObs)
	{
		OutError = FString::Printf(TEXT("NumObs (%d) does not match Level 0 layout 3*%d+%d = %d"), NumObs, NumActions, NumCommands, ExpectedObs);
		return false;
	}
	if (PolicyHz <= 0.f)
	{
		OutError = TEXT("PolicyHz must be > 0");
		return false;
	}
	for (int32 i = 0; i < N; ++i)
	{
		if (JointNames[i].IsNone()) { OutError = FString::Printf(TEXT("JointNames[%d] is None"), i); return false; }
		if (Kp[i] < 0.f || Kd[i] < 0.f) { OutError = FString::Printf(TEXT("Negative gain at %d"), i); return false; }
	}
	OutError.Reset();
	return true;
}

FR1DeployConfig FR1DeployConfig::MakeLevel0Default()
{
	// Source of truth: RobotControl/configs/r1_joint_map.yaml (policy_index 19..25 minus left arm).
	FR1DeployConfig C;
	C.JointNames = {
		TEXT("right_shoulder_pitch"),
		TEXT("right_shoulder_roll"),
		TEXT("right_shoulder_yaw"),
		TEXT("right_elbow"),
		TEXT("right_wrist_roll"),
		TEXT("head_pitch"),
		TEXT("head_yaw"),
	};
	C.DefaultJointPos = { 0.35f, -0.18f, 0.0f, 0.87f, 0.0f, 0.0f, 0.0f };
	C.Kp              = { 40.f,  40.f,   20.f, 20.f,  20.f, 50.f, 10.f };
	C.Kd              = { 2.f,   2.f,    1.f,  1.f,   1.f,  2.f,  0.1f };
	C.ActionScale     = { 0.375f, 0.375f, 0.4125f, 0.4125f, 0.4125f, 0.30f, 0.25f }; // must match deploy.yaml (head yaw retrained at 0.25)
	C.SoftLimitLo     = { -2.87980f, -2.34323f, -1.72791f, -0.81760f, -1.72791f, -0.56549f, -1.80639f }; // r1_joint_map.yaml, 0.9 x range
	C.SoftLimitHi     = { 1.83260f, 0.09154f, 1.72791f, 2.02716f, 1.72791f, 0.56549f, 1.80639f };
	C.NumActions  = 7;
	C.NumCommands = 2;
	C.NumObs      = Level0ObsSize(C.NumActions, C.NumCommands); // 23
	C.PolicyHz    = 50.f;
	C.ActionClipAbs = 5.f; // deploy.yaml action_clip
	return C;
}

#if WITH_EDITOR
EDataValidationResult UR1DeployConfigAsset::IsDataValid(FDataValidationContext& Context) const
{
	EDataValidationResult Result = Super::IsDataValid(Context);
	FString Error;
	if (!Config.Validate(Error))
	{
		Context.AddError(FText::FromString(FString::Printf(TEXT("R1 deploy config invalid: %s"), *Error)));
		Result = EDataValidationResult::Invalid;
	}
	if (PolicyModel == nullptr)
	{
		Context.AddWarning(FText::FromString(TEXT("No PolicyModel assigned - the policy component will hold the default pose only.")));
	}
	return Result;
}
#endif
