#include "R1RewardConfig.h"

#include UE_INLINE_GENERATED_CPP_BY_NAME(R1RewardConfig)

UR1RewardConfig::UR1RewardConfig()
{
	// Same order and values as RobotControl/configs/reward_config.json "curriculum".
	AddStage(TEXT("L0_start"), 0.3f, TEXT("fraction of the full command amplitude at iteration 0"));
	AddStage(TEXT("L0_advance"), 3.0f, TEXT("mean tracking error (deg) below which the amplitude fraction grows by step per iteration"));
	AddStage(TEXT("L0_full"), 1.0f, TEXT("amplitude fraction at which the Level 0 curriculum is complete"));
}

void UR1RewardConfig::AddStage(const TCHAR* Stage, float Threshold, const TCHAR* Metric)
{
	FR1CurriculumStage& S = CurriculumStages.AddDefaulted_GetRef();
	S.Stage = Stage;
	S.Threshold = Threshold;
	S.Metric = Metric;
}

int32 UR1RewardConfig::FindStage(const FString& Stage) const
{
	return CurriculumStages.IndexOfByPredicate([&Stage](const FR1CurriculumStage& S) { return S.Stage == Stage; });
}
