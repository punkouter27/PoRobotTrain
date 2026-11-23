#include "R1MotorModel.h"
#include "R1Twin.h"

#include "Dom/JsonObject.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"

// =====================================================================================================
// config
// =====================================================================================================

FString FR1MotorModelConfig::DefaultPath()
{
	return FPaths::ConvertRelativePathToFull(FPaths::Combine(FPaths::ProjectDir(), TEXT(".."), TEXT("RobotControl"), TEXT("configs"), TEXT("motor_model.json")));
}

const FR1MotorModelConfig& FR1MotorModelConfig::Get()
{
	static const FR1MotorModelConfig Config = []
	{
		FR1MotorModelConfig C;
		C.LoadFromFile(DefaultPath());
		return C;
	}();
	return Config;
}

bool FR1MotorModelConfig::LoadFromFile(const FString& Path)
{
	FString Text;
	TSharedPtr<FJsonObject> J;
	if (!FFileHelper::LoadFileToString(Text, *Path) || !FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Text), J) || !J.IsValid())
	{
		UE_LOG(LogR1Twin, Log, TEXT("R1MotorModel: %s not found, using the built-in defaults"), *Path);
		return false;
	}
	auto Read = [&J](const TCHAR* Key, float& Out)
	{
		double V = 0.0;
		if (J->TryGetNumberField(Key, V) && FMath::IsFinite(V))
		{
			Out = static_cast<float>(V);
		}
	};
	Read(TEXT("ambient_c"), Ambient);
	Read(TEXT("thermal_time_constant_s"), TimeConstant);
	Read(TEXT("heat_rate_c_per_s_at_full_torque"), HeatRate);
	Read(TEXT("warn_c"), Warn);
	Read(TEXT("limit_c"), Limit);
	Read(TEXT("battery_wh"), Battery);
	Read(TEXT("idle_w"), Idle);
	Read(TEXT("drive_efficiency"), Efficiency);
	Read(TEXT("copper_loss_w_at_full_torque"), CopperLoss);
	Read(TEXT("torque_alert_fraction"), AlertFraction);
	Read(TEXT("action_filter_alpha"), FilterAlpha);
	Read(TEXT("jitter_window_s"), JitterWindow);
	TimeConstant = FMath::Max(TimeConstant, 1e-3f);
	Battery = FMath::Max(Battery, 1e-3f);
	Efficiency = FMath::Clamp(Efficiency, 0.05f, 1.f);
	FilterAlpha = FMath::Clamp(FilterAlpha, 0.f, 1.f);
	UE_LOG(LogR1Twin, Log, TEXT("R1MotorModel: loaded %s"), *Path);
	return true;
}

// =====================================================================================================
// readout
// =====================================================================================================

int32 FR1MotorReadout::MaxFracIndex() const
{
	int32 Best = INDEX_NONE;
	for (int32 i = 0; i < TorqueFrac.Num(); ++i)
	{
		if (Best == INDEX_NONE || TorqueFrac[i] > TorqueFrac[Best])
		{
			Best = i;
		}
	}
	return Best;
}

// =====================================================================================================
// motor model
// =====================================================================================================

float FR1MotorModel::JointMapEffort(FName Joint)
{
	// RobotControl/configs/r1_joint_map.yaml (effort, N m)
	static const TMap<FName, float> Table = {
		{TEXT("left_hip_pitch_joint"), 60.f}, {TEXT("left_hip_roll_joint"), 60.f}, {TEXT("left_hip_yaw_joint"), 60.f},
		{TEXT("left_knee_joint"), 60.f}, {TEXT("left_ankle_pitch_joint"), 50.f}, {TEXT("left_ankle_roll_joint"), 50.f},
		{TEXT("right_hip_pitch_joint"), 60.f}, {TEXT("right_hip_roll_joint"), 60.f}, {TEXT("right_hip_yaw_joint"), 60.f},
		{TEXT("right_knee_joint"), 60.f}, {TEXT("right_ankle_pitch_joint"), 50.f}, {TEXT("right_ankle_roll_joint"), 50.f},
		{TEXT("waist_roll_joint"), 60.f}, {TEXT("waist_yaw_joint"), 60.f},
		{TEXT("left_shoulder_pitch_joint"), 60.f}, {TEXT("left_shoulder_roll_joint"), 60.f}, {TEXT("left_shoulder_yaw_joint"), 33.f},
		{TEXT("left_elbow_joint"), 33.f}, {TEXT("left_wrist_roll_joint"), 33.f},
		{TEXT("right_shoulder_pitch_joint"), 60.f}, {TEXT("right_shoulder_roll_joint"), 60.f}, {TEXT("right_shoulder_yaw_joint"), 33.f},
		{TEXT("right_elbow_joint"), 33.f}, {TEXT("right_wrist_roll_joint"), 33.f},
		{TEXT("head_pitch_joint"), 33.f}, {TEXT("head_yaw_joint"), 0.86f},
	};
	const float* E = Table.Find(Joint);
	return E ? *E : 33.f;
}

void FR1MotorModel::Reset(const TArray<FName>& Joints, const TArray<float>& InEffort)
{
	const FR1MotorModelConfig& C = FR1MotorModelConfig::Get();
	Effort = InEffort;
	Effort.SetNum(Joints.Num());
	for (float& E : Effort)
	{
		E = FMath::Max(E, 1e-3f);
	}
	Temp.Init(C.Ambient, Joints.Num());
	Soc = 1.f;
	Readout = FR1MotorReadout();
	for (const FName& N : Joints)
	{
		Readout.Names.Add(N.ToString());
	}
	Readout.Speed.SetNumZeroed(Joints.Num());
	Readout.Torque.SetNumZeroed(Joints.Num());
	Readout.TorqueFrac.SetNumZeroed(Joints.Num());
	Readout.TempC = Temp;
	Readout.HottestC = C.Ambient;
}

void FR1MotorModel::Step(float Dt, const TArray<float>& Tau, const TArray<float>& Dq)
{
	const FR1MotorModelConfig& C = FR1MotorModelConfig::Get();
	const int32 N = Effort.Num();
	if (N == 0 || Tau.Num() < N || Dq.Num() < N || Dt <= 0.f)
	{
		return;
	}
	float Mech = 0.f, Copper = 0.f;
	int32 Hot = 0;
	Readout.Warn.Reset();
	Readout.Limit.Reset();
	for (int32 i = 0; i < N; ++i)
	{
		const float F = FMath::Abs(Tau[i]) / Effort[i];
		Temp[i] += Dt * (C.HeatRate * F * F - (Temp[i] - C.Ambient) / C.TimeConstant);
		Mech += FMath::Max(Tau[i] * Dq[i], 0.f);
		Copper += C.CopperLoss * F * F;
		Readout.Speed[i] = Dq[i];
		Readout.Torque[i] = Tau[i];
		Readout.TorqueFrac[i] = F;
		Readout.TempC[i] = Temp[i];
		if (Temp[i] > Temp[Hot]) Hot = i;
		if (Temp[i] >= C.Limit) Readout.Limit.Add(Readout.Names[i]);
		else if (Temp[i] >= C.Warn) Readout.Warn.Add(Readout.Names[i]);
	}
	const float P = C.Idle + Mech / C.Efficiency + Copper;
	Soc = FMath::Max(0.f, Soc - P * Dt / (C.Battery * 3600.f));
	Readout.bValid = true;
	Readout.Hottest = Readout.Names[Hot];
	Readout.HottestC = Temp[Hot];
	Readout.SocPct = Soc * 100.f;
	Readout.PowerW = P;
	Readout.MinutesLeft = P > 1e-3f ? Soc * C.Battery * 60.f / P : -1.f;
}

// =====================================================================================================
// action filter
// =====================================================================================================

void FR1ActionFilter::Reset(int32 NumJoints, float Dt)
{
	const FR1MotorModelConfig& C = FR1MotorModelConfig::Get();
	Filtered.Init(0.f, NumJoints);
	Prev1.Init(0.f, NumJoints);
	Prev2.Init(0.f, NumJoints);
	Seen = 0;
	SecondDiffSq.Init(0.f, FMath::Max(1, FMath::RoundToInt(C.JitterWindow / FMath::Max(Dt, 1e-3f))));
	RingHead = 0;
	RingCount = 0;
	JitterScore = 0.f;
	HistoryRaw.SetNum(NumJoints);
	HistoryFiltered.SetNum(NumJoints);
	for (int32 j = 0; j < NumJoints; ++j)
	{
		HistoryRaw[j].Reset(HistoryLen);
		HistoryFiltered[j].Reset(HistoryLen);
	}
}

void FR1ActionFilter::Push(const float* Raw, int32 Count)
{
	const float Alpha = FR1MotorModelConfig::Get().FilterAlpha;
	const int32 K = FMath::Min(Count, Filtered.Num());
	float Sq = 0.f;
	for (int32 j = 0; j < K; ++j)
	{
		const float A = Raw[j];
		Filtered[j] = Seen == 0 ? A : Filtered[j] + Alpha * (A - Filtered[j]);
		if (Seen >= 2)
		{
			const float D2 = A - 2.f * Prev1[j] + Prev2[j];
			Sq += D2 * D2;
		}
		Prev2[j] = Prev1[j];
		Prev1[j] = A;
		TArray<float>& HR = HistoryRaw[j];
		TArray<float>& HF = HistoryFiltered[j];
		if (HR.Num() >= HistoryLen)
		{
			HR.RemoveAt(0, 1, EAllowShrinking::No);
			HF.RemoveAt(0, 1, EAllowShrinking::No);
		}
		HR.Add(A);
		HF.Add(Filtered[j]);
	}
	if (Seen >= 2 && K > 0)
	{
		SecondDiffSq[RingHead] = Sq / K;
		RingHead = (RingHead + 1) % SecondDiffSq.Num();
		RingCount = FMath::Min(RingCount + 1, SecondDiffSq.Num());
		float Sum = 0.f;
		for (int32 i = 0; i < RingCount; ++i)
		{
			Sum += SecondDiffSq[i];
		}
		JitterScore = FMath::Sqrt(Sum / RingCount);
	}
	++Seen;
}
