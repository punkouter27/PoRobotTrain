// Motor heat + battery model and action-filter view for the scenario lab (docs/sim_features.md #5, #15, #17).
// C++ port of robotcontrol.sim_features.motor_model; the defaults below mirror RobotControl/configs/motor_model.json
// (a pytest parses the `static constexpr float` lines, so keep them one per line and in this exact form).
// DISPLAY ONLY: nothing here derates a motor, and nothing reaches the physical robot.
#pragma once

#include "CoreMinimal.h"

/** Shared constants (RobotControl/configs/motor_model.json). */
struct R1TWIN_API FR1MotorModelConfig
{
	static constexpr float AmbientC = 25.0f;
	static constexpr float ThermalTimeConstantS = 120.0f;
	static constexpr float HeatRateCPerSAtFullTorque = 0.8333f;
	static constexpr float WarnC = 70.0f;
	static constexpr float LimitC = 90.0f;
	static constexpr float BatteryWh = 500.0f;
	static constexpr float IdleW = 60.0f;
	static constexpr float DriveEfficiency = 0.8f;
	static constexpr float CopperLossWAtFullTorque = 40.0f;
	static constexpr float TorqueAlertFraction = 0.8f;
	static constexpr float ActionFilterAlpha = 0.5f;
	static constexpr float JitterWindowS = 1.0f;

	// runtime values (defaults above, optionally overridden from the JSON file)
	float Ambient = AmbientC;
	float TimeConstant = ThermalTimeConstantS;
	float HeatRate = HeatRateCPerSAtFullTorque;
	float Warn = WarnC;
	float Limit = LimitC;
	float Battery = BatteryWh;
	float Idle = IdleW;
	float Efficiency = DriveEfficiency;
	float CopperLoss = CopperLossWAtFullTorque;
	float AlertFraction = TorqueAlertFraction;
	float FilterAlpha = ActionFilterAlpha;
	float JitterWindow = JitterWindowS;

	/** RobotControl/configs/motor_model.json next to UnrealProject/ (missing file or field = default). */
	static FString DefaultPath();
	/** Loaded once per process (the file is read on first use). */
	static const FR1MotorModelConfig& Get();
	bool LoadFromFile(const FString& Path);
};

/** What the HUD and the motor sound read (Lab: computed here; Walk: parsed from /walk/state). */
struct R1TWIN_API FR1MotorReadout
{
	bool bValid = false;
	TArray<FString> Names;
	TArray<float> Speed;       // rad/s
	TArray<float> Torque;      // N m
	TArray<float> TorqueFrac;  // |tau| / effort
	TArray<float> TempC;
	FString Hottest;
	float HottestC = FR1MotorModelConfig::AmbientC;
	TArray<FString> Warn;      // joints above warn_c
	TArray<FString> Limit;     // joints above limit_c
	float SocPct = 100.f;
	float PowerW = 0.f;
	float MinutesLeft = -1.f;  // < 0 = unknown

	/** Largest torque fraction and its joint (INDEX_NONE if empty). */
	int32 MaxFracIndex() const;
};

/** Per-joint thermal + whole-robot battery integration (see docs/sim_features.md for the equations). */
class R1TWIN_API FR1MotorModel
{
public:
	void Reset(const TArray<FName>& Joints, const TArray<float>& Effort);
	int32 Num() const { return Effort.Num(); }
	/** One step of dt seconds with the joint torques (N m) and speeds (rad/s), same order as Reset. */
	void Step(float Dt, const TArray<float>& Tau, const TArray<float>& Dq);
	const FR1MotorReadout& GetReadout() const { return Readout; }

	/** Effort (N m) of a joint from RobotControl/configs/r1_joint_map.yaml (33 if unknown). */
	static float JointMapEffort(FName Joint);

private:
	TArray<float> Effort;
	TArray<float> Temp;
	float Soc = 1.f;
	FR1MotorReadout Readout;
};

/** EMA low-pass of the policy action + jitter score (RMS of the second difference over the jitter window). */
class R1TWIN_API FR1ActionFilter
{
public:
	void Reset(int32 NumJoints, float Dt);
	void Push(const float* Raw, int32 Count);
	int32 NumJoints() const { return Filtered.Num(); }
	float Jitter() const { return JitterScore; }
	const TArray<float>& GetFiltered() const { return Filtered; }

	/** Recent history (oldest first) for the mini plot: HistoryRaw[j][k], HistoryFiltered[j][k]. */
	static constexpr int32 HistoryLen = 100;
	TArray<TArray<float>> HistoryRaw;
	TArray<TArray<float>> HistoryFiltered;

private:
	TArray<float> Filtered;
	TArray<float> Prev1, Prev2;
	int32 Seen = 0;
	TArray<float> SecondDiffSq;   // ring: mean over joints of (second difference)^2 per step
	int32 RingHead = 0;
	int32 RingCount = 0;
	float JitterScore = 0.f;
};
