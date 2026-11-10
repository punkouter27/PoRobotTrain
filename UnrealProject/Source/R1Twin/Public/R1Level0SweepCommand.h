// Level 0 command source: produces the 2-dim command appended to the policy observation.
// Written for UE 5.7; NOT YET COMPILED (see docs/ue5_setup.md).
//
// Command layout (must match the Level 0 training env in RobotControl - TODO confirm there):
//   command[0] = head_yaw target, rad               (absolute joint angle)
//   command[1] = right_shoulder_pitch target, rad   (absolute joint angle)
//
// Modes
//   Sinusoid : head yaw  = A_yaw * sin(2*pi*f_yaw*t),            A_yaw = 30 deg, f_yaw = 0.25 Hz
//              shoulder  = q_def + (Delta/2) * (1 - cos(2*pi*f_sh*t)), Delta = -1.2 rad, f_sh = 0.15 Hz
//              (shoulder sweeps smoothly between q_def and q_def + Delta, starting at q_def)
//   Manual   : keyboard axes integrate the two targets at a fixed rate, clamped to joint limits.
//   Hold     : both targets frozen (useful for comparing against the Python env).
//
// Enhanced Input actions (created in code by AR1FollowCamera - no .uasset required):
//   IA_R1_HeadYawAxis      Axis1D   A (negate) / D          -> SetManualAxes(x, _)
//   IA_R1_ShoulderAxis     Axis1D   S (negate) / W          -> SetManualAxes(_, y)
//   IA_R1_ToggleCmdMode    Digital  Tab                      -> ToggleMode()
//   IA_R1_ResetClock       Digital  R                        -> ResetClock()
#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "R1Level0SweepCommand.generated.h"

UENUM(BlueprintType)
enum class ER1CommandMode : uint8
{
	Sinusoid UMETA(DisplayName = "Sinusoid sweep"),
	Manual   UMETA(DisplayName = "Manual (keyboard)"),
	Hold     UMETA(DisplayName = "Hold current"),
};

UCLASS(ClassGroup = (R1), meta = (BlueprintSpawnableComponent))
class R1TWIN_API UR1Level0SweepCommand : public UActorComponent
{
	GENERATED_BODY()

public:
	static constexpr int32 NumCommands = 2;

	UR1Level0SweepCommand();

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "R1|Command")
	ER1CommandMode Mode = ER1CommandMode::Sinusoid;

	/** Multiplies the internal clock (1 = real time). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "R1|Command", meta = (ClampMin = 0.0))
	float TimeScale = 1.f;

	// ---- head yaw sinusoid
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "R1|Command|Head Yaw", meta = (ClampMin = 0.0, ClampMax = 115.0))
	float HeadYawAmplitudeDeg = 30.f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "R1|Command|Head Yaw", meta = (ClampMin = 0.0))
	float HeadYawFrequencyHz = 0.25f;

	/** head_yaw_joint range from the MJCF: +-2.0071 rad. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "R1|Command|Head Yaw")
	float HeadYawLimitRad = 2.0071f;

	// ---- shoulder pitch sinusoid
	/** right_shoulder_pitch default from r1_joint_map.yaml. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "R1|Command|Shoulder Pitch")
	float ShoulderPitchDefaultRad = 0.35f;

	/** Sweep goes from default to default + Delta (Level 0 spec: default - 1.2 rad, i.e. arm forward/up). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "R1|Command|Shoulder Pitch")
	float ShoulderPitchDeltaRad = -1.2f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "R1|Command|Shoulder Pitch", meta = (ClampMin = 0.0))
	float ShoulderPitchFrequencyHz = 0.15f;

	/** right_shoulder_pitch_joint range from the MJCF. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "R1|Command|Shoulder Pitch")
	float ShoulderPitchMinRad = -3.1416f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "R1|Command|Shoulder Pitch")
	float ShoulderPitchMaxRad = 2.0944f;

	// ---- manual mode
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "R1|Command|Manual", meta = (ClampMin = 0.0))
	float ManualHeadYawRateRadPerSec = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "R1|Command|Manual", meta = (ClampMin = 0.0))
	float ManualShoulderRateRadPerSec = 0.8f;

	/** Writes [head_yaw_target_rad, shoulder_pitch_target_rad] into Out (Out.Num() must be >= NumCommands). No allocation. */
	void FillCommand(TArrayView<float> Out) const;

	/** Manual-mode input: axes in [-1, 1]. Ignored unless Mode == Manual. */
	UFUNCTION(BlueprintCallable, Category = "R1|Command")
	void SetManualAxes(float HeadYawAxis, float ShoulderAxis);

	UFUNCTION(BlueprintCallable, Category = "R1|Command")
	void ToggleMode();

	UFUNCTION(BlueprintCallable, Category = "R1|Command")
	void SetMode(ER1CommandMode NewMode);

	/** Restart the sinusoid clock (and re-centre manual targets on the defaults). */
	UFUNCTION(BlueprintCallable, Category = "R1|Command")
	void ResetClock();

	UFUNCTION(BlueprintPure, Category = "R1|Command")
	float GetHeadYawTargetRad() const { return HeadYawTargetRad; }

	UFUNCTION(BlueprintPure, Category = "R1|Command")
	float GetShoulderPitchTargetRad() const { return ShoulderPitchTargetRad; }

	UFUNCTION(BlueprintPure, Category = "R1|Command")
	float GetClockSeconds() const { return Clock; }

	virtual void BeginPlay() override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

private:
	void UpdateTargets(float Dt);

	float Clock = 0.f;
	float HeadYawTargetRad = 0.f;
	float ShoulderPitchTargetRad = 0.35f;
	float ManualHeadYawAxis = 0.f;
	float ManualShoulderAxis = 0.f;
};
