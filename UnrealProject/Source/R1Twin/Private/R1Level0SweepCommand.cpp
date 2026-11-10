// Written for UE 5.7; NOT YET COMPILED (see docs/ue5_setup.md).
#include "R1Level0SweepCommand.h"
#include "R1Twin.h"

UR1Level0SweepCommand::UR1Level0SweepCommand()
{
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.bStartWithTickEnabled = true;
	// Run early so UR1PolicyComponent (which also declares us as a tick prerequisite) sees fresh targets.
	PrimaryComponentTick.TickGroup = TG_PrePhysics;
}

void UR1Level0SweepCommand::BeginPlay()
{
	Super::BeginPlay();
	ResetClock();
	UE_LOG(LogR1Twin, Log, TEXT("Level0 command source ready: mode=%d, head yaw %.0f deg @ %.2f Hz, shoulder %.2f..%.2f rad @ %.2f Hz"),
		static_cast<int32>(Mode), HeadYawAmplitudeDeg, HeadYawFrequencyHz,
		ShoulderPitchDefaultRad, ShoulderPitchDefaultRad + ShoulderPitchDeltaRad, ShoulderPitchFrequencyHz);
}

void UR1Level0SweepCommand::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
	UpdateTargets(DeltaTime * TimeScale);
}

void UR1Level0SweepCommand::UpdateTargets(float Dt)
{
	switch (Mode)
	{
	case ER1CommandMode::Sinusoid:
	{
		Clock += Dt;
		const float YawAmpRad = FMath::DegreesToRadians(HeadYawAmplitudeDeg);
		HeadYawTargetRad = YawAmpRad * FMath::Sin(2.f * PI * HeadYawFrequencyHz * Clock);
		// (1 - cos)/2 ramps 0 -> 1 -> 0, so the sweep starts exactly at the default pose.
		const float Phase = 0.5f * (1.f - FMath::Cos(2.f * PI * ShoulderPitchFrequencyHz * Clock));
		ShoulderPitchTargetRad = ShoulderPitchDefaultRad + ShoulderPitchDeltaRad * Phase;
		break;
	}
	case ER1CommandMode::Manual:
		HeadYawTargetRad += ManualHeadYawAxis * ManualHeadYawRateRadPerSec * Dt;
		ShoulderPitchTargetRad += ManualShoulderAxis * ManualShoulderRateRadPerSec * Dt;
		break;
	case ER1CommandMode::Hold:
	default:
		break;
	}

	HeadYawTargetRad = FMath::Clamp(HeadYawTargetRad, -HeadYawLimitRad, HeadYawLimitRad);
	ShoulderPitchTargetRad = FMath::Clamp(ShoulderPitchTargetRad, ShoulderPitchMinRad, ShoulderPitchMaxRad);
}

void UR1Level0SweepCommand::FillCommand(TArrayView<float> Out) const
{
	if (Out.Num() < NumCommands)
	{
		return;
	}
	Out[0] = HeadYawTargetRad;
	Out[1] = ShoulderPitchTargetRad;
}

void UR1Level0SweepCommand::SetManualAxes(float HeadYawAxis, float ShoulderAxis)
{
	ManualHeadYawAxis = FMath::Clamp(HeadYawAxis, -1.f, 1.f);
	ManualShoulderAxis = FMath::Clamp(ShoulderAxis, -1.f, 1.f);
}

void UR1Level0SweepCommand::ToggleMode()
{
	switch (Mode)
	{
	case ER1CommandMode::Sinusoid: SetMode(ER1CommandMode::Manual); break;
	case ER1CommandMode::Manual:   SetMode(ER1CommandMode::Hold); break;
	default:                       SetMode(ER1CommandMode::Sinusoid); break;
	}
}

void UR1Level0SweepCommand::SetMode(ER1CommandMode NewMode)
{
	if (Mode == NewMode)
	{
		return;
	}
	Mode = NewMode;
	ManualHeadYawAxis = 0.f;
	ManualShoulderAxis = 0.f;
	UE_LOG(LogR1Twin, Log, TEXT("Level0 command mode -> %d"), static_cast<int32>(Mode));
}

void UR1Level0SweepCommand::ResetClock()
{
	Clock = 0.f;
	HeadYawTargetRad = 0.f;
	ShoulderPitchTargetRad = ShoulderPitchDefaultRad;
}
