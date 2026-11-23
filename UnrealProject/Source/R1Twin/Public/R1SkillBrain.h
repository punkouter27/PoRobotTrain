// Scenario lab brain: runs the trained head skills (validated ONNX from the web app) inside Unreal, on the virtual
// R1 only. NNE / ONNX Runtime CPU at the trained 50 Hz, same observation, action, torque cap and soft limits as the
// Python/Jetson runtime (RobotControl/behaviours/head_skills.py), so what you see here is what the brain does.
//
// Scenario tools: a movable look-at target (Script / Manual / Orbit), a head-camera picture-in-picture, a skill
// timeline, and disturbances (push the head, action delay, sensor noise, weaker motors). It never connects to the
// physical robot; while a Python client drives URLab (web app twin preview, walk sandbox) it lets go of the actuators.
#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "GameFramework/Actor.h"
#include "Templates/UniquePtr.h"
#include "NNERuntimeCPU.h"
#include "R1ArticulationBridge.h"
#include "R1SkillLibrary.h"
#include "R1SkillBrain.generated.h"

class UNNEModelData;
class UStaticMeshComponent;
class USceneCaptureComponent2D;
class UTextureRenderTarget2D;
class UMaterialInstanceDynamic;

UENUM()
enum class ER1TargetMode : uint8
{
	Script,   // the web app's Look At demo sequence (centre, left, right, down, up, half-left)
	Manual,   // arrow keys / panel nudges move the ball
	Orbit,    // the ball circles a figure-8 in front of the robot
};

/** One timeline step: run Skill at SizeDeg for DurationS (periodic skills round up to whole cycles). */
struct FR1TimelineStep
{
	int32 Skill = 0;
	float SizeDeg = 10.f;
	float DurationS = 6.f;
};

/** What the panel shows (updated every policy step). */
struct FR1BrainReadout
{
	bool bRunning = false;
	bool bSettling = false;
	bool bExternal = false;     // a Python client owns URLab right now
	bool bConnected = false;    // URLab articulation found (false = mock joints)
	FString SkillLabel;
	float T = 0.f;
	float DurationS = 0.f;
	int32 TimelineStep = -1;    // -1 = single skill
	TArray<FString> JointLabels;
	TArray<float> QDeg, RefDeg, TargetDeg;
	float ErrNowDeg = 0.f, ErrMeanDeg = 0.f, ErrMaxDeg = 0.f;
	float OtherMaxDeg = 0.f;
	float InferMs = 0.f;
	float ValidatedErrDeg = -1.f;
	FVector2D LookCmdDeg = FVector2D::ZeroVector;   // (yaw, pitch-down) the brain is asked to look at
	bool bLookClamped = false;
	/** Empty = the real robot's safety envelope would let this run continue; else why the runner would stop it. */
	FString RobotWouldStop;
};

/** The look-at ball and the head camera (spawned by the brain; transforms driven every tick). */
UCLASS(NotPlaceable, Transient)
class R1TWIN_API AR1ScenarioProps : public AActor
{
	GENERATED_BODY()
public:
	AR1ScenarioProps();
	UPROPERTY(VisibleAnywhere) TObjectPtr<USceneComponent> Root;
	UPROPERTY(VisibleAnywhere) TObjectPtr<UStaticMeshComponent> Ball;
	UPROPERTY(VisibleAnywhere) TObjectPtr<USceneCaptureComponent2D> HeadCam;
	UPROPERTY(Transient) TObjectPtr<UTextureRenderTarget2D> HeadCamRT;
	UPROPERTY(Transient) TObjectPtr<UMaterialInstanceDynamic> BallMat;
	virtual void BeginPlay() override;
};

UCLASS(ClassGroup = (R1), meta = (BlueprintSpawnableComponent))
class R1TWIN_API UR1SkillBrainComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	UR1SkillBrainComponent();

	UPROPERTY(EditAnywhere, Category = "R1|Lab")
	FName ArticulationActorTag = TEXT("R1Articulation");

	UPROPERTY(EditAnywhere, Category = "R1|Lab")
	FString RuntimeName = TEXT("NNERuntimeORTCpu");

	// ------------------------------------------------------------------ skills
	const TArray<FR1SkillDef>& GetSkills() const { return Skills; }
	/** Re-read RobotControl/models/head_skills (after training a new skill in the web app). */
	void ReloadSkills();
	int32 FindSkill(const FString& Id) const;

	/** Start one skill; false + OutWhy when it is locked (not validated) or the twin is busy. */
	bool Play(int32 SkillIndex, float SizeDeg, float DurationS, FString& OutWhy);
	/** Run the timeline steps in order (Loop: start over at the end). */
	bool PlayTimeline(FString& OutWhy);
	void Stop();
	bool IsRunning() const { return Active >= 0; }
	const FR1BrainReadout& GetReadout() const { return Readout; }

	// ------------------------------------------------------------------ look-at target
	ER1TargetMode TargetMode = ER1TargetMode::Script;
	/** Ball direction from the head (deg): yaw left +, pitch down +; distance (m). */
	float TargetYawDeg = 0.f;
	float TargetPitchDeg = 0.f;
	float TargetDistM = 1.2f;
	void NudgeTarget(float DYawDeg, float DPitchDeg);
	void SetTargetMode(ER1TargetMode Mode);

	// ------------------------------------------------------------------ disturbances
	int32 DelaySteps = 0;          // extra action delay, 0..4 policy steps (0..80 ms)
	float NoiseDeg = 0.f;          // uniform joint-angle sensor noise, deg
	float StrengthPct = 100.f;     // head motor strength (scales kp/kd)
	/** A short shove on the head: Dir.X = yaw (+ left), Dir.Y = pitch (+ down). */
	void Push(const FVector2D& Dir);

	// ------------------------------------------------------------------ timeline
	TArray<FR1TimelineStep> Timeline;
	bool bLoopTimeline = false;

	// ------------------------------------------------------------------ head camera (picture-in-picture)
	UTextureRenderTarget2D* GetHeadCamTexture() const;

	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type Reason) override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

private:
	struct FModel
	{
		TSharedPtr<UE::NNE::IModelInstanceCPU> Instance;
		FString Error;
	};

	bool EnsureBridge();
	bool EnsureModel(int32 SkillIndex, FString& OutWhy);
	void StartSkill(int32 SkillIndex, float SizeDeg, float DurationS);
	void PolicyStep();
	void ApplyGains();
	void HoldPose();
	void FillCommand(const FR1SkillDef& S, float T, float Cmd[R1Skills::NumCmd]);
	void UpdateTargetDirection(float DeltaTime);
	void UpdateProps();
	void FinishActive();

	TArray<FR1SkillDef> Skills;
	TMap<int32, FModel> Models;
	UPROPERTY(Transient) TArray<TObjectPtr<UNNEModelData>> ModelData;
	TUniquePtr<IR1ArticulationBridge> Bridge;
	TArray<FName> BridgeJoints;   // union of the head joints and every skill's hold joints
	bool bSessionStarted = false;
	bool bHadExternal = false;

	UPROPERTY(Transient) TObjectPtr<AR1ScenarioProps> Props;
	double NextCapture = 0.0;

	// active run
	int32 Active = -1;
	int32 TimelineIndex = -1;
	float SizeDeg = 10.f;
	float RunS = 0.f;
	float T = 0.f;
	float Accumulator = 0.f;
	TArray<float> Q, Dq, LastAction, Target, Ref, PrevRef, Obs, Action;
	TArray<TArray<float>> DelayQueue;
	int32 DelayHead = 0;
	float LastJumpT = 0.f;
	TMap<FName, float> HoldStart;
	float ErrSum = 0.f;
	int32 ErrCount = 0;
	double ErrSince = -1.0;
	float AppliedStrength = -1.f;
	FVector PushTorqueMj = FVector::ZeroVector;
	float PushLeftS = 0.f;
	float OrbitPhase = 0.f;
	FR1BrainReadout Readout;
};
