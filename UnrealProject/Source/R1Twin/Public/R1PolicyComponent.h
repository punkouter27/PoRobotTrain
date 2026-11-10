// ONNX policy runner for the R1 digital twin (NNE / ONNX Runtime CPU, fixed 50 Hz).
// Written for UE 5.7; NOT YET COMPILED (see docs/ue5_setup.md).
//
// Data flow per policy step (game thread):
//   IR1ArticulationBridge  --GetJointPos/GetJointVel-->  FR1ObservationBuilder  -->  obs (1,N) float32
//   obs  --IModelInstanceCPU::RunSync-->  actions (1,A) float32
//   q_target[i] = default[i] + action_scale[i] * action[i]  --SetActuatorTarget-->  URLab UMjPDController
//
// All buffers are allocated once in BeginPlay; nothing in PolicyStep() touches the heap
// (NNE bindings are views onto those buffers; ORT itself may allocate internally - outside our control).
#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "Containers/ArrayView.h"
#include "Templates/UniquePtr.h"
#include "Templates/SharedPointer.h"
#include "NNERuntimeCPU.h"            // UE::NNE::IModelInstanceCPU, UE::NNE::FTensorBindingCPU (engine module "NNE")
#include "R1DeployConfig.h"
#include "R1ArticulationBridge.h"
#include "R1PolicyComponent.generated.h"

class UNNEModelData;
class UR1Level0SweepCommand;

/** Level 0 observation layout: [ joint_pos_rel(N) | joint_vel(N) | last_action(N) | command(C) ]. */
struct R1TWIN_API FR1Level0ObsLayout
{
	int32 NumJoints = 0;
	int32 NumCommands = 0;

	int32 JointPosRelOffset() const { return 0; }
	int32 JointVelOffset() const    { return NumJoints; }
	int32 LastActionOffset() const  { return 2 * NumJoints; }
	int32 CommandOffset() const     { return 3 * NumJoints; }
	int32 Size() const              { return 3 * NumJoints + NumCommands; }
};

/** Builds the Level 0 observation vector into a caller-provided buffer. Pure, allocation-free. */
class R1TWIN_API FR1ObservationBuilder
{
public:
	void Configure(int32 NumJoints, int32 NumCommands, float InJointPosScale, float InJointVelScale);
	const FR1Level0ObsLayout& GetLayout() const { return Layout; }

	/**
	 * Out[0..N)   = (JointPos - DefaultJointPos) * JointPosScale
	 * Out[N..2N)  = JointVel * JointVelScale
	 * Out[2N..3N) = LastAction
	 * Out[3N..)   = Command
	 * Returns false (and leaves Out untouched) if any view has the wrong size.
	 */
	bool Build(TConstArrayView<float> JointPos,
	           TConstArrayView<float> DefaultJointPos,
	           TConstArrayView<float> JointVel,
	           TConstArrayView<float> LastAction,
	           TConstArrayView<float> Command,
	           TArrayView<float> Out) const;

private:
	FR1Level0ObsLayout Layout;
	float JointPosScale = 1.f;
	float JointVelScale = 1.f;
};

UENUM(BlueprintType)
enum class ER1PolicyState : uint8
{
	Uninitialized,
	/** Bridge ready but no ONNX model: holds the default pose. */
	NoModel,
	/** No articulation bridge at all: ticks but does nothing. */
	NoBridge,
	Ready,
	Error,
};

UCLASS(ClassGroup = (R1), meta = (BlueprintSpawnableComponent))
class R1TWIN_API UR1PolicyComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	UR1PolicyComponent();

	// ------------------------------------------------------------------ configuration

	/** Deploy config + model. If null and bUseLevel0DefaultsIfNoAsset, FR1DeployConfig::MakeLevel0Default() is used. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "R1|Policy")
	TObjectPtr<UR1DeployConfigAsset> DeployConfigAsset;

	/** Optional: overrides DeployConfigAsset->PolicyModel. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "R1|Policy")
	TObjectPtr<UNNEModelData> PolicyModelOverride;

	UPROPERTY(EditAnywhere, Category = "R1|Policy")
	bool bUseLevel0DefaultsIfNoAsset = true;

	/** NNE runtime name. "NNERuntimeORTCpu" is registered by the NNERuntimeORT plugin. */
	UPROPERTY(EditAnywhere, Category = "R1|Policy")
	FString RuntimeName = TEXT("NNERuntimeORTCpu");

	/** Set to false to freeze targets at the current values (observations still update). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "R1|Policy")
	bool bInferenceEnabled = true;

	/** When no model is loaded, keep sending q_default so the PD controller holds the robot. */
	UPROPERTY(EditAnywhere, Category = "R1|Policy")
	bool bHoldDefaultPoseWhenNoModel = false;

	/** Catch-up cap for the fixed-rate accumulator (avoids a death spiral after a hitch). */
	UPROPERTY(EditAnywhere, Category = "R1|Policy", meta = (ClampMin = 1, ClampMax = 50))
	int32 MaxPolicyStepsPerFrame = 4;

	// ------------------------------------------------------------------ articulation

	/** Actor tag used to find the URLab Articulation Blueprint instance in the level. */
	UPROPERTY(EditAnywhere, Category = "R1|Articulation")
	FName ArticulationActorTag = TEXT("R1Articulation");

	/** Explicit articulation actor (optional; found by tag/class when null). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "R1|Articulation")
	TObjectPtr<AActor> ArticulationActor;

	/** Use FR1MockBridge when no URLab actor is found / the URLab API is unverified. */
	UPROPERTY(EditAnywhere, Category = "R1|Articulation")
	bool bAllowMockBridge = true;

	/** Push deploy kp/kd into the URLab PD controller on BeginPlay. */
	UPROPERTY(EditAnywhere, Category = "R1|Articulation")
	bool bApplyGainsOnBeginPlay = true;

	UPROPERTY(EditAnywhere, Category = "R1|Articulation")
	FName RootBodyName = TEXT("pelvis");

	// ------------------------------------------------------------------ command source

	/** Level 0 command generator. If null, the owner is searched for one. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "R1|Command")
	TObjectPtr<UR1Level0SweepCommand> CommandSource;

	// ------------------------------------------------------------------ debug

	/** Log obs/action every N policy steps (0 = off). Formatting allocates; keep off for timing runs. */
	UPROPERTY(EditAnywhere, Category = "R1|Debug", meta = (ClampMin = 0))
	int32 LogEveryNSteps = 0;

	// ------------------------------------------------------------------ runtime state (read-only)

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category = "R1|State")
	ER1PolicyState State = ER1PolicyState::Uninitialized;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category = "R1|State")
	int32 StepCount = 0;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category = "R1|State")
	float LastInferenceMs = 0.f;

	/** Rolling profile window (reset every PerfLogPeriodS). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "R1|Debug")
	float PerfLogPeriodS = 5.f;
	double PerfSumMs = 0.0;
	float PerfMaxMs = 0.f;
	int32 PerfCount = 0;
	double PerfWindowStart = 0.0;
	int32 PerfExternalSkips = 0;
	bool bSessionStarted = false;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category = "R1|State")
	FString BridgeDescription;

	// ------------------------------------------------------------------ API

	IR1ArticulationBridge* GetBridge() const { return Bridge.Get(); }
	const FR1DeployConfig& GetConfig() const { return Config; }
	TConstArrayView<float> GetLastObservation() const { return Obs; }
	TConstArrayView<float> GetLastAction() const { return LastAction; }
	TConstArrayView<float> GetLastTargets() const { return Targets; }
	TConstArrayView<FName> GetJointNames() const { return JointNames; }

	UFUNCTION(BlueprintCallable, Category = "R1|Policy")
	void SetInferenceEnabled(bool bEnabled) { bInferenceEnabled = bEnabled; }

	/** Zero last_action, reset accumulator and step counter (does not reload the model). */
	UFUNCTION(BlueprintCallable, Category = "R1|Policy")
	void ResetPolicy();

	UFUNCTION(BlueprintPure, Category = "R1|Policy")
	bool IsUsingMockBridge() const { return bUsingMockBridge; }

	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

private:
	bool ResolveConfig();
	bool CreateBridge();
	bool LoadModel();
	void AllocateBuffers();
	void ApplyGains();

	/** One fixed-rate policy step. Allocation-free. */
	void PolicyStep();
	void ReadJointState();
	bool RunInference();
	void ApplyActions();
	void HoldDefaultPose();

	FR1DeployConfig Config;
	TUniquePtr<IR1ArticulationBridge> Bridge;
	bool bUsingMockBridge = false;

	TSharedPtr<UE::NNE::IModelInstanceCPU> ModelInstance;
	TArray<UE::NNE::FTensorBindingCPU> InputBindings;
	TArray<UE::NNE::FTensorBindingCPU> OutputBindings;

	/** Resolved MJCF names, policy order (filled once). */
	TArray<FName> JointNames;
	TArray<FName> ActuatorNames;

	/** Pre-allocated buffers (policy order). */
	TArray<float> DefaultJointPos;
	TArray<float> ActionScale;
	TArray<float> JointPos;
	TArray<float> JointVel;
	TArray<float> LastAction;
	TArray<float> Command;
	TArray<float> Obs;
	TArray<float> Action;
	TArray<float> Targets;

	FR1ObservationBuilder ObsBuilder;

	float PolicyPeriod = 0.02f;
	float Accumulator = 0.f;
};
