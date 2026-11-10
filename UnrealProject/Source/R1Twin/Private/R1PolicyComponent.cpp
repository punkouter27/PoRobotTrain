// Written for UE 5.7; NOT YET COMPILED (see docs/ue5_setup.md).
//
// NNE API used (engine module "NNE", runtime from the NNERuntimeORT plugin). Written against the
// UE 5.4-5.6 public headers; re-check against 5.7 when compiling:
//   NNE.h              UE::NNE::GetRuntime<INNERuntimeCPU>(FString)      -> TWeakInterfacePtr<INNERuntimeCPU>
//   NNERuntimeCPU.h    INNERuntimeCPU::CreateModelCPU(TObjectPtr<UNNEModelData>) -> TSharedPtr<IModelCPU>
//                      IModelCPU::CreateModelInstanceCPU()               -> TSharedPtr<IModelInstanceCPU>
//   NNERuntimeRunSync.h IModelInstanceRunSync::SetInputTensorShapes(...)  -> ESetInputTensorShapesStatus::Ok
//                      IModelInstanceRunSync::RunSync(In, Out)           -> ERunSyncStatus::Ok
//   NNETypes.h         FTensorShape::Make(TConstArrayView<uint32>), FTensorShape::Volume(), FTensorDesc
#include "R1PolicyComponent.h"
#include "R1Twin.h"
#include "R1Level0SweepCommand.h"

#include "NNE.h"
#include "NNEModelData.h"
#include "NNERuntimeCPU.h"
#include "NNERuntimeRunSync.h"
#include "NNETypes.h"

#include "GameFramework/Actor.h"
#include "Engine/World.h"
#include "HAL/PlatformTime.h"

// =====================================================================================================
// FR1ObservationBuilder
// =====================================================================================================

void FR1ObservationBuilder::Configure(int32 NumJoints, int32 NumCommands, float InJointPosScale, float InJointVelScale)
{
	Layout.NumJoints = NumJoints;
	Layout.NumCommands = NumCommands;
	JointPosScale = InJointPosScale;
	JointVelScale = InJointVelScale;
}

bool FR1ObservationBuilder::Build(TConstArrayView<float> JointPos,
                                  TConstArrayView<float> DefaultJointPos,
                                  TConstArrayView<float> JointVel,
                                  TConstArrayView<float> LastAction,
                                  TConstArrayView<float> Command,
                                  TArrayView<float> Out) const
{
	const int32 N = Layout.NumJoints;
	const int32 C = Layout.NumCommands;
	if (JointPos.Num() != N || DefaultJointPos.Num() != N || JointVel.Num() != N || LastAction.Num() != N
		|| Command.Num() != C || Out.Num() != Layout.Size())
	{
		return false;
	}

	float* O = Out.GetData();
	const int32 PosOff = Layout.JointPosRelOffset();
	const int32 VelOff = Layout.JointVelOffset();
	const int32 ActOff = Layout.LastActionOffset();
	const int32 CmdOff = Layout.CommandOffset();

	for (int32 i = 0; i < N; ++i)
	{
		O[PosOff + i] = (JointPos[i] - DefaultJointPos[i]) * JointPosScale;
		O[VelOff + i] = JointVel[i] * JointVelScale;
		O[ActOff + i] = LastAction[i];
	}
	for (int32 c = 0; c < C; ++c)
	{
		O[CmdOff + c] = Command[c];
	}
	return true;
}

// =====================================================================================================
// UR1PolicyComponent
// =====================================================================================================

UR1PolicyComponent::UR1PolicyComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.bStartWithTickEnabled = true;
	// MuJoCo runs on URLab's async thread; we only read the latest snapshot, so any tick group works.
	// PrePhysics keeps us ahead of Chaos (which is disabled anyway) and of rendering.
	PrimaryComponentTick.TickGroup = TG_PrePhysics;
}

void UR1PolicyComponent::BeginPlay()
{
	Super::BeginPlay();

	State = ER1PolicyState::Uninitialized;

	if (!ResolveConfig())
	{
		State = ER1PolicyState::Error;
		SetComponentTickEnabled(false);
		return;
	}

	AllocateBuffers();

	if (!CreateBridge())
	{
		State = ER1PolicyState::NoBridge;
		UE_LOG(LogR1Twin, Error, TEXT("R1Policy: no articulation bridge available - component idle."));
		return;
	}
	BridgeDescription = Bridge->GetDescription();

	if (bApplyGainsOnBeginPlay)
	{
		ApplyGains();
	}

	if (!CommandSource)
	{
		CommandSource = GetOwner() ? GetOwner()->FindComponentByClass<UR1Level0SweepCommand>() : nullptr;
	}
	if (CommandSource)
	{
		// Make sure the command is refreshed before we sample it in the same frame.
		AddTickPrerequisiteComponent(CommandSource);
	}
	else if (Config.NumCommands > 0)
	{
		UE_LOG(LogR1Twin, Warning, TEXT("R1Policy: no UR1Level0SweepCommand found - command slice of the observation stays zero."));
	}

	State = LoadModel() ? ER1PolicyState::Ready : ER1PolicyState::NoModel;

	UE_LOG(LogR1Twin, Log, TEXT("R1Policy: state=%d, bridge=%s, %d joints, obs=%d, actions=%d, %.0f Hz"),
		static_cast<int32>(State), *BridgeDescription, JointNames.Num(), Config.NumObs, Config.NumActions, Config.PolicyHz);
}

void UR1PolicyComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	ModelInstance.Reset();
	Bridge.Reset();
	Super::EndPlay(EndPlayReason);
}

bool UR1PolicyComponent::ResolveConfig()
{
	if (DeployConfigAsset)
	{
		Config = DeployConfigAsset->Config;
	}
	else if (bUseLevel0DefaultsIfNoAsset)
	{
		Config = FR1DeployConfig::MakeLevel0Default();
		UE_LOG(LogR1Twin, Warning, TEXT("R1Policy: no DeployConfigAsset - using built-in Level 0 defaults from r1_joint_map.yaml."));
	}
	else
	{
		UE_LOG(LogR1Twin, Error, TEXT("R1Policy: no DeployConfigAsset assigned."));
		return false;
	}

	FString Error;
	if (!Config.Validate(Error))
	{
		UE_LOG(LogR1Twin, Error, TEXT("R1Policy: deploy config invalid: %s"), *Error);
		return false;
	}

	PolicyPeriod = 1.f / Config.PolicyHz;
	Accumulator = 0.f;
	return true;
}

void UR1PolicyComponent::AllocateBuffers()
{
	const int32 N = Config.NumActions;
	const int32 C = Config.NumCommands;

	JointNames.SetNum(N);
	ActuatorNames.SetNum(N);
	DefaultJointPos.SetNumUninitialized(N);
	ActionScale.SetNumUninitialized(N);
	for (int32 i = 0; i < N; ++i)
	{
		JointNames[i] = Config.GetJointName(i);
		ActuatorNames[i] = Config.GetActuatorName(i);
		DefaultJointPos[i] = Config.DefaultJointPos[i];
		ActionScale[i] = Config.ActionScale[i];
	}

	JointPos.SetNumZeroed(N);
	JointVel.SetNumZeroed(N);
	LastAction.SetNumZeroed(N);
	Action.SetNumZeroed(N);
	Targets = DefaultJointPos;
	Command.SetNumZeroed(C);
	Obs.SetNumZeroed(Config.NumObs);

	ObsBuilder.Configure(N, C, Config.ObsJointPosScale, Config.ObsJointVelScale);
	check(ObsBuilder.GetLayout().Size() == Config.NumObs);

	// Tensor bindings are plain {void* Data; uint64 SizeInBytes;} views onto the buffers above.
	InputBindings.SetNum(1);
	InputBindings[0].Data = Obs.GetData();
	InputBindings[0].SizeInBytes = static_cast<uint64>(Obs.Num()) * sizeof(float);

	OutputBindings.SetNum(1);
	OutputBindings[0].Data = Action.GetData();
	OutputBindings[0].SizeInBytes = static_cast<uint64>(Action.Num()) * sizeof(float);
}

bool UR1PolicyComponent::CreateBridge()
{
	FR1BridgeInitParams Params;
	Params.JointNames = JointNames;
	Params.ActuatorNames = ActuatorNames;
	Params.RootBodyName = RootBodyName;
	Params.InitialJointPos.SetNumUninitialized(DefaultJointPos.Num());
	for (int32 i = 0; i < DefaultJointPos.Num(); ++i)
	{
		Params.InitialJointPos[i] = DefaultJointPos[i];
	}

	AActor* Art = ArticulationActor;
	if (!Art)
	{
		Art = R1Urlab::FindArticulationActor(GetWorld(), ArticulationActorTag);
	}
	Params.ArticulationActor = Art;
	if (Art)
	{
		UE_LOG(LogR1Twin, Log, TEXT("R1Policy: articulation actor candidate '%s' (class %s)"), *Art->GetName(), *Art->GetClass()->GetName());
	}
	else
	{
		UE_LOG(LogR1Twin, Warning, TEXT("R1Policy: no articulation actor found (tag '%s'); %s"),
			*ArticulationActorTag.ToString(), bAllowMockBridge ? TEXT("using mock bridge") : TEXT("mock bridge disabled"));
	}

	Bridge = R1Urlab::CreateBridge(Params, bAllowMockBridge);
	if (!Bridge)
	{
		return false;
	}
	bUsingMockBridge = (Art == nullptr) || (Bridge->GetDescription().Contains(TEXT("Mock")));
	return true;
}

void UR1PolicyComponent::ApplyGains()
{
	for (int32 i = 0; i < ActuatorNames.Num(); ++i)
	{
		Bridge->SetActuatorGains(ActuatorNames[i], Config.Kp[i], Config.Kd[i]);
	}
	UE_LOG(LogR1Twin, Log, TEXT("R1Policy: pushed kp/kd for %d actuators to %s"), ActuatorNames.Num(), *Bridge->GetDescription());
}

bool UR1PolicyComponent::LoadModel()
{
	UNNEModelData* ModelData = PolicyModelOverride ? PolicyModelOverride.Get()
		: (DeployConfigAsset ? DeployConfigAsset->PolicyModel.Get() : nullptr);
	if (!ModelData)
	{
		UE_LOG(LogR1Twin, Warning, TEXT("R1Policy: no UNNEModelData assigned - holding default pose (import the .onnx and set PolicyModel)."));
		return false;
	}

	TWeakInterfacePtr<INNERuntimeCPU> Runtime = UE::NNE::GetRuntime<INNERuntimeCPU>(RuntimeName);
	if (!Runtime.IsValid())
	{
		UE_LOG(LogR1Twin, Error, TEXT("R1Policy: NNE runtime '%s' not found. Is the NNERuntimeORT plugin enabled?"), *RuntimeName);
		return false;
	}

	if (Runtime->CanCreateModelCPU(ModelData) != INNERuntimeCPU::ECanCreateModelCPUStatus::Ok)
	{
		UE_LOG(LogR1Twin, Error, TEXT("R1Policy: runtime '%s' cannot create a CPU model from '%s'."), *RuntimeName, *ModelData->GetName());
		return false;
	}

	TSharedPtr<UE::NNE::IModelCPU> Model = Runtime->CreateModelCPU(ModelData);
	if (!Model.IsValid())
	{
		UE_LOG(LogR1Twin, Error, TEXT("R1Policy: CreateModelCPU failed for '%s'."), *ModelData->GetName());
		return false;
	}

	ModelInstance = Model->CreateModelInstanceCPU();
	if (!ModelInstance.IsValid())
	{
		UE_LOG(LogR1Twin, Error, TEXT("R1Policy: CreateModelInstanceCPU failed."));
		return false;
	}

	// ---- validate the ONNX contract: input "obs" (1,N) float32, output "actions" (1,A) float32
	TConstArrayView<UE::NNE::FTensorDesc> InputDescs = ModelInstance->GetInputTensorDescs();
	TConstArrayView<UE::NNE::FTensorDesc> OutputDescs = ModelInstance->GetOutputTensorDescs();
	if (InputDescs.Num() != 1 || OutputDescs.Num() != 1)
	{
		UE_LOG(LogR1Twin, Error, TEXT("R1Policy: expected 1 input / 1 output, model has %d / %d."), InputDescs.Num(), OutputDescs.Num());
		ModelInstance.Reset();
		return false;
	}
	if (InputDescs[0].GetName() != Config.InputTensorName || OutputDescs[0].GetName() != Config.OutputTensorName)
	{
		UE_LOG(LogR1Twin, Warning, TEXT("R1Policy: tensor names are '%s'/'%s', deploy config expects '%s'/'%s' (continuing by position)."),
			*InputDescs[0].GetName(), *OutputDescs[0].GetName(), *Config.InputTensorName, *Config.OutputTensorName);
	}
	if (InputDescs[0].GetDataType() != ENNETensorDataType::Float || OutputDescs[0].GetDataType() != ENNETensorDataType::Float)
	{
		UE_LOG(LogR1Twin, Error, TEXT("R1Policy: tensors must be float32."));
		ModelInstance.Reset();
		return false;
	}

	// Concrete input shape (1, NumObs) - the batch dim is usually symbolic (-1) in the exported model.
	const uint32 InputDims[2] = { 1u, static_cast<uint32>(Config.NumObs) };
	const UE::NNE::FTensorShape InputShape = UE::NNE::FTensorShape::Make(MakeArrayView(InputDims, 2));
	if (ModelInstance->SetInputTensorShapes(MakeArrayView(&InputShape, 1)) != UE::NNE::IModelInstanceRunSync::ESetInputTensorShapesStatus::Ok)
	{
		UE_LOG(LogR1Twin, Error, TEXT("R1Policy: SetInputTensorShapes((1,%d)) failed - check NumObs against the exported model."), Config.NumObs);
		ModelInstance.Reset();
		return false;
	}

	// Runtimes such as NNERuntimeORTCpu only resolve output shapes on the first run (Volume() == 0 here).
	// In that case validate the symbolic output description instead: its last dim must equal NumActions.
	TConstArrayView<UE::NNE::FTensorShape> OutputShapes = ModelInstance->GetOutputTensorShapes();
	bool bOutputOk = OutputShapes.Num() == 1 && OutputShapes[0].Volume() == static_cast<uint64>(Config.NumActions);
	if (!bOutputOk && OutputDescs.Num() == 1 && (OutputShapes.Num() == 0 || OutputShapes[0].Volume() == 0))
	{
		const TConstArrayView<int32> Sym = OutputDescs[0].GetShape().GetData();
		bOutputOk = Sym.Num() >= 1 && Sym[Sym.Num() - 1] == Config.NumActions;
	}
	if (!bOutputOk)
	{
		UE_LOG(LogR1Twin, Error, TEXT("R1Policy: output volume %llu != NumActions %d."),
			OutputShapes.Num() ? OutputShapes[0].Volume() : 0ull, Config.NumActions);
		ModelInstance.Reset();
		return false;
	}

	UE_LOG(LogR1Twin, Log, TEXT("R1Policy: loaded '%s' on %s: %s(1,%d) -> %s(1,%d)"),
		*ModelData->GetName(), *RuntimeName, *InputDescs[0].GetName(), Config.NumObs, *OutputDescs[0].GetName(), Config.NumActions);
	return true;
}

void UR1PolicyComponent::ResetPolicy()
{
	FMemory::Memzero(LastAction.GetData(), LastAction.Num() * sizeof(float));
	FMemory::Memzero(Action.GetData(), Action.Num() * sizeof(float));
	Targets = DefaultJointPos;
	Accumulator = 0.f;
	StepCount = 0;
	if (CommandSource)
	{
		CommandSource->ResetClock();
	}
}

void UR1PolicyComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	if (!Bridge)
	{
		return;
	}

	// Fixed-rate accumulator: run PolicyStep every PolicyPeriod of wall/game time, with a catch-up cap.
	// URLab's MuJoCo thread is free-running (SimSpeedPercent=100 => real time), so game-time stepping
	// keeps the policy at its trained 50 Hz relative to the physics.
	Accumulator += DeltaTime;
	int32 Steps = 0;
	while (Accumulator >= PolicyPeriod && Steps < MaxPolicyStepsPerFrame)
	{
		Bridge->Advance(PolicyPeriod); // mock only; real bridge ignores
		PolicyStep();
		Accumulator -= PolicyPeriod;
		++Steps;
	}
	if (Accumulator > PolicyPeriod * MaxPolicyStepsPerFrame)
	{
		// Hitch: drop the backlog rather than spiralling.
		Accumulator = 0.f;
	}
}

void UR1PolicyComponent::PolicyStep()
{
	// A Python runner driving URLab in Direct/Puppet mode owns the actuators: observe only.
	if (Bridge && Bridge->IsExternallyStepped())
	{
		Bridge->SetInEngineControl(false);
		++PerfExternalSkips;
		return;
	}
	if (Bridge && State == ER1PolicyState::Ready && ModelInstance.IsValid())
	{
		if (!bSessionStarted)
		{
			Bridge->StartInEngineSession();
			bSessionStarted = true;
			ResetPolicy();
		}
		Bridge->SetInEngineControl(true);
	}
	ReadJointState();

	if (CommandSource)
	{
		CommandSource->FillCommand(Command);
	}

	if (State != ER1PolicyState::Ready || !ModelInstance.IsValid())
	{
		if (bHoldDefaultPoseWhenNoModel)
		{
			HoldDefaultPose();
		}
		return;
	}

	if (!ObsBuilder.Build(JointPos, DefaultJointPos, JointVel, LastAction, Command, Obs))
	{
		UE_LOG(LogR1Twin, Error, TEXT("R1Policy: observation build failed (size mismatch) - disabling."));
		State = ER1PolicyState::Error;
		return;
	}

	if (!bInferenceEnabled)
	{
		return; // keep last targets
	}

	if (!RunInference())
	{
		State = ER1PolicyState::Error;
		return;
	}

	ApplyActions();
	++StepCount;

	if (LogEveryNSteps > 0 && (StepCount % LogEveryNSteps) == 0)
	{
		// Debug only: FString formatting allocates.
		FString ObsStr, ActStr;
		for (float V : Obs)    { ObsStr += FString::Printf(TEXT("%.3f "), V); }
		for (float V : Action) { ActStr += FString::Printf(TEXT("%.3f "), V); }
		UE_LOG(LogR1Twin, Log, TEXT("R1Policy step %d (%.2f ms): obs=[%s] act=[%s]"), StepCount, LastInferenceMs, *ObsStr, *ActStr);
	}
}

void UR1PolicyComponent::ReadJointState()
{
	for (int32 i = 0; i < JointNames.Num(); ++i)
	{
		JointPos[i] = static_cast<float>(Bridge->GetJointPos(JointNames[i]));
		JointVel[i] = static_cast<float>(Bridge->GetJointVel(JointNames[i]));
	}
}

bool UR1PolicyComponent::RunInference()
{
	const double T0 = FPlatformTime::Seconds();
	const UE::NNE::IModelInstanceRunSync::ERunSyncStatus Status = ModelInstance->RunSync(InputBindings, OutputBindings);
	LastInferenceMs = static_cast<float>((FPlatformTime::Seconds() - T0) * 1000.0);
	PerfSumMs += LastInferenceMs;
	PerfMaxMs = FMath::Max(PerfMaxMs, LastInferenceMs);
	++PerfCount;
	const double Now = FPlatformTime::Seconds();
	if (PerfWindowStart <= 0.0) { PerfWindowStart = Now; }
	if (Now - PerfWindowStart >= PerfLogPeriodS && PerfCount > 0)
	{
		UE_LOG(LogR1Twin, Log, TEXT("R1Policy perf: %d steps in %.2f s (%.1f Hz), NNE inference avg %.3f ms max %.3f ms"),
			PerfCount, Now - PerfWindowStart, PerfCount / (Now - PerfWindowStart), PerfSumMs / PerfCount, PerfMaxMs);
		PerfSumMs = 0.0; PerfMaxMs = 0.f; PerfCount = 0; PerfWindowStart = Now;
	}

	if (Status != UE::NNE::IModelInstanceRunSync::ERunSyncStatus::Ok)
	{
		UE_LOG(LogR1Twin, Error, TEXT("R1Policy: RunSync failed at step %d."), StepCount);
		return false;
	}

	// Guard against NaN/Inf escaping into the PD controller.
	for (int32 i = 0; i < Action.Num(); ++i)
	{
		if (!FMath::IsFinite(Action[i]))
		{
			UE_LOG(LogR1Twin, Error, TEXT("R1Policy: non-finite action[%d] at step %d."), i, StepCount);
			return false;
		}
	}
	return true;
}

void UR1PolicyComponent::ApplyActions()
{
	const float Clip = Config.ActionClipAbs;
	for (int32 i = 0; i < Action.Num(); ++i)
	{
		float A = Action[i];
		if (Clip > 0.f)
		{
			A = FMath::Clamp(A, -Clip, Clip);
		}
		LastAction[i] = A;                                   // next observation sees the clipped action
		Targets[i] = DefaultJointPos[i] + ActionScale[i] * A;
		if (Config.SoftLimitLo.Num() == Action.Num() && Config.SoftLimitHi.Num() == Action.Num())
		{
			Targets[i] = FMath::Clamp(Targets[i], Config.SoftLimitLo[i], Config.SoftLimitHi[i]);
		}
		Bridge->SetActuatorTarget(ActuatorNames[i], Targets[i]);
	}
}

void UR1PolicyComponent::HoldDefaultPose()
{
	for (int32 i = 0; i < ActuatorNames.Num(); ++i)
	{
		Targets[i] = DefaultJointPos[i];
		Bridge->SetActuatorTarget(ActuatorNames[i], Targets[i]);
	}
}
