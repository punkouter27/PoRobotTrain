#include "R1SkillBrain.h"
#include "R1Twin.h"
#include "R1Showcase.h"
#include "MjUeConvert.h"

#include "NNE.h"
#include "NNEModelData.h"
#include "NNERuntimeRunSync.h"
#include "NNETypes.h"

#include "Components/SceneCaptureComponent2D.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "Engine/TextureRenderTarget2D.h"
#include "Engine/World.h"
#include "GameFramework/PlayerController.h"
#include "HAL/FileManager.h"
#include "HAL/PlatformTime.h"
#include "ImageUtils.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "TextureResource.h"

namespace
{
	const FName HeadPitchLink(TEXT("head_pitch_link"));
	const FName HeadYawLink(TEXT("head_yaw_link"));
	const FName Pelvis(TEXT("pelvis"));
	// head_rgbd in the MJCF (scripts/build_mjcf.py): on head_yaw_link, 20 deg down, 58 deg vertical FOV.
	const FVector CamPosMj(0.066971, 0.0, -0.021637);
	const FVector CamFwdMj(0.93969, 0.0, -0.34202);
	const FVector CamUpMj(0.34202, 0.0, 0.93969);
	constexpr float CamHFovDeg = 72.9f;   // 58 deg vertical at 4:3
	constexpr float LookStepS = 2.5f;     // Look At demo sequence, as robotcontrol.behaviours.head_skills.look_at_script

	FString PrettyJoint(FName N)
	{
		FString S = N.ToString();
		S.RemoveFromEnd(TEXT("_joint"));
		S.ReplaceInline(TEXT("_"), TEXT(" "));
		return S;
	}
}

// =====================================================================================================
// AR1ScenarioProps
// =====================================================================================================

AR1ScenarioProps::AR1ScenarioProps()
{
	Root = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
	SetRootComponent(Root);
	Ball = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Ball"));
	Ball->SetupAttachment(Root);
	Ball->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	Ball->SetCastShadow(true);
	Ball->SetWorldScale3D(FVector(0.12f));
	if (UStaticMesh* Sphere = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Sphere.Sphere")))
	{
		Ball->SetStaticMesh(Sphere);
	}
	HeadCam = CreateDefaultSubobject<USceneCaptureComponent2D>(TEXT("HeadCam"));
	HeadCam->SetupAttachment(Root);
	HeadCam->bCaptureEveryFrame = false;
	HeadCam->bCaptureOnMovement = false;
	HeadCam->FOVAngle = CamHFovDeg;
	HeadCam->CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR;
}

void AR1ScenarioProps::BeginPlay()
{
	Super::BeginPlay();
	if (UMaterialInterface* Mat = LoadObject<UMaterialInterface>(nullptr, TEXT("/Engine/BasicShapes/BasicShapeMaterial.BasicShapeMaterial")))
	{
		BallMat = UMaterialInstanceDynamic::Create(Mat, this);
		BallMat->SetVectorParameterValue(TEXT("Color"), FLinearColor(1.f, 0.35f, 0.08f, 1.f));
		Ball->SetMaterial(0, BallMat);
	}
	HeadCamRT = NewObject<UTextureRenderTarget2D>(this);
	HeadCamRT->InitCustomFormat(320, 240, PF_B8G8R8A8, /*bForceLinearGamma*/ false);
	HeadCamRT->ClearColor = FLinearColor::Black;
	HeadCam->TextureTarget = HeadCamRT;
}

// =====================================================================================================
// UR1SkillBrainComponent
// =====================================================================================================

UR1SkillBrainComponent::UR1SkillBrainComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.bTickEvenWhenPaused = false;
}

void UR1SkillBrainComponent::BeginPlay()
{
	Super::BeginPlay();
	ReloadSkills();
	if (UWorld* World = GetWorld())
	{
		FActorSpawnParameters P;
		P.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
		P.ObjectFlags |= RF_Transient;
		Props = World->SpawnActor<AR1ScenarioProps>(AR1ScenarioProps::StaticClass(), FTransform::Identity, P);
		if (Props)
		{
			Props->Ball->SetVisibility(false);
		}
	}
}

void UR1SkillBrainComponent::EndPlay(const EEndPlayReason::Type Reason)
{
	StopRecording();   // keep the labels of the frames already written
	if (Bridge && PushLeftS > 0.f)
	{
		Bridge->ClearBodyWrench(HeadYawLink);
	}
	if (Props)
	{
		Props->Destroy();
		Props = nullptr;
	}
	Super::EndPlay(Reason);
}

void UR1SkillBrainComponent::ReloadSkills()
{
	Stop();
	Skills = R1Skills::LoadAll(R1Skills::DefaultDir());
	Models.Reset();
	ModelData.Reset();
	// joints the brain reads and commands: both head joints plus every skill's hold set (waist + arms)
	TArray<FName> Joints = {R1Skills::HeadPitch, R1Skills::HeadYaw};
	for (const FR1SkillDef& S : Skills)
	{
		for (const FName& N : S.HoldJoints)
		{
			Joints.AddUnique(N);
		}
	}
	if (Joints != BridgeJoints)
	{
		BridgeJoints = Joints;
		Bridge.Reset();   // rebuilt on the next tick with the new joint set
		bSessionStarted = false;
	}
	UE_LOG(LogR1Twin, Log, TEXT("R1SkillBrain: %d skill(s) from %s"), Skills.Num(), *R1Skills::DefaultDir());
}

int32 UR1SkillBrainComponent::FindSkill(const FString& Id) const
{
	return Skills.IndexOfByPredicate([&Id](const FR1SkillDef& S) { return S.Id == Id; });
}

bool UR1SkillBrainComponent::EnsureBridge()
{
	if (Bridge)
	{
		return true;
	}
	if (BridgeJoints.Num() == 0)
	{
		return false;
	}
	FR1BridgeInitParams Params;
	Params.JointNames = BridgeJoints;
	for (const FName& N : BridgeJoints)
	{
		FString A = N.ToString();
		A.RemoveFromEnd(TEXT("_joint"));
		Params.ActuatorNames.Add(FName(*A));
		Params.InitialJointPos.Add(0.0);
	}
	Params.RootBodyName = Pelvis;
	Params.ExtraBodyNames = {HeadPitchLink, HeadYawLink};
	AActor* Art = R1Urlab::FindArticulationActor(GetWorld(), ArticulationActorTag);
	Params.ArticulationActor = Art;
	Bridge = R1Urlab::CreateBridge(Params, /*bAllowMock*/ true);
	if (!Bridge)
	{
		return false;
	}
	Readout.bConnected = Art != nullptr && !Bridge->GetDescription().Contains(TEXT("Mock"));
	// motor model: one slot per commanded joint; effort from the skills' deploy.json, else the joint map
	const int32 N = BridgeJoints.Num();
	ActuatorSlot.Reset();
	MotorEffort.Reset(N);
	for (int32 i = 0; i < N; ++i)
	{
		ActuatorSlot.Add(Params.ActuatorNames[i], i);
		float Effort = FR1MotorModel::JointMapEffort(BridgeJoints[i]);
		for (const FR1SkillDef& S : Skills)
		{
			const int32 J = S.JointIndex(BridgeJoints[i]);
			if (J != INDEX_NONE && S.Effort.IsValidIndex(J))
			{
				Effort = S.Effort[J];
				break;
			}
		}
		MotorEffort.Add(Effort);
	}
	MotorTarget.Init(0.f, N);
	MotorKp.Init(0.f, N);
	MotorKd.Init(0.f, N);
	MotorTau.Init(0.f, N);
	MotorDq.Init(0.f, N);
	Motor.Reset(BridgeJoints, MotorEffort);
	MotorAccumulator = 0.f;
	if (Props && Art)
	{
		Props->HeadCam->HiddenActors.AddUnique(Art);   // the camera sits inside the head shell: show the world, not the robot
	}
	UE_LOG(LogR1Twin, Log, TEXT("R1SkillBrain: bridge %s (%d joints)"), *Bridge->GetDescription(), BridgeJoints.Num());
	return true;
}

bool UR1SkillBrainComponent::EnsureModel(int32 SkillIndex, FString& OutWhy)
{
	if (const FModel* M = Models.Find(SkillIndex))
	{
		OutWhy = M->Error;
		return M->Instance.IsValid();
	}
	FModel& M = Models.Add(SkillIndex);
	const FR1SkillDef& S = Skills[SkillIndex];
	TArray64<uint8> Bytes;
	if (!FFileHelper::LoadFileToArray(Bytes, *S.OnnxPath))
	{
		M.Error = TEXT("cannot read policy.onnx");
	}
	else
	{
		TWeakInterfacePtr<INNERuntimeCPU> Runtime = UE::NNE::GetRuntime<INNERuntimeCPU>(RuntimeName);
		UNNEModelData* Data = NewObject<UNNEModelData>(this);
		Data->Init(TEXT("onnx"), Bytes);
		ModelData.Add(Data);
		if (!Runtime.IsValid())
		{
			M.Error = FString::Printf(TEXT("NNE runtime %s not found"), *RuntimeName);
		}
		else if (Runtime->CanCreateModelCPU(Data) != INNERuntimeCPU::ECanCreateModelCPUStatus::Ok)
		{
			M.Error = TEXT("ONNX Runtime cannot load this policy");
		}
		else if (TSharedPtr<UE::NNE::IModelCPU> Model = Runtime->CreateModelCPU(Data))
		{
			M.Instance = Model->CreateModelInstanceCPU();
			const uint32 Dims[2] = {1u, static_cast<uint32>(S.NumObs)};
			const UE::NNE::FTensorShape Shape = UE::NNE::FTensorShape::Make(MakeArrayView(Dims, 2));
			if (!M.Instance.IsValid()
				|| M.Instance->SetInputTensorShapes(MakeArrayView(&Shape, 1)) != UE::NNE::IModelInstanceRunSync::ESetInputTensorShapesStatus::Ok)
			{
				M.Instance.Reset();
				M.Error = FString::Printf(TEXT("policy does not take %d observations"), S.NumObs);
			}
		}
		else
		{
			M.Error = TEXT("CreateModelCPU failed");
		}
	}
	OutWhy = M.Error;
	UE_LOG(LogR1Twin, Log, TEXT("R1SkillBrain: %s model %s"), *S.Id, M.Instance.IsValid() ? TEXT("loaded") : *M.Error);
	return M.Instance.IsValid();
}

bool UR1SkillBrainComponent::Play(int32 SkillIndex, float InSizeDeg, float DurationS, FString& OutWhy)
{
	EndScenario();   // a manual Play replaces a running scenario script (PlayScenario sets it again afterwards)
	if (!Skills.IsValidIndex(SkillIndex))
	{
		OutWhy = TEXT("no such skill");
		return false;
	}
	const FR1SkillDef& S = Skills[SkillIndex];
	if (!S.bReady)
	{
		OutWhy = FString::Printf(TEXT("%s is locked: %s"), *S.Label, *S.Reason);
		return false;
	}
	if (Readout.bExternal)
	{
		OutWhy = TEXT("a Python client is driving the virtual robot (web app preview or walk sandbox)");
		return false;
	}
	if (!EnsureBridge() || !EnsureModel(SkillIndex, OutWhy))
	{
		if (OutWhy.IsEmpty()) OutWhy = TEXT("no articulation");
		return false;
	}
	TimelineIndex = -1;
	StartSkill(SkillIndex, InSizeDeg, DurationS);
	return true;
}

bool UR1SkillBrainComponent::PlayTimeline(FString& OutWhy)
{
	EndScenario();
	if (Timeline.Num() == 0)
	{
		OutWhy = TEXT("the timeline is empty: add steps first");
		return false;
	}
	for (const FR1TimelineStep& Step : Timeline)
	{
		FString Why;
		if (!Skills.IsValidIndex(Step.Skill) || !Skills[Step.Skill].bReady || !EnsureBridge() || !EnsureModel(Step.Skill, Why))
		{
			OutWhy = FString::Printf(TEXT("step %s cannot run%s%s"), Skills.IsValidIndex(Step.Skill) ? *Skills[Step.Skill].Label : TEXT("?"),
				Why.IsEmpty() ? TEXT("") : TEXT(": "), *Why);
			return false;
		}
	}
	if (Readout.bExternal)
	{
		OutWhy = TEXT("a Python client is driving the virtual robot");
		return false;
	}
	TimelineIndex = 0;
	StartSkill(Timeline[0].Skill, Timeline[0].SizeDeg, Timeline[0].DurationS);
	return true;
}

void UR1SkillBrainComponent::StartSkill(int32 SkillIndex, float InSizeDeg, float DurationS)
{
	const FR1SkillDef& S = Skills[SkillIndex];
	Active = SkillIndex;
	SizeDeg = FMath::Min(InSizeDeg, S.MaxDeg);
	RunS = FMath::Max(DurationS, 2.f);
	if (S.PeriodS > 0.f)
	{
		RunS = FMath::CeilToFloat(RunS / S.PeriodS) * S.PeriodS;   // end on a whole cycle, near centre
	}
	T = 0.f;
	Accumulator = 0.f;
	const int32 K = S.NumJoints();
	Q.SetNumZeroed(K);
	Dq.SetNumZeroed(K);
	LastAction.SetNumZeroed(K);
	Target = S.Default;
	Ref.SetNumZeroed(K);
	Obs.SetNumZeroed(S.NumObs);
	Action.SetNumZeroed(S.bUnitePortable ? R1Skills::UniteNumAct : K);
	PrevRef.SetNumZeroed(K);
	DelayQueue.SetNum(5);   // up to 4 steps (80 ms) of extra delay
	for (TArray<float>& D : DelayQueue)
	{
		D = S.Default;
	}
	DelayHead = 0;
	ActionView.Reset(K, R1Skills::PolicyDt);
	LastJumpT = 0.f;
	ErrSum = 0.f;
	ErrCount = 0;
	ErrSince = -1.0;
	OrbitPhase = 0.f;
	Readout.RobotWouldStop.Reset();
	Readout.ErrMaxDeg = Readout.ErrMeanDeg = Readout.OtherMaxDeg = 0.f;
	Readout.ValidatedErrDeg = S.TrackErrDeg;
	Readout.SkillLabel = S.Label;
	Readout.JointLabels.Reset();
	for (const FName& N : S.Joints) Readout.JointLabels.Add(PrettyJoint(N));
	HoldStart.Reset();
	if (Bridge)
	{
		for (const FName& N : S.HoldJoints) HoldStart.Add(N, static_cast<float>(Bridge->GetJointPos(N)));
		// the real runner refuses to start unless both head joints face forward (within 10 deg)
		for (const FName& N : {R1Skills::HeadPitch, R1Skills::HeadYaw})
		{
			const float Deg = FMath::RadiansToDegrees(static_cast<float>(Bridge->GetJointPos(N)));
			if (FMath::Abs(Deg) > 10.f)
			{
				Readout.RobotWouldStop = FString::Printf(TEXT("would refuse to start: %s at %+.0f deg (needs within 10 deg of centre)"), *PrettyJoint(N), Deg);
			}
		}
	}
	AppliedStrength = -1.f;   // re-apply gains (strength) for the new skill
	UE_LOG(LogR1Twin, Log, TEXT("R1SkillBrain: play %s size %.0f deg for %.1f s"), *S.Id, SizeDeg, RunS);
}

void UR1SkillBrainComponent::Stop()
{
	EndScenario();
	Active = -1;
	TimelineIndex = -1;
	Readout.bRunning = false;
	Readout.bSettling = false;
	if (Bridge && bSessionStarted && !Readout.bExternal)
	{
		HoldPose();
	}
}

void UR1SkillBrainComponent::FinishActive()
{
	if (TimelineIndex >= 0)
	{
		++TimelineIndex;
		if (TimelineIndex >= Timeline.Num() && bLoopTimeline)
		{
			TimelineIndex = 0;
		}
		if (Timeline.IsValidIndex(TimelineIndex))
		{
			const FR1TimelineStep& Step = Timeline[TimelineIndex];
			StartSkill(Step.Skill, Step.SizeDeg, Step.DurationS);
			return;
		}
	}
	UE_LOG(LogR1Twin, Log, TEXT("R1SkillBrain: done (mean err %.2f deg, max %.2f deg)"), Readout.ErrMeanDeg, Readout.ErrMaxDeg);
	Active = -1;
	TimelineIndex = -1;
	Readout.bRunning = false;
	Readout.bSettling = false;
	// keep the final targets (the 1 s Hold Centre settle left the head facing forward)
}

void UR1SkillBrainComponent::NudgeTarget(float DYawDeg, float DPitchDeg)
{
	if (TargetMode != ER1TargetMode::Manual)
	{
		SetTargetMode(ER1TargetMode::Manual);
	}
	TargetYawDeg = FMath::Clamp(TargetYawDeg + DYawDeg, -60.f, 60.f);
	TargetPitchDeg = FMath::Clamp(TargetPitchDeg + DPitchDeg, -35.f, 35.f);
}

void UR1SkillBrainComponent::SetTargetMode(ER1TargetMode Mode)
{
	TargetMode = Mode;
	OrbitPhase = 0.f;
}

void UR1SkillBrainComponent::Push(const FVector2D& Dir)
{
	// a shove a hand would give: ~1.2 Nm about the yaw axis (the yaw motor's limit is 0.86 Nm), ~5 Nm about pitch
	PushTorqueMj = FVector(0.0, 5.0 * Dir.Y, 1.2 * Dir.X);
	PushLeftS = 0.25f;   // docs/sim_features.md: the scripted push is the same 0.25 s shove
}

void UR1SkillBrainComponent::ApplyGains()
{
	if (!Bridge)
	{
		return;
	}
	const float Scale = StrengthPct / 100.f;
	for (const FR1SkillDef& S : Skills)
	{
		for (int32 i = 0; i < S.HoldJoints.Num(); ++i)
		{
			const bool bHead = S.HoldJoints[i] == R1Skills::HeadPitch || S.HoldJoints[i] == R1Skills::HeadYaw;
			SetGains(S.HoldActuators[i], S.HoldKp[i] * (bHead ? Scale : 1.f), S.HoldKd[i] * (bHead ? Scale : 1.f));
		}
		for (int32 i = 0; i < S.NumJoints(); ++i)
		{
			SetGains(S.Actuators[i], S.Kp[i] * Scale, S.Kd[i] * Scale);
		}
	}
	AppliedStrength = StrengthPct;
}

void UR1SkillBrainComponent::HoldPose()
{
	// every commanded joint at its default (head facing forward, waist and arms at the stand pose)
	for (const FR1SkillDef& S : Skills)
	{
		for (int32 i = 0; i < S.HoldJoints.Num(); ++i)
		{
			SetTarget(S.HoldActuators[i], S.HoldDefault[i]);
		}
	}
}

void UR1SkillBrainComponent::FillCommand(const FR1SkillDef& S, float InT, float Cmd[R1Skills::NumCmd])
{
	Cmd[0] = Cmd[1] = Cmd[2] = 0.f;
	Readout.bLookClamped = false;
	if (InT >= RunS || S.Id == TEXT("hold_centre"))
	{
		return;   // settle = Hold Centre with this skill's policy, as the runtime does
	}
	if (S.Id == TEXT("look_at"))
	{
		const float YawLim = FMath::Min(SizeDeg, FMath::RadiansToDegrees(S.YawRange));
		const float PitchLim = FMath::Min(SizeDeg, FMath::RadiansToDegrees(S.PitchRange));
		const float Yaw = FMath::Clamp(TargetYawDeg, -YawLim, YawLim);
		const float Pitch = FMath::Clamp(TargetPitchDeg, -PitchLim, PitchLim);
		Readout.bLookClamped = !FMath::IsNearlyEqual(Yaw, TargetYawDeg, 0.01f) || !FMath::IsNearlyEqual(Pitch, TargetPitchDeg, 0.01f);
		Cmd[0] = FMath::DegreesToRadians(Yaw);
		Cmd[1] = FMath::DegreesToRadians(Pitch);
		Readout.LookCmdDeg = FVector2D(Yaw, Pitch);
		return;
	}
	if (S.bPose)
	{
		for (int32 c = 0; c < R1Skills::NumCmd; ++c)
		{
			Cmd[c] = S.PoseCommand[c];
		}
		return;
	}
	if (S.PeriodS > 0.f)
	{
		R1Skills::PhaseCommand(S, InT, SizeDeg / FMath::Max(S.MaxDeg, 1e-3f), Cmd);
	}
}

void UR1SkillBrainComponent::UpdateTargetDirection(float DeltaTime)
{
	const bool bLookActive = Skills.IsValidIndex(Active) && Skills[Active].Id == TEXT("look_at");
	const float Reach = bLookActive ? SizeDeg : 20.f;
	if (TargetMode == ER1TargetMode::Script)
	{
		if (bLookActive)
		{
			const float P = FMath::Min(Reach, 20.f) * 0.5f;
			const FVector2D Seq[] = {{0.f, 0.f}, {Reach, 0.f}, {-Reach, 0.f}, {0.f, P}, {0.f, -P}, {0.5f * Reach, 0.5f * P}};
			const FVector2D& Cur = Seq[FMath::FloorToInt(FMath::Max(T, 0.f) / LookStepS) % UE_ARRAY_COUNT(Seq)];
			TargetYawDeg = Cur.X;
			TargetPitchDeg = Cur.Y;
		}
	}
	else if (TargetMode == ER1TargetMode::Orbit)
	{
		OrbitPhase = FMath::Fmod(OrbitPhase + DeltaTime * 0.12f, 1.f);   // one figure-8 every ~8 s
		TargetYawDeg = Reach * FMath::Sin(2.f * PI * OrbitPhase);
		TargetPitchDeg = 0.5f * Reach * FMath::Sin(4.f * PI * OrbitPhase);
	}
	else if (APlayerController* PC = GetWorld() ? GetWorld()->GetFirstPlayerController() : nullptr)
	{
		// Manual: arrow keys (left = look left), 40 deg/s; the walk tab owns the arrows only while walking
		const float Rate = 40.f * DeltaTime;
		const float DY = (PC->IsInputKeyDown(EKeys::Left) ? 1.f : 0.f) - (PC->IsInputKeyDown(EKeys::Right) ? 1.f : 0.f);
		const float DP = (PC->IsInputKeyDown(EKeys::Down) ? 1.f : 0.f) - (PC->IsInputKeyDown(EKeys::Up) ? 1.f : 0.f);
		if (DY != 0.f || DP != 0.f)
		{
			NudgeTarget(DY * Rate, DP * Rate);
		}
	}
}

void UR1SkillBrainComponent::UpdateProps()
{
	if (!Props || !Bridge)
	{
		return;
	}
	FVector PelvisPos, HeadPos, YawPos;
	FQuat PelvisQ(0, 0, 0, 1), HeadQ, YawQ;
	const bool bPelvis = Bridge->GetBodyPoseMj(Pelvis, PelvisPos, PelvisQ);
	if (!Bridge->GetBodyPoseMj(HeadPitchLink, HeadPos, HeadQ))
	{
		HeadPos = (bPelvis ? PelvisPos : FVector(0, 0, 1)) + FVector(0, 0, 0.45);
	}
	// look-at ball: direction (yaw left +, pitch down +) in the robot's frame, TargetDistM from the head
	const bool bLookActive = Skills.IsValidIndex(Active) && Skills[Active].Id == TEXT("look_at");
	const bool bShowBall = bLookActive || TargetMode != ER1TargetMode::Script;
	Props->Ball->SetVisibility(bShowBall);
	if (bShowBall)
	{
		const float Y = FMath::DegreesToRadians(TargetYawDeg), P = FMath::DegreesToRadians(TargetPitchDeg);
		const FVector DirLocal(FMath::Cos(P) * FMath::Cos(Y), FMath::Cos(P) * FMath::Sin(Y), -FMath::Sin(P));
		const FVector BallMj = HeadPos + PelvisQ.RotateVector(DirLocal) * TargetDistM;
		Props->Ball->SetWorldLocation(MjUe::PositionToUe(BallMj));
	}
	// head camera follows head_yaw_link
	if (Bridge->GetBodyPoseMj(HeadYawLink, YawPos, YawQ))
	{
		const FVector CamMj = YawPos + YawQ.RotateVector(CamPosMj);
		const FVector Fwd = MjUe::DirectionToUe(YawQ.RotateVector(CamFwdMj));
		const FVector Up = MjUe::DirectionToUe(YawQ.RotateVector(CamUpMj));
		Props->HeadCam->SetWorldLocationAndRotation(MjUe::PositionToUe(CamMj), FRotationMatrix::MakeFromXZ(Fwd, Up).Rotator());
		const double Now = FPlatformTime::Seconds();
		if (Now >= NextCapture)
		{
			Props->HeadCam->CaptureScene();
			NextCapture = Now + 1.0 / 15.0;
		}
	}
}

UTextureRenderTarget2D* UR1SkillBrainComponent::GetHeadCamTexture() const
{
	return Props && Bridge && Readout.bConnected ? Props->HeadCamRT.Get() : nullptr;
}

void UR1SkillBrainComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
	if (!EnsureBridge())
	{
		return;
	}
	Bridge->Advance(DeltaTime);   // mock only

	// a Python client (web app twin preview over URLab, walk sandbox puppet) owns the actuators: let go
	Readout.bExternal = Bridge->IsExternallyStepped();
	if (Readout.bExternal)
	{
		if (!bHadExternal)
		{
			Bridge->SetInEngineControl(false);
			Active = -1;
			TimelineIndex = -1;
			Readout.bRunning = false;
		}
		bHadExternal = true;
		UpdateProps();
		return;
	}
	if (!bSessionStarted || bHadExternal)
	{
		if (!bSessionStarted)
		{
			Bridge->StartInEngineSession();   // home keyframe, physics running
		}
		Bridge->SetInEngineControl(true);
		ApplyGains();
		HoldPose();
		bSessionStarted = true;
		bHadExternal = false;
	}
	if (!FMath::IsNearlyEqual(AppliedStrength, StrengthPct))
	{
		ApplyGains();
	}

	TickScenario(DeltaTime);   // before the target and the policy step, so a step lands on its time
	UpdateTargetDirection(DeltaTime);
	if (PushLeftS > 0.f)
	{
		FVector P;
		FQuat PelvisQ(0, 0, 0, 1);
		Bridge->GetBodyPoseMj(Pelvis, P, PelvisQ);
		Bridge->ApplyBodyWrenchMj(HeadYawLink, FVector::ZeroVector, PelvisQ.RotateVector(PushTorqueMj));
		PushLeftS -= DeltaTime;
		if (PushLeftS <= 0.f)
		{
			Bridge->ClearBodyWrench(HeadYawLink);
		}
	}

	if (Active >= 0)
	{
		// a scenario is playing: the idle demo mode must not take over the camera and hide the lab
		if (UR1ShowcaseSubsystem* Showcase = UR1ShowcaseSubsystem::Get(this))
		{
			Showcase->NotifyUserInput();
		}
		Accumulator += DeltaTime;
		int32 Steps = 0;
		while (Active >= 0 && Accumulator >= R1Skills::PolicyDt && Steps < 4)
		{
			PolicyStep();
			Accumulator -= R1Skills::PolicyDt;
			++Steps;
		}
		if (Accumulator > 4.f * R1Skills::PolicyDt)
		{
			Accumulator = 0.f;   // hitch: drop the backlog
		}
	}
	StepMotorModel(DeltaTime);
	UpdateProps();
	TickRecording(DeltaTime);
}

// =====================================================================================================
// simulation features (docs/sim_features.md): motor model, scenario scripts, dataset recording
// =====================================================================================================

void UR1SkillBrainComponent::SetTarget(FName Actuator, float TargetRad)
{
	Bridge->SetActuatorTarget(Actuator, TargetRad);
	if (const int32* Slot = ActuatorSlot.Find(Actuator))
	{
		MotorTarget[*Slot] = TargetRad;
	}
}

void UR1SkillBrainComponent::SetGains(FName Actuator, float Kp, float Kd)
{
	Bridge->SetActuatorGains(Actuator, Kp, Kd);
	if (const int32* Slot = ActuatorSlot.Find(Actuator))
	{
		MotorKp[*Slot] = Kp;
		MotorKd[*Slot] = Kd;
	}
}

void UR1SkillBrainComponent::StepMotorModel(float DeltaTime)
{
	// the lab's torque source is the PD estimate kp*(target-q) - kd*dq, clipped to effort (display only)
	MotorAccumulator += DeltaTime;
	if (MotorAccumulator < R1Skills::PolicyDt || !Bridge || Motor.Num() != BridgeJoints.Num())
	{
		return;
	}
	const float Dt = FMath::Min(MotorAccumulator, 0.1f);
	MotorAccumulator = 0.f;
	for (int32 i = 0; i < BridgeJoints.Num(); ++i)
	{
		const float Qi = static_cast<float>(Bridge->GetJointPos(BridgeJoints[i]));
		MotorDq[i] = static_cast<float>(Bridge->GetJointVel(BridgeJoints[i]));
		MotorTau[i] = FMath::Clamp(MotorKp[i] * (MotorTarget[i] - Qi) - MotorKd[i] * MotorDq[i], -MotorEffort[i], MotorEffort[i]);
	}
	Motor.Step(Dt, MotorTau, MotorDq);
}

bool UR1SkillBrainComponent::GetBodyPoseUe(FName Body, FVector& OutPosUe, FQuat& OutRotUe) const
{
	FVector PosMj;
	FQuat RotMj;
	if (!Bridge || !Readout.bConnected || !Bridge->GetBodyPoseMj(Body, PosMj, RotMj))
	{
		return false;
	}
	OutPosUe = MjUe::PositionToUe(PosMj);
	OutRotUe = MjUe::QuatToUe(RotMj);
	return true;
}

bool UR1SkillBrainComponent::PlayScenario(const FR1LabScenario& In, FString& OutWhy)
{
	const int32 Idx = FindSkill(In.Skill);
	if (Idx == INDEX_NONE)
	{
		OutWhy = FString::Printf(TEXT("the scenario needs the skill %s, which is not trained here"), *In.Skill);
		return false;
	}
	EndScenario();
	SavedDelay = DelaySteps;
	SavedNoise = NoiseDeg;
	SavedStrength = StrengthPct;
	SavedTargetMode = TargetMode;
	if (!Play(Idx, In.SizeDeg, In.DurationS, OutWhy))
	{
		return false;
	}
	Scenario = In;
	ScenarioNext = 0;
	ScenarioClock = 0.f;
	NoiseRng.Initialize(In.Seed);
	Readout.ScenarioName = In.Name;
	Readout.ScenarioDurationS = In.DurationS;
	Readout.ScenarioSteps = In.Steps.Num();
	Readout.ScenarioStep = 0;
	Readout.ScenarioT = 0.f;
	Readout.ScenarioLast.Reset();
	UE_LOG(LogR1Twin, Log, TEXT("R1SkillBrain: scenario %s (%s, %d steps)"), *In.Id, *In.Skill, In.Steps.Num());
	return true;
}

void UR1SkillBrainComponent::EndScenario()
{
	if (!Scenario.IsSet())
	{
		return;
	}
	Scenario.Reset();
	DelaySteps = SavedDelay;
	NoiseDeg = SavedNoise;
	StrengthPct = SavedStrength;
	SetTargetMode(SavedTargetMode);
	Readout.ScenarioName.Reset();
	Readout.ScenarioSteps = 0;
}

void UR1SkillBrainComponent::TickScenario(float DeltaTime)
{
	if (!Scenario.IsSet())
	{
		return;
	}
	if (Active < 0)
	{
		EndScenario();   // the skill finished (or was stopped): the script ends with it
		return;
	}
	ScenarioClock += DeltaTime;
	const TArray<FR1LabScenarioStep>& Steps = Scenario->Steps;
	while (Steps.IsValidIndex(ScenarioNext) && Steps[ScenarioNext].T <= ScenarioClock)
	{
		const FR1LabScenarioStep& St = Steps[ScenarioNext++];
		if (St.Do == TEXT("target"))
		{
			SetTargetMode(ER1TargetMode::Manual);
			TargetYawDeg = FMath::Clamp(St.YawDeg, -60.f, 60.f);
			TargetPitchDeg = FMath::Clamp(St.PitchDeg, -35.f, 35.f);
			Readout.ScenarioLast = FString::Printf(TEXT("target yaw %+.0f pitch %+.0f deg"), St.YawDeg, St.PitchDeg);
		}
		else if (St.Do == TEXT("push"))
		{
			Push(FVector2D(St.Yaw, St.Pitch));
			Readout.ScenarioLast = FString::Printf(TEXT("push (yaw %+.0f, pitch %+.0f)"), St.Yaw, St.Pitch);
		}
		else if (St.Do == TEXT("noise"))
		{
			NoiseDeg = FMath::Clamp(St.Deg, 0.f, 10.f);
			Readout.ScenarioLast = FString::Printf(TEXT("sensor noise %g deg"), NoiseDeg);
		}
		else if (St.Do == TEXT("delay"))
		{
			DelaySteps = FMath::Clamp(St.Steps, 0, 4);
			Readout.ScenarioLast = FString::Printf(TEXT("action delay %d ms"), DelaySteps * 20);
		}
		else if (St.Do == TEXT("strength"))
		{
			StrengthPct = FMath::Clamp(St.Pct, 30.f, 100.f);
			Readout.ScenarioLast = FString::Printf(TEXT("motor strength %.0f%%"), StrengthPct);
		}
		else
		{
			Readout.ScenarioLast = FString::Printf(TEXT("unknown step '%s' skipped"), *St.Do);
		}
	}
	Readout.ScenarioT = ScenarioClock;
	Readout.ScenarioStep = ScenarioNext;
}

bool UR1SkillBrainComponent::StartRecording(const FString& Dir, int32 Frames, FString& OutWhy)
{
	if (!GetHeadCamTexture())
	{
		OutWhy = TEXT("no head camera (needs the URLab articulation in the level)");
		return false;
	}
	if (!IFileManager::Get().MakeDirectory(*FPaths::Combine(Dir, TEXT("rgb")), /*Tree*/ true))
	{
		OutWhy = FString::Printf(TEXT("cannot create %s"), *Dir);
		return false;
	}
	Recording = FR1DatasetRecording();
	Recording.bActive = true;
	Recording.Dir = Dir;
	Recording.Wanted = FMath::Clamp(Frames, 1, 2000);
	UE_LOG(LogR1Twin, Log, TEXT("R1SkillBrain: recording %d head-camera frames to %s"), Recording.Wanted, *Dir);
	return true;
}

void UR1SkillBrainComponent::StopRecording()
{
	if (!Recording.bActive)
	{
		return;
	}
	Recording.bActive = false;
	UTextureRenderTarget2D* RT = Props ? Props->HeadCamRT.Get() : nullptr;
	const FString Json = FString::Printf(
		TEXT("{\n \"source\": \"unreal_head_cam\",\n \"simulation_only\": true,\n \"camera\": \"head_rgbd\",\n \"width\": %d,\n \"height\": %d,\n")
		TEXT(" \"hfov_deg\": %.1f,\n \"ball_radius_m\": 0.06,\n \"frames\": [\n%s\n ]\n}\n"),
		RT ? RT->SizeX : 0, RT ? RT->SizeY : 0, CamHFovDeg, *Recording.LabelsJson);
	FFileHelper::SaveStringToFile(Json, *FPaths::Combine(Recording.Dir, TEXT("labels.json")));
	UE_LOG(LogR1Twin, Log, TEXT("R1SkillBrain: dataset done, %d frame(s) in %s"), Recording.Frames, *Recording.Dir);
}

void UR1SkillBrainComponent::TickRecording(float DeltaTime)
{
	if (!Recording.bActive)
	{
		return;
	}
	Recording.NextS -= DeltaTime;
	if (Recording.NextS > 0.f)
	{
		return;
	}
	Recording.NextS = Recording.IntervalS;
	UTextureRenderTarget2D* RT = GetHeadCamTexture();
	FTextureRenderTargetResource* Res = RT ? RT->GameThread_GetRenderTargetResource() : nullptr;
	if (!Res)
	{
		Recording.LastError = TEXT("the head camera went away");
		StopRecording();
		return;
	}
	Props->HeadCam->CaptureScene();   // this frame's view, so the label matches the picture
	TArray<FColor> Pixels;
	if (!Res->ReadPixels(Pixels) || Pixels.Num() != RT->SizeX * RT->SizeY)
	{
		Recording.LastError = TEXT("could not read the head camera");
		StopRecording();
		return;
	}
	for (FColor& C : Pixels)
	{
		C.A = 255;   // scene captures leave alpha empty
	}
	TArray64<uint8> Png;
	FImageUtils::PNGCompressImageArray(RT->SizeX, RT->SizeY, TArrayView64<const FColor>(Pixels.GetData(), Pixels.Num()), Png);
	const FString Rel = FString::Printf(TEXT("rgb/%05d.png"), Recording.Frames);
	if (!FFileHelper::SaveArrayToFile(Png, *FPaths::Combine(Recording.Dir, Rel)))
	{
		Recording.LastError = TEXT("could not write the PNG");
		StopRecording();
		return;
	}

	// label: the ball's box in the picture (pinhole projection of a 6 cm sphere) and the head angles
	const float W = static_cast<float>(RT->SizeX), H = static_cast<float>(RT->SizeY);
	const float Focal = 0.5f * W / FMath::Tan(FMath::DegreesToRadians(0.5f * Props->HeadCam->FOVAngle));
	const FVector Local = Props->HeadCam->GetComponentTransform().InverseTransformPosition(Props->Ball->GetComponentLocation());
	bool bVisible = Props->Ball->IsVisible() && Local.X > 5.0;
	float X0 = 0.f, Y0 = 0.f, X1 = 0.f, Y1 = 0.f, U = 0.f, V = 0.f;
	if (bVisible)
	{
		const float R = Focal * 6.f / static_cast<float>(Local.X);
		U = 0.5f * W + Focal * static_cast<float>(Local.Y / Local.X);
		V = 0.5f * H - Focal * static_cast<float>(Local.Z / Local.X);
		X0 = FMath::Clamp(U - R, 0.f, W);
		X1 = FMath::Clamp(U + R, 0.f, W);
		Y0 = FMath::Clamp(V - R, 0.f, H);
		Y1 = FMath::Clamp(V + R, 0.f, H);
		bVisible = X1 > X0 && Y1 > Y0;
	}
	const float YawDeg = FMath::RadiansToDegrees(static_cast<float>(Bridge->GetJointPos(R1Skills::HeadYaw)));
	const float PitchDeg = FMath::RadiansToDegrees(static_cast<float>(Bridge->GetJointPos(R1Skills::HeadPitch)));
	const FString Ball = bVisible
		? FString::Printf(TEXT("{\"class\": \"ball\", \"visible\": true, \"bbox\": [%.1f, %.1f, %.1f, %.1f], \"center\": [%.1f, %.1f], \"dist_m\": %.3f, \"yaw_deg\": %.2f, \"pitch_deg\": %.2f}"),
			X0, Y0, X1, Y1, U, V, Local.X / 100.0, TargetYawDeg, TargetPitchDeg)
		: FString(TEXT("{\"class\": \"ball\", \"visible\": false}"));
	Recording.LabelsJson += FString::Printf(TEXT("%s  {\"frame\": %d, \"rgb\": \"%s\", \"head_yaw_deg\": %.2f, \"head_pitch_deg\": %.2f, \"skill\": \"%s\", \"objects\": [%s]}"),
		Recording.Frames > 0 ? TEXT(",\n") : TEXT(""), Recording.Frames, *Rel, YawDeg, PitchDeg,
		Skills.IsValidIndex(Active) ? *Skills[Active].Id : TEXT(""), *Ball);
	if (++Recording.Frames >= Recording.Wanted)
	{
		StopRecording();
	}
}

void UR1SkillBrainComponent::PolicyStep()
{
	const FR1SkillDef& S = Skills[Active];
	const FModel* M = Models.Find(Active);
	if (!M || !M->Instance.IsValid())
	{
		Stop();
		return;
	}
	const int32 K = S.NumJoints();
	for (int32 i = 0; i < K; ++i)
	{
		Q[i] = static_cast<float>(Bridge->GetJointPos(S.Joints[i]));
		Dq[i] = static_cast<float>(Bridge->GetJointVel(S.Joints[i]));
	}
	float Other = 0.f;
	for (const TPair<FName, float>& H : HoldStart)
	{
		Other = FMath::Max(Other, FMath::Abs(static_cast<float>(Bridge->GetJointPos(H.Key)) - H.Value));
	}
	Readout.OtherMaxDeg = FMath::Max(Readout.OtherMaxDeg, FMath::RadiansToDegrees(Other));

	// observation: UNITE 131-dim role-ordered or legacy [q - default (+ sensor noise), dq, last_action, command]
	float Cmd[R1Skills::NumCmd];
	FillCommand(S, T, Cmd);
	const float Noise = FMath::DegreesToRadians(NoiseDeg);
	if (S.bUnitePortable)
	{
		FMemory::Memzero(Obs.GetData(), Obs.Num() * sizeof(float));
		Obs[8] = -1.f;   // projected_gravity = [0, 0, -1] upright on stand
		for (int32 i = 0; i < K; ++i)
		{
			const int32 R = S.RoleIndices[i];
			Obs[9 + R] = Q[i] - S.Default[i] + (Noise > 0.f ? NoiseRng.FRandRange(-Noise, Noise) : 0.f);
			Obs[37 + R] = Dq[i];
			Obs[65 + R] = LastAction[i];
		}
		for (int32 j = 0; j < R1Skills::UniteNumAct; ++j)
		{
			Obs[93 + j] = (j != 13 && j != 27) ? 1.f : 0.f;   // R1 A5 role_mask
		}
		for (int32 c = 0; c < R1Skills::NumCmd; ++c)
		{
			Obs[121 + c] = Cmd[c];
		}
	}
	else
	{
		for (int32 i = 0; i < K; ++i)
		{
			Obs[i] = Q[i] - S.Default[i] + (Noise > 0.f ? NoiseRng.FRandRange(-Noise, Noise) : 0.f);
			Obs[K + i] = Dq[i];
			Obs[2 * K + i] = LastAction[i];
		}
		for (int32 c = 0; c < R1Skills::NumCmd; ++c)
		{
			Obs[3 * K + c] = Cmd[c];
		}
	}

	// bindings are views onto the preallocated buffers: no heap work in the policy loop
	const UE::NNE::FTensorBindingCPU In{Obs.GetData(), static_cast<uint64>(Obs.Num()) * sizeof(float)};
	const UE::NNE::FTensorBindingCPU Out{Action.GetData(), static_cast<uint64>(Action.Num()) * sizeof(float)};
	const double T0 = FPlatformTime::Seconds();
	const bool bOk = M->Instance->RunSync(MakeArrayView(&In, 1), MakeArrayView(&Out, 1)) == UE::NNE::IModelInstanceRunSync::ERunSyncStatus::Ok;
	Readout.InferMs = static_cast<float>((FPlatformTime::Seconds() - T0) * 1000.0);
	// disturbance: extra action delay (a ring of DelaySteps + 1 preallocated target vectors)
	DelayHead = (DelayHead + 1) % DelayQueue.Num();
	TArray<float>& Tgt = DelayQueue[DelayHead];
	for (int32 i = 0; i < K; ++i)
	{
		const float RawAct = S.bUnitePortable ? Action[S.RoleIndices[i]] : Action[i];
		if (!bOk || !FMath::IsFinite(RawAct))
		{
			Readout.RobotWouldStop = TEXT("policy produced a non-finite action");
			Stop();
			return;
		}
		const float A = FMath::Clamp(RawAct, -R1Skills::ActionClip, R1Skills::ActionClip);
		LastAction[i] = A;
		Tgt[i] = FMath::Clamp(S.Default[i] + S.ActionScale[i] * A, S.SoftLo[i], S.SoftHi[i]);
	}
	ActionView.Push(LastAction.GetData(), K);   // #15: shown only; the brain still applies the raw action
	const int32 Lag = FMath::Clamp(DelaySteps, 0, DelayQueue.Num() - 1);
	const TArray<float>& Applied = DelayQueue[(DelayHead - Lag + DelayQueue.Num()) % DelayQueue.Num()];
	// the runtime's torque cap: kp * |target - q| <= effort, then the soft limits
	for (int32 i = 0; i < K; ++i)
	{
		const float Band = S.Effort[i] / FMath::Max(S.Kp[i], 1e-6f);
		Target[i] = FMath::Clamp(FMath::Clamp(Applied[i], Q[i] - Band, Q[i] + Band), S.SoftLo[i], S.SoftHi[i]);
		SetTarget(S.Actuators[i], Target[i]);
	}

	// reference + tracking, as the robot's safety envelope sees it
	const float Phase = S.PeriodS > 0.f ? FMath::Fmod(T / S.PeriodS, 1.f) : -1.f;
	PrevRef = Ref;   // same size every step: no reallocation
	R1Skills::Reference(S, Cmd, Phase, Ref);
	const bool bSettle = T >= RunS;
	float ErrNow = 0.f, ErrWorst = 0.f, RefJump = 0.f;
	for (int32 i = 0; i < K; ++i)
	{
		const float E = FMath::RadiansToDegrees(FMath::Abs(Q[i] - Ref[i]));
		ErrNow += E / K;
		ErrWorst = FMath::Max(ErrWorst, E);
		RefJump = FMath::Max(RefJump, FMath::RadiansToDegrees(FMath::Abs(Ref[i] - (PrevRef.IsValidIndex(i) ? PrevRef[i] : Ref[i]))));
	}
	if (RefJump > 1.f || T < 1e-3f)
	{
		LastJumpT = T;   // Look At target jumped: score only once settled (0.5 s), like the validation gate
	}
	if (!bSettle && T >= 0.5f && T - LastJumpT >= 0.5f)
	{
		ErrSum += ErrNow;
		++ErrCount;
		Readout.ErrMeanDeg = ErrSum / ErrCount;
		Readout.ErrMaxDeg = FMath::Max(Readout.ErrMaxDeg, ErrNow);
	}
	const bool bSteps = S.Id == TEXT("look_at") || S.bPose;   // a taught pose starts with a step away from centre
	const float Thr = bSteps ? 15.f : 8.f, Window = bSteps ? 1.f : 0.4f;
	if (ErrWorst > Thr)
	{
		ErrSince = ErrSince < 0.0 ? T : ErrSince;
		if (T - ErrSince > Window && Readout.RobotWouldStop.IsEmpty())
		{
			Readout.RobotWouldStop = FString::Printf(TEXT("would STOP: tracking error %.0f deg for > %.1f s"), ErrWorst, Window);
		}
	}
	else
	{
		ErrSince = -1.0;
	}
	if (Readout.OtherMaxDeg > 5.f && Readout.RobotWouldStop.IsEmpty())
	{
		Readout.RobotWouldStop = FString::Printf(TEXT("would STOP: another joint moved %.1f deg (> 5)"), Readout.OtherMaxDeg);
	}

	Readout.bRunning = true;
	Readout.bSettling = bSettle;
	Readout.T = T;
	Readout.DurationS = RunS;
	Readout.TimelineStep = TimelineIndex;
	Readout.ErrNowDeg = ErrNow;
	Readout.QDeg.SetNum(K);
	Readout.RefDeg.SetNum(K);
	Readout.TargetDeg.SetNum(K);
	for (int32 i = 0; i < K; ++i)
	{
		Readout.QDeg[i] = FMath::RadiansToDegrees(Q[i]);
		Readout.RefDeg[i] = FMath::RadiansToDegrees(Ref[i]);
		Readout.TargetDeg[i] = FMath::RadiansToDegrees(Target[i]);
	}

	T += R1Skills::PolicyDt;
	if (T >= RunS + 1.f)
	{
		FinishActive();
	}
}
