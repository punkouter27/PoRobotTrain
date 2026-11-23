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
#include "HAL/PlatformTime.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Misc/FileHelper.h"

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
	Action.SetNumZeroed(K);
	PrevRef.SetNumZeroed(K);
	DelayQueue.SetNum(5);   // up to 4 steps (80 ms) of extra delay
	for (TArray<float>& D : DelayQueue)
	{
		D = S.Default;
	}
	DelayHead = 0;
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
	PushLeftS = 0.2f;
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
			Bridge->SetActuatorGains(S.HoldActuators[i], S.HoldKp[i] * (bHead ? Scale : 1.f), S.HoldKd[i] * (bHead ? Scale : 1.f));
		}
		for (int32 i = 0; i < S.NumJoints(); ++i)
		{
			Bridge->SetActuatorGains(S.Actuators[i], S.Kp[i] * Scale, S.Kd[i] * Scale);
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
			Bridge->SetActuatorTarget(S.HoldActuators[i], S.HoldDefault[i]);
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
	UpdateProps();
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

	// observation: [q - default (+ sensor noise), dq, last_action, command]
	float Cmd[R1Skills::NumCmd];
	FillCommand(S, T, Cmd);
	const float Noise = FMath::DegreesToRadians(NoiseDeg);
	for (int32 i = 0; i < K; ++i)
	{
		Obs[i] = Q[i] - S.Default[i] + (Noise > 0.f ? FMath::FRandRange(-Noise, Noise) : 0.f);
		Obs[K + i] = Dq[i];
		Obs[2 * K + i] = LastAction[i];
	}
	for (int32 c = 0; c < R1Skills::NumCmd; ++c)
	{
		Obs[3 * K + c] = Cmd[c];
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
		if (!bOk || !FMath::IsFinite(Action[i]))
		{
			Readout.RobotWouldStop = TEXT("policy produced a non-finite action");
			Stop();
			return;
		}
		const float A = FMath::Clamp(Action[i], -R1Skills::ActionClip, R1Skills::ActionClip);
		LastAction[i] = A;
		Tgt[i] = FMath::Clamp(S.Default[i] + S.ActionScale[i] * A, S.SoftLo[i], S.SoftHi[i]);
	}
	const int32 Lag = FMath::Clamp(DelaySteps, 0, DelayQueue.Num() - 1);
	const TArray<float>& Applied = DelayQueue[(DelayHead - Lag + DelayQueue.Num()) % DelayQueue.Num()];
	// the runtime's torque cap: kp * |target - q| <= effort, then the soft limits
	for (int32 i = 0; i < K; ++i)
	{
		const float Band = S.Effort[i] / FMath::Max(S.Kp[i], 1e-6f);
		Target[i] = FMath::Clamp(FMath::Clamp(Applied[i], Q[i] - Band, Q[i] + Band), S.SoftLo[i], S.SoftHi[i]);
		Bridge->SetActuatorTarget(S.Actuators[i], Target[i]);
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
	const bool bSteps = S.Id == TEXT("look_at");
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
