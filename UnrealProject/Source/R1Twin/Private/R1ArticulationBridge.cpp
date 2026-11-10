// Written for UE 5.7; NOT YET COMPILED (see docs/ue5_setup.md).
#include "R1ArticulationBridge.h"
#include "R1Twin.h"
#include "GameFramework/Actor.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "UObject/UObjectGlobals.h"

#include "MjUeConvert.h"
#if R1_WITH_URLAB && R1_URLAB_API_VERIFIED
#include "MuJoCo/Core/MjArticulation.h"
#include "MuJoCo/Components/Joints/MjJoint.h"
#include "MuJoCo/Components/Actuators/MjActuator.h"
#include "MuJoCo/Components/Bodies/MjBody.h"
#include "MuJoCo/Components/Sensors/MjSensor.h"
#include "MuJoCo/Core/AMjManager.h"
#endif

// =====================================================================================================
// FR1MockBridge
// =====================================================================================================

bool FR1MockBridge::Initialize(const FR1BridgeInitParams& Params)
{
	const int32 N = Params.JointNames.Num();
	if (N == 0 || Params.ActuatorNames.Num() != N)
	{
		UE_LOG(LogR1Twin, Error, TEXT("FR1MockBridge: joint/actuator name arrays empty or mismatched (%d vs %d)"),
			N, Params.ActuatorNames.Num());
		return false;
	}

	RootBodyName = Params.RootBodyName;
	JointIndex.Empty(N);
	ActuatorIndex.Empty(N);
	Q.SetNumZeroed(N);
	V.SetNumZeroed(N);
	Target.SetNumZeroed(N);
	Kp.Init(20.0, N);
	Kd.Init(1.0, N);

	for (int32 i = 0; i < N; ++i)
	{
		JointIndex.Add(Params.JointNames[i], i);
		ActuatorIndex.Add(Params.ActuatorNames[i], i);
		if (Params.InitialJointPos.IsValidIndex(i))
		{
			Q[i] = Params.InitialJointPos[i];
			Target[i] = Params.InitialJointPos[i];
		}
	}

	bInitialized = true;
	UE_LOG(LogR1Twin, Warning, TEXT("FR1MockBridge active with %d joints - the robot you see (if any) is NOT being driven."), N);
	return true;
}

double FR1MockBridge::GetJointPos(FName JointName) const
{
	const int32* Idx = JointIndex.Find(JointName);
	return Idx ? Q[*Idx] : 0.0;
}

double FR1MockBridge::GetJointVel(FName JointName) const
{
	const int32* Idx = JointIndex.Find(JointName);
	return Idx ? V[*Idx] : 0.0;
}

void FR1MockBridge::SetActuatorTarget(FName ActuatorName, double TargetRad)
{
	if (const int32* Idx = ActuatorIndex.Find(ActuatorName))
	{
		Target[*Idx] = TargetRad;
	}
}

void FR1MockBridge::SetActuatorGains(FName ActuatorName, double InKp, double InKd)
{
	if (const int32* Idx = ActuatorIndex.Find(ActuatorName))
	{
		Kp[*Idx] = InKp;
		Kd[*Idx] = InKd;
	}
}

FQuat FR1MockBridge::GetImuQuat() const
{
	// Raw MuJoCo identity (w=1, x=y=z=0) packed as FQuat(X=x, Y=y, Z=z, W=w).
	return FQuat(0.0, 0.0, 0.0, 1.0);
}

FVector FR1MockBridge::GetImuGyro() const
{
	return FVector::ZeroVector;
}

bool FR1MockBridge::GetBodyPoseMj(FName BodyName, FVector& OutPosMeters, FQuat& OutQuatRaw) const
{
	if (BodyName != RootBodyName)
	{
		return false;
	}
	OutPosMeters = RootPosMeters;
	OutQuatRaw = FQuat(0.0, 0.0, 0.0, 1.0);
	return true;
}

void FR1MockBridge::Advance(float DeltaSeconds)
{
	if (!bInitialized || DeltaSeconds <= 0.f)
	{
		return;
	}
	// First-order response whose speed scales with sqrt(kp) - good enough to make the
	// joint_pos_rel / joint_vel observations non-trivial while the real bridge is unverified.
	const double Dt = static_cast<double>(DeltaSeconds);
	for (int32 i = 0; i < Q.Num(); ++i)
	{
		const double Rate = FMath::Clamp(FMath::Sqrt(FMath::Max(Kp[i], 1.0)), 1.0, 30.0);
		const double Alpha = FMath::Clamp(Rate * Dt, 0.0, 1.0);
		const double Prev = Q[i];
		Q[i] = Prev + (Target[i] - Prev) * Alpha;
		V[i] = (Q[i] - Prev) / Dt;
	}
}

// =====================================================================================================
// FR1UrlabBridge
// =====================================================================================================

bool FR1UrlabBridge::Initialize(const FR1BridgeInitParams& Params)
{
	ArticulationActor = Params.ArticulationActor;
	ImuSiteName = Params.ImuSiteName;
	RootBodyName = Params.RootBodyName;
	bConnected = false;

	if (!ArticulationActor.IsValid())
	{
		UE_LOG(LogR1Twin, Warning, TEXT("FR1UrlabBridge: no articulation actor supplied."));
		return false;
	}

#if R1_WITH_URLAB && R1_URLAB_API_VERIFIED
	AMjArticulation* Art = Cast<AMjArticulation>(ArticulationActor.Get());
	if (!Art)
	{
		UE_LOG(LogR1Twin, Error, TEXT("FR1UrlabBridge: actor %s is not an AMjArticulation."), *ArticulationActor->GetName());
		return false;
	}

	// URLab keys components by UE component name; imported names may carry an actor prefix, so fall back
	// to an exact or suffix match on the MuJoCo name. Resolution happens once, never per tick.
	auto Matches = [](const FString& MjName, const FString& Want)
	{
		return MjName == Want || MjName.EndsWith(FString(TEXT("_")) + Want);
	};
	auto ResolveJoint = [Art, &Matches](const FString& Name) -> UMjJoint*
	{
		if (UMjJoint* J = Art->GetJoint(Name)) return J;
		for (UMjJoint* J : Art->GetJoints()) { if (J && Matches(J->GetMjName(), Name)) return J; }
		return nullptr;
	};
	auto ResolveActuator = [Art, &Matches](const FString& Name) -> UMjActuator*
	{
		if (UMjActuator* A = Art->GetActuator(Name)) return A;
		for (UMjActuator* A : Art->GetActuators()) { if (A && Matches(A->GetMjName(), Name)) return A; }
		return nullptr;
	};
	auto ResolveBody = [Art, &Matches](const FString& Name) -> UMjBody*
	{
		if (UMjBody* B = Art->GetBody(Name)) return B;
		for (UMjBody* B : Art->GetBodies()) { if (B && Matches(B->GetMjName(), Name)) return B; }
		return nullptr;
	};

	Joints.Empty(Params.JointNames.Num());
	for (const FName& Joint : Params.JointNames)
	{
		UMjJoint* J = ResolveJoint(Joint.ToString());
		if (!J)
		{
			UE_LOG(LogR1Twin, Error, TEXT("FR1UrlabBridge: joint '%s' not found. Available: %s"),
				*Joint.ToString(), *FString::Join(Art->GetJointNames(), TEXT(", ")));
			return false;
		}
		Joints.Add(Joint, J);
	}
	Actuators.Empty(Params.ActuatorNames.Num());
	for (const FName& Act : Params.ActuatorNames)
	{
		UMjActuator* A = ResolveActuator(Act.ToString());
		if (!A)
		{
			UE_LOG(LogR1Twin, Error, TEXT("FR1UrlabBridge: actuator '%s' not found. Available: %s"),
				*Act.ToString(), *FString::Join(Art->GetActuatorNames(), TEXT(", ")));
			return false;
		}
		Actuators.Add(Act, A);
	}
	RootBody = ResolveBody(RootBodyName.IsNone() ? FString(TEXT("pelvis")) : RootBodyName.ToString());
	if (!RootBody.IsValid())
	{
		UE_LOG(LogR1Twin, Warning, TEXT("FR1UrlabBridge: root body '%s' not found; IMU quaternion falls back to identity."),
			*RootBodyName.ToString());
	}
	for (UMjSensor* S : Art->GetSensors())
	{
		if (S && Matches(S->GetMjName(), TEXT("imu_ang_vel"))) { GyroSensor = S; break; }
	}

	bConnected = true;
	UE_LOG(LogR1Twin, Log, TEXT("FR1UrlabBridge connected to %s (%d joints, %d actuators, root body %s, gyro %s)."),
		*Art->GetName(), Joints.Num(), Actuators.Num(), RootBody.IsValid() ? TEXT("ok") : TEXT("missing"),
		GyroSensor.IsValid() ? TEXT("ok") : TEXT("missing"));
	return true;
#else
	UE_LOG(LogR1Twin, Warning,
		TEXT("FR1UrlabBridge: articulation actor '%s' found but URLab bindings are compiled out (R1_WITH_URLAB=%d, R1_URLAB_API_VERIFIED=%d)."),
		*ArticulationActor->GetName(), static_cast<int32>(R1_WITH_URLAB), static_cast<int32>(R1_URLAB_API_VERIFIED));
	return false;
#endif
}

FString FR1UrlabBridge::GetDescription() const
{
	return FString::Printf(TEXT("FR1UrlabBridge -> %s (%s)"),
		ArticulationActor.IsValid() ? *ArticulationActor->GetName() : TEXT("<none>"),
		bConnected ? TEXT("connected") : TEXT("NOT connected"));
}

double FR1UrlabBridge::GetJointPos(FName JointName) const
{
#if R1_WITH_URLAB && R1_URLAB_API_VERIFIED
	if (const TWeakObjectPtr<UObject>* P = Joints.Find(JointName))
	{
		if (const UMjJoint* J = static_cast<const UMjJoint*>(P->Get())) return J->GetPosition();
	}
#endif
	return 0.0;
}

double FR1UrlabBridge::GetJointVel(FName JointName) const
{
#if R1_WITH_URLAB && R1_URLAB_API_VERIFIED
	if (const TWeakObjectPtr<UObject>* P = Joints.Find(JointName))
	{
		if (const UMjJoint* J = static_cast<const UMjJoint*>(P->Get())) return J->GetVelocity();
	}
#endif
	return 0.0;
}

void FR1UrlabBridge::SetActuatorTarget(FName ActuatorName, double TargetRad)
{
#if R1_WITH_URLAB && R1_URLAB_API_VERIFIED
	// The R1 MJCF uses <position> actuators, so ctrl IS the joint position target (raw control mode).
	if (const TWeakObjectPtr<UObject>* P = Actuators.Find(ActuatorName))
	{
		if (UMjActuator* A = static_cast<UMjActuator*>(P->Get())) A->SetControl(static_cast<float>(TargetRad));
	}
#endif
}

void FR1UrlabBridge::SetActuatorGains(FName ActuatorName, double Kp, double Kd)
{
	// Raw control mode: gains are the MJCF <position kp kv> values generated from r1_joint_map.yaml
	// (the stand scene bakes passive legs kp 0 / kv 3). They already equal the deploy.yaml gains.
	if (!bWarnedGains)
	{
		bWarnedGains = true;
		UE_LOG(LogR1Twin, Log, TEXT("FR1UrlabBridge: gains are fixed by the MJCF (raw control); SetActuatorGains is informational."));
	}
}

FQuat FR1UrlabBridge::GetImuQuat() const
{
#if R1_WITH_URLAB && R1_URLAB_API_VERIFIED
	// The IMU site sits at the pelvis origin with identity orientation, so its orientation == pelvis xquat.
	if (const UMjBody* B = static_cast<const UMjBody*>(RootBody.Get()))
	{
		return MjUe::QuatToMj(B->GetWorldRotation());
	}
#endif
	return FQuat(0.0, 0.0, 0.0, 1.0);
}

FVector FR1UrlabBridge::GetImuGyro() const
{
#if R1_WITH_URLAB && R1_URLAB_API_VERIFIED
	// NOTE: UMjSensor::GetReading returns a TArray (one small allocation). Only observation layouts with
	// base_ang_vel (Level 2+) call this; the Level 0 layout does not.
	if (const UMjSensor* S = static_cast<const UMjSensor*>(GyroSensor.Get()))
	{
		const TArray<float> R = S->GetReading();
		if (R.Num() == 3) return FVector(R[0], R[1], R[2]);
	}
#endif
	return FVector::ZeroVector;
}

bool FR1UrlabBridge::GetBodyPoseMj(FName BodyName, FVector& OutPosMeters, FQuat& OutQuatRaw) const
{
#if R1_WITH_URLAB && R1_URLAB_API_VERIFIED
	// Only the root body is cached (camera tracking and IMU are the consumers).
	if (const UMjBody* B = static_cast<const UMjBody*>(RootBody.Get()))
	{
		OutPosMeters = MjUe::PositionToMj(B->GetWorldPosition());
		OutQuatRaw = MjUe::QuatToMj(B->GetWorldRotation());
		return true;
	}
#endif
	return false;
}

bool FR1UrlabBridge::IsExternallyStepped() const
{
#if R1_WITH_URLAB && R1_URLAB_API_VERIFIED
	if (const AAMjManager* M = AAMjManager::GetManager())
	{
		return M->EffectiveStepMode.load() != EStepMode::Live;
	}
#endif
	return false;
}

void FR1UrlabBridge::SetInEngineControl(bool bClaim)
{
#if R1_WITH_URLAB && R1_URLAB_API_VERIFIED
	const int32 Want = bClaim ? 1 : 0;
	if (Want == ClaimedState) return;
	if (AMjArticulation* Art = Cast<AMjArticulation>(ArticulationActor.Get()))
	{
		Art->ControlSource = static_cast<uint8>(Want); // 0 = ZMQ (network / Python), 1 = UI (in-engine)
		ClaimedState = Want;
		UE_LOG(LogR1Twin, Log, TEXT("FR1UrlabBridge: actuators %s"), bClaim ? TEXT("claimed by in-engine policy") : TEXT("released to external client"));
	}
#endif
}

void FR1UrlabBridge::StartInEngineSession()
{
#if R1_WITH_URLAB && R1_URLAB_API_VERIFIED
	AMjArticulation* Art = Cast<AMjArticulation>(ArticulationActor.Get());
	if (!Art) return;
	FString Home;
	for (const FString& K : Art->GetKeyframeNames())
	{
		if (K == TEXT("home") || K.EndsWith(TEXT("_home"))) { Home = K; break; }
	}
	const bool bReset = !Home.IsEmpty() && Art->ResetToKeyframe(Home);
	if (AAMjManager* M = AAMjManager::GetManager()) { M->SetPaused(false); }
	UE_LOG(LogR1Twin, Log, TEXT("FR1UrlabBridge: in-engine session started (reset to '%s': %s, physics unpaused)"),
		*Home, bReset ? TEXT("ok") : TEXT("failed"));
#endif
}

// =====================================================================================================
// Helpers
// =====================================================================================================

namespace R1Urlab
{
	AActor* FindArticulationActor(UWorld* World, FName PreferredTag)
	{
		if (!World)
		{
			return nullptr;
		}

		// 1) Explicit tag wins (set "R1Articulation" on the placed Articulation Blueprint instance).
		if (!PreferredTag.IsNone())
		{
			for (TActorIterator<AActor> It(World); It; ++It)
			{
				if (It->ActorHasTag(PreferredTag))
				{
					return *It;
				}
			}
		}

		// 2) Class match by script path - avoids a compile-time include of the URLab header.
		//    TODO verify: module/class path "/Script/UnrealRoboticsLab.MjArticulation".
		static const TCHAR* CandidateClassPaths[] =
		{
						TEXT("/Script/URLab.MjArticulation"),
		};
		for (const TCHAR* Path : CandidateClassPaths)
		{
			if (UClass* ArtClass = FindObject<UClass>(nullptr, Path))
			{
				for (TActorIterator<AActor> It(World, ArtClass); It; ++It)
				{
					return *It;
				}
			}
		}

		// 3) Last resort: class-name heuristic (imported BPs are usually named after the MJCF model).
		for (TActorIterator<AActor> It(World); It; ++It)
		{
			const FString ClassName = It->GetClass()->GetName();
			if (ClassName.Contains(TEXT("MjArticulation")) || ClassName.Contains(TEXT("scene_stand")) || ClassName.Contains(TEXT("r1_twin")))
			{
				return *It;
			}
		}
		return nullptr;
	}

	TUniquePtr<IR1ArticulationBridge> CreateBridge(const FR1BridgeInitParams& Params, bool bAllowMock)
	{
		if (Params.ArticulationActor.IsValid())
		{
			TUniquePtr<IR1ArticulationBridge> Real = MakeUnique<FR1UrlabBridge>();
			if (Real->Initialize(Params))
			{
				return Real;
			}
		}

		if (bAllowMock)
		{
			TUniquePtr<IR1ArticulationBridge> Mock = MakeUnique<FR1MockBridge>();
			if (Mock->Initialize(Params))
			{
				return Mock;
			}
		}
		return nullptr;
	}
}
