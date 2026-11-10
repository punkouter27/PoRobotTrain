// Written for UE 5.7; NOT YET COMPILED (see docs/ue5_setup.md).
#include "R1FollowCamera.h"
#include "R1Twin.h"
#include "R1PolicyComponent.h"
#include "R1Level0SweepCommand.h"
#include "R1ArticulationBridge.h"
#include "MjUeConvert.h"

#include "Camera/CameraComponent.h"
#include "Components/SceneComponent.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "GameFramework/PlayerController.h"
#include "Engine/LocalPlayer.h"

// Enhanced Input (plugin "EnhancedInput", module dependency in R1Twin.Build.cs)
#include "EnhancedInputComponent.h"
#include "EnhancedInputSubsystems.h"
#include "InputAction.h"
#include "InputActionValue.h"
#include "InputMappingContext.h"
#include "InputModifiers.h"

AR1FollowCamera::AR1FollowCamera()
{
	PrimaryActorTick.bCanEverTick = true;
	// Camera runs after the policy/command components so it sees this frame's pose.
	PrimaryActorTick.TickGroup = TG_PostUpdateWork;

	Root = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
	SetRootComponent(Root);

	Camera = CreateDefaultSubobject<UCameraComponent>(TEXT("Camera"));
	Camera->SetupAttachment(Root);
	Camera->bUsePawnControlRotation = false;
	Camera->FieldOfView = 70.f;

	AutoPossessPlayer = EAutoReceiveInput::Disabled; // GameMode spawns/possesses via DefaultPawnClass
	bUseControllerRotationYaw = false;
	bUseControllerRotationPitch = false;
}

void AR1FollowCamera::BeginPlay()
{
	Super::BeginPlay();

	SmoothedYaw = OrbitYawDeg;
	SmoothedPitch = OrbitPitchDeg;
	SmoothedDistance = Distance;

	if (APlayerController* PC = Cast<APlayerController>(GetController()))
	{
		PC->bShowMouseCursor = true;
		PC->bEnableClickEvents = false;
		PC->bEnableMouseOverEvents = false;
	}

	TryResolvePolicyComponent();
}

void AR1FollowCamera::TryResolvePolicyComponent()
{
	if (PolicyComponent)
	{
		return;
	}
	UWorld* World = GetWorld();
	if (!World)
	{
		return;
	}
	for (TActorIterator<AActor> It(World); It; ++It)
	{
		if (UR1PolicyComponent* Comp = It->FindComponentByClass<UR1PolicyComponent>())
		{
			PolicyComponent = Comp;
			CommandSource = Comp->CommandSource ? Comp->CommandSource.Get() : It->FindComponentByClass<UR1Level0SweepCommand>();
			UE_LOG(LogR1Twin, Log, TEXT("R1FollowCamera: bound to policy component on '%s'."), *It->GetName());
			return;
		}
	}
}

bool AR1FollowCamera::GetFocusTarget(FVector& OutFocusUe) const
{
	if (bLockToStand)
	{
		OutFocusUe = StandFocusUe + FocusOffsetUe;
		return true;
	}

	if (PolicyComponent)
	{
		if (const IR1ArticulationBridge* Bridge = PolicyComponent->GetBridge())
		{
			FVector PosMj;
			FQuat QuatRaw;
			if (Bridge->GetBodyPoseMj(TargetBodyName, PosMj, QuatRaw))
			{
				OutFocusUe = MjUe::PositionToUe(PosMj) + FocusOffsetUe;
				return true;
			}
		}
	}

	// Nothing to follow yet: keep whatever we had, or fall back to the stand.
	OutFocusUe = bFocusInitialised ? FocusSmoothed : (StandFocusUe + FocusOffsetUe);
	return false;
}

void AR1FollowCamera::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);

	if (!PolicyComponent)
	{
		ResolveRetryTimer -= DeltaSeconds;
		if (ResolveRetryTimer <= 0.f)
		{
			ResolveRetryTimer = 1.f;
			TryResolvePolicyComponent();
		}
	}

	// Forward manual axes every frame (the command component only integrates while Mode == Manual).
	if (CommandSource)
	{
		CommandSource->SetManualAxes(ManualHeadYawAxis, ManualShoulderAxis);
	}

	FVector FocusTarget;
	GetFocusTarget(FocusTarget);
	if (!bFocusInitialised)
	{
		FocusSmoothed = FocusTarget;
		bFocusInitialised = true;
	}
	else
	{
		FocusSmoothed = FMath::VInterpTo(FocusSmoothed, FocusTarget, DeltaSeconds, FocusSmoothingSpeed);
	}

	OrbitPitchDeg = FMath::Clamp(OrbitPitchDeg, MinPitchDeg, MaxPitchDeg);
	Distance = FMath::Clamp(Distance, MinDistance, MaxDistance);
	SmoothedYaw = FMath::FInterpTo(SmoothedYaw, OrbitYawDeg, DeltaSeconds, OrbitSmoothingSpeed);
	SmoothedPitch = FMath::FInterpTo(SmoothedPitch, OrbitPitchDeg, DeltaSeconds, OrbitSmoothingSpeed);
	SmoothedDistance = FMath::FInterpTo(SmoothedDistance, Distance, DeltaSeconds, OrbitSmoothingSpeed);

	const FRotator ViewRot(SmoothedPitch, SmoothedYaw, 0.f);
	const FVector CamLoc = FocusSmoothed - ViewRot.Vector() * SmoothedDistance;
	SetActorLocationAndRotation(CamLoc, ViewRot);
}

// -----------------------------------------------------------------------------------------------------
// Input
// -----------------------------------------------------------------------------------------------------

void AR1FollowCamera::CreateInputAssets()
{
	if (MappingContext)
	{
		return;
	}

	auto MakeAction = [this](const TCHAR* Name, EInputActionValueType Type) -> UInputAction*
	{
		UInputAction* Action = NewObject<UInputAction>(this, Name);
		Action->ValueType = Type;
		return Action;
	};

	IA_OrbitEnable     = MakeAction(TEXT("IA_R1_OrbitEnable"),     EInputActionValueType::Boolean);
	IA_Orbit           = MakeAction(TEXT("IA_R1_Orbit"),           EInputActionValueType::Axis2D);
	IA_Zoom            = MakeAction(TEXT("IA_R1_Zoom"),            EInputActionValueType::Axis1D);
	IA_ToggleLock      = MakeAction(TEXT("IA_R1_ToggleLock"),      EInputActionValueType::Boolean);
	IA_ToggleInference = MakeAction(TEXT("IA_R1_ToggleInference"), EInputActionValueType::Boolean);
	IA_HeadYawAxis     = MakeAction(TEXT("IA_R1_HeadYawAxis"),     EInputActionValueType::Axis1D);
	IA_ShoulderAxis    = MakeAction(TEXT("IA_R1_ShoulderAxis"),    EInputActionValueType::Axis1D);
	IA_ToggleCmdMode   = MakeAction(TEXT("IA_R1_ToggleCmdMode"),   EInputActionValueType::Boolean);
	IA_ResetClock      = MakeAction(TEXT("IA_R1_ResetClock"),      EInputActionValueType::Boolean);

	MappingContext = NewObject<UInputMappingContext>(this, TEXT("IMC_R1Camera"));

	MappingContext->MapKey(IA_OrbitEnable, EKeys::RightMouseButton);
	MappingContext->MapKey(IA_Orbit, EKeys::Mouse2D);
	MappingContext->MapKey(IA_Zoom, EKeys::MouseWheelAxis);
	MappingContext->MapKey(IA_ToggleLock, EKeys::L);
	MappingContext->MapKey(IA_ToggleInference, EKeys::P);
	MappingContext->MapKey(IA_ToggleCmdMode, EKeys::Tab);
	MappingContext->MapKey(IA_ResetClock, EKeys::R);

	// A/D -> head yaw axis (A negated), W/S -> shoulder axis (S negated)
	MappingContext->MapKey(IA_HeadYawAxis, EKeys::D);
	{
		FEnhancedActionKeyMapping& Neg = MappingContext->MapKey(IA_HeadYawAxis, EKeys::A);
		Neg.Modifiers.Add(NewObject<UInputModifierNegate>(MappingContext));
	}
	MappingContext->MapKey(IA_ShoulderAxis, EKeys::W);
	{
		FEnhancedActionKeyMapping& Neg = MappingContext->MapKey(IA_ShoulderAxis, EKeys::S);
		Neg.Modifiers.Add(NewObject<UInputModifierNegate>(MappingContext));
	}
}

void AR1FollowCamera::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)
{
	Super::SetupPlayerInputComponent(PlayerInputComponent);

	CreateInputAssets();

	if (APlayerController* PC = Cast<APlayerController>(GetController()))
	{
		if (ULocalPlayer* LP = PC->GetLocalPlayer())
		{
			if (UEnhancedInputLocalPlayerSubsystem* Subsystem = LP->GetSubsystem<UEnhancedInputLocalPlayerSubsystem>())
			{
				Subsystem->AddMappingContext(MappingContext, /*Priority*/ 0);
			}
		}
	}

	UEnhancedInputComponent* EIC = Cast<UEnhancedInputComponent>(PlayerInputComponent);
	if (!EIC)
	{
		UE_LOG(LogR1Twin, Error, TEXT("R1FollowCamera: input component is not UEnhancedInputComponent - check DefaultInput.ini (DefaultInputComponentClass)."));
		return;
	}

	EIC->BindAction(IA_OrbitEnable, ETriggerEvent::Started,   this, &AR1FollowCamera::OnOrbitEnableStarted);
	EIC->BindAction(IA_OrbitEnable, ETriggerEvent::Completed, this, &AR1FollowCamera::OnOrbitEnableCompleted);
	EIC->BindAction(IA_Orbit,       ETriggerEvent::Triggered, this, &AR1FollowCamera::OnOrbit);
	EIC->BindAction(IA_Zoom,        ETriggerEvent::Triggered, this, &AR1FollowCamera::OnZoom);
	EIC->BindAction(IA_ToggleLock,  ETriggerEvent::Started,   this, &AR1FollowCamera::OnToggleLock);
	EIC->BindAction(IA_ToggleInference, ETriggerEvent::Started, this, &AR1FollowCamera::OnToggleInference);
	// Axis actions: Triggered while held (value), Completed on release (value 0).
	EIC->BindAction(IA_HeadYawAxis,  ETriggerEvent::Triggered, this, &AR1FollowCamera::OnHeadYawAxis);
	EIC->BindAction(IA_HeadYawAxis,  ETriggerEvent::Completed, this, &AR1FollowCamera::OnHeadYawAxis);
	EIC->BindAction(IA_ShoulderAxis, ETriggerEvent::Triggered, this, &AR1FollowCamera::OnShoulderAxis);
	EIC->BindAction(IA_ShoulderAxis, ETriggerEvent::Completed, this, &AR1FollowCamera::OnShoulderAxis);
	EIC->BindAction(IA_ToggleCmdMode, ETriggerEvent::Started,  this, &AR1FollowCamera::OnToggleCmdMode);
	EIC->BindAction(IA_ResetClock,    ETriggerEvent::Started,  this, &AR1FollowCamera::OnResetClock);
}

void AR1FollowCamera::OnOrbitEnableStarted(const FInputActionValue& /*Value*/)
{
	bOrbiting = true;
}

void AR1FollowCamera::OnOrbitEnableCompleted(const FInputActionValue& /*Value*/)
{
	bOrbiting = false;
}

void AR1FollowCamera::OnOrbit(const FInputActionValue& Value)
{
	if (!bOrbiting)
	{
		return;
	}
	const FVector2D Delta = Value.Get<FVector2D>();
	OrbitYawDeg += Delta.X * MouseSensitivity;
	OrbitPitchDeg += (bInvertPitch ? -Delta.Y : Delta.Y) * MouseSensitivity;
	OrbitPitchDeg = FMath::Clamp(OrbitPitchDeg, MinPitchDeg, MaxPitchDeg);
}

void AR1FollowCamera::OnZoom(const FInputActionValue& Value)
{
	const float Wheel = Value.Get<float>();
	Distance = FMath::Clamp(Distance - Wheel * ZoomStep, MinDistance, MaxDistance);
}

void AR1FollowCamera::OnToggleLock(const FInputActionValue& /*Value*/)
{
	bLockToStand = !bLockToStand;
	UE_LOG(LogR1Twin, Log, TEXT("R1FollowCamera: lock to stand = %s"), bLockToStand ? TEXT("on") : TEXT("off"));
}

void AR1FollowCamera::OnToggleInference(const FInputActionValue& /*Value*/)
{
	if (PolicyComponent)
	{
		PolicyComponent->SetInferenceEnabled(!PolicyComponent->bInferenceEnabled);
		UE_LOG(LogR1Twin, Log, TEXT("R1FollowCamera: inference %s"), PolicyComponent->bInferenceEnabled ? TEXT("enabled") : TEXT("paused"));
	}
}

void AR1FollowCamera::OnHeadYawAxis(const FInputActionValue& Value)
{
	ManualHeadYawAxis = Value.Get<float>();
}

void AR1FollowCamera::OnShoulderAxis(const FInputActionValue& Value)
{
	ManualShoulderAxis = Value.Get<float>();
}

void AR1FollowCamera::OnToggleCmdMode(const FInputActionValue& /*Value*/)
{
	if (CommandSource)
	{
		CommandSource->ToggleMode();
	}
}

void AR1FollowCamera::OnResetClock(const FInputActionValue& /*Value*/)
{
	if (PolicyComponent)
	{
		PolicyComponent->ResetPolicy(); // also resets the command clock
	}
	else if (CommandSource)
	{
		CommandSource->ResetClock();
	}
}
