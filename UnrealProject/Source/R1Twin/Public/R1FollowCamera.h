// Orbit camera pawn that follows the robot pelvis. Written for UE 5.7; NOT YET COMPILED.
//
// The pelvis pose comes from IR1ArticulationBridge::GetBodyPoseMj (MuJoCo metres / raw quaternion)
// and is converted with MjUe::PositionToUe. With bLockToStand (Level 0) the focus point is the fixed
// gantry-stand location instead (pelvis welded at z = 1.0 m -> UE (0, 0, 100)).
//
// Controls (Enhanced Input, actions + mapping context created in code - no .uasset needed):
//   Right mouse (hold) + move   orbit          IA_R1_OrbitEnable (Digital, RightMouseButton), IA_R1_Orbit (Axis2D, Mouse2D)
//   Mouse wheel                 zoom           IA_R1_Zoom (Axis1D, MouseWheelAxis)
//   L                           toggle stand lock      IA_R1_ToggleLock (Digital)
//   P                           toggle inference       IA_R1_ToggleInference (Digital)
//   A / D                       manual head yaw        IA_R1_HeadYawAxis (Axis1D, A negated)
//   W / S                       manual shoulder pitch  IA_R1_ShoulderAxis (Axis1D, S negated)
//   Tab                         cycle command mode     IA_R1_ToggleCmdMode (Digital)
//   R                           reset command clock    IA_R1_ResetClock (Digital)
// DefaultInput.ini sets DefaultViewportMouseCaptureMode=CaptureDuringRightMouseDown so the cursor is
// free unless orbiting.
#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"
#include "R1FollowCamera.generated.h"

class UCameraComponent;
class USceneComponent;
class UInputAction;
class UInputMappingContext;
class UR1PolicyComponent;
class UR1Level0SweepCommand;
struct FInputActionValue;

UCLASS()
class R1TWIN_API AR1FollowCamera : public APawn
{
	GENERATED_BODY()

public:
	AR1FollowCamera();

	// ------------------------------------------------------------------ follow

	/** MJCF body to follow (pelvis carries the "imu" site). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "R1|Follow")
	FName TargetBodyName = TEXT("pelvis");

	/** Level 0: ignore the pelvis and orbit the fixed stand focus. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "R1|Follow")
	bool bLockToStand = true;

	/** UE cm. scene_stand.xml welds the pelvis at MuJoCo (0,0,1.0) m. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "R1|Follow")
	FVector StandFocusUe = FVector(0.0, 0.0, 100.0);

	/** Added to the focus point (cm), e.g. to look at the chest rather than the pelvis. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "R1|Follow")
	FVector FocusOffsetUe = FVector(0.0, 0.0, 25.0);

	/** Exponential smoothing speeds (1/s). Higher = snappier. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "R1|Follow", meta = (ClampMin = 0.1))
	float FocusSmoothingSpeed = 6.f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "R1|Follow", meta = (ClampMin = 0.1))
	float OrbitSmoothingSpeed = 14.f;

	// ------------------------------------------------------------------ orbit

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "R1|Orbit")
	float OrbitYawDeg = -150.f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "R1|Orbit")
	float OrbitPitchDeg = -12.f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "R1|Orbit", meta = (ClampMin = 10.0))
	float Distance = 320.f;

	UPROPERTY(EditAnywhere, Category = "R1|Orbit")
	float MinDistance = 60.f;

	UPROPERTY(EditAnywhere, Category = "R1|Orbit")
	float MaxDistance = 2000.f;

	UPROPERTY(EditAnywhere, Category = "R1|Orbit")
	float MinPitchDeg = -85.f;

	UPROPERTY(EditAnywhere, Category = "R1|Orbit")
	float MaxPitchDeg = 20.f;

	/** Degrees per mouse unit. */
	UPROPERTY(EditAnywhere, Category = "R1|Orbit")
	float MouseSensitivity = 0.3f;

	UPROPERTY(EditAnywhere, Category = "R1|Orbit")
	bool bInvertPitch = false;

	/** cm per wheel notch. */
	UPROPERTY(EditAnywhere, Category = "R1|Orbit")
	float ZoomStep = 30.f;

	UFUNCTION(BlueprintCallable, Category = "R1|Follow")
	void SetLockToStand(bool bLock) { bLockToStand = bLock; }

	virtual void BeginPlay() override;
	virtual void Tick(float DeltaSeconds) override;
	virtual void SetupPlayerInputComponent(UInputComponent* PlayerInputComponent) override;

protected:
	UPROPERTY(VisibleAnywhere, Category = "R1")
	TObjectPtr<USceneComponent> Root;

	UPROPERTY(VisibleAnywhere, Category = "R1")
	TObjectPtr<UCameraComponent> Camera;

	// Enhanced Input assets, built at runtime in CreateInputAssets().
	UPROPERTY(Transient) TObjectPtr<UInputMappingContext> MappingContext;
	UPROPERTY(Transient) TObjectPtr<UInputAction> IA_OrbitEnable;
	UPROPERTY(Transient) TObjectPtr<UInputAction> IA_Orbit;
	UPROPERTY(Transient) TObjectPtr<UInputAction> IA_Zoom;
	UPROPERTY(Transient) TObjectPtr<UInputAction> IA_ToggleLock;
	UPROPERTY(Transient) TObjectPtr<UInputAction> IA_ToggleInference;
	UPROPERTY(Transient) TObjectPtr<UInputAction> IA_HeadYawAxis;
	UPROPERTY(Transient) TObjectPtr<UInputAction> IA_ShoulderAxis;
	UPROPERTY(Transient) TObjectPtr<UInputAction> IA_ToggleCmdMode;
	UPROPERTY(Transient) TObjectPtr<UInputAction> IA_ResetClock;

	/** Resolved lazily (the GameMode may spawn the policy host after this pawn). */
	UPROPERTY(Transient) TObjectPtr<UR1PolicyComponent> PolicyComponent;
	UPROPERTY(Transient) TObjectPtr<UR1Level0SweepCommand> CommandSource;

	void CreateInputAssets();
	void TryResolvePolicyComponent();
	bool GetFocusTarget(FVector& OutFocusUe) const;

	// input handlers
	void OnOrbitEnableStarted(const FInputActionValue& Value);
	void OnOrbitEnableCompleted(const FInputActionValue& Value);
	void OnOrbit(const FInputActionValue& Value);
	void OnZoom(const FInputActionValue& Value);
	void OnToggleLock(const FInputActionValue& Value);
	void OnToggleInference(const FInputActionValue& Value);
	void OnHeadYawAxis(const FInputActionValue& Value);
	void OnShoulderAxis(const FInputActionValue& Value);
	void OnToggleCmdMode(const FInputActionValue& Value);
	void OnResetClock(const FInputActionValue& Value);

private:
	bool bOrbiting = false;
	bool bFocusInitialised = false;
	FVector FocusSmoothed = FVector::ZeroVector;
	float SmoothedYaw = 0.f;
	float SmoothedPitch = 0.f;
	float SmoothedDistance = 0.f;
	float ManualHeadYawAxis = 0.f;
	float ManualShoulderAxis = 0.f;
	float ResolveRetryTimer = 0.f;
};
