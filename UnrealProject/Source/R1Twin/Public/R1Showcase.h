// Diagnostics overlay (Tab / 44 px corner button on touch) and idle demo mode (Po Game Standard C12 / C13).
//
// UR1ShowcaseSubsystem lives in every game world. Game code pushes its numbers in (episode reward, inference
// latency, curriculum stage) and binds OnDemoModeChanged / OnDemoCamera to hide its HUD, hand every agent to the
// trained policy and switch cameras. Every tunable (idle threshold, camera list, dwell time) is in
// UR1ShowcaseConfig, a DataAsset; the asset in use is picked in Project Settings > Game > R1 Showcase.
//
// Command line (used by the runtime screenshot check, docs/showcase.md):
//   -DiagOverlay              start with the overlay visible
//   -ShowcaseTouch            show the touch corner button on desktop
//   -ShowcaseIdle=<seconds>   override the idle threshold
//   -ShowcaseCapture=<dir>    self-capture: overlay_on, overlay_off, demo_a, demo_b screenshots into <dir>, then quit
#pragma once

#include "CoreMinimal.h"
#include "Engine/DataAsset.h"
#include "Engine/DeveloperSettings.h"
#include "Subsystems/WorldSubsystem.h"
#include "R1Showcase.generated.h"

class SWidget;
class FR1ShowcaseInput;

/** Idle demo + diagnostics overlay tunables. Create one in the editor (Miscellaneous > Data Asset) to override. */
UCLASS(BlueprintType)
class R1TWIN_API UR1ShowcaseConfig : public UPrimaryDataAsset
{
	GENERATED_BODY()

public:
	UR1ShowcaseConfig();

	/** Seconds without keyboard, mouse, touch or gamepad input before demo mode starts. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Demo", meta = (ClampMin = "5.0"))
	float IdleSeconds = 30.f;

	/** Seconds each showcase camera is held while demo mode runs. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Demo", meta = (ClampMin = "2.0"))
	float CameraDwellSeconds = 7.f;

	/** Showcase cameras, in order. The names are the game's camera modes (see the game's OnDemoCamera handler). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Demo")
	TArray<FName> DemoCameras;

	/** Key that toggles the diagnostics overlay. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Overlay")
	FKey OverlayKey;

	/** Inference latency is shown as the rolling mean of this many samples. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Overlay", meta = (ClampMin = "1", ClampMax = "600"))
	int32 LatencyWindow = 60;

	/** Show the 44 px corner button that toggles the overlay on touch devices. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Overlay")
	bool bTouchCornerButton = true;
};

/** Picks the DataAssets the game uses (Project Settings > Game > R1 Showcase). Empty = built-in defaults. */
UCLASS(Config = Game, DefaultConfig, meta = (DisplayName = "R1 Showcase"))
class R1TWIN_API UR1ShowcaseSettings : public UDeveloperSettings
{
	GENERATED_BODY()

public:
	UPROPERTY(Config, EditAnywhere, BlueprintReadOnly, Category = "Showcase")
	TSoftObjectPtr<UR1ShowcaseConfig> ShowcaseConfig;

	UPROPERTY(Config, EditAnywhere, BlueprintReadOnly, Category = "Showcase")
	TSoftObjectPtr<class UR1RewardConfig> RewardConfig;

	virtual FName GetCategoryName() const override { return TEXT("Game"); }

	static const UR1ShowcaseConfig* GetShowcaseConfig();
	static const class UR1RewardConfig* GetRewardConfig();
};

DECLARE_MULTICAST_DELEGATE_OneParam(FR1OnDemoModeChanged, bool /*bActive*/);
DECLARE_MULTICAST_DELEGATE_TwoParams(FR1OnDemoCamera, FName /*Camera*/, int32 /*Index*/);

UCLASS()
class R1TWIN_API UR1ShowcaseSubsystem : public UTickableWorldSubsystem
{
	GENERATED_BODY()

public:
	static UR1ShowcaseSubsystem* Get(const UObject* WorldContext);

	// --- numbers the game reports -------------------------------------------------------------------------
	UFUNCTION(BlueprintCallable, Category = "Showcase") void ReportEpisodeReward(float Reward);
	UFUNCTION(BlueprintCallable, Category = "Showcase") void ReportInferenceMs(float Milliseconds);
	UFUNCTION(BlueprintCallable, Category = "Showcase") void SetCurriculumStage(const FString& Stage);
	/** Shown after the reward when the game reports a proxy (e.g. "proxy: lead distance, m"). */
	UFUNCTION(BlueprintCallable, Category = "Showcase") void SetRewardNote(const FString& Note) { RewardNote = Note; }

	// --- state ----------------------------------------------------------------------------------------------
	UFUNCTION(BlueprintCallable, Category = "Showcase") void SetOverlayVisible(bool bVisible);
	UFUNCTION(BlueprintCallable, Category = "Showcase") void ToggleOverlay() { SetOverlayVisible(!bOverlayVisible); }
	UFUNCTION(BlueprintPure, Category = "Showcase") bool IsOverlayVisible() const { return bOverlayVisible; }
	UFUNCTION(BlueprintPure, Category = "Showcase") bool IsDemoActive() const { return bDemoActive; }
	UFUNCTION(BlueprintPure, Category = "Showcase") float GetIdleSeconds() const { return IdleTime; }
	UFUNCTION(BlueprintPure, Category = "Showcase") float GetFps() const { return Fps; }
	UFUNCTION(BlueprintPure, Category = "Showcase") float GetMeanInferenceMs() const;
	/** Starts demo mode now (e.g. a level opened by demo mode keeps showing it). */
	UFUNCTION(BlueprintCallable, Category = "Showcase") void StartDemo() { IdleTime = IdleThreshold; SetDemoActive(true); }
	/** Any real input: leaves demo mode and restarts the idle timer. */
	void NotifyUserInput();

	FR1OnDemoModeChanged OnDemoModeChanged;
	FR1OnDemoCamera OnDemoCamera;

	/** Overlay text, one "label  value" line per row (also used by tests). */
	FString BuildOverlayText() const;

	// UTickableWorldSubsystem
	virtual bool ShouldCreateSubsystem(UObject* Outer) const override;
	virtual void Initialize(FSubsystemCollectionBase& Collection) override;
	virtual void Deinitialize() override;
	virtual void OnWorldBeginPlay(UWorld& InWorld) override;
	virtual void Tick(float DeltaTime) override;
	virtual TStatId GetStatId() const override;
	virtual bool IsTickableWhenPaused() const override { return true; }

protected:
	virtual bool DoesSupportWorldType(const EWorldType::Type WorldType) const override;

private:
	void SetDemoActive(bool bActive);
	void AddWidgets();
	void RemoveWidgets();
	void TickCapture(float DeltaTime);
	void Shot(const TCHAR* Name);

	TSharedPtr<FR1ShowcaseInput> Input;
	TSharedPtr<SWidget> OverlayWidget;
	TSharedPtr<SWidget> ButtonWidget;
	TSharedPtr<SWidget> DemoHintWidget;

	bool bOverlayVisible = false;
	bool bDemoActive = false;
	bool bTouchButton = false;
	float IdleTime = 0.f;
	float IdleThreshold = 30.f;
	float CameraTime = 0.f;
	int32 CameraIndex = 0;
	float Fps = 0.f;
	float EpisodeReward = 0.f;
	bool bHaveReward = false;
	FString CurriculumStage;
	FString RewardNote;
	TArray<float> LatencySamples;
	int32 LatencyNext = 0;

	FString CaptureDir;
	float CaptureClock = 0.f;
	int32 CaptureStep = 0;
};
