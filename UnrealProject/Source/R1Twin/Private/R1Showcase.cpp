#include "R1Showcase.h"

#include "R1RewardConfig.h"
#include "Async/Async.h"
#include "HAL/PlatformTime.h"
#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Core/MjPhysicsEngine.h"
#include "Widgets/SBoxPanel.h"
#include "Engine/Engine.h"
#include "Engine/GameViewportClient.h"
#include "Engine/World.h"
#include "Framework/Application/IInputProcessor.h"
#include "Framework/Application/SlateApplication.h"
#include "HAL/PlatformMisc.h"
#include "InputCoreTypes.h"
#include "Misc/App.h"
#include "Misc/CommandLine.h"
#include "Misc/Paths.h"
#include "Misc/Parse.h"
#include "Styling/CoreStyle.h"
#include "UnrealClient.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Layout/SBorder.h"
#include "Widgets/Layout/SBox.h"
#include "Widgets/SOverlay.h"
#include "Widgets/Text/STextBlock.h"

#include UE_INLINE_GENERATED_CPP_BY_NAME(R1Showcase)

DEFINE_LOG_CATEGORY_STATIC(LogR1Showcase, Log, All);

namespace R1ShowcaseUi
{
	// Po Game Standard tokens: opaque surface panel, text >= 4.5:1 on it (#e6e9f2 on #121826 is ~14:1).
	static FLinearColor Hex(const TCHAR* H) { return FLinearColor::FromSRGBColor(FColor::FromHex(H)); }
	static FLinearColor Surface() { return Hex(TEXT("121826")); }
	static FLinearColor Text() { return Hex(TEXT("e6e9f2")); }
	static FLinearColor Muted() { return Hex(TEXT("9aa3b5")); }
	static FLinearColor Accent() { return Hex(TEXT("6366f1")); }
}

// ------------------------------------------------------------------------------------------------------------
// Input: every key, button, touch, wheel, real mouse move and gamepad stick counts as "the player is here".

class FR1ShowcaseInput : public IInputProcessor
{
public:
	FR1ShowcaseInput(UR1ShowcaseSubsystem* InOwner, const FKey& InToggleKey) : Owner(InOwner), ToggleKey(InToggleKey) {}

	virtual void Tick(const float DeltaTime, FSlateApplication& SlateApp, TSharedRef<ICursor> Cursor) override {}

	virtual bool HandleKeyDownEvent(FSlateApplication& SlateApp, const FKeyEvent& InKeyEvent) override
	{
		UR1ShowcaseSubsystem* O = Owner.Get();
		if (!O) return false;
		O->NotifyUserInput();
		if (InKeyEvent.GetKey() == ToggleKey)
		{
			if (!InKeyEvent.IsRepeat()) O->ToggleOverlay();
			return true;   // the overlay owns its key
		}
		return false;
	}

	virtual bool HandleAnalogInputEvent(FSlateApplication& SlateApp, const FAnalogInputEvent& InAnalogInputEvent) override
	{
		if (FMath::Abs(InAnalogInputEvent.GetAnalogValue()) > 0.25f) Touch();
		return false;
	}

	virtual bool HandleMouseMoveEvent(FSlateApplication& SlateApp, const FPointerEvent& MouseEvent) override
	{
		if (MouseEvent.GetCursorDelta().SizeSquared() > 4.f) Touch();   // synthetic moves carry no delta
		return false;
	}

	virtual bool HandleMouseButtonDownEvent(FSlateApplication& SlateApp, const FPointerEvent& MouseEvent) override
	{
		Touch();   // mouse buttons and touches
		return false;
	}

	virtual bool HandleMouseWheelOrGestureEvent(FSlateApplication& SlateApp, const FPointerEvent& InWheelEvent, const FPointerEvent* InGestureEvent) override
	{
		Touch();
		return false;
	}

	virtual const TCHAR* GetDebugName() const override { return TEXT("R1Showcase"); }

private:
	void Touch() { if (UR1ShowcaseSubsystem* O = Owner.Get()) O->NotifyUserInput(); }

	TWeakObjectPtr<UR1ShowcaseSubsystem> Owner;
	FKey ToggleKey;
};

// ------------------------------------------------------------------------------------------------------------

UR1ShowcaseConfig::UR1ShowcaseConfig()
{
	OverlayKey = EKeys::Tab;
	DemoCameras = { FName(TEXT("Front")), FName(TEXT("ThreeQuarter")), FName(TEXT("Side")) };
}

const UR1ShowcaseConfig* UR1ShowcaseSettings::GetShowcaseConfig()
{
	const UR1ShowcaseSettings* S = GetDefault<UR1ShowcaseSettings>();
	if (S && !S->ShowcaseConfig.IsNull())
	{
		if (const UR1ShowcaseConfig* Asset = S->ShowcaseConfig.LoadSynchronous()) return Asset;
	}
	return GetDefault<UR1ShowcaseConfig>();
}

const UR1RewardConfig* UR1ShowcaseSettings::GetRewardConfig()
{
	const UR1ShowcaseSettings* S = GetDefault<UR1ShowcaseSettings>();
	if (S && !S->RewardConfig.IsNull())
	{
		if (const UR1RewardConfig* Asset = S->RewardConfig.LoadSynchronous()) return Asset;
	}
	return GetDefault<UR1RewardConfig>();
}

// ------------------------------------------------------------------------------------------------------------

UR1ShowcaseSubsystem* UR1ShowcaseSubsystem::Get(const UObject* WorldContext)
{
	const UWorld* World = WorldContext ? WorldContext->GetWorld() : nullptr;
	return World ? World->GetSubsystem<UR1ShowcaseSubsystem>() : nullptr;
}

bool UR1ShowcaseSubsystem::ShouldCreateSubsystem(UObject* Outer) const
{
	return !IsRunningCommandlet() && Super::ShouldCreateSubsystem(Outer);
}

bool UR1ShowcaseSubsystem::DoesSupportWorldType(const EWorldType::Type WorldType) const
{
	return WorldType == EWorldType::Game || WorldType == EWorldType::PIE;
}

void UR1ShowcaseSubsystem::Initialize(FSubsystemCollectionBase& Collection)
{
	Super::Initialize(Collection);
	const UR1ShowcaseConfig* Cfg = UR1ShowcaseSettings::GetShowcaseConfig();
	IdleThreshold = Cfg->IdleSeconds;
	FParse::Value(FCommandLine::Get(), TEXT("ShowcaseIdle="), IdleThreshold);
	IdleThreshold = FMath::Max(1.f, IdleThreshold);
	LatencySamples.Reset();
	bOverlayVisible = FParse::Param(FCommandLine::Get(), TEXT("DiagOverlay"));
	bTouchButton = Cfg->bTouchCornerButton
		&& (PLATFORM_ANDROID || PLATFORM_IOS || FPlatformMisc::GetUseVirtualJoysticks() || FParse::Param(FCommandLine::Get(), TEXT("ShowcaseTouch")));
	FString Dir;
	if (FParse::Value(FCommandLine::Get(), TEXT("ShowcaseCapture="), Dir)) CaptureDir = FPaths::ConvertRelativePathToFull(Dir);
}

void UR1ShowcaseSubsystem::OnWorldBeginPlay(UWorld& InWorld)
{
	Super::OnWorldBeginPlay(InWorld);
	if (!FSlateApplication::IsInitialized()) return;
	const UR1ShowcaseConfig* Cfg = UR1ShowcaseSettings::GetShowcaseConfig();
	Input = MakeShared<FR1ShowcaseInput>(this, Cfg->OverlayKey);
	FSlateApplication::Get().RegisterInputPreProcessor(Input);
	AddWidgets();
	UE_LOG(LogR1Showcase, Log, TEXT("Showcase: overlay key %s, demo after %.0f s idle, %d demo cameras, touch button %s%s"),
		*Cfg->OverlayKey.ToString(), IdleThreshold, Cfg->DemoCameras.Num(), bTouchButton ? TEXT("on") : TEXT("off"),
		CaptureDir.IsEmpty() ? TEXT("") : *FString::Printf(TEXT(", capturing to %s"), *CaptureDir));
}

void UR1ShowcaseSubsystem::Deinitialize()
{
	if (Input.IsValid() && FSlateApplication::IsInitialized()) FSlateApplication::Get().UnregisterInputPreProcessor(Input);
	Input.Reset();
	RemoveWidgets();
	Super::Deinitialize();
}

TStatId UR1ShowcaseSubsystem::GetStatId() const
{
	RETURN_QUICK_DECLARE_CYCLE_STAT(UR1ShowcaseSubsystem, STATGROUP_Tickables);
}

void UR1ShowcaseSubsystem::AddWidgets()
{
	UWorld* World = GetWorld();
	UGameViewportClient* VC = World ? World->GetGameViewport() : nullptr;
	if (!VC) return;
	const TWeakObjectPtr<UR1ShowcaseSubsystem> Weak(this);
	const FSlateFontInfo Mono = FCoreStyle::GetDefaultFontStyle("Mono", 13);
	const FSlateFontInfo Body = FCoreStyle::GetDefaultFontStyle("Regular", 13);
	const FSlateBrush* White = FCoreStyle::Get().GetBrush("WhiteBrush");

	OverlayWidget = SNew(SBox)
		.HAlign(HAlign_Left).VAlign(VAlign_Top).Padding(FMargin(16.f, 64.f, 16.f, 16.f))
		.Visibility_Lambda([Weak] { return Weak.IsValid() && Weak->IsOverlayVisible() && !Weak->IsDemoActive() ? EVisibility::SelfHitTestInvisible : EVisibility::Collapsed; })
		[
			SNew(SBorder).BorderImage(White).BorderBackgroundColor(R1ShowcaseUi::Surface()).Padding(FMargin(14.f, 10.f))
			[
				SNew(SVerticalBox)
				+ SVerticalBox::Slot().AutoHeight()
				[
					SNew(STextBlock).Font(Mono).ColorAndOpacity(R1ShowcaseUi::Text())
					.Text_Lambda([Weak] { return Weak.IsValid() ? FText::FromString(Weak->BuildOverlayText()) : FText::GetEmpty(); })
				]
				+ SVerticalBox::Slot().AutoHeight().HAlign(HAlign_Left).Padding(FMargin(0.f, 8.f, 0.f, 0.f))
				[
					SNew(SBox).MinDesiredHeight(44.f)
					[
						SNew(SButton)
						.ButtonColorAndOpacity(R1ShowcaseUi::Accent())
						.VAlign(VAlign_Center)
						.ToolTipText(NSLOCTEXT("R1Showcase", "BenchTip", "Steps a copy of this MuJoCo scene for 1 s: physics steps/s of one training environment on this PC"))
						.OnClicked_Lambda([Weak] { if (Weak.IsValid()) Weak->RunBenchmark(); return FReply::Handled(); })
						[
							SNew(STextBlock).Font(Body).ColorAndOpacity(FLinearColor::White)
							.Text(NSLOCTEXT("R1Showcase", "BenchLabel", "Benchmark training speed"))
						]
					]
				]
			]
		];
	VC->AddViewportWidgetContent(OverlayWidget.ToSharedRef(), 1000);

	DemoHintWidget = SNew(SBox)
		.HAlign(HAlign_Center).VAlign(VAlign_Bottom).Padding(FMargin(16.f, 16.f, 16.f, 40.f))
		.Visibility_Lambda([Weak] { return Weak.IsValid() && Weak->IsDemoActive() ? EVisibility::HitTestInvisible : EVisibility::Collapsed; })
		[
			SNew(SBorder).BorderImage(White).BorderBackgroundColor(R1ShowcaseUi::Surface()).Padding(FMargin(14.f, 8.f))
			[
				SNew(STextBlock).Font(Body).ColorAndOpacity(R1ShowcaseUi::Text())
				.Text(NSLOCTEXT("R1Showcase", "DemoHint", "Demo  -  press any key or touch to play"))
			]
		];
	VC->AddViewportWidgetContent(DemoHintWidget.ToSharedRef(), 1000);

	if (bTouchButton)
	{
		ButtonWidget = SNew(SBox)
			.HAlign(HAlign_Right).VAlign(VAlign_Top).Padding(FMargin(8.f))
			.Visibility_Lambda([Weak] { return Weak.IsValid() && !Weak->IsDemoActive() ? EVisibility::SelfHitTestInvisible : EVisibility::Collapsed; })
			[
				SNew(SBox).WidthOverride(44.f).HeightOverride(44.f)
				[
					SNew(SButton)
					.ButtonColorAndOpacity(R1ShowcaseUi::Accent())
					.HAlign(HAlign_Center).VAlign(VAlign_Center)
					.ToolTipText(NSLOCTEXT("R1Showcase", "ButtonTip", "Diagnostics"))
					.OnClicked_Lambda([Weak] { if (Weak.IsValid()) Weak->ToggleOverlay(); return FReply::Handled(); })
					[
						SNew(STextBlock).Font(FCoreStyle::GetDefaultFontStyle("Bold", 15)).ColorAndOpacity(FLinearColor::White)
						.Text(NSLOCTEXT("R1Showcase", "ButtonLabel", "i"))
					]
				]
			];
		VC->AddViewportWidgetContent(ButtonWidget.ToSharedRef(), 1001);
	}
}

void UR1ShowcaseSubsystem::RemoveWidgets()
{
	UWorld* World = GetWorld();
	UGameViewportClient* VC = World ? World->GetGameViewport() : nullptr;
	for (TSharedPtr<SWidget>* W : { &OverlayWidget, &DemoHintWidget, &ButtonWidget })
	{
		if (VC && W->IsValid()) VC->RemoveViewportWidgetContent(W->ToSharedRef());
		W->Reset();
	}
}

void UR1ShowcaseSubsystem::ReportEpisodeReward(float Reward)
{
	EpisodeReward = Reward;
	bHaveReward = true;
}

void UR1ShowcaseSubsystem::ReportInferenceMs(float Milliseconds)
{
	if (!FMath::IsFinite(Milliseconds) || Milliseconds < 0.f) return;
	const int32 Window = FMath::Clamp(UR1ShowcaseSettings::GetShowcaseConfig()->LatencyWindow, 1, 600);
	if (LatencySamples.Num() < Window)
	{
		LatencySamples.Add(Milliseconds);
	}
	else
	{
		LatencySamples[LatencyNext % LatencySamples.Num()] = Milliseconds;
	}
	LatencyNext = (LatencyNext + 1) % Window;
}

void UR1ShowcaseSubsystem::SetCurriculumStage(const FString& Stage)
{
	CurriculumStage = Stage;
}

float UR1ShowcaseSubsystem::GetMeanInferenceMs() const
{
	if (LatencySamples.Num() == 0) return -1.f;
	float Sum = 0.f;
	for (const float V : LatencySamples) Sum += V;
	return Sum / LatencySamples.Num();
}

FString UR1ShowcaseSubsystem::BuildOverlayText() const
{
	const UR1ShowcaseConfig* Cfg = UR1ShowcaseSettings::GetShowcaseConfig();
	const float Lat = GetMeanInferenceMs();
	FString Out = FString::Printf(TEXT("DIAGNOSTICS              [%s]\n"), *Cfg->OverlayKey.GetDisplayName().ToString());
	Out += FString::Printf(TEXT("FPS                      %.1f\n"), Fps);
	const FString Note = RewardNote.IsEmpty() ? FString() : FString::Printf(TEXT("  (%s)"), *RewardNote);
	Out += bHaveReward ? FString::Printf(TEXT("Episode reward           %.2f%s\n"), EpisodeReward, *Note) : FString(TEXT("Episode reward           n/a\n"));
	Out += Lat >= 0.f ? FString::Printf(TEXT("Inference latency        %.2f ms (mean of %d)\n"), Lat, LatencySamples.Num())
	                  : FString(TEXT("Inference latency        n/a (no policy step yet)\n"));
	Out += FString::Printf(TEXT("Curriculum stage         %s\n"), CurriculumStage.IsEmpty() ? TEXT("n/a") : *CurriculumStage);
	Out += FString::Printf(TEXT("Training speed (bench)   %s\n"), BenchText.IsEmpty() ? TEXT("press the button below") : *BenchText);
	Out += FString::Printf(TEXT("Demo mode                after %.0f s idle (idle %.0f s)"), IdleThreshold, IdleTime);
	return Out;
}

void UR1ShowcaseSubsystem::RunBenchmark()
{
	if (bBenchRunning) return;
	const AAMjManager* Manager = AAMjManager::GetManager();
	UMjPhysicsEngine* Engine = Manager ? Manager->PhysicsEngine : nullptr;
	const mjModel* Model = Engine ? Engine->GetModel() : nullptr;
	if (!Model || !Engine->GetData())
	{
		BenchText = TEXT("no MuJoCo scene loaded");
		return;
	}
	// Private copy of the live state, taken under the physics thread's lock; the live simulation keeps running.
	mjModel* ModelCopy = mj_copyModel(nullptr, Model);
	mjData* DataCopy = mj_makeData(ModelCopy);
	{
		FScopeLock Lock(&Engine->CallbackMutex);
		mj_copyData(DataCopy, Model, Engine->GetData());
	}
	bBenchRunning = true;
	BenchText = TEXT("running (1 s)...");
	const TWeakObjectPtr<UR1ShowcaseSubsystem> Weak(this);
	Async(EAsyncExecution::ThreadPool, [Weak, ModelCopy, DataCopy]
	{
		const double T0 = FPlatformTime::Seconds();
		int32 Steps = 0;
		while (Steps < 1000000 && FPlatformTime::Seconds() - T0 < 1.0)
		{
			mj_step(ModelCopy, DataCopy);
			++Steps;
		}
		const double Seconds = FPlatformTime::Seconds() - T0;
		const double Timestep = ModelCopy->opt.timestep;
		mj_deleteData(DataCopy);
		mj_deleteModel(ModelCopy);
		AsyncTask(ENamedThreads::GameThread, [Weak, Steps, Seconds, Timestep]
		{
			if (Weak.IsValid()) Weak->FinishBenchmark(Steps, Seconds, Timestep);
		});
	});
}

void UR1ShowcaseSubsystem::FinishBenchmark(int32 Steps, double Seconds, double Timestep)
{
	bBenchRunning = false;
	const double Rate = Seconds > 0.0 ? Steps / Seconds : 0.0;
	BenchText = FString::Printf(TEXT("%.0f steps/s, %.0fx real time"), Rate, Rate * Timestep);
	UE_LOG(LogR1Showcase, Log, TEXT("Showcase: benchmark ran %d MuJoCo steps of a scene copy in %.2f s"), Steps, Seconds);
	UE_LOG(LogR1Showcase, Log, TEXT("Showcase: benchmark %s"), *BenchText);
}

void UR1ShowcaseSubsystem::SetOverlayVisible(bool bVisible)
{
	bOverlayVisible = bVisible;
}

void UR1ShowcaseSubsystem::NotifyUserInput()
{
	IdleTime = 0.f;
	if (bDemoActive) SetDemoActive(false);
}

void UR1ShowcaseSubsystem::SetDemoActive(bool bActive)
{
	if (bDemoActive == bActive) return;
	bDemoActive = bActive;
	CameraTime = 0.f;
	CameraIndex = 0;
	UE_LOG(LogR1Showcase, Log, TEXT("Showcase: demo mode %s"), bActive ? TEXT("on (idle)") : TEXT("off (input)"));
	OnDemoModeChanged.Broadcast(bActive);
	const TArray<FName>& Cams = UR1ShowcaseSettings::GetShowcaseConfig()->DemoCameras;
	if (bActive && Cams.Num() > 0) OnDemoCamera.Broadcast(Cams[0], 0);
}

void UR1ShowcaseSubsystem::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	const float Dt = static_cast<float>(FApp::GetDeltaTime());   // real time: pause and slow motion don't stop the idle clock
	if (Dt > 0.f)
	{
		const float Inst = 1.f / Dt;
		Fps = Fps <= 0.f ? Inst : FMath::Lerp(Fps, Inst, 0.05f);
	}
	if (Dt < 0.5f) IdleTime += Dt;   // a long frame (shader warm-up, level load) is not idle time
	if (!bDemoActive && IdleTime >= IdleThreshold) SetDemoActive(true);
	if (bDemoActive)
	{
		const UR1ShowcaseConfig* Cfg = UR1ShowcaseSettings::GetShowcaseConfig();
		CameraTime += Dt;
		if (Cfg->DemoCameras.Num() > 0 && CameraTime >= Cfg->CameraDwellSeconds)
		{
			CameraTime = 0.f;
			CameraIndex = (CameraIndex + 1) % Cfg->DemoCameras.Num();
			OnDemoCamera.Broadcast(Cfg->DemoCameras[CameraIndex], CameraIndex);
		}
	}
	if (!CaptureDir.IsEmpty()) TickCapture(Dt);
}

void UR1ShowcaseSubsystem::Shot(const TCHAR* Name)
{
	const FString File = FPaths::Combine(CaptureDir, FString::Printf(TEXT("%s.png"), Name));
	FScreenshotRequest::RequestScreenshot(File, /*bShowUI*/ true, /*bAddFilenameSuffix*/ false);
	UE_LOG(LogR1Showcase, Log, TEXT("Showcase: screenshot %s (overlay %d, demo %d, idle %.1f s)"), *File, bOverlayVisible ? 1 : 0, bDemoActive ? 1 : 0, IdleTime);
}

void UR1ShowcaseSubsystem::TickCapture(float Dt)
{
	// Scripted check of C12 / C13 without touching the input clock: overlay on, overlay off, demo mode after the
	// idle threshold (two shots one camera dwell apart), then quit.
	CaptureClock += Dt;
	const float Dwell = UR1ShowcaseSettings::GetShowcaseConfig()->CameraDwellSeconds;
	const float DemoAt = IdleThreshold + 5.f;
	switch (CaptureStep)
	{
	case 0: if (CaptureClock >= 6.f) { SetOverlayVisible(true); if (FParse::Param(FCommandLine::Get(), TEXT("ShowcaseBench"))) RunBenchmark(); ++CaptureStep; } break;
	case 1: if (CaptureClock >= 9.f) { Shot(TEXT("overlay_on")); ++CaptureStep; } break;
	case 2: if (CaptureClock >= 11.f) { SetOverlayVisible(false); ++CaptureStep; } break;
	case 3: if (CaptureClock >= 12.f) { Shot(TEXT("overlay_off")); ++CaptureStep; } break;
	case 4: if (CaptureClock >= DemoAt && bDemoActive) { Shot(TEXT("demo_a")); ++CaptureStep; } break;
	case 5: if (CaptureClock >= DemoAt + Dwell + 1.f) { Shot(TEXT("demo_b")); ++CaptureStep; } break;
	case 6: if (CaptureClock >= DemoAt + Dwell + 4.f)
		{
			UE_LOG(LogR1Showcase, Log, TEXT("Showcase: capture done (demo %d), exiting"), bDemoActive ? 1 : 0);
			++CaptureStep;
			FPlatformMisc::RequestExit(false, TEXT("ShowcaseCapture"));
		}
		break;
	default: break;
	}
}
