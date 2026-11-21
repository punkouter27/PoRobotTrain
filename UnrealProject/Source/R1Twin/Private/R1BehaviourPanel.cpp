#include "R1BehaviourPanel.h"
#include "R1Twin.h"
#include "R1ArticulationBridge.h"
#include "R1VoiceSapi.h"
#include "R1PanelStyle.h"
#include "R1WalkCourse.h"
#include "MjUeConvert.h"

#include "Blueprint/UserWidget.h"
#include "CanvasTypes.h"
#include "Math/ScaleMatrix.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "Misc/Paths.h"
#include "UnrealClient.h"
#include "Framework/Application/IInputProcessor.h"
#include "Framework/Application/SlateApplication.h"
#include "Engine/Canvas.h"
#include "Engine/Engine.h"
#include "Engine/Font.h"
#include "EngineUtils.h"
#include "GameFramework/PlayerController.h"
#include "HttpModule.h"
#include "Interfaces/IHttpResponse.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"
#if R1_WITH_URLAB && R1_URLAB_API_VERIFIED
#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Core/MjArticulation.h"
#include "MuJoCo/Components/Joints/MjJoint.h"
#include "MuJoCo/Components/Bodies/MjBody.h"
#endif

using namespace R1PanelStyle;

namespace
{
	const FLinearColor PanelBg(0.03f, 0.03f, 0.04f, 0.88f);
	const FLinearColor HeaderStripBg(0.05f, 0.06f, 0.08f, 0.96f);
	// Virtual = blue, real robot = orange, everywhere (section stripe, heading, main button).
	const FLinearColor RealCol(0.88f, 0.45f, 0.08f, 1.f);
	const FLinearColor RealSoftCol(0.5f, 0.3f, 0.1f, 1.f);
	const FLinearColor MonitorCol(0.18f, 0.48f, 0.54f, 1.f);
	const FLinearColor OffCol(0.24f, 0.24f, 0.26f, 1.f);
	const FName IdPreview(TEXT("preview")), IdCheck(TEXT("check")), IdMonitor(TEXT("monitor")), IdRun(TEXT("run")), IdStop(TEXT("stop"));
	const FName IdConfirm(TEXT("confirm")), IdCancel(TEXT("cancel")), IdDetails(TEXT("details")), IdJoints(TEXT("joints"));
	const FName IdAmp10(TEXT("amp10")), IdAmp20(TEXT("amp20")), IdAmp30(TEXT("amp30"));
	const FName IdTabBeh(TEXT("tab_behaviours")), IdTabRob(TEXT("tab_robustness")), IdTabWalk(TEXT("tab_walk"));
	const FName IdBehIds[] = {
		FName(TEXT("beh_neck_turn")),
		FName(TEXT("beh_neck_nod")),
		FName(TEXT("beh_arm_raise")),
		FName(TEXT("beh_hand_wave")),
	};
	constexpr float CheckValidS = 300.f;   // matches CHECK_VALID_S in behaviours/service.py

	/** Sees every left mouse press before Slate routes it; never consumes it (the camera keeps right-drag orbit). */
	class FPanelClickProcessor : public IInputProcessor
	{
	public:
		explicit FPanelClickProcessor(AR1BehaviourPanel* InPanel) : Panel(InPanel) {}
		virtual void Tick(const float, FSlateApplication&, TSharedRef<ICursor>) override {}
		virtual bool HandleMouseButtonDownEvent(FSlateApplication&, const FPointerEvent& E) override
		{
			if (E.GetEffectingButton() == EKeys::LeftMouseButton && Panel.IsValid())
			{
				Panel->QueueClick();
			}
			return false;
		}
		virtual bool HandleMouseWheelOrGestureEvent(FSlateApplication&, const FPointerEvent& E, const FPointerEvent*) override
		{
			if (Panel.IsValid())
			{
				Panel->QueueWheel(E.GetWheelDelta());   // Help tab scrolling; the camera still zooms
			}
			return false;
		}
		virtual const TCHAR* GetDebugName() const override { return TEXT("R1BehaviourPanelClicks"); }
	private:
		TWeakObjectPtr<AR1BehaviourPanel> Panel;
	};
}

AR1BehaviourPanel::AR1BehaviourPanel(FVTableHelper& Helper)
	: Super(Helper)
{
}

AR1BehaviourPanel::~AR1BehaviourPanel() = default;

AR1BehaviourPanel::AR1BehaviourPanel()
{
	PrimaryActorTick.bCanEverTick = true;
	PrimaryActorTick.bTickEvenWhenPaused = true;
	PushToTalkKey = EKeys::V;
	Behaviours = {
		{TEXT("neck_turn"), TEXT("Neck Turn"), TEXT("Turns head left/right. Only neck yaw motor moves."), TEXT("HEAD YAW"), 30.f},
		{TEXT("neck_nod"),  TEXT("Neck Nod"),  TEXT("Nods head up/down (max 20 deg). Only neck pitch moves."), TEXT("HEAD PITCH"), 20.f},
		{TEXT("arm_raise"), TEXT("Arm Raise"), TEXT("Gently raises/lowers right arm forward (elbow held)."), TEXT("R.SHOULDER"), 30.f},
		{TEXT("hand_wave"), TEXT("Hand Wave"), TEXT("Gentle stand-safe right-arm wave (max 20 deg)."), TEXT("R.ARM WAVE"), 20.f},
	};
	// Source of truth defaults/limits matching RobotControl/configs/r1_joint_map.yaml (updated from /status when online).
	JointMetas = {
		{TEXT("left_hip_pitch_joint"),       TEXT("L Hip Pitch"),    -168.0f, 146.0f,  -5.7f},
		{TEXT("left_hip_roll_joint"),        TEXT("L Hip Roll"),      -60.0f, 100.0f,   0.0f},
		{TEXT("left_hip_yaw_joint"),         TEXT("L Hip Yaw"),      -157.0f, 157.0f,   0.0f},
		{TEXT("left_knee_joint"),            TEXT("L Knee"),          -10.0f, 139.0f,  17.2f},
		{TEXT("left_ankle_pitch_joint"),     TEXT("L Ankle Pitch"),   -50.0f,  33.0f, -11.5f},
		{TEXT("left_ankle_roll_joint"),      TEXT("L Ankle Roll"),    -15.0f,  15.0f,   0.0f},
		{TEXT("right_hip_pitch_joint"),      TEXT("R Hip Pitch"),    -168.0f, 146.0f,  -5.7f},
		{TEXT("right_hip_roll_joint"),       TEXT("R Hip Roll"),     -100.0f,  60.0f,   0.0f},
		{TEXT("right_hip_yaw_joint"),        TEXT("R Hip Yaw"),      -157.0f, 157.0f,   0.0f},
		{TEXT("right_knee_joint"),           TEXT("R Knee"),          -10.0f, 139.0f,  17.2f},
		{TEXT("right_ankle_pitch_joint"),    TEXT("R Ankle Pitch"),   -50.0f,  33.0f, -11.5f},
		{TEXT("right_ankle_roll_joint"),     TEXT("R Ankle Roll"),    -15.0f,  15.0f,   0.0f},
		{TEXT("waist_roll_joint"),           TEXT("Waist Roll"),      -30.0f,  30.0f,   0.0f},
		{TEXT("waist_yaw_joint"),            TEXT("Waist Yaw"),      -150.0f, 150.0f,   0.0f},
		{TEXT("left_shoulder_pitch_joint"),  TEXT("L Shld Pitch"),   -180.0f, 120.0f,  20.1f},
		{TEXT("left_shoulder_roll_joint"),   TEXT("L Shld Roll"),     -13.0f, 142.0f,  10.3f},
		{TEXT("left_shoulder_yaw_joint"),    TEXT("L Shld Yaw"),     -110.0f, 110.0f,   0.0f},
		{TEXT("left_elbow_joint"),           TEXT("L Elbow"),         -55.9f, 125.2f,  49.8f},
		{TEXT("left_wrist_roll_joint"),      TEXT("L Wrist Roll"),   -110.0f, 110.0f,   0.0f},
		{TEXT("right_shoulder_pitch_joint"), TEXT("R Shld Pitch"),   -180.0f, 120.0f,  20.1f},
		{TEXT("right_shoulder_roll_joint"),  TEXT("R Shld Roll"),    -142.0f,  13.0f, -10.3f},
		{TEXT("right_shoulder_yaw_joint"),   TEXT("R Shld Yaw"),     -110.0f, 110.0f,   0.0f},
		{TEXT("right_elbow_joint"),          TEXT("R Elbow"),         -55.9f, 125.2f,  49.8f},
		{TEXT("right_wrist_roll_joint"),     TEXT("R Wrist Roll"),   -110.0f, 110.0f,   0.0f},
		{TEXT("head_pitch_joint"),           TEXT("Head Pitch"),      -36.0f,  36.0f,   0.0f},
		{TEXT("head_yaw_joint"),             TEXT("Head Yaw"),       -115.0f, 115.0f,   0.0f},
	};
}

void AR1BehaviourPanel::BeginPlay()
{
	Super::BeginPlay();
	if (APlayerController* PC = GetOwningPlayerController())
	{
		PC->bShowMouseCursor = true;
		PC->bEnableClickEvents = true;   // HUD hit boxes receive left clicks; orbit stays on the right button
	}
	if (FSlateApplication::IsInitialized())
	{
		ClickProcessor = MakeShared<FPanelClickProcessor>(this);
		FSlateApplication::Get().RegisterInputPreProcessor(ClickProcessor);
	}
	ResolveTwinJoints();

	FParse::Value(FCommandLine::Get(), TEXT("R1Service="), ServiceUrl);   // e.g. -R1Service=http://127.0.0.1:8766
	FString TabArg;
	if (FParse::Value(FCommandLine::Get(), TEXT("PanelTab="), TabArg))
	{
		auto Is = [&TabArg](const TCHAR* S) { return TabArg.Equals(S, ESearchCase::IgnoreCase); };
		SetTab(Is(TEXT("robustness")) ? ER1PanelTab::Robustness : Is(TEXT("walk")) ? ER1PanelTab::Walk
			: Is(TEXT("health")) ? ER1PanelTab::Health : Is(TEXT("motions")) ? ER1PanelTab::Motions
			: Is(TEXT("setup")) ? ER1PanelTab::Setup : Is(TEXT("help")) ? ER1PanelTab::Help : ER1PanelTab::Behaviours);
	}
	FString Dir;
	if (FParse::Value(FCommandLine::Get(), TEXT("PanelCapture="), Dir))
	{
		PanelCaptureDir = FPaths::ConvertRelativePathToFull(Dir);
		FParse::Value(FCommandLine::Get(), TEXT("PanelCaptureTag="), PanelCaptureTag);
		bSpeakReplies = false;   // unattended runs stay silent
	}
	if (FParse::Value(FCommandLine::Get(), TEXT("VoiceTest="), VoiceTestText))
	{
		VoiceTestText.ReplaceInline(TEXT("_"), TEXT(" "));   // -VoiceTest=nod_the_head_twenty_degrees
	}

	// Voice: one COM thread for SAPI recognition + speech; the microphone stays off until Mic / push-to-talk.
	Voice = MakeUnique<FR1VoiceSapi>();
	Voice->Start();
	UE_LOG(LogR1Twin, Log, TEXT("R1BehaviourPanel: service %s, behaviour %s, push-to-talk %s"), *ServiceUrl, *Behaviour, *PushToTalkKey.ToString());
}

void AR1BehaviourPanel::EndPlay(const EEndPlayReason::Type Reason)
{
	if (ClickProcessor.IsValid() && FSlateApplication::IsInitialized())
	{
		FSlateApplication::Get().UnregisterInputPreProcessor(ClickProcessor);
	}
	ClickProcessor.Reset();
	if (Voice)
	{
		Voice->Shutdown();
		Voice.Reset();
	}
	if (bWalkVisualsOn)
	{
		SetWalkRunningVisuals(false);
	}
	if (AR1WalkCourse* Course = CourseActor.Get())
	{
		Course->Destroy();
	}
	Super::EndPlay(Reason);
}

void AR1BehaviourPanel::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);
	const double Now = FPlatformTime::Seconds();
	// Runs even while demo mode hides the HUD: the walk watchdog, voice and course need it.
	TickVoice();
	TickRobustness(Now);
	TickWalk(Now);
	TickConsole(Now);
	TickPanelCapture(Now);
}

bool AR1BehaviourPanel::GetPanelMouse(float& OutX, float& OutY) const
{
	APlayerController* PC = GetOwningPlayerController();
	if (!PC || !PC->GetMousePosition(OutX, OutY))
	{
		return false;
	}
	const float S = FMath::Max(PanelScale, 0.1f);
	OutX /= S;
	OutY /= S;
	return true;
}

void AR1BehaviourPanel::SetTab(ER1PanelTab NewTab)
{
	Tab = NewTab;
	SelKind = SelRow = SelCol = -1;
	if (Tab == ER1PanelTab::Robustness)
	{
		NextRobustFetch = 0.0;   // refresh on entry
	}
	if (Tab == ER1PanelTab::Walk)
	{
		NextWalkPoll = 0.0;
	}
	ConsoleNextFetch.Reset();   // console tabs refresh on entry
	bCalibConfirm = false;
}

void AR1BehaviourPanel::PostJson(const FString& Path, const FString& Body, TFunction<void(int32, TSharedPtr<FJsonObject>)> OnDone)
{
	TSharedRef<IHttpRequest, ESPMode::ThreadSafe> Req = FHttpModule::Get().CreateRequest();
	Req->SetURL(ServiceUrl + Path);
	Req->SetVerb(TEXT("POST"));
	Req->SetHeader(TEXT("Content-Type"), TEXT("application/json"));
	Req->SetContentAsString(Body.IsEmpty() ? FString(TEXT("{}")) : Body);
	Req->SetTimeout(5.f);
	Req->OnProcessRequestComplete().BindWeakLambda(this, [OnDone = MoveTemp(OnDone)](FHttpRequestPtr, FHttpResponsePtr Resp, bool bOk)
	{
		const int32 Code = bOk && Resp.IsValid() ? Resp->GetResponseCode() : 0;
		TSharedPtr<FJsonObject> J;
		if (Code != 0)
		{
			FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Resp->GetContentAsString()), J);
		}
		if (OnDone)
		{
			OnDone(Code, J);
		}
	});
	Req->ProcessRequest();
}

void AR1BehaviourPanel::GetJson(const FString& Path, TFunction<void(int32, const FString&)> OnDone)
{
	TSharedRef<IHttpRequest, ESPMode::ThreadSafe> Req = FHttpModule::Get().CreateRequest();
	Req->SetURL(ServiceUrl + Path);
	Req->SetVerb(TEXT("GET"));
	Req->SetTimeout(3.f);
	Req->OnProcessRequestComplete().BindWeakLambda(this, [OnDone = MoveTemp(OnDone)](FHttpRequestPtr, FHttpResponsePtr Resp, bool bOk)
	{
		const int32 Code = bOk && Resp.IsValid() ? Resp->GetResponseCode() : 0;
		OnDone(Code, Code != 0 ? Resp->GetContentAsString() : FString());
	});
	Req->ProcessRequest();
}

void AR1BehaviourPanel::DrawWrapped(const FString& Text, float X, float& Y, float MaxW, const FLinearColor& Color, float Scale)
{
	UFont* Font = GEngine->GetMediumFont();
	TArray<FString> Words;
	Text.ParseIntoArray(Words, TEXT(" "));
	FString Line;
	for (const FString& Word : Words)
	{
		const FString Try = Line.IsEmpty() ? Word : Line + TEXT(" ") + Word;
		float W = 0.f, H = 0.f;
		GetTextSize(Try, W, H, Font, Scale);
		if (W > MaxW && !Line.IsEmpty())
		{
			DrawLine(Line, X, Y, Color, Scale);
			Line = Word;
		}
		else
		{
			Line = Try;
		}
	}
	if (!Line.IsEmpty())
	{
		DrawLine(Line, X, Y, Color, Scale);
	}
}

void AR1BehaviourPanel::CaptureShot(const TCHAR* Name)
{
	const FString File = FPaths::Combine(PanelCaptureDir, FString::Printf(TEXT("%s%s.png"), Name, *PanelCaptureTag));
	FScreenshotRequest::RequestScreenshot(File, /*bShowUI*/ true, /*bAddFilenameSuffix*/ false);
	UE_LOG(LogR1Twin, Log, TEXT("R1BehaviourPanel: capture %s"), *File);
}

void AR1BehaviourPanel::TickPanelCapture(double Now)
{
	// Unattended screenshots of each tab (docs: SCREENSHOTS/), using whatever service is at ServiceUrl.
	if (PanelCaptureDir.IsEmpty())
	{
		return;
	}
	if (CaptureStartAt < 0.0)
	{
		CaptureStartAt = Now;
	}
	const double T = Now - CaptureStartAt;
	switch (CaptureStep)
	{
	case 0: if (T > 10.0)
		{
			SetTab(ER1PanelTab::Behaviours);
			if (!VoiceTestText.IsEmpty())
			{
				// as if SAPI had heard it: exercises /assistant/interpret + ApplyIntent without a microphone
				HeardText = VoiceTestText;
				HeardAt = Now;
				SendInterpret(VoiceTestText);
			}
			++CaptureStep;
		}
		break;
	case 1: if (T > 12.0) { CaptureShot(TEXT("panel_behaviours")); ++CaptureStep; } break;
	case 2: if (T > 13.0) { SetTab(ER1PanelTab::Robustness); ++CaptureStep; } break;
	case 3: if (T > 16.0) { CaptureShot(TEXT("panel_robustness")); ++CaptureStep; } break;
	case 4: if (T > 17.0)
		{
			// pick a failing interaction cell so the details line shows up
			SelKind = Robust.Interaction.Num() > 0 ? 1 : -1;
			SelRow = Robust.Interaction.Num() - 1;
			SelCol = 0;
			++CaptureStep;
		}
		break;
	case 5: if (T > 18.0) { CaptureShot(TEXT("panel_robustness_cell")); ++CaptureStep; } break;
	case 6: if (T > 19.0) { SetTab(ER1PanelTab::Walk); ++CaptureStep; } break;
	case 7: if (T > 22.0) { CaptureShot(TEXT("panel_walk_idle")); ++CaptureStep; } break;
	case 8: if (T > 23.0)
		{
			PostJson(TEXT("/walk/start"), FString::Printf(TEXT("{\"course\":\"%s\",\"mode\":\"hold\"}"), *WalkCourse));
			++CaptureStep;
		}
		break;
	case 9: if (T > 31.0) { CaptureShot(TEXT("panel_walk_hold")); ++CaptureStep; } break;
	case 10: if (T > 32.0) { PostJson(TEXT("/walk/stop"), TEXT("{}")); ++CaptureStep; } break;
	// operator console tabs (docs/service_api.md sections 4-7)
	case 11: if (T > 34.0) { SetTab(ER1PanelTab::Health); ++CaptureStep; } break;
	case 12: if (T > 40.0) { CaptureShot(TEXT("panel_health")); ++CaptureStep; } break;
	case 13: if (T > 41.0) { SetTab(ER1PanelTab::Motions); ++CaptureStep; } break;
	case 14: if (T > 44.0) { CaptureShot(TEXT("panel_motions")); ++CaptureStep; } break;
	case 15: if (T > 45.0)
		{
			ConsolePost(TEXT("/poses/preview"), TEXT("{\"name\":\"look_left\",\"target\":\"urlab\"}"), TEXT("Pose preview started on the twin."));
			++CaptureStep;
		}
		break;
	case 16: if (T > 46.5) { CaptureShot(TEXT("panel_motions_pose")); ++CaptureStep; } break;
	case 17: if (T > 50.0) { SetTab(ER1PanelTab::Setup); ++CaptureStep; } break;
	case 18: if (T > 54.0) { CaptureShot(TEXT("panel_setup")); ++CaptureStep; } break;
	case 19: if (T > 55.0) { SetTab(ER1PanelTab::Help); OpenDoc(TEXT("service_api")); ++CaptureStep; } break;
	case 20: if (T > 58.0) { CaptureShot(TEXT("panel_help")); ++CaptureStep; } break;
	case 21: if (T > 59.0)
		{
			// the assistant: a status question is answered from the live service state (nothing changes)
			SetTab(ER1PanelTab::Health);
			HeardText = TEXT("how hot are the motors?");
			HeardAt = Now;
			SendInterpret(HeardText);
			++CaptureStep;
		}
		break;
	case 22: if (T > 62.0) { CaptureShot(TEXT("panel_chat_answer")); ++CaptureStep; } break;
	case 23: if (T > 63.0)
		{
			HeardText = TEXT("go to the arms out pose");
			HeardAt = Now;
			SendInterpret(HeardText);
			++CaptureStep;
		}
		break;
	case 24: if (T > 66.0) { CaptureShot(TEXT("panel_chat_pose")); ++CaptureStep; } break;
	case 25: if (T > 67.0)
		{
			++CaptureStep;
			UE_LOG(LogR1Twin, Log, TEXT("R1BehaviourPanel: capture done, exiting"));
			FPlatformMisc::RequestExit(false, TEXT("PanelCapture"));
		}
		break;
	default: break;
	}
}

void AR1BehaviourPanel::ResolveTwinJoints()
{
	bTwinResolved = true;
	TwinJointObjs.Reset();
	TwinRootBody.Reset();
#if R1_WITH_URLAB && R1_URLAB_API_VERIFIED
	AActor* ArtActor = R1Urlab::FindArticulationActor(GetWorld(), TEXT("r1_stand"));
	AMjArticulation* Art = Cast<AMjArticulation>(ArtActor);
	if (!Art)
	{
		return;
	}
	auto Matches = [](const FString& MjName, const FString& Want)
	{
		return MjName == Want || MjName.EndsWith(FString(TEXT("_")) + Want);
	};
	TwinJointObjs.SetNum(JointMetas.Num());
	for (int32 i = 0; i < JointMetas.Num(); ++i)
	{
		const FString Want = JointMetas[i].MjName.ToString();
		UMjJoint* Found = Art->GetJoint(Want);
		if (!Found)
		{
			for (UMjJoint* J : Art->GetJoints())
			{
				if (J && Matches(J->GetMjName(), Want))
				{
					Found = J;
					break;
				}
			}
		}
		TwinJointObjs[i] = Found;
	}
	for (UMjBody* B : Art->GetBodies())
	{
		if (B && Matches(B->GetMjName(), TEXT("pelvis")))
		{
			TwinRootBody = B;
			break;
		}
	}
#endif
}

bool AR1BehaviourPanel::SampleLiveTwin(FTelemetryReadout& Out) const
{
#if R1_WITH_URLAB && R1_URLAB_API_VERIFIED
	if (TwinJointObjs.Num() != JointMetas.Num())
	{
		return false;
	}
	Out.QDeg.SetNum(JointMetas.Num());
	int32 ValidCount = 0;
	for (int32 i = 0; i < JointMetas.Num(); ++i)
	{
		if (const UMjJoint* J = static_cast<const UMjJoint*>(TwinJointObjs[i].Get()))
		{
			Out.QDeg[i] = static_cast<float>(FMath::RadiansToDegrees(J->GetPosition()));
			++ValidCount;
		}
		else
		{
			Out.QDeg[i] = JointMetas[i].DefaultDeg;
		}
	}
	if (ValidCount == 0)
	{
		return false;
	}
	Out.bValid = true;
	Out.Source = TEXT("twin_live");
	Out.AgeS = 0.f;
	if (const UMjBody* Pelvis = static_cast<const UMjBody*>(TwinRootBody.Get()))
	{
		const FQuat Raw = MjUe::QuatToMj(Pelvis->GetWorldRotation());
		// MuJoCo raw wxyz packed in UE FQuat -> ZYX RPY degrees (same convention as base_interface.quat_to_rpy)
		const double W = Raw.W, X = Raw.X, Y = Raw.Y, Z = Raw.Z;
		const double SinR = 2.0 * (W * X + Y * Z), CosR = 1.0 - 2.0 * (X * X + Y * Y);
		const double SinP = FMath::Clamp(2.0 * (W * Y - Z * X), -1.0, 1.0);
		const double SinY = 2.0 * (W * Z + X * Y), CosY = 1.0 - 2.0 * (Y * Y + Z * Z);
		Out.bHasImu = true;
		Out.ImuRoll = static_cast<float>(FMath::RadiansToDegrees(FMath::Atan2(SinR, CosR)));
		Out.ImuPitch = static_cast<float>(FMath::RadiansToDegrees(FMath::Asin(SinP)));
		Out.ImuYaw = static_cast<float>(FMath::RadiansToDegrees(FMath::Atan2(SinY, CosY)));
	}
	return true;
#else
	return false;
#endif
}

void AR1BehaviourPanel::SetUrlabDebugVisible(bool bVisible)
{
#if R1_WITH_URLAB && R1_URLAB_API_VERIFIED
	for (TActorIterator<AAMjManager> It(GetWorld()); It; ++It)
	{
		if (UUserWidget* W = It->SimulateWidget)
		{
			W->SetVisibility(bVisible ? ESlateVisibility::Visible : ESlateVisibility::Collapsed);
			bUrlabDebugHidden = !bVisible;
		}
	}
#endif
}

const AR1BehaviourPanel::FBehaviourInfo* AR1BehaviourPanel::ActiveBehaviourInfo() const
{
	for (const FBehaviourInfo& Info : Behaviours)
	{
		if (Info.Id == Behaviour)
		{
			return &Info;
		}
	}
	return Behaviours.Num() > 0 ? &Behaviours[0] : nullptr;
}

FString AR1BehaviourPanel::ParamsJson(bool bConfirm) const
{
	return FString::Printf(TEXT("{\"behaviour\":\"%s\",\"amplitude\":%.1f,\"freq\":%.3f,\"duration\":%.1f%s}"),
		*Behaviour, AmplitudeDeg, FrequencyHz, DurationS, bConfirm ? TEXT(",\"confirm\":true") : TEXT(""));
}

void AR1BehaviourPanel::PollStatus()
{
	if (bPollInFlight)
	{
		return;
	}
	bPollInFlight = true;
	TSharedRef<IHttpRequest, ESPMode::ThreadSafe> Req = FHttpModule::Get().CreateRequest();
	Req->SetURL(ServiceUrl + TEXT("/status"));
	Req->SetVerb(TEXT("GET"));
	Req->SetTimeout(2.f);
	Req->OnProcessRequestComplete().BindUObject(this, &AR1BehaviourPanel::OnStatus);
	Req->ProcessRequest();
}

float AR1BehaviourPanel::ReadAmplitude(const TSharedPtr<FJsonObject>& J, const TCHAR* Field)
{
	const TSharedPtr<FJsonObject>* P = nullptr;
	double A = -1.0;
	if (J->TryGetObjectField(Field, P) && P && P->IsValid())
	{
		(*P)->TryGetNumberField(TEXT("amplitude"), A);
	}
	return static_cast<float>(A);
}

FString AR1BehaviourPanel::ReadBehaviour(const TSharedPtr<FJsonObject>& J, const TCHAR* Field)
{
	const TSharedPtr<FJsonObject>* P = nullptr;
	FString B;
	if (J->TryGetObjectField(Field, P) && P && P->IsValid())
	{
		(*P)->TryGetStringField(TEXT("behaviour"), B);
	}
	return B;
}

AR1BehaviourPanel::FJobResult AR1BehaviourPanel::ReadResult(const TSharedPtr<FJsonObject>& Results, const TCHAR* Job)
{
	FJobResult R;
	const TSharedPtr<FJsonObject>* P = nullptr;
	if (Results.IsValid() && Results->TryGetObjectField(Job, P) && P && P->IsValid())
	{
		R.bValid = true;
		(*P)->TryGetBoolField(TEXT("ok"), R.bOk);
		double A = -1.0;
		(*P)->TryGetNumberField(TEXT("amplitude"), A);
		R.Amplitude = static_cast<float>(A);
		(*P)->TryGetStringField(TEXT("reason"), R.Reason);
	}
	return R;
}

AR1BehaviourPanel::FTelemetryReadout AR1BehaviourPanel::ReadTelemetry(
	const TSharedPtr<FJsonObject>& Telemetry, const TSharedPtr<FJsonObject>& Head, const TCHAR* Who)
{
	FTelemetryReadout Out;
	const TSharedPtr<FJsonObject>* TP = nullptr;
	if (Telemetry.IsValid() && Telemetry->TryGetObjectField(Who, TP) && TP && TP->IsValid())
	{
		const TSharedPtr<FJsonObject>& Obj = *TP;
		double V = 0.0;
		if (Obj->TryGetNumberField(TEXT("actual_deg"), V))
		{
			Out.bValid = true;
			Out.ActualDeg = static_cast<float>(V);
		}
		if (Obj->TryGetNumberField(TEXT("target_deg"), V))
		{
			Out.bHasTarget = true;
			Out.TargetDeg = static_cast<float>(V);
		}
		if (Obj->TryGetNumberField(TEXT("other_max_deg"), V))
		{
			Out.OtherDeg = static_cast<float>(V);
		}
		if (Obj->TryGetNumberField(TEXT("age_s"), V))
		{
			Out.AgeS = static_cast<float>(V);
		}
		Obj->TryGetStringField(TEXT("primary_label"), Out.PrimaryLabel);
		Obj->TryGetStringField(TEXT("source"), Out.Source);
		Obj->TryGetStringField(TEXT("phase"), Out.Phase);
		Obj->TryGetStringField(TEXT("hot_joint"), Out.HotJoint);

		const TArray<TSharedPtr<FJsonValue>>* ImuArr = nullptr;
		if (Obj->TryGetArrayField(TEXT("imu_rpy_deg"), ImuArr) && ImuArr && ImuArr->Num() >= 3)
		{
			Out.bHasImu = true;
			Out.ImuRoll = static_cast<float>((*ImuArr)[0]->AsNumber());
			Out.ImuPitch = static_cast<float>((*ImuArr)[1]->AsNumber());
			Out.ImuYaw = static_cast<float>((*ImuArr)[2]->AsNumber());
		}
		int32 TempVal = 0;
		if (Obj->TryGetNumberField(TEXT("max_temp_c"), TempVal))
		{
			Out.bHasTemp = true;
			Out.MaxTempC = TempVal;
		}
		if (Obj->TryGetNumberField(TEXT("bus_v"), V))
		{
			Out.bHasBusV = true;
			Out.BusV = static_cast<float>(V);
		}
		int32 ActVal = 0;
		if (Obj->TryGetNumberField(TEXT("active_motors"), ActVal))
		{
			Out.bHasActive = true;
			Out.ActiveMotors = ActVal;
		}
		Obj->TryGetNumberField(TEXT("mode_machine"), Out.ModeMachine);

		const TArray<TSharedPtr<FJsonValue>>* QArr = nullptr;
		if (Obj->TryGetArrayField(TEXT("q_deg"), QArr) && QArr)
		{
			Out.bValid = true;
			for (const TSharedPtr<FJsonValue>& Item : *QArr)
			{
				Out.QDeg.Add(static_cast<float>(Item->AsNumber()));
			}
			if (Out.PrimaryLabel.IsEmpty() && Out.QDeg.Num() >= 26)
			{
				Out.PrimaryLabel = TEXT("HEAD YAW");
				Out.ActualDeg = Out.QDeg[25];
			}
		}
		const TArray<TSharedPtr<FJsonValue>>* TempArr = nullptr;
		if (Obj->TryGetArrayField(TEXT("temps_c"), TempArr) && TempArr)
		{
			for (const TSharedPtr<FJsonValue>& Item : *TempArr)
			{
				Out.TempsC.Add(static_cast<int32>(Item->AsNumber()));
			}
		}
	}

	// Fallback to legacy head field if telemetry hasn't populated primary angle yet
	const TSharedPtr<FJsonObject>* HP = nullptr;
	if (!Out.bValid && Head.IsValid() && Head->TryGetObjectField(Who, HP) && HP && HP->IsValid())
	{
		double V = 0.0;
		if ((*HP)->TryGetNumberField(TEXT("yaw"), V))
		{
			Out.bValid = true;
			Out.PrimaryLabel = TEXT("HEAD YAW");
			Out.ActualDeg = static_cast<float>(V);
		}
		if ((*HP)->TryGetNumberField(TEXT("target"), V))
		{
			Out.bHasTarget = true;
			Out.TargetDeg = static_cast<float>(V);
		}
		if ((*HP)->TryGetNumberField(TEXT("other"), V))
		{
			Out.OtherDeg = static_cast<float>(V);
		}
		if ((*HP)->TryGetNumberField(TEXT("age_s"), V))
		{
			Out.AgeS = static_cast<float>(V);
		}
		(*HP)->TryGetStringField(TEXT("source"), Out.Source);
		(*HP)->TryGetStringField(TEXT("phase"), Out.Phase);
	}
	return Out;
}

void AR1BehaviourPanel::OnStatus(FHttpRequestPtr, FHttpResponsePtr Resp, bool bOk)
{
	bPollInFlight = false;
	TSharedPtr<FJsonObject> J;
	if (!bOk || !Resp.IsValid() || Resp->GetResponseCode() != 200
		|| !FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Resp->GetContentAsString()), J) || !J.IsValid())
	{
		bServiceOnline = false;
		bCanRunRobot = false;
		return;
	}
	bServiceOnline = true;
	Job.Reset();
	J->TryGetStringField(TEXT("job"), Job);
	// {"progress": {"job": "robustness", "done": k, "total": n}} while a sweep runs, else null
	StatusProgressJob.Reset();
	ProgressDone = ProgressTotal = 0;
	const TSharedPtr<FJsonObject>* ProgObj = nullptr;
	if (J->TryGetObjectField(TEXT("progress"), ProgObj) && ProgObj && ProgObj->IsValid())
	{
		(*ProgObj)->TryGetStringField(TEXT("job"), StatusProgressJob);
		(*ProgObj)->TryGetNumberField(TEXT("done"), ProgressDone);
		(*ProgObj)->TryGetNumberField(TEXT("total"), ProgressTotal);
	}
	LastResult.Reset();
	J->TryGetStringField(TEXT("last_result"), LastResult);
	WhyNotRun.Reset();
	J->TryGetStringField(TEXT("why_not_run"), WhyNotRun);
	bCanRunRobot = false;
	J->TryGetBoolField(TEXT("can_run_robot"), bCanRunRobot);
	MirrorSamples = 0;
	J->TryGetNumberField(TEXT("mirror_samples"), MirrorSamples);
	PreviewedAmp = ReadAmplitude(J, TEXT("previewed"));
	CheckedAmp = ReadAmplitude(J, TEXT("checked"));
	PreviewedBehaviour = ReadBehaviour(J, TEXT("previewed"));
	CheckedBehaviour = ReadBehaviour(J, TEXT("checked"));
	double Age = -1.0;
	CheckAgeS = J->TryGetNumberField(TEXT("check_age_s"), Age) ? static_cast<float>(Age) : -1.f;
	const TSharedPtr<FJsonObject>* Results = nullptr;
	const TSharedPtr<FJsonObject> R = J->TryGetObjectField(TEXT("results"), Results) && Results ? *Results : nullptr;
	PreviewRes = ReadResult(R, TEXT("preview"));
	CheckRes = ReadResult(R, TEXT("check"));
	RunRes = ReadResult(R, TEXT("run"));
	MonitorRes = ReadResult(R, TEXT("monitor"));

	const TSharedPtr<FJsonObject>* HeadObj = nullptr;
	const TSharedPtr<FJsonObject> Head = J->TryGetObjectField(TEXT("head"), HeadObj) && HeadObj ? *HeadObj : nullptr;
	const TSharedPtr<FJsonObject>* TelObj = nullptr;
	const TSharedPtr<FJsonObject> Tel = J->TryGetObjectField(TEXT("telemetry"), TelObj) && TelObj ? *TelObj : nullptr;
	VirtualTel = ReadTelemetry(Tel, Head, TEXT("virtual"));
	RealTel = ReadTelemetry(Tel, Head, TEXT("real"));

	const TArray<TSharedPtr<FJsonValue>>* BehArr = nullptr;
	if (J->TryGetArrayField(TEXT("behaviours"), BehArr) && BehArr && BehArr->Num() > 0)
	{
		Behaviours.Reset();
		for (const TSharedPtr<FJsonValue>& V : *BehArr)
		{
			const TSharedPtr<FJsonObject>* BO = nullptr;
			if (V->TryGetObject(BO) && BO && BO->IsValid())
			{
				FBehaviourInfo Info;
				(*BO)->TryGetStringField(TEXT("id"), Info.Id);
				(*BO)->TryGetStringField(TEXT("label"), Info.Label);
				(*BO)->TryGetStringField(TEXT("desc"), Info.Desc);
				(*BO)->TryGetStringField(TEXT("primary_label"), Info.PrimaryLabel);
				double Md = 30.0;
				(*BO)->TryGetNumberField(TEXT("max_deg"), Md);
				Info.MaxDeg = static_cast<float>(Md);
				Behaviours.Add(Info);
			}
		}
	}

	const TArray<TSharedPtr<FJsonValue>>* LimArr = nullptr;
	if (J->TryGetArrayField(TEXT("joint_limits_deg"), LimArr) && LimArr && LimArr->Num() == JointMetas.Num())
	{
		for (int32 i = 0; i < JointMetas.Num(); ++i)
		{
			const TArray<TSharedPtr<FJsonValue>>* Pair = nullptr;
			if ((*LimArr)[i]->TryGetArray(Pair) && Pair && Pair->Num() >= 2)
			{
				JointMetas[i].MinDeg = static_cast<float>((*Pair)[0]->AsNumber());
				JointMetas[i].MaxDeg = static_cast<float>((*Pair)[1]->AsNumber());
			}
		}
	}
	const TArray<TSharedPtr<FJsonValue>>* DefArr = nullptr;
	if (J->TryGetArrayField(TEXT("joint_defaults_deg"), DefArr) && DefArr && DefArr->Num() == JointMetas.Num())
	{
		for (int32 i = 0; i < JointMetas.Num(); ++i)
		{
			JointMetas[i].DefaultDeg = static_cast<float>((*DefArr)[i]->AsNumber());
		}
	}

	Lines.Reset();
	const TArray<TSharedPtr<FJsonValue>>* Arr = nullptr;
	if (J->TryGetArrayField(TEXT("lines"), Arr))
	{
		for (const TSharedPtr<FJsonValue>& V : *Arr)
		{
			Lines.Add(V->AsString());
		}
	}
	if (ProgressTotal <= 0 && Job == TEXT("robustness"))
	{
		R1Panel::ParseProgress(Lines, ProgressDone, ProgressTotal);   // older service: "@progress k/n" lines
	}
}

void AR1BehaviourPanel::Post(const FString& Path, bool bConfirm)
{
	TSharedRef<IHttpRequest, ESPMode::ThreadSafe> Req = FHttpModule::Get().CreateRequest();
	Req->SetURL(ServiceUrl + Path);
	Req->SetVerb(TEXT("POST"));
	Req->SetHeader(TEXT("Content-Type"), TEXT("application/json"));
	const bool bEmptyBody = (Path == TEXT("/stop") || Path == TEXT("/monitor"));
	Req->SetContentAsString(bEmptyBody ? FString(TEXT("{}")) : ParamsJson(bConfirm));
	Req->SetTimeout(5.f);
	Req->OnProcessRequestComplete().BindUObject(this, &AR1BehaviourPanel::OnPost);
	Req->ProcessRequest();
	NextPoll = 0.0;   // refresh status right away
}

void AR1BehaviourPanel::OnPost(FHttpRequestPtr, FHttpResponsePtr Resp, bool bOk)
{
	LastError.Reset();
	TSharedPtr<FJsonObject> J;
	if (!bOk || !Resp.IsValid())
	{
		LastError = TEXT("behaviour service not reachable");
		return;
	}
	if (FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Resp->GetContentAsString()), J) && J.IsValid())
	{
		bool bReqOk = false;
		J->TryGetBoolField(TEXT("ok"), bReqOk);
		if (!bReqOk)
		{
			J->TryGetStringField(TEXT("error"), LastError);
		}
	}
}

void AR1BehaviourPanel::ShowToast(const FString& Text, const FLinearColor& Color)
{
	Toast = Text;
	ToastColor = Color;
	ToastAt = FPlatformTime::Seconds();
}

void AR1BehaviourPanel::HandleClick(FName BoxName)
{
	PressedId = BoxName;
	PressedAt = FPlatformTime::Seconds();
	if (const FString* Why = DisabledWhy.Find(BoxName))
	{
		const FString* Label = ButtonLabel.Find(BoxName);
		ShowToast(FString::Printf(TEXT("\"%s\" is locked: %s"), Label ? **Label : *BoxName.ToString(), **Why), WarnCol);
		return;
	}
	const bool bIdle = bServiceOnline && Job.IsEmpty() && !WalkState.bRunning;
	if (BoxName == IdStop)
	{
		bConfirmOpen = false;
		Post(TEXT("/stop"));
		VoiceWalkCmd = FVector::ZeroVector;
		ShowToast(TEXT("STOP sent."), BadCol);
	}
	else if (BoxName == IdEStop)
	{
		bConfirmOpen = false;
		SendEStop();
	}
	else if (!bConfirmOpen && (BoxName == IdTabBeh || BoxName == IdTabRob || BoxName == IdTabWalk || BoxName == IdTabHealth
		|| BoxName == IdTabMotions || BoxName == IdTabSetup || BoxName == IdTabHelp))
	{
		SetTab(BoxName == IdTabRob ? ER1PanelTab::Robustness : BoxName == IdTabWalk ? ER1PanelTab::Walk
			: BoxName == IdTabHealth ? ER1PanelTab::Health : BoxName == IdTabMotions ? ER1PanelTab::Motions
			: BoxName == IdTabSetup ? ER1PanelTab::Setup : BoxName == IdTabHelp ? ER1PanelTab::Help : ER1PanelTab::Behaviours);
	}
	else if (!bConfirmOpen && HandleConsoleClick(BoxName))
	{
	}
	else if (!bConfirmOpen && HandleVoiceClick(BoxName))
	{
	}
	else if (!bConfirmOpen && Tab == ER1PanelTab::Robustness && HandleRobustClick(BoxName))
	{
	}
	else if (!bConfirmOpen && Tab == ER1PanelTab::Walk && HandleWalkClick(BoxName))
	{
	}
	else if (BoxName == IdDetails)
	{
		bShowDetails = !bShowDetails;
		if (bShowDetails)
		{
			bShowJoints = false;   // URLab's right-side ACTUATORS/JOINTS drawer occupies the right edge
		}
		SetUrlabDebugVisible(bShowDetails);
	}
	else if (BoxName == IdJoints)
	{
		bShowJoints = !bShowJoints;
		if (bShowJoints && bShowDetails)
		{
			bShowDetails = false;
			SetUrlabDebugVisible(false);
		}
	}
	else if (bConfirmOpen)
	{
		if (BoxName == IdConfirm && bCanRunRobot && bIdle)
		{
			Post(TEXT("/run"), /*bConfirm=*/true);
			ShowToast(TEXT("Starting the REAL robot..."), RealCol * 1.2f);
		}
		else
		{
			ShowToast(TEXT("Cancelled. The real robot did not move."), DimCol);
		}
		bConfirmOpen = false;
	}
	else if (BoxName == IdPreview && bIdle)
	{
		Post(TEXT("/preview"));
		ShowToast(TEXT("Starting the VIRTUAL robot..."), VirtualCol * 1.35f);
	}
	else if (BoxName == IdCheck && bIdle)
	{
		Post(TEXT("/check"));
		ShowToast(TEXT("Checking the robot connection (read-only)..."), RealCol * 1.2f);
	}
	else if (BoxName == IdMonitor && bIdle)
	{
		Post(TEXT("/monitor"));
		ShowToast(TEXT("Starting read-only live monitor + twin mirror..."), MonitorCol * 1.6f);
	}
	else if (BoxName == IdRun && bIdle && bCanRunRobot)
	{
		bConfirmOpen = true;
		ConfirmOpenedAt = FPlatformTime::Seconds();
	}
	else if (bIdle && (BoxName == IdAmp10 || BoxName == IdAmp20 || BoxName == IdAmp30))
	{
		AmplitudeDeg = BoxName == IdAmp10 ? 10.f : BoxName == IdAmp20 ? 20.f : 30.f;
		ShowToast(FString::Printf(TEXT("Size set to %.0f deg."), AmplitudeDeg), TextCol);
	}
	else if (bIdle)
	{
		for (int32 i = 0; i < Behaviours.Num() && i < UE_ARRAY_COUNT(IdBehIds); ++i)
		{
			if (BoxName == IdBehIds[i])
			{
				Behaviour = Behaviours[i].Id;
				ShowToast(FString::Printf(TEXT("Behaviour set to %s."), *Behaviours[i].Label), TextCol);
				break;
			}
		}
	}
}

void AR1BehaviourPanel::DrawButton(const FButton& B)
{
	const double Now = FPlatformTime::Seconds();
	const float Flash = B.Id == PressedId ? FMath::Clamp(1.f - static_cast<float>(Now - PressedAt) / 0.5f, 0.f, 1.f) : 0.f;
	bool bHover = false;
	{
		float Mx = 0.f, My = 0.f;
		if (GetPanelMouse(Mx, My))
		{
			bHover = Mx >= B.Pos.X && Mx <= B.Pos.X + B.Size.X && My >= B.Pos.Y && My <= B.Pos.Y + B.Size.Y;
		}
	}
	FLinearColor Fill = B.bEnabled ? B.Color : OffCol;
	if (B.bEnabled && bHover)
	{
		Fill = FMath::Lerp(Fill, FLinearColor::White, 0.18f);
	}
	if (Flash > 0.f)
	{
		Fill = FMath::Lerp(Fill, FLinearColor::White, 0.55f * Flash);
	}
	const FVector2D Off = Flash > 0.5f ? FVector2D(0.f, 2.f) : FVector2D::ZeroVector;
	DrawRect(Fill, B.Pos.X + Off.X, B.Pos.Y + Off.Y, B.Size.X, B.Size.Y);
	if (Flash > 0.f || (B.bEnabled && bHover))
	{
		const FLinearColor Edge(1.f, 1.f, 1.f, Flash > 0.f ? 1.f : 0.6f);
		const float T = Flash > 0.f ? 3.f : 2.f;
		const float X = B.Pos.X + Off.X, Y = B.Pos.Y + Off.Y;
		DrawRect(Edge, X, Y, B.Size.X, T);
		DrawRect(Edge, X, Y + B.Size.Y - T, B.Size.X, T);
		DrawRect(Edge, X, Y, T, B.Size.Y);
		DrawRect(Edge, X + B.Size.X - T, Y, T, B.Size.Y);
	}
	float W = 0.f, H = 0.f;
	UFont* Font = GEngine->GetMediumFont();
	const float Scale = B.bLarge ? 1.28f : 0.95f;
	GetTextSize(B.Label, W, H, Font, Scale);
	const bool bDarkText = Fill.GetLuminance() > 0.6f;
	const FLinearColor Col = !B.bEnabled && Flash <= 0.f ? DimCol : bDarkText ? FLinearColor(0.05f, 0.05f, 0.06f, 1.f) : TextCol;
	DrawText(B.Label, Col, B.Pos.X + Off.X + (B.Size.X - W) * 0.5f, B.Pos.Y + Off.Y + (B.Size.Y - H) * 0.5f, Font, Scale);

	NextRects.Emplace(B.Id, FBox2D(B.Pos, B.Pos + B.Size));
	ButtonLabel.Add(B.Id, B.Label);
	if (B.bEnabled)
	{
		DisabledWhy.Remove(B.Id);
	}
	else
	{
		FString Why;
		if (!bServiceOnline)
		{
			Why = TEXT("the behaviour service is not running");
		}
		else if (WalkState.bRunning && Job.IsEmpty())
		{
			Why = TEXT("the walk sandbox is running (Stop walk on the Walk tab, or STOP)");
		}
		else if (!Job.IsEmpty())
		{
			Why = Job == TEXT("preview") ? TEXT("the virtual robot is still moving (STOP cancels it)")
				: Job == TEXT("check") ? TEXT("the robot check is still running")
				: Job == TEXT("monitor") ? TEXT("live monitor is running (STOP cancels it)")
				: Job == TEXT("snapshot") ? TEXT("the start-up robot snapshot is running (a few seconds)")
				: Job == TEXT("robustness") ? TEXT("a robustness sweep is running (STOP cancels it)")
				: Job == TEXT("walk") ? TEXT("the walk sandbox is running (Stop walk on the Walk tab, or STOP)")
				: Job == TEXT("run") ? TEXT("the real robot is moving (STOP cancels it)")
				: Job == TEXT("motion") ? TEXT("a motion is playing on the twin (STOP cancels it)")
				: Job == TEXT("pose") ? TEXT("a pose preview is running on the twin (STOP cancels it)")
				: Job == TEXT("mode") ? TEXT("a mode is active on the twin (Off on the Motions tab, or STOP)")
				: FString::Printf(TEXT("the service is busy (%s)"), *Job);
		}
		else if (B.Id == IdRun || B.Id == IdConfirm)
		{
			Why = RunLockedWhy;
		}
		DisabledWhy.Add(B.Id, Why.IsEmpty() ? FString(TEXT("not available right now")) : Why);
	}
}

void AR1BehaviourPanel::DrawLine(const FString& Text, float X, float& Y, const FLinearColor& Color, float Scale)
{
	UFont* Font = GEngine->GetMediumFont();
	DrawText(Text, Color, X, Y, Font, Scale);
	float W = 0.f, H = 0.f;
	GetTextSize(Text, W, H, Font, Scale);
	Y += H + 3.f;
}

void AR1BehaviourPanel::DrawTelemetry(const FTelemetryReadout& T, bool bLive, const FString& NoData,
	const FLinearColor& Accent, float X, float W, float& Y)
{
	if (!T.bValid)
	{
		DrawLine(FString::Printf(TEXT("JOINT  %s"), *NoData), X, Y, DimCol, 0.88f);
		return;
	}
	const FString& Label = T.PrimaryLabel.IsEmpty() ? FString(TEXT("JOINT")) : T.PrimaryLabel;
	FString S = FString::Printf(TEXT("%s  %+.1f deg"), *Label, T.ActualDeg);
	if (T.bHasTarget && bLive && T.Source != TEXT("check") && T.Source != TEXT("monitor"))
	{
		S += FString::Printf(TEXT("   target %+.1f"), T.TargetDeg);
	}
	const FString When = bLive ? FString(TEXT("live"))
		: T.AgeS < 90.f ? FString::Printf(TEXT("%.0f s ago"), T.AgeS) : FString::Printf(TEXT("%.0f min ago"), T.AgeS / 60.f);
	const float RowY = Y;
	DrawLine(S, X, Y, TextCol, 0.88f);

	const float GW = 140.f, GH = 8.f, GX = X + W - GW - 26.f, GY = RowY + 7.f, Range = 30.f;
	auto ToX = [&](float Deg) { return GX + (FMath::Clamp(Deg, -Range, Range) + Range) / (2.f * Range) * GW; };
	DrawRect(FLinearColor(1.f, 1.f, 1.f, 0.12f), GX, GY, GW, GH);
	DrawRect(FLinearColor(1.f, 1.f, 1.f, 0.35f), ToX(0.f) - 1.f, GY - 3.f, 2.f, GH + 6.f);
	if (T.bHasTarget && bLive && T.Source != TEXT("check") && T.Source != TEXT("monitor"))
	{
		DrawRect(Accent, ToX(T.TargetDeg) - 2.f, GY - 5.f, 4.f, GH + 10.f);
	}
	DrawRect(bLive ? FLinearColor::White : DimCol, ToX(T.ActualDeg) - 5.f, GY - 2.f, 10.f, GH + 4.f);
	DrawText(TEXT("-30"), DimCol, GX - 22.f, GY - 5.f, GEngine->GetSmallFont());
	DrawText(TEXT("+30"), DimCol, GX + GW + 4.f, GY - 5.f, GEngine->GetSmallFont());

	FString Health = FString::Printf(TEXT("%s   drift %.1f deg"), *When, T.OtherDeg);
	if (T.bHasImu)
	{
		Health += FString::Printf(TEXT("   IMU %+.0f/%+.0f/%+.0f"), T.ImuRoll, T.ImuPitch, T.ImuYaw);
	}
	if (T.bHasTemp)
	{
		Health += FString::Printf(TEXT("   Temp %dC"), T.MaxTempC);
	}
	if (T.bHasBusV)
	{
		Health += FString::Printf(TEXT("   %.1fV"), T.BusV);
	}
	if (T.bHasActive)
	{
		Health += FString::Printf(TEXT("   %d/26 on"), T.ActiveMotors);
	}
	FLinearColor HealthCol = bLive ? OkCol : DimCol;
	if (T.bHasTemp && T.MaxTempC >= 65)
	{
		HealthCol = BadCol;
	}
	else if (T.bHasTemp && T.MaxTempC >= 50)
	{
		HealthCol = WarnCol;
	}
	DrawLine(Health, X, Y, HealthCol, 0.78f);
}

void AR1BehaviourPanel::DrawJointMatrix(float X, float Y, float W)
{
	if (!bTwinResolved)
	{
		ResolveTwinJoints();
	}
	FTelemetryReadout TwinLive;
	const bool bHaveTwinLive = SampleLiveTwin(TwinLive);

	const bool bRealActive = RealTel.bValid && RealTel.QDeg.Num() >= 26
		&& (Job == TEXT("monitor") || Job == TEXT("run") || Job == TEXT("check") || RealTel.AgeS < 15.f);
	const bool bVirtualSvcActive = VirtualTel.bValid && VirtualTel.QDeg.Num() >= 26
		&& (Job == TEXT("preview") || !bHaveTwinLive);

	const FTelemetryReadout* TelPtr = bRealActive ? &RealTel : bVirtualSvcActive ? &VirtualTel : bHaveTwinLive ? &TwinLive : nullptr;
	const bool bIsReal = bRealActive;
	const FLinearColor Accent = bIsReal ? RealCol : VirtualCol;
	const FString BadgeText = bIsReal
		? ((Job == TEXT("monitor") || Job == TEXT("run")) && RealTel.AgeS < 2.f ? TEXT("REAL LIVE") : TEXT("REAL SNAPSHOT"))
		: (Job == TEXT("preview") ? TEXT("VIRTUAL PREVIEW") : TEXT("LIVE TWIN"));

	const float H = 706.f;
	const float Pad = 14.f;
	DrawRect(PanelBg, X, Y, W, H);
	DrawRect(HeaderStripBg, X, Y, W, 56.f);
	DrawRect(Accent, X, Y, W, 4.f);
	DrawRect(DividerCol, X, Y + 56.f, W, 1.f);

	UFont* Font = GEngine->GetMediumFont();
	UFont* SmallFont = GEngine->GetSmallFont();
	DrawText(TEXT("R1 JOINT TELEMETRY  (26 DOF)"), TextCol, X + Pad, Y + 12.f, Font, 1.05f);

	// Status pill badge on top right
	float BW = 0.f, BH = 0.f;
	GetTextSize(BadgeText, BW, BH, Font, 0.8f);
	const float PillW = BW + 18.f, PillH = 22.f;
	const float PillX = X + W - Pad - PillW, PillY = Y + 10.f;
	DrawRect(bIsReal ? RealCol : Job == TEXT("preview") ? VirtualCol : MonitorCol, PillX, PillY, PillW, PillH);
	DrawText(BadgeText, TextCol, PillX + 9.f, PillY + 3.f, Font, 0.8f);

	if (!TelPtr || TelPtr->QDeg.Num() < 26)
	{
		DrawText(TEXT("Waiting for URLab articulation or RobotControl telemetry..."), DimCol, X + Pad, Y + 72.f, Font, 0.9f);
		return;
	}
	const FTelemetryReadout& Tel = *TelPtr;

	// Metrics summary strip inside header
	FString Metrics = Tel.bHasImu
		? FString::Printf(TEXT("IMU R/P/Y  %+5.1f / %+5.1f / %+5.1f deg"), Tel.ImuRoll, Tel.ImuPitch, Tel.ImuYaw)
		: TEXT("IMU  --");
	if (bIsReal)
	{
		Metrics += FString::Printf(TEXT("   |   Mode %d"), Tel.ModeMachine);
		if (Tel.bHasActive)
		{
			Metrics += FString::Printf(TEXT("   |   Motors %d/26"), Tel.ActiveMotors);
		}
		if (Tel.bHasTemp)
		{
			Metrics += FString::Printf(TEXT("   |   Max Temp %d C (%s)"), Tel.MaxTempC, *Tel.HotJoint);
		}
		if (Tel.bHasBusV)
		{
			Metrics += FString::Printf(TEXT("   |   Bus %.1f V"), Tel.BusV);
		}
	}
	else
	{
		Metrics += TEXT("   |   MuJoCo / URLab 60 fps readout   |   Limits from r1_joint_map.yaml");
	}
	DrawText(Metrics, DimCol, X + Pad, Y + 35.f, Font, 0.78f);

	// Two columns of grouped sub-cards
	struct FGroupCard
	{
		const TCHAR* Name;
		int32 Start;
		int32 Count;
		int32 Col;
		FLinearColor TagCol;
	};
	static const FGroupCard Groups[] = {
		{TEXT("HEAD"),       24, 2, 0, CyanCol},
		{TEXT("WAIST"),      12, 2, 0, FLinearColor(0.65f, 0.55f, 0.95f, 1.f)},
		{TEXT("LEFT ARM"),   14, 5, 0, VirtualCol * 1.35f},
		{TEXT("RIGHT ARM"),  19, 5, 0, OkCol},
		{TEXT("LEFT LEG"),    0, 6, 1, WarnCol},
		{TEXT("RIGHT LEG"),   6, 6, 1, RealCol * 1.15f},
	};

	const float ColGap = 12.f;
	const float ColW = (W - 2.f * Pad - ColGap) * 0.5f;
	float ColY[2] = {Y + 68.f, Y + 68.f};
	const float ColX[2] = {X + Pad, X + Pad + ColW + ColGap};
	constexpr float RowH = 27.f;
	constexpr float CardHdrH = 26.f;

	for (const FGroupCard& G : Groups)
	{
		const float CX = ColX[G.Col];
		const float CY = ColY[G.Col];
		const float CH = CardHdrH + G.Count * RowH + 8.f;
		DrawRect(CardBg, CX, CY, ColW, CH);
		DrawRect(G.TagCol, CX, CY, 3.f, CH);
		DrawRect(DividerCol, CX, CY + CardHdrH - 2.f, ColW, 1.f);
		DrawText(G.Name, G.TagCol, CX + 10.f, CY + 5.f, Font, 0.85f);

		// Column mini-headers
		DrawText(TEXT("RANGE"), DimCol, CX + 126.f, CY + 6.f, SmallFont, 0.85f);
		DrawText(TEXT("ANGLE"), DimCol, CX + ColW - 78.f, CY + 6.f, SmallFont, 0.85f);
		if (bIsReal)
		{
			DrawText(TEXT("T"), DimCol, CX + ColW - 22.f, CY + 6.f, SmallFont, 0.85f);
		}

		for (int32 i = 0; i < G.Count; ++i)
		{
			const int32 Idx = G.Start + i;
			const FJointMeta& Meta = JointMetas[Idx];
			const float ValDeg = Tel.QDeg[Idx];
			const float RY = CY + CardHdrH + i * RowH;
			if (Meta.MjName == HighlightJoint && FPlatformTime::Seconds() - HighlightAt < 8.0)
			{
				// voice asked about this joint: point at its row
				DrawRect(FLinearColor(0.39f, 0.40f, 0.95f, 0.35f), CX + 4.f, RY + 1.f, ColW - 8.f, RowH - 2.f);
				DrawRect(FLinearColor(0.39f, 0.40f, 0.95f, 1.f), CX + 4.f, RY + 1.f, 3.f, RowH - 2.f);
			}
			if (i > 0)
			{
				DrawRect(FLinearColor(1.f, 1.f, 1.f, 0.03f), CX + 8.f, RY, ColW - 14.f, 1.f);
			}

			// 1. Clean joint name
			DrawText(Meta.Label, TextCol, CX + 10.f, RY + 5.f, Font, 0.78f);

			// 2. Normalized limit bar [MinDeg .. MaxDeg] with default tick
			const float BarX = CX + 104.f;
			const float BarW = ColW - 190.f;
			const float BarY = RY + 10.f;
			const float BarH = 7.f;
			const float Span = FMath::Max(1.f, Meta.MaxDeg - Meta.MinDeg);
			const float NormVal = FMath::Clamp((ValDeg - Meta.MinDeg) / Span, 0.f, 1.f);
			const float NormDef = FMath::Clamp((Meta.DefaultDeg - Meta.MinDeg) / Span, 0.f, 1.f);

			const float Margin = FMath::Min(NormVal, 1.f - NormVal);
			const float DegMargin = FMath::Min(ValDeg - Meta.MinDeg, Meta.MaxDeg - ValDeg);
			const FLinearColor BarCol = (Margin < 0.03f && DegMargin < 4.f) ? BadCol
				: (Margin < 0.07f && DegMargin < 8.f) ? WarnCol
				: bIsReal ? RealCol * 1.15f : CyanCol;

			DrawRect(FLinearColor(1.f, 1.f, 1.f, 0.10f), BarX, BarY, BarW, BarH);
			// Fill from default tick to current value
			const float X0 = BarX + NormDef * BarW;
			const float X1 = BarX + NormVal * BarW;
			const float FillX = FMath::Min(X0, X1);
			const float FillW = FMath::Max(2.f, FMath::Abs(X1 - X0));
			DrawRect(FLinearColor(BarCol.R, BarCol.G, BarCol.B, 0.55f), FillX, BarY, FillW, BarH);
			// Default home tick
			DrawRect(FLinearColor(1.f, 1.f, 1.f, 0.45f), X0 - 1.f, BarY - 2.f, 2.f, BarH + 4.f);
			// Current position marker
			DrawRect(BarCol, X1 - 2.f, BarY - 2.f, 4.f, BarH + 4.f);

			// 3. Right-aligned angle readout
			const FString AngStr = FString::Printf(TEXT("%+6.1f°"), ValDeg);
			float AW = 0.f, AH = 0.f;
			GetTextSize(AngStr, AW, AH, Font, 0.78f);
			const float AngRight = bIsReal ? (CX + ColW - 36.f) : (CX + ColW - 10.f);
			DrawText(AngStr, (Margin < 0.07f && DegMargin < 8.f) ? BarCol : TextCol, AngRight - AW, RY + 5.f, Font, 0.78f);

			// 4. Real-robot motor temperature badge
			if (bIsReal && Tel.TempsC.IsValidIndex(Idx))
			{
				const int32 Tc = Tel.TempsC[Idx];
				const FLinearColor TC = Tc >= 65 ? BadCol : Tc >= 50 ? WarnCol : DimCol;
				DrawText(FString::Printf(TEXT("%d°"), Tc), TC, CX + ColW - 30.f, RY + 5.f, Font, 0.75f);
			}
		}
		ColY[G.Col] += CH + 10.f;
	}

	// Compact legend card below Right Leg in column 1
	const float LegX = ColX[1], LegY = ColY[1], LegH = (Y + H - 12.f) - LegY;
	if (LegH >= 48.f)
	{
		DrawRect(CardBg, LegX, LegY, ColW, LegH);
		float LY = LegY + 10.f;
		DrawLine(TEXT("LEGEND & LIMITS"), LegX + 10.f, LY, DimCol, 0.8f);
		DrawLine(TEXT("Bar = joint position inside r1_joint_map limits"), LegX + 10.f, LY, DimCol, 0.74f);
		DrawLine(TEXT("White tick = default home   |   Amber/Red = near limit"), LegX + 10.f, LY, DimCol, 0.74f);
	}
}

void AR1BehaviourPanel::DrawStep(bool bDone, const FString& Text, float X, float& Y)
{
	const float S = 12.f, Yc = Y + 4.f;
	if (bDone)
	{
		DrawRect(OkCol, X, Yc, S, S);
	}
	else
	{
		DrawRect(DimCol, X, Yc, S, 2.f);
		DrawRect(DimCol, X, Yc + S - 2.f, S, 2.f);
		DrawRect(DimCol, X, Yc, 2.f, S);
		DrawRect(DimCol, X + S - 2.f, Yc, 2.f, S);
	}
	DrawLine(Text, X + S + 8.f, Y, bDone ? TextCol : DimCol, 0.92f);
}

void AR1BehaviourPanel::DrawTabStrip(float X0, float X, float Inner, float PanelW, float& Y)
{
	// title + voice controls
	DrawText(TEXT("R1 STAND EXPLORE"), TextCol, X, Y + 2.f, GEngine->GetMediumFont(), 1.4f);
	const bool bRecoOk = Voice && Voice->IsRecognizerReady();
	const bool bListening = Voice && Voice->IsListening();
	const FString MicLabel = bListening ? TEXT("Listening...") : bMicOn ? TEXT("Mic: on") : TEXT("Mic: off");
	const float MuteW = 92.f, MicW = 104.f, Gap = 6.f, EStopW = 96.f;
	const float MuteX = X0 + PanelW - 18.f - MuteW;
	DrawButton({IdMute, bSpeakReplies ? TEXT("Voice: on") : TEXT("Voice: muted"), {MuteX, Y}, {MuteW, 28.f}, true,
		bSpeakReplies ? ChipCol : OffCol});
	DrawButton({IdMic, MicLabel, {MuteX - Gap - MicW, Y}, {MicW, 28.f}, bRecoOk || !Voice,
		bListening ? CyanCol * 0.8f : bMicOn ? ChipOnCol : ChipCol});
	// E-STOP on every tab (End key): stops every job, the walk sandbox and the twin player
	DrawEStop(MuteX - 2.f * Gap - MicW - EStopW, Y, EStopW, 28.f);
	if (!bRecoOk)
	{
		DisabledWhy.Add(IdMic, Voice && !Voice->GetRecognizerError().IsEmpty() ? Voice->GetRecognizerError() : FString(TEXT("speech recogniser is starting")));
	}
	Y += 36.f;

	// tabs
	struct FTabDef { FName Id; const TCHAR* Label; ER1PanelTab T; };
	const FTabDef Tabs[] = {
		{IdTabBeh, TEXT("Behaviours"), ER1PanelTab::Behaviours},
		{IdTabRob, TEXT("Robustness"), ER1PanelTab::Robustness},
		{IdTabWalk, WalkState.bRunning ? TEXT("Walk (running)") : TEXT("Walk (sim)"), ER1PanelTab::Walk},
		{IdTabHealth, TEXT("Health"), ER1PanelTab::Health},
		{IdTabMotions, TEXT("Motions"), ER1PanelTab::Motions},
		{IdTabSetup, TEXT("Setup"), ER1PanelTab::Setup},
		{IdTabHelp, TEXT("Help"), ER1PanelTab::Help},
	};
	// two rows: the original three tabs, then the operator console tabs
	const float TabW3 = (Inner - 2.f * Gap) / 3.f, TabW4 = (Inner - 3.f * Gap) / 4.f;
	for (int32 i = 0; i < UE_ARRAY_COUNT(Tabs); ++i)
	{
		const bool bSel = Tab == Tabs[i].T;
		const bool bRow2 = i >= 3;
		const float TW = bRow2 ? TabW4 : TabW3;
		const int32 Col = bRow2 ? i - 3 : i;
		DrawButton({Tabs[i].Id, Tabs[i].Label, {X + Col * (TW + Gap), Y + (bRow2 ? 34.f : 0.f)}, {TW, bRow2 ? 28.f : 30.f}, true,
			bSel ? ChipOnCol : ChipCol});
	}
	Y += 72.f;
	DrawVoiceLine(X, Inner, Y);
	DrawPendingPose(X, Inner, Y);
	DrawRect(DividerCol, X0, Y + 2.f, PanelW, 1.f);
	Y += 10.f;
}

void AR1BehaviourPanel::DrawHUD()
{
	Super::DrawHUD();
	if (!Canvas)
	{
		return;
	}
	{
		// version stamp bottom-right: UTC yyMMdd of the HEAD commit (R1Twin.Build.cs), the same stamp as the web page
		const FString Stamp = FString::Printf(TEXT("v%s"), TEXT(R1_BUILD_VERSION));
		float TW = 0.f, TH = 0.f;
		Canvas->TextSize(GEngine->GetSmallFont(), Stamp, TW, TH);
		Canvas->SetDrawColor(FColor(154, 163, 181));
		Canvas->DrawText(GEngine->GetSmallFont(), Stamp, Canvas->ClipX - TW - 12.f, Canvas->ClipY - TH - 10.f);
	}

	// The panel is designed at 640 px wide; narrower or shorter windows (portrait 450x800) draw it scaled down.
	const float Pad = 18.f, PanelW = 600.f;
	const float X0 = 20.f, Y0 = 20.f;
	PanelScale = FMath::Clamp(FMath::Min((Canvas->ClipX - 8.f) / (X0 + PanelW + 8.f), (Canvas->ClipY - 8.f) / (Y0 + LastPanelH + 30.f)), 0.4f, 1.f);
	ViewW = Canvas->ClipX / PanelScale;
	ViewH = Canvas->ClipY / PanelScale;
	const bool bScaled = PanelScale < 0.999f && Canvas->Canvas;
	if (bScaled)
	{
		Canvas->Canvas->PushRelativeTransform(FScaleMatrix(FVector(PanelScale, PanelScale, 1.f)));
	}

	const double Now = FPlatformTime::Seconds();
	if (bClickQueued)
	{
		bClickQueued = false;
		float Mx = 0.f, My = 0.f;
		if (GetPanelMouse(Mx, My))
		{
			const FVector2D M(Mx, My);
			for (int32 i = ButtonRects.Num() - 1; i >= 0; --i)
			{
				if (ButtonRects[i].Value.IsInside(M))
				{
					UE_LOG(LogR1Twin, Log, TEXT("R1BehaviourPanel: click %s"), *ButtonRects[i].Key.ToString());
					HandleClick(ButtonRects[i].Key);
					break;
				}
			}
		}
	}
	NextRects.Reset();
	if (Now >= NextPoll)
	{
		PollStatus();
		NextPoll = Now + 0.4;
	}
	if (!bShowDetails && !bUrlabDebugHidden)
	{
		SetUrlabDebugVisible(false);
	}
	if (APlayerController* PC = GetOwningPlayerController())
	{
		PC->bEnableClickEvents = true;
		if (!PC->bShowMouseCursor)
		{
			PC->bShowMouseCursor = true;
			FInputModeGameAndUI Mode;
			Mode.SetLockMouseToViewportBehavior(EMouseLockMode::DoNotLock);
			Mode.SetHideCursorDuringCapture(false);
			PC->SetInputMode(Mode);
		}
	}
	if (bConfirmOpen && Now - ConfirmOpenedAt > 15.0)
	{
		bConfirmOpen = false;
	}

	const bool bIdle = bServiceOnline && Job.IsEmpty() && !WalkState.bRunning;
	const float Amp = AmplitudeDeg;
	const FBehaviourInfo* Info = ActiveBehaviourInfo();
	const float EffAmp = Info ? FMath::Min(Amp, Info->MaxDeg) : Amp;
	const bool bPreviewedThis = PreviewedAmp > 0.f && FMath::IsNearlyEqual(PreviewedAmp, Amp) && PreviewedBehaviour == Behaviour;
	const bool bCheckedThis = CheckedAmp > 0.f && FMath::IsNearlyEqual(CheckedAmp, Amp)
		&& CheckedBehaviour == Behaviour && CheckAgeS >= 0.f && CheckAgeS < CheckValidS;
	const bool bRobotReady = bIdle && bCanRunRobot && bPreviewedThis && bCheckedThis;

	const float X = X0 + Pad, Inner = PanelW - 2.f * Pad;
	DrawRect(PanelBg, X0, Y0, PanelW, LastPanelH);
	float Y = Y0 + Pad - 4.f;

	DrawTabStrip(X0, X, Inner, PanelW, Y);

	if (Tab == ER1PanelTab::Behaviours)
	{
		// ---------------------------------------------------------------- size chips + behaviour selector
		const float ChipW = 58.f, ChipH = 28.f, ChipX = X0 + PanelW - Pad - 3.f * (ChipW + 6.f) + 6.f;
		DrawText(TEXT("Behaviour"), DimCol, X, Y + 5.f, GEngine->GetMediumFont(), 0.9f);
		DrawText(TEXT("Size"), DimCol, ChipX - 42.f, Y + 5.f, GEngine->GetMediumFont(), 0.9f);
		for (int32 i = 0; i < 3; ++i)
		{
			const float A = 10.f * (i + 1);
			const FName Id = i == 0 ? IdAmp10 : i == 1 ? IdAmp20 : IdAmp30;
			const bool bSel = FMath::IsNearlyEqual(Amp, A);
			DrawButton({Id, FString::Printf(TEXT("%.0f deg"), A), {ChipX + i * (ChipW + 6.f), Y}, {ChipW, ChipH},
				bIdle || bSel, bSel ? ChipOnCol : ChipCol});
		}
		Y += 34.f;

		const int32 NumBeh = FMath::Min(Behaviours.Num(), static_cast<int32>(UE_ARRAY_COUNT(IdBehIds)));
		const float BehGap = 6.f;
		const float BehW = NumBeh > 0 ? (Inner - (NumBeh - 1) * BehGap) / NumBeh : Inner;
		for (int32 i = 0; i < NumBeh; ++i)
		{
			const bool bSel = Behaviours[i].Id == Behaviour;
			DrawButton({IdBehIds[i], Behaviours[i].Label, {X + i * (BehW + BehGap), Y}, {BehW, 28.f},
				bIdle || bSel, bSel ? ChipOnCol : ChipCol});
		}
		Y += 34.f;
		const FString Desc = Info ? Info->Desc : TEXT("Stand-safe upper-body behaviour.");
		DrawLine(FString::Printf(TEXT("%s  (%.0f deg)"), *Desc, EffAmp), X, Y, DimCol, 0.88f);
		Y += 8.f;

		// ---------------------------------------------------------------- 1. virtual robot (blue)
		{
			const float SecY = Y;
			DrawRect(VirtualCol, X0, SecY, 6.f, 134.f);
			DrawLine(TEXT("VIRTUAL ROBOT  (on this screen)"), X, Y, VirtualCol * 1.35f, 1.05f);
			Y += 4.f;
			DrawButton({IdPreview, Job == TEXT("preview") ? TEXT("Moving the virtual robot...") : TEXT("Move VIRTUAL robot"),
				{X, Y}, {Inner, 44.f}, bIdle, VirtualCol, true});
			Y += 50.f;
			FString S;
			FLinearColor C = DimCol;
			if (Job == TEXT("preview"))
			{
				S = TEXT("Moving the virtual robot...");
				C = WarnCol;
			}
			else if (PreviewRes.bValid && !PreviewRes.bOk)
			{
				S = FString::Printf(TEXT("Stopped: %s"), *PreviewRes.Reason);
				C = BadCol;
			}
			else if (bPreviewedThis)
			{
				S = FString::Printf(TEXT("Done - virtual %s ran cleanly (%.0f deg)."), Info ? *Info->Label : *Behaviour, EffAmp);
				C = OkCol;
			}
			else
			{
				S = TEXT("Safe: nothing real moves. Watch the robot on screen.");
			}
			DrawLine(S, X, Y, C, 0.92f);
			Y += 3.f;
			DrawTelemetry(VirtualTel, Job == TEXT("preview") && VirtualTel.AgeS < 2.f,
				TEXT("no reading yet - move the virtual robot"), VirtualCol * 1.35f, X, Inner, Y);
			Y = SecY + 144.f;
		}

		// ---------------------------------------------------------------- 2. real robot (orange)
		{
			const float SecY = Y;
			DrawRect(RealCol, X0, SecY, 6.f, 166.f);
			DrawLine(TEXT("REAL ROBOT  (Unitree R1)"), X, Y, RealCol * 1.2f, 1.05f);
			Y += 4.f;
			const float CheckW = 144.f, MonW = 136.f, Gap = 8.f;
			RunLockedWhy = !bPreviewedThis ? FString::Printf(TEXT("do step 1 first - preview %s at %.0f deg"), Info ? *Info->Label : *Behaviour, Amp)
				: !bCheckedThis ? FString(TEXT("do step 2 first - Check connection"))
				: FString(TEXT("not ready yet"));
			DrawButton({IdCheck, Job == TEXT("check") ? TEXT("Checking...") : TEXT("Check connection"),
				{X, Y}, {CheckW, 44.f}, bIdle, RealSoftCol});
			DrawButton({IdMonitor, Job == TEXT("monitor") ? TEXT("Monitoring...") : TEXT("Monitor (Mirror)"),
				{X + CheckW + Gap, Y}, {MonW, 44.f}, bIdle, MonitorCol});
			DrawButton({IdRun, Job == TEXT("run") ? TEXT("REAL robot moving...") : TEXT("Move REAL robot"),
				{X + CheckW + MonW + 2.f * Gap, Y}, {Inner - CheckW - MonW - 2.f * Gap, 44.f}, bRobotReady, RealCol, true});
			Y += 52.f;
			DrawStep(bPreviewedThis, FString::Printf(TEXT("Step 1: preview %s (%.0f deg) on virtual robot"), Info ? *Info->Label : *Behaviour, Amp), X, Y);
			const FString CheckText = bCheckedThis
				? FString::Printf(TEXT("Step 2: robot connected & ready (%.0f s ago)"), CheckAgeS)
				: TEXT("Step 2: check connection (read-only, nothing moves)");
			DrawStep(bCheckedThis, CheckText, X, Y);
			Y += 2.f;
			FString S;
			FLinearColor C = DimCol;
			if (Job == TEXT("check"))
			{
				S = TEXT("Checking the robot (read-only)...");
				C = WarnCol;
			}
			else if (Job == TEXT("monitor"))
			{
				S = FString::Printf(TEXT("MONITOR (read-only) - live telemetry & twin mirror (%d samples)"), MirrorSamples);
				C = OkCol;
			}
			else if (Job == TEXT("run"))
			{
				S = FString::Printf(TEXT("REAL ROBOT MOVING - screen mirrors the robot (%d samples)"), MirrorSamples);
				C = RealCol * 1.2f;
			}
			else if (CheckRes.bValid && !CheckRes.bOk && !bCheckedThis)
			{
				S = FString::Printf(TEXT("Check failed: %s"), *CheckRes.Reason);
				C = BadCol;
			}
			else if (MonitorRes.bValid && !MonitorRes.bOk && LastResult.StartsWith(TEXT("monitor")))
			{
				S = FString::Printf(TEXT("Monitor failed: %s"), *MonitorRes.Reason);
				C = BadCol;
			}
			else if (RunRes.bValid && !RunRes.bOk)
			{
				S = FString::Printf(TEXT("Last robot run stopped: %s"), *RunRes.Reason);
				C = BadCol;
			}
			else if (bRobotReady)
			{
				S = TEXT("Ready. Confirmation required before any motor moves.");
				C = OkCol;
			}
			else if (RunRes.bValid && RunRes.bOk)
			{
				S = TEXT("Last robot run finished. Check again before the next one.");
				C = OkCol;
			}
			else
			{
				S = TEXT("Locked until steps 1 and 2 are done (Monitor is always read-only).");
			}
			DrawLine(S, X, Y, C, 0.92f);
			Y += 3.f;
			DrawTelemetry(RealTel, (Job == TEXT("check") || Job == TEXT("run") || Job == TEXT("monitor")) && RealTel.AgeS < 2.f,
				TEXT("no reading yet - Check or Monitor reads it (nothing moves)"), RealCol * 1.2f, X, Inner, Y);
			Y = SecY + 176.f;
		}
	}
	else if (Tab == ER1PanelTab::Robustness)
	{
		DrawRobustnessTab(X, Inner, Y);
	}
	else if (Tab == ER1PanelTab::Health)
	{
		DrawHealthTab(X, Inner, Y);
	}
	else if (Tab == ER1PanelTab::Motions)
	{
		DrawMotionsTab(X, Inner, Y);
	}
	else if (Tab == ER1PanelTab::Setup)
	{
		DrawSetupTab(X, Inner, Y);
	}
	else if (Tab == ER1PanelTab::Help)
	{
		DrawHelpTab(X, Inner, Y);
	}
	else
	{
		DrawWalkTab(X, Inner, Y);
	}

	// ---------------------------------------------------------------- stop + service state (every tab)
	Y += 4.f;
	DrawButton({IdStop, TEXT("STOP"), {X, Y}, {108.f, 40.f}, bServiceOnline, StopCol, true});
	{
		float Ty = Y + 2.f;
		DrawLine(TEXT("Emergency stop on robot:"), X + 118.f, Ty, WarnCol, 0.85f);
		DrawLine(TEXT("remote L2 + B, hold 5 s"), X + 118.f, Ty, WarnCol, 0.85f);
	}
	const bool bRoomForJoints = ViewW >= X0 + PanelW + 20.f + 540.f;
	DrawButton({IdJoints, bShowJoints ? TEXT("Hide joints") : TEXT("Joints (26)"), {X0 + PanelW - Pad - 220.f, Y}, {106.f, 40.f},
		bRoomForJoints, bShowJoints && bRoomForJoints ? ChipOnCol : ChipCol});
	if (!bRoomForJoints)
	{
		DisabledWhy.Add(IdJoints, TEXT("the joint view needs a wider window"));
	}
	DrawButton({IdDetails, bShowDetails ? TEXT("Hide details") : TEXT("Details"), {X0 + PanelW - Pad - 106.f, Y}, {106.f, 40.f},
		true, ChipCol});
	Y += 50.f;
	{
		const double Age = FPlatformTime::Seconds() - ToastAt;
		if (Age < 4.0 && !Toast.IsEmpty())
		{
			const float Alpha = Age < 3.0 ? 1.f : static_cast<float>(4.0 - Age);
			FLinearColor C = ToastColor;
			C.A = Alpha;
			const float ToastY = Y;
			float Ty = Y;
			DrawWrapped(Toast, X, Ty, Inner, C, 1.f);
			DrawRect(FLinearColor(1.f, 1.f, 1.f, 0.08f * Alpha), X - 6.f, ToastY - 4.f, Inner + 12.f, Ty - ToastY + 6.f);
			Y = Ty + 4.f;
		}
	}
	if (!bServiceOnline)
	{
		DrawWrapped(FString::Printf(TEXT("Not connected (%s). Start:  python -m robotcontrol.behaviours.service"), *ServiceUrl), X, Y, Inner, BadCol, 0.9f);
	}
	else if (!LastError.IsEmpty())
	{
		DrawWrapped(FString::Printf(TEXT("Refused: %s"), *LastError), X, Y, Inner, BadCol, 0.9f);
	}

	if (bShowJoints && bRoomForJoints)
	{
		const float RightW = FMath::Clamp(ViewW - (X0 + PanelW + 300.f), 540.f, 650.f);
		const float RightX = FMath::Max(X0 + PanelW + 20.f, ViewW - RightW - 20.f);
		DrawJointMatrix(RightX, Y0, RightW);
	}
	if (bShowDetails)
	{
		Y += 4.f;
		const int32 First = FMath::Max(0, Lines.Num() - 7);
		for (int32 i = First; i < Lines.Num(); ++i)
		{
			DrawLine(Lines[i].Left(92), X, Y, DimCol, 0.75f);
		}
	}
	LastPanelH = FMath::Max(200.f, Y - Y0 + 8.f);

	// ---------------------------------------------------------------- confirmation
	if (bConfirmOpen)
	{
		const float CW = 600.f, CH = 210.f;
		const float CX = (ViewW - CW) * 0.5f, CY = (ViewH - CH) * 0.5f;
		DrawRect(FLinearColor(0.f, 0.f, 0.f, 0.55f), 0.f, 0.f, ViewW, ViewH);
		DrawRect(FLinearColor(0.07f, 0.04f, 0.02f, 0.98f), CX, CY, CW, CH);
		DrawRect(RealCol, CX, CY, CW, 6.f);
		float CYl = CY + 20.f;
		DrawLine(TEXT("Move the REAL robot?"), CX + 20.f, CYl, RealCol * 1.2f, 1.4f);
		CYl += 6.f;
		DrawLine(FString::Printf(TEXT("Behaviour: %s (%.0f deg). Unpowered joints stay disabled."),
			Info ? *Info->Label : *Behaviour, EffAmp), CX + 20.f, CYl, TextCol);
		DrawLine(TEXT("Robot on its stand, clear workspace, remote in your hand?"), CX + 20.f, CYl, TextCol);
		DrawButton({IdConfirm, TEXT("Yes, move the real robot"), {CX + 20.f, CY + CH - 58.f}, {300.f, 42.f}, bRobotReady, RealCol, true});
		DrawButton({IdCancel, TEXT("Cancel"), {CX + 334.f, CY + CH - 58.f}, {120.f, 42.f}, true, ChipCol});
	}
	ButtonRects = MoveTemp(NextRects);
	if (bScaled)
	{
		Canvas->Canvas->PopTransform();
	}
}
