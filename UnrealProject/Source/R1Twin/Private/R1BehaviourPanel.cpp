#include "R1BehaviourPanel.h"
#include "R1Twin.h"
#include "R1ArticulationBridge.h"
#include "R1PanelStyle.h"
#include "R1Showcase.h"
#include "R1SkillBrain.h"
#include "R1WalkCourse.h"
#include "MjUeConvert.h"

#include "Blueprint/UserWidget.h"
#include "CanvasItem.h"
#include "CanvasTypes.h"
#include "Engine/TextureRenderTarget2D.h"
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
	const FLinearColor OffCol(0.24f, 0.24f, 0.26f, 1.f);
	const FName IdStop(TEXT("stop")), IdDetails(TEXT("details")), IdJoints(TEXT("joints"));
	const FName IdTabLab(TEXT("tab_lab")), IdTabWalk(TEXT("tab_walk"));

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
		virtual const TCHAR* GetDebugName() const override { return TEXT("R1BehaviourPanelClicks"); }
	private:
		TWeakObjectPtr<AR1BehaviourPanel> Panel;
	};
}

AR1BehaviourPanel::AR1BehaviourPanel()
{
	PrimaryActorTick.bCanEverTick = true;
	PrimaryActorTick.bTickEvenWhenPaused = true;
	// Source of truth defaults/limits: RobotControl/configs/r1_joint_map.yaml
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

	FParse::Value(FCommandLine::Get(), TEXT("R1Service="), ServiceUrl);
	FString TabArg;
	if (FParse::Value(FCommandLine::Get(), TEXT("PanelTab="), TabArg))
	{
		SetTab(TabArg.Equals(TEXT("walk"), ESearchCase::IgnoreCase) ? ER1PanelTab::Walk : ER1PanelTab::Lab);
	}
	FString Dir;
	if (FParse::Value(FCommandLine::Get(), TEXT("PanelCapture="), Dir))
	{
		PanelCaptureDir = FPaths::ConvertRelativePathToFull(Dir);
	}
	UE_LOG(LogR1Twin, Log, TEXT("R1BehaviourPanel: scenario lab (simulation only); walk service %s"), *ServiceUrl);
}

void AR1BehaviourPanel::EndPlay(const EEndPlayReason::Type Reason)
{
	if (ClickProcessor.IsValid() && FSlateApplication::IsInitialized())
	{
		FSlateApplication::Get().UnregisterInputPreProcessor(ClickProcessor);
	}
	ClickProcessor.Reset();
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
	TickWalk(Now);
	TickLab(Now);
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
	if (Tab == ER1PanelTab::Walk)
	{
		NextWalkPoll = 0.0;
	}
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

void AR1BehaviourPanel::DrawSection(const FString& Title, float X, float Inner, float& Y, const FLinearColor& Accent)
{
	Y += 4.f;
	DrawRect(Accent, X, Y + 3.f, 3.f, 14.f);
	DrawText(Title, Accent, X + 9.f, Y, GEngine->GetMediumFont(), 0.86f);
	DrawRect(DividerCol, X + 9.f, Y + 21.f, Inner - 9.f, 1.f);
	Y += 26.f;
}

void AR1BehaviourPanel::CaptureShot(const TCHAR* Name)
{
	const FString File = FPaths::Combine(PanelCaptureDir, FString::Printf(TEXT("%s.png"), Name));
	FScreenshotRequest::RequestScreenshot(File, /*bShowUI*/ true, /*bAddFilenameSuffix*/ false);
	UE_LOG(LogR1Twin, Log, TEXT("R1BehaviourPanel: capture %s"), *File);
}

void AR1BehaviourPanel::TickPanelCapture(double Now)
{
	// Unattended scenario run with screenshots (SCREENSHOTS/): the trained brains in the lab, then the walk tab.
	if (PanelCaptureDir.IsEmpty())
	{
		return;
	}
	UR1SkillBrainComponent* B = FindBrain();
	if (UR1ShowcaseSubsystem* Showcase = UR1ShowcaseSubsystem::Get(this))
	{
		Showcase->NotifyUserInput();   // a scripted run is activity: keep the idle demo mode away
	}
	if (CaptureStartAt < 0.0)
	{
		CaptureStartAt = Now;
	}
	const double T = Now - CaptureStartAt;
	FString Why;
	auto Skill = [B](const TCHAR* Id) { return B ? B->FindSkill(Id) : INDEX_NONE; };
	switch (CaptureStep)
	{
	case 0: if (T > 8.0)
		{
			SetTab(ER1PanelTab::Lab);
			bExperiment = false;   // Watch first (not saved: the capture never calls SaveConfig)
			if (B)
			{
				LabSkill = Skill(TEXT("look_at"));
				LabSizeDeg = 20.f;
				B->SetTargetMode(ER1TargetMode::Orbit);
				B->Play(LabSkill, 20.f, 30.f, Why);
			}
			++CaptureStep;
		}
		break;
	case 1: if (T > 15.0) { CaptureShot(TEXT("lab_watch")); ++CaptureStep; } break;
	case 2: if (T > 16.0) { bExperiment = true; if (B) B->Push(FVector2D(1.f, 0.f)); ++CaptureStep; } break;
	case 3: if (T > 16.25) { CaptureShot(TEXT("lab_push")); ++CaptureStep; } break;
	case 4: if (T > 19.0)
		{
			if (B)
			{
				B->Stop();
				B->SetTargetMode(ER1TargetMode::Script);
				B->Timeline = {{Skill(TEXT("hold_centre")), 10.f, 3.f}, {Skill(TEXT("nod_yes")), 15.f, 4.f},
					{Skill(TEXT("shake_no")), 20.f, 6.f}, {Skill(TEXT("glance_around")), 25.f, 12.f}};
				B->PlayTimeline(Why);
			}
			++CaptureStep;
		}
		break;
	case 5: if (T > 30.0) { CaptureShot(TEXT("lab_timeline")); ++CaptureStep; } break;
	case 6: if (T > 31.0)
		{
			if (B)
			{
				B->Stop();
				B->DelaySteps = 4;
				B->NoiseDeg = 2.f;
				B->StrengthPct = 50.f;
				LabSkill = Skill(TEXT("shake_no"));
				B->Play(LabSkill, 20.f, 12.f, Why);
			}
			++CaptureStep;
		}
		break;
	case 7: if (T > 39.0) { CaptureShot(TEXT("lab_disturbed")); ++CaptureStep; } break;
	case 8: if (T > 40.0)
		{
			if (B)
			{
				B->Stop();
				B->DelaySteps = 0;
				B->NoiseDeg = 0.f;
				B->StrengthPct = 100.f;
			}
			bShowJoints = true;
			++CaptureStep;
		}
		break;
	case 9: if (T > 42.0) { CaptureShot(TEXT("lab_joints")); bShowJoints = false; SetTab(ER1PanelTab::Walk); ++CaptureStep; } break;
	case 10: if (T > 45.0) { CaptureShot(TEXT("walk_tab")); ++CaptureStep; } break;
	case 11: if (T > 46.0)
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
	AMjArticulation* Art = Cast<AMjArticulation>(R1Urlab::FindArticulationActor(GetWorld(), TEXT("r1_stand")));
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

bool AR1BehaviourPanel::SampleLiveTwin(FTwinReadout& Out) const
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
	if (const UMjBody* Pelvis = static_cast<const UMjBody*>(TwinRootBody.Get()))
	{
		const FQuat Raw = MjUe::QuatToMj(Pelvis->GetWorldRotation());
		const double W = Raw.W, X = Raw.X, Y = Raw.Y, Z = Raw.Z;
		const double SinR = 2.0 * (W * X + Y * Z), CosR = 1.0 - 2.0 * (X * X + Y * Y);
		const double SinP = FMath::Clamp(2.0 * (W * Y - Z * X), -1.0, 1.0);
		const double SinY = 2.0 * (W * Z + X * Y), CosY = 1.0 - 2.0 * (Y * Y + Z * Z);
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

void AR1BehaviourPanel::OnStatus(FHttpRequestPtr, FHttpResponsePtr Resp, bool bOk)
{
	// The lab runs without the service; the walk tab needs it (it steps the Level 2 sandbox).
	bPollInFlight = false;
	TSharedPtr<FJsonObject> J;
	if (!bOk || !Resp.IsValid() || Resp->GetResponseCode() != 200
		|| !FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Resp->GetContentAsString()), J) || !J.IsValid())
	{
		bServiceOnline = false;
		Job.Reset();
		return;
	}
	bServiceOnline = true;
	Job.Reset();
	J->TryGetStringField(TEXT("job"), Job);
}

void AR1BehaviourPanel::ShowToast(const FString& Text, const FLinearColor& Color)
{
	Toast = Text;
	ToastColor = Color;
	ToastAt = FPlatformTime::Seconds();
}

void AR1BehaviourPanel::StopAll()
{
	if (UR1SkillBrainComponent* B = FindBrain())
	{
		B->Stop();
	}
	if (WalkState.bRunning)
	{
		PostJson(TEXT("/walk/stop"), TEXT("{}"));
	}
	ShowToast(TEXT("STOP: brain stopped (and the walk sandbox, if it was running)."), BadCol);
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
	if (BoxName == IdStop)
	{
		StopAll();
	}
	else if (BoxName == IdTabLab || BoxName == IdTabWalk)
	{
		SetTab(BoxName == IdTabWalk ? ER1PanelTab::Walk : ER1PanelTab::Lab);
	}
	else if (BoxName == IdDetails)
	{
		bShowDetails = !bShowDetails;
		SetUrlabDebugVisible(bShowDetails);
	}
	else if (BoxName == IdJoints)
	{
		bShowJoints = !bShowJoints;
	}
	else if (Tab == ER1PanelTab::Walk && HandleWalkClick(BoxName))
	{
	}
	else if (Tab == ER1PanelTab::Lab)
	{
		HandleLabClick(BoxName);
	}
}

void AR1BehaviourPanel::DrawButton(const FButton& B)
{
	float Mx = -1.f, My = -1.f;
	GetPanelMouse(Mx, My);
	const bool bHover = Mx >= B.Pos.X && Mx <= B.Pos.X + B.Size.X && My >= B.Pos.Y && My <= B.Pos.Y + B.Size.Y;
	const float Flash = PressedId == B.Id ? FMath::Clamp(1.f - static_cast<float>(FPlatformTime::Seconds() - PressedAt) / 0.35f, 0.f, 1.f) : 0.f;
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
	const float Scale = B.bLarge ? 1.2f : 0.9f;
	GetTextSize(B.Label, W, H, Font, Scale);
	const bool bDarkText = Fill.GetLuminance() > 0.6f;
	const FLinearColor Col = !B.bEnabled && Flash <= 0.f ? DimCol : bDarkText ? DarkInk : TextCol;
	DrawText(B.Label, Col, B.Pos.X + Off.X + (B.Size.X - W) * 0.5f, B.Pos.Y + Off.Y + (B.Size.Y - H) * 0.5f, Font, Scale);

	NextRects.Emplace(B.Id, FBox2D(B.Pos, B.Pos + B.Size));
	ButtonLabel.Add(B.Id, B.Label);
	if (B.bEnabled)
	{
		DisabledWhy.Remove(B.Id);
	}
	else if (!DisabledWhy.Contains(B.Id))
	{
		DisabledWhy.Add(B.Id, TEXT("not available right now"));
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

void AR1BehaviourPanel::DrawHeadCamera(float X, float Y, float W)
{
	const float H = W * 0.75f;
	DrawRect(PanelBg, X - 8.f, Y - 8.f, W + 16.f, H + 46.f);
	DrawRect(CyanCol, X - 8.f, Y - 8.f, W + 16.f, 3.f);
	DrawText(TEXT("HEAD CAMERA  (head_rgbd: 58 deg FOV, 20 deg down)"), TextCol, X, Y, GEngine->GetMediumFont(), 0.82f);
	const float ImgY = Y + 24.f;
	UR1SkillBrainComponent* B = FindBrain();
	UTextureRenderTarget2D* RT = B ? B->GetHeadCamTexture() : nullptr;
	if (RT && RT->GetResource() && Canvas)
	{
		FCanvasTileItem Tile(FVector2D(X, ImgY), RT->GetResource(), FVector2D(W, H), FLinearColor::White);
		Tile.BlendMode = SE_BLEND_Opaque;
		Canvas->DrawItem(Tile);
		// crosshair: where the camera points; a centred ball means the brain is looking at it
		DrawRect(FLinearColor(1.f, 1.f, 1.f, 0.6f), X + W * 0.5f - 10.f, ImgY + H * 0.5f - 1.f, 20.f, 2.f);
		DrawRect(FLinearColor(1.f, 1.f, 1.f, 0.6f), X + W * 0.5f - 1.f, ImgY + H * 0.5f - 10.f, 2.f, 20.f);
	}
	else
	{
		DrawRect(CardBg, X, ImgY, W, H);
		float Ty = ImgY + H * 0.45f;
		DrawLine(TEXT("No camera: the URLab articulation is not in the level"), X + 16.f, Ty, DimCol, 0.85f);
	}
}

void AR1BehaviourPanel::DrawJointMatrix(float X, float Y, float W)
{
	if (!bTwinResolved)
	{
		ResolveTwinJoints();
	}
	FTwinReadout Tel;
	const float H = 706.f;
	const float Pad = 14.f;
	DrawRect(PanelBg, X, Y, W, H);
	DrawRect(HeaderStripBg, X, Y, W, 56.f);
	DrawRect(VirtualCol, X, Y, W, 4.f);
	DrawRect(DividerCol, X, Y + 56.f, W, 1.f);
	UFont* Font = GEngine->GetMediumFont();
	UFont* SmallFont = GEngine->GetSmallFont();
	DrawText(TEXT("R1 JOINT TELEMETRY  (26 DOF, virtual)"), TextCol, X + Pad, Y + 12.f, Font, 1.05f);
	if (!SampleLiveTwin(Tel) || Tel.QDeg.Num() < 26)
	{
		DrawText(TEXT("Waiting for the URLab articulation..."), DimCol, X + Pad, Y + 72.f, Font, 0.9f);
		return;
	}
	DrawText(FString::Printf(TEXT("IMU R/P/Y  %+5.1f / %+5.1f / %+5.1f deg   |   MuJoCo / URLab   |   limits from r1_joint_map.yaml"),
		Tel.ImuRoll, Tel.ImuPitch, Tel.ImuYaw), DimCol, X + Pad, Y + 35.f, Font, 0.78f);

	struct FGroupCard { const TCHAR* Name; int32 Start; int32 Count; int32 Col; FLinearColor TagCol; };
	static const FGroupCard Groups[] = {
		{TEXT("HEAD"),       24, 2, 0, CyanCol},
		{TEXT("WAIST"),      12, 2, 0, FLinearColor(0.65f, 0.55f, 0.95f, 1.f)},
		{TEXT("LEFT ARM"),   14, 5, 0, VirtualCol * 1.35f},
		{TEXT("RIGHT ARM"),  19, 5, 0, OkCol},
		{TEXT("LEFT LEG"),    0, 6, 1, WarnCol},
		{TEXT("RIGHT LEG"),   6, 6, 1, FLinearColor(0.95f, 0.55f, 0.2f, 1.f)},
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
		DrawText(TEXT("RANGE"), DimCol, CX + 126.f, CY + 6.f, SmallFont, 0.85f);
		DrawText(TEXT("ANGLE"), DimCol, CX + ColW - 78.f, CY + 6.f, SmallFont, 0.85f);
		for (int32 i = 0; i < G.Count; ++i)
		{
			const int32 Idx = G.Start + i;
			const FJointMeta& Meta = JointMetas[Idx];
			const float ValDeg = Tel.QDeg[Idx];
			const float RY = CY + CardHdrH + i * RowH;
			DrawText(Meta.Label, TextCol, CX + 10.f, RY + 5.f, Font, 0.78f);
			const float BarX = CX + 104.f, BarW = ColW - 190.f, BarY = RY + 10.f, BarH = 7.f;
			const float Span = FMath::Max(1.f, Meta.MaxDeg - Meta.MinDeg);
			const float NormVal = FMath::Clamp((ValDeg - Meta.MinDeg) / Span, 0.f, 1.f);
			const float NormDef = FMath::Clamp((Meta.DefaultDeg - Meta.MinDeg) / Span, 0.f, 1.f);
			const float Margin = FMath::Min(NormVal, 1.f - NormVal);
			const float DegMargin = FMath::Min(ValDeg - Meta.MinDeg, Meta.MaxDeg - ValDeg);
			const FLinearColor BarCol = (Margin < 0.03f && DegMargin < 4.f) ? BadCol : (Margin < 0.07f && DegMargin < 8.f) ? WarnCol : CyanCol;
			DrawRect(FLinearColor(1.f, 1.f, 1.f, 0.10f), BarX, BarY, BarW, BarH);
			const float X0 = BarX + NormDef * BarW, X1 = BarX + NormVal * BarW;
			DrawRect(FLinearColor(BarCol.R, BarCol.G, BarCol.B, 0.55f), FMath::Min(X0, X1), BarY, FMath::Max(2.f, FMath::Abs(X1 - X0)), BarH);
			DrawRect(FLinearColor(1.f, 1.f, 1.f, 0.45f), X0 - 1.f, BarY - 2.f, 2.f, BarH + 4.f);
			DrawRect(BarCol, X1 - 2.f, BarY - 2.f, 4.f, BarH + 4.f);
			const FString AngStr = FString::Printf(TEXT("%+6.1f°"), ValDeg);
			float AW = 0.f, AH = 0.f;
			GetTextSize(AngStr, AW, AH, Font, 0.78f);
			DrawText(AngStr, (Margin < 0.07f && DegMargin < 8.f) ? BarCol : TextCol, CX + ColW - 10.f - AW, RY + 5.f, Font, 0.78f);
		}
		ColY[G.Col] += CH + 10.f;
	}
}

void AR1BehaviourPanel::DrawTabStrip(float X0, float X, float Inner, float PanelW, float& Y)
{
	DrawText(TEXT("R1 VIRTUAL ROBOT"), TextCol, X, Y + 2.f, GEngine->GetMediumFont(), 1.4f);
	DrawButton({IdStop, TEXT("STOP"), {X0 + PanelW - 18.f - 110.f, Y}, {110.f, 30.f}, true, StopCol, false});
	Y += 38.f;
	if (bExperiment)
	{
		// the extra tools: the lab's experiment sections and the walk sandbox
		const float Gap = 6.f, TabW = (Inner - Gap) * 0.5f;
		DrawButton({IdTabLab, TEXT("Lab"), {X, Y}, {TabW, 30.f}, true, Tab == ER1PanelTab::Lab ? ChipOnCol : ChipCol});
		DrawButton({IdTabWalk, WalkState.bRunning ? TEXT("Walk (running)") : TEXT("Walk (sim)"), {X + TabW + Gap, Y}, {TabW, 30.f}, true,
			Tab == ER1PanelTab::Walk ? ChipOnCol : ChipCol});
		Y += 36.f;
	}
	DrawLine(TEXT("Pick a behaviour and press Play. Nothing physical moves."), X, Y, DimCol, 0.8f);
	DrawRect(DividerCol, X0, Y + 2.f, PanelW, 1.f);
	Y += 8.f;
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
		NextPoll = Now + (Tab == ER1PanelTab::Walk ? 0.4 : 2.0);
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

	if (!bExperiment && !WalkState.bRunning)
	{
		// simple view: the lab only, no joint list, URLab's debug panels hidden
		Tab = ER1PanelTab::Lab;
		bShowJoints = false;
		if (bShowDetails)
		{
			bShowDetails = false;
			SetUrlabDebugVisible(false);
		}
	}
	const float X = X0 + Pad, Inner = PanelW - 2.f * Pad;
	DrawRect(PanelBg, X0, Y0, PanelW, LastPanelH);
	float Y = Y0 + Pad - 4.f;
	DrawTabStrip(X0, X, Inner, PanelW, Y);
	if (Tab == ER1PanelTab::Lab)
	{
		DrawLabTab(X, Inner, Y);
	}
	else
	{
		DrawWalkTab(X, Inner, Y);
	}

	// ---------------------------------------------------------------- right column + details (every tab)
	Y += 6.f;
	const bool bRoomRight = ViewW >= X0 + PanelW + 20.f + 420.f;
	if (bExperiment)
	{
		DrawButton({IdJoints, bShowJoints ? TEXT("Head camera") : TEXT("Joints (26)"), {X, Y}, {150.f, 34.f}, bRoomRight, ChipCol});
		if (!bRoomRight)
		{
			DisabledWhy.Add(IdJoints, TEXT("the right column needs a wider window"));
		}
		DrawButton({IdDetails, bShowDetails ? TEXT("Hide physics panels") : TEXT("Physics panels"), {X + 158.f, Y}, {170.f, 34.f}, true, ChipCol});
		Y += 42.f;
	}
	{
		const double Age = FPlatformTime::Seconds() - ToastAt;
		if (Age < 4.0 && !Toast.IsEmpty())
		{
			const float Alpha = Age < 3.0 ? 1.f : static_cast<float>(4.0 - Age);
			FLinearColor C = ToastColor;
			C.A = Alpha;
			DrawWrapped(Toast, X, Y, Inner, C, 0.92f);
		}
	}
	if (bRoomRight)
	{
		if (bShowJoints)
		{
			const float RightW = FMath::Clamp(ViewW - (X0 + PanelW + 300.f), 540.f, 650.f);
			DrawJointMatrix(FMath::Max(X0 + PanelW + 20.f, ViewW - RightW - 20.f), Y0, RightW);
		}
		else if (Tab == ER1PanelTab::Lab && (bExperiment || LabShowsCamera()))
		{
			const float CamW = bExperiment ? FMath::Clamp(ViewW - (X0 + PanelW + 60.f), 320.f, 480.f) : 320.f;
			DrawHeadCamera(ViewW - CamW - 28.f, Y0 + 8.f, CamW);
		}
	}
	LastPanelH = FMath::Max(200.f, Y - Y0 + 8.f);
	ButtonRects = MoveTemp(NextRects);
	if (bScaled)
	{
		Canvas->Canvas->PopTransform();
	}
}
