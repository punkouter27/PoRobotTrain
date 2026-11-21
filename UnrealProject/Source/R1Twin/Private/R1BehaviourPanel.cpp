#include "R1BehaviourPanel.h"
#include "R1Twin.h"

#include "Blueprint/UserWidget.h"
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
#endif

namespace
{
	const FLinearColor PanelBg(0.03f, 0.03f, 0.04f, 0.86f);
	const FLinearColor TextCol(0.95f, 0.95f, 0.95f, 1.f);
	const FLinearColor DimCol(0.62f, 0.62f, 0.66f, 1.f);
	const FLinearColor OkCol(0.42f, 0.85f, 0.48f, 1.f);
	const FLinearColor WarnCol(1.f, 0.75f, 0.3f, 1.f);
	const FLinearColor BadCol(1.f, 0.42f, 0.36f, 1.f);
	// Virtual = blue, real robot = orange, everywhere (section stripe, heading, main button).
	const FLinearColor VirtualCol(0.15f, 0.42f, 0.85f, 1.f);
	const FLinearColor RealCol(0.88f, 0.45f, 0.08f, 1.f);
	const FLinearColor RealSoftCol(0.5f, 0.3f, 0.1f, 1.f);
	const FLinearColor StopCol(0.78f, 0.1f, 0.08f, 1.f);
	const FLinearColor OffCol(0.24f, 0.24f, 0.26f, 1.f);
	const FLinearColor ChipCol(0.2f, 0.2f, 0.23f, 1.f);
	const FLinearColor ChipOnCol(0.85f, 0.85f, 0.88f, 1.f);
	const FName IdPreview(TEXT("preview")), IdCheck(TEXT("check")), IdRun(TEXT("run")), IdStop(TEXT("stop"));
	const FName IdConfirm(TEXT("confirm")), IdCancel(TEXT("cancel")), IdDetails(TEXT("details"));
	const FName IdAmp10(TEXT("amp10")), IdAmp20(TEXT("amp20")), IdAmp30(TEXT("amp30"));
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
		virtual const TCHAR* GetDebugName() const override { return TEXT("R1BehaviourPanelClicks"); }
	private:
		TWeakObjectPtr<AR1BehaviourPanel> Panel;
	};
}

AR1BehaviourPanel::AR1BehaviourPanel()
{
	PrimaryActorTick.bCanEverTick = true;
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
	UE_LOG(LogR1Twin, Log, TEXT("R1BehaviourPanel: service %s, behaviour %s"), *ServiceUrl, *Behaviour);
}

void AR1BehaviourPanel::EndPlay(const EEndPlayReason::Type Reason)
{
	if (ClickProcessor.IsValid() && FSlateApplication::IsInitialized())
	{
		FSlateApplication::Get().UnregisterInputPreProcessor(ClickProcessor);
	}
	ClickProcessor.Reset();
	Super::EndPlay(Reason);
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

AR1BehaviourPanel::FHeadReadout AR1BehaviourPanel::ReadHead(const TSharedPtr<FJsonObject>& Head, const TCHAR* Who)
{
	FHeadReadout H;
	const TSharedPtr<FJsonObject>* P = nullptr;
	if (!Head.IsValid() || !Head->TryGetObjectField(Who, P) || !P || !P->IsValid())
	{
		return H;
	}
	double V = 0.0;
	if ((*P)->TryGetNumberField(TEXT("yaw"), V))
	{
		H.bValid = true;
		H.Yaw = static_cast<float>(V);
	}
	if ((*P)->TryGetNumberField(TEXT("pitch"), V))
	{
		H.bHasPitch = true;
		H.Pitch = static_cast<float>(V);
	}
	if ((*P)->TryGetNumberField(TEXT("target"), V))
	{
		H.bHasTarget = true;
		H.Target = static_cast<float>(V);
	}
	if ((*P)->TryGetNumberField(TEXT("other"), V))
	{
		H.Other = static_cast<float>(V);
	}
	if ((*P)->TryGetNumberField(TEXT("age_s"), V))
	{
		H.AgeS = static_cast<float>(V);
	}
	(*P)->TryGetStringField(TEXT("source"), H.Source);
	(*P)->TryGetStringField(TEXT("phase"), H.Phase);
	return H;
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
	double Age = -1.0;
	CheckAgeS = J->TryGetNumberField(TEXT("check_age_s"), Age) ? static_cast<float>(Age) : -1.f;
	const TSharedPtr<FJsonObject>* Results = nullptr;
	const TSharedPtr<FJsonObject> R = J->TryGetObjectField(TEXT("results"), Results) && Results ? *Results : nullptr;
	PreviewRes = ReadResult(R, TEXT("preview"));
	CheckRes = ReadResult(R, TEXT("check"));
	RunRes = ReadResult(R, TEXT("run"));
	const TSharedPtr<FJsonObject>* HeadObj = nullptr;
	const TSharedPtr<FJsonObject> Head = J->TryGetObjectField(TEXT("head"), HeadObj) && HeadObj ? *HeadObj : nullptr;
	VirtualHead = ReadHead(Head, TEXT("virtual"));
	RealHead = ReadHead(Head, TEXT("real"));
	Lines.Reset();
	const TArray<TSharedPtr<FJsonValue>>* Arr = nullptr;
	if (J->TryGetArrayField(TEXT("lines"), Arr))
	{
		for (const TSharedPtr<FJsonValue>& V : *Arr)
		{
			Lines.Add(V->AsString());
		}
	}
}

void AR1BehaviourPanel::Post(const FString& Path, bool bConfirm)
{
	TSharedRef<IHttpRequest, ESPMode::ThreadSafe> Req = FHttpModule::Get().CreateRequest();
	Req->SetURL(ServiceUrl + Path);
	Req->SetVerb(TEXT("POST"));
	Req->SetHeader(TEXT("Content-Type"), TEXT("application/json"));
	Req->SetContentAsString(Path == TEXT("/stop") ? FString(TEXT("{}")) : ParamsJson(bConfirm));
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
	const bool bIdle = bServiceOnline && Job.IsEmpty();
	if (BoxName == IdStop)
	{
		bConfirmOpen = false;
		Post(TEXT("/stop"));
		ShowToast(TEXT("STOP sent."), BadCol);
	}
	else if (BoxName == IdDetails)
	{
		bShowDetails = !bShowDetails;
		SetUrlabDebugVisible(bShowDetails);
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
	else if (BoxName == IdRun && bIdle && bCanRunRobot)
	{
		bConfirmOpen = true;
		ConfirmOpenedAt = FPlatformTime::Seconds();
	}
	else if (bIdle && (BoxName == IdAmp10 || BoxName == IdAmp20 || BoxName == IdAmp30))
	{
		AmplitudeDeg = BoxName == IdAmp10 ? 10.f : BoxName == IdAmp20 ? 20.f : 30.f;
		ShowToast(FString::Printf(TEXT("Turn size set to %.0f deg."), AmplitudeDeg), TextCol);
	}
}

void AR1BehaviourPanel::DrawButton(const FButton& B)
{
	const double Now = FPlatformTime::Seconds();
	const float Flash = B.Id == PressedId ? FMath::Clamp(1.f - static_cast<float>(Now - PressedAt) / 0.5f, 0.f, 1.f) : 0.f;
	bool bHover = false;
	if (APlayerController* PC = GetOwningPlayerController())
	{
		float Mx = 0.f, My = 0.f;
		if (PC->GetMousePosition(Mx, My))
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
	// pressed: the button sinks 2 px for the first part of the flash
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
	UFont* Font = GEngine->GetMediumFont();   // GetLargeFont() is a small distance-field font
	const float Scale = B.bLarge ? 1.35f : 1.f;
	GetTextSize(B.Label, W, H, Font, Scale);
	const bool bDarkText = Fill.GetLuminance() > 0.6f;
	const FLinearColor Col = !B.bEnabled && Flash <= 0.f ? DimCol : bDarkText ? FLinearColor(0.05f, 0.05f, 0.06f, 1.f) : TextCol;
	DrawText(B.Label, Col, B.Pos.X + Off.X + (B.Size.X - W) * 0.5f, B.Pos.Y + Off.Y + (B.Size.Y - H) * 0.5f, Font, Scale);

	// every button is clickable, so a click on a locked one still answers (with the reason)
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
		else if (!Job.IsEmpty())
		{
			Why = Job == TEXT("preview") ? TEXT("the virtual robot is still moving (STOP cancels it)")
				: Job == TEXT("check") ? TEXT("the robot check is still running")
				: TEXT("the real robot is moving (STOP cancels it)");
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

void AR1BehaviourPanel::DrawHead(const FHeadReadout& H, bool bLive, const FString& NoData, const FLinearColor& Accent,
	float X, float W, float& Y)
{
	// "HEAD  yaw +5.1 deg  pitch -0.3 deg  target +6.9 deg   live"   [-30 ....|..o.. +30] yaw gauge
	if (!H.bValid)
	{
		DrawLine(FString::Printf(TEXT("HEAD   %s"), *NoData), X, Y, DimCol, 0.9f);
		return;
	}
	FString S = FString::Printf(TEXT("HEAD   yaw %+.1f deg"), H.Yaw);
	if (H.bHasPitch)
	{
		S += FString::Printf(TEXT("   pitch %+.1f deg"), H.Pitch);
	}
	if (H.bHasTarget && bLive && H.Source != TEXT("check"))   // a read-only check sends no targets
	{
		S += FString::Printf(TEXT("   target %+.1f deg"), H.Target);
	}
	const FString When = bLive ? FString(TEXT("live"))
		: H.AgeS < 90.f ? FString::Printf(TEXT("%.0f s ago"), H.AgeS) : FString::Printf(TEXT("%.0f min ago"), H.AgeS / 60.f);
	const float RowY = Y;
	DrawLine(S, X, Y, TextCol, 0.9f);
	{
		float Ty = Y;
		const FString Sub = FString::Printf(TEXT("%s   other joints moved max %.1f deg   (%s)"), *When, H.Other,
			H.Source == TEXT("preview") ? TEXT("virtual run") : H.Source == TEXT("check") ? TEXT("read-only check") : TEXT("robot run"));
		DrawLine(Sub, X, Ty, bLive ? OkCol : DimCol, 0.8f);
		Y = Ty;
	}

	// yaw gauge, -30..+30 deg, right-aligned on the readout row
	const float GW = 150.f, GH = 8.f, GX = X + W - GW, GY = RowY + 7.f, Range = 30.f;
	auto ToX = [&](float Deg) { return GX + (FMath::Clamp(Deg, -Range, Range) + Range) / (2.f * Range) * GW; };
	DrawRect(FLinearColor(1.f, 1.f, 1.f, 0.12f), GX, GY, GW, GH);
	DrawRect(FLinearColor(1.f, 1.f, 1.f, 0.35f), ToX(0.f) - 1.f, GY - 3.f, 2.f, GH + 6.f);          // centre
	if (H.bHasTarget && bLive && H.Source != TEXT("check"))   // a read-only check sends no targets
	{
		DrawRect(Accent, ToX(H.Target) - 2.f, GY - 5.f, 4.f, GH + 10.f);                             // target tick
	}
	DrawRect(bLive ? FLinearColor::White : DimCol, ToX(H.Yaw) - 5.f, GY - 2.f, 10.f, GH + 4.f);      // actual
	DrawText(TEXT("L"), DimCol, GX - 14.f, GY - 6.f, GEngine->GetSmallFont());
	DrawText(TEXT("R"), DimCol, GX + GW + 5.f, GY - 6.f, GEngine->GetSmallFont());
}

void AR1BehaviourPanel::DrawStep(bool bDone, const FString& Text, float X, float& Y)
{
	// a filled green square = done, a hollow grey square = still to do
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
	DrawLine(Text, X + S + 8.f, Y, bDone ? TextCol : DimCol, 0.95f);
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
	const double Now = FPlatformTime::Seconds();
	if (bClickQueued)
	{
		bClickQueued = false;
		float Mx = 0.f, My = 0.f;
		APlayerController* PC = GetOwningPlayerController();
		if (PC && PC->GetMousePosition(Mx, My))
		{
			const FVector2D M(Mx, My);
			for (int32 i = ButtonRects.Num() - 1; i >= 0; --i)   // topmost (confirmation box) first
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
		NextPoll = Now + 0.5;
	}
	if (!bShowDetails && !bUrlabDebugHidden)
	{
		SetUrlabDebugVisible(false);   // URLab creates its widget after its first compile; hide it once it exists
	}
	// Keep the panel clickable: the follow camera and URLab's widget (GameOnly input, hidden cursor) change these
	// after our BeginPlay. Re-assert every frame; set the input mode only when something switched the cursor off.
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
		bConfirmOpen = false;   // the confirmation expires
	}

	const bool bIdle = bServiceOnline && Job.IsEmpty();
	const float Amp = AmplitudeDeg;
	const bool bPreviewedThis = PreviewedAmp > 0.f && FMath::IsNearlyEqual(PreviewedAmp, Amp);
	const bool bCheckedThis = CheckedAmp > 0.f && FMath::IsNearlyEqual(CheckedAmp, Amp)
		&& CheckAgeS >= 0.f && CheckAgeS < CheckValidS;
	const bool bRobotReady = bIdle && bCanRunRobot && bPreviewedThis && bCheckedThis;

	const float Pad = 18.f, PanelW = 600.f;
	const float X0 = 20.f, Y0 = 20.f;
	const float X = X0 + Pad, Inner = PanelW - 2.f * Pad;
	const float PanelH = (bShowDetails ? 624.f : 479.f) + (FPlatformTime::Seconds() - ToastAt < 4.0 ? 32.f : 0.f);
	DrawRect(PanelBg, X0, Y0, PanelW, PanelH);
	float Y = Y0 + Pad - 4.f;

	// ---------------------------------------------------------------- title + turn size
	DrawText(TEXT("HEAD TURN"), TextCol, X, Y + 2.f, GEngine->GetMediumFont(), 1.5f);
	const float ChipW = 58.f, ChipH = 30.f, ChipX = X0 + PanelW - Pad - 3.f * (ChipW + 6.f) + 6.f;
	{
		float Ty = Y + 6.f;
		DrawText(TEXT("Turn size"), DimCol, ChipX - 92.f, Ty, GEngine->GetMediumFont());
	}
	for (int32 i = 0; i < 3; ++i)
	{
		const float A = 10.f * (i + 1);
		const FName Id = i == 0 ? IdAmp10 : i == 1 ? IdAmp20 : IdAmp30;
		const bool bSel = FMath::IsNearlyEqual(Amp, A);
		DrawButton({Id, FString::Printf(TEXT("%.0f deg"), A), {ChipX + i * (ChipW + 6.f), Y}, {ChipW, ChipH},
			bIdle || bSel, bSel ? ChipOnCol : ChipCol});
	}
	Y += 38.f;
	DrawLine(FString::Printf(TEXT("Turns the head left and right %.0f deg. Only the neck motor moves."), Amp), X, Y, DimCol, 0.95f);
	Y += 12.f;

	// ---------------------------------------------------------------- 1. virtual robot (blue)
	{
		const float SecY = Y;
		DrawRect(VirtualCol, X0, SecY, 6.f, 132.f);
		DrawLine(TEXT("VIRTUAL ROBOT  (on this screen)"), X, Y, VirtualCol * 1.35f, 1.05f);
		Y += 4.f;
		DrawButton({IdPreview, Job == TEXT("preview") ? TEXT("Moving the virtual robot...") : TEXT("Move VIRTUAL robot"),
			{X, Y}, {Inner, 46.f}, bIdle, VirtualCol, true});
		Y += 54.f;
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
			S = FString::Printf(TEXT("Done - the virtual robot turned its head %.0f deg."), Amp);
			C = OkCol;
		}
		else
		{
			S = TEXT("Safe: nothing real moves. Watch the robot on screen.");
		}
		DrawLine(S, X, Y, C, 0.95f);
		Y += 4.f;
		DrawHead(VirtualHead, Job == TEXT("preview") && VirtualHead.AgeS < 2.f, TEXT("no reading yet - move the virtual robot"),
			VirtualCol * 1.35f, X, Inner, Y);
		Y = SecY + 144.f;
	}

	// ---------------------------------------------------------------- 2. real robot (orange)
	{
		const float SecY = Y;
		DrawRect(RealCol, X0, SecY, 6.f, 160.f);
		DrawLine(TEXT("REAL ROBOT  (Unitree R1)"), X, Y, RealCol * 1.2f, 1.05f);
		Y += 4.f;
		const float CheckW = 190.f;
		RunLockedWhy = !bPreviewedThis ? FString::Printf(TEXT("do step 1 first - move the virtual robot at %.0f deg"), Amp)
			: !bCheckedThis ? FString(TEXT("do step 2 first - Check connection"))
			: FString(TEXT("not ready yet"));
		DrawButton({IdCheck, Job == TEXT("check") ? TEXT("Checking...") : TEXT("Check connection"), {X, Y}, {CheckW, 46.f}, bIdle, RealSoftCol});
		DrawButton({IdRun, Job == TEXT("run") ? TEXT("REAL robot moving...") : TEXT("Move REAL robot"), {X + CheckW + 10.f, Y}, {Inner - CheckW - 10.f, 46.f}, bRobotReady, RealCol, true});
		Y += 56.f;
		DrawStep(bPreviewedThis, FString::Printf(TEXT("Step 1: move the virtual robot (%.0f deg) first"), Amp), X, Y);
		const FString CheckText = bCheckedThis
			? FString::Printf(TEXT("Step 2: robot connected, head centred (%.0f s ago)"), CheckAgeS)
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
		else if (RunRes.bValid && !RunRes.bOk)
		{
			S = FString::Printf(TEXT("Last robot run stopped: %s"), *RunRes.Reason);
			C = BadCol;
		}
		else if (bRobotReady)
		{
			S = TEXT("Ready. The button asks you to confirm before anything moves.");
			C = OkCol;
		}
		else if (RunRes.bValid && RunRes.bOk)
		{
			S = TEXT("Last robot run finished. Check again before the next one.");
			C = OkCol;
		}
		else
		{
			S = TEXT("Locked until steps 1 and 2 are done.");
		}
		DrawLine(S, X, Y, C, 0.95f);
		Y += 4.f;
		DrawHead(RealHead, (Job == TEXT("check") || Job == TEXT("run")) && RealHead.AgeS < 2.f,
			TEXT("no reading yet - Check connection reads it (nothing moves)"), RealCol * 1.2f, X, Inner, Y);
		Y = SecY + 172.f;
	}

	// ---------------------------------------------------------------- stop + service state
	DrawButton({IdStop, TEXT("STOP"), {X, Y}, {120.f, 40.f}, bServiceOnline, StopCol, true});
	{
		float Ty = Y + 2.f;
		DrawLine(TEXT("Emergency stop on the robot:"), X + 134.f, Ty, WarnCol, 0.9f);
		DrawLine(TEXT("remote L2 + B, hold 5 s"), X + 134.f, Ty, WarnCol, 0.9f);
	}
	DrawButton({IdDetails, bShowDetails ? TEXT("Hide details") : TEXT("Details"), {X0 + PanelW - Pad - 120.f, Y}, {120.f, 40.f},
		true, ChipCol});
	Y += 50.f;
	{
		const double Age = FPlatformTime::Seconds() - ToastAt;
		if (Age < 4.0 && !Toast.IsEmpty())
		{
			const float Alpha = Age < 3.0 ? 1.f : static_cast<float>(4.0 - Age);
			FLinearColor C = ToastColor;
			C.A = Alpha;
			DrawRect(FLinearColor(1.f, 1.f, 1.f, 0.08f * Alpha), X - 6.f, Y - 4.f, Inner + 12.f, 30.f);
			float Ty = Y;
			DrawLine(Toast, X, Ty, C, 1.f);
			Y += 32.f;
		}
	}
	if (!bServiceOnline)
	{
		DrawLine(TEXT("Not connected. Start:  python -m robotcontrol.behaviours.service"), X, Y, BadCol, 0.9f);
	}
	else if (!LastError.IsEmpty())
	{
		DrawLine(FString::Printf(TEXT("Refused: %s"), *LastError), X, Y, BadCol, 0.9f);
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

	// ---------------------------------------------------------------- confirmation
	if (bConfirmOpen)
	{
		const float CW = 600.f, CH = 210.f;
		const float CX = (Canvas->ClipX - CW) * 0.5f, CY = (Canvas->ClipY - CH) * 0.5f;
		DrawRect(FLinearColor(0.f, 0.f, 0.f, 0.55f), 0.f, 0.f, Canvas->ClipX, Canvas->ClipY);
		DrawRect(FLinearColor(0.07f, 0.04f, 0.02f, 0.98f), CX, CY, CW, CH);
		DrawRect(RealCol, CX, CY, CW, 6.f);
		float CYl = CY + 20.f;
		DrawLine(TEXT("Move the REAL robot?"), CX + 20.f, CYl, RealCol * 1.2f, 1.4f);
		CYl += 6.f;
		DrawLine(FString::Printf(TEXT("Only the neck motor will turn the head %.0f deg left and right."), Amp), CX + 20.f, CYl, TextCol);
		DrawLine(TEXT("Robot on its stand, head facing forward, remote in your hand?"), CX + 20.f, CYl, TextCol);
		DrawButton({IdConfirm, TEXT("Yes, move the real robot"), {CX + 20.f, CY + CH - 58.f}, {300.f, 42.f}, bRobotReady, RealCol, true});
		DrawButton({IdCancel, TEXT("Cancel"), {CX + 334.f, CY + CH - 58.f}, {120.f, 42.f}, true, ChipCol});
	}
	ButtonRects = MoveTemp(NextRects);
}
