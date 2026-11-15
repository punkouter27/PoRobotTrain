#include "R1BehaviourPanel.h"
#include "R1Twin.h"

#include "Engine/Canvas.h"
#include "Engine/Engine.h"
#include "Engine/Font.h"
#include "GameFramework/PlayerController.h"
#include "HttpModule.h"
#include "Interfaces/IHttpResponse.h"
#include "Dom/JsonObject.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"

namespace
{
	const FLinearColor PanelBg(0.02f, 0.02f, 0.03f, 0.78f);
	const FLinearColor TextCol(0.92f, 0.92f, 0.92f, 1.f);
	const FLinearColor DimCol(0.6f, 0.6f, 0.62f, 1.f);
	const FLinearColor OkCol(0.45f, 0.85f, 0.5f, 1.f);
	const FLinearColor WarnCol(1.f, 0.75f, 0.3f, 1.f);
	const FLinearColor BadCol(1.f, 0.4f, 0.35f, 1.f);
	const FLinearColor BtnCol(0.16f, 0.32f, 0.55f, 1.f);
	const FLinearColor RobotCol(0.6f, 0.38f, 0.1f, 1.f);
	const FLinearColor StopCol(0.7f, 0.12f, 0.1f, 1.f);
	const FLinearColor OffCol(0.22f, 0.22f, 0.24f, 1.f);
	const FName IdPreview(TEXT("preview")), IdCheck(TEXT("check")), IdRun(TEXT("run")), IdStop(TEXT("stop"));
	const FName IdConfirm(TEXT("confirm")), IdCancel(TEXT("cancel"));
	const FName IdAmp10(TEXT("amp10")), IdAmp20(TEXT("amp20")), IdAmp30(TEXT("amp30"));
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
	UE_LOG(LogR1Twin, Log, TEXT("R1BehaviourPanel: service %s, behaviour %s"), *ServiceUrl, *Behaviour);
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

void AR1BehaviourPanel::NotifyHitBoxClick(FName BoxName)
{
	Super::NotifyHitBoxClick(BoxName);
	const bool bIdle = bServiceOnline && Job.IsEmpty();
	if (BoxName == IdStop)
	{
		bConfirmOpen = false;
		Post(TEXT("/stop"));
	}
	else if (bConfirmOpen)
	{
		if (BoxName == IdConfirm && bCanRunRobot && bIdle)
		{
			Post(TEXT("/run"), /*bConfirm=*/true);
		}
		bConfirmOpen = false;
	}
	else if (BoxName == IdPreview && bIdle)
	{
		Post(TEXT("/preview"));
	}
	else if (BoxName == IdCheck && bIdle)
	{
		Post(TEXT("/check"));
	}
	else if (BoxName == IdRun && bIdle && bCanRunRobot)
	{
		bConfirmOpen = true;
		ConfirmOpenedAt = FPlatformTime::Seconds();
	}
	else if (bIdle && (BoxName == IdAmp10 || BoxName == IdAmp20 || BoxName == IdAmp30))
	{
		AmplitudeDeg = BoxName == IdAmp10 ? 10.f : BoxName == IdAmp20 ? 20.f : 30.f;
	}
}

void AR1BehaviourPanel::DrawButton(const FButton& B)
{
	DrawRect(B.bEnabled ? B.Color : OffCol, B.Pos.X, B.Pos.Y, B.Size.X, B.Size.Y);
	float W = 0.f, H = 0.f;
	UFont* Font = GEngine->GetMediumFont();
	GetTextSize(B.Label, W, H, Font);
	DrawText(B.Label, B.bEnabled ? TextCol : DimCol, B.Pos.X + (B.Size.X - W) * 0.5f, B.Pos.Y + (B.Size.Y - H) * 0.5f, Font);
	if (B.bEnabled)
	{
		AddHitBox(B.Pos, B.Size, B.Id, /*bConsumesInput=*/true);
	}
}

void AR1BehaviourPanel::DrawLine(const FString& Text, float X, float& Y, const FLinearColor& Color, float Scale)
{
	UFont* Font = GEngine->GetSmallFont();
	DrawText(Text, Color, X, Y, Font, Scale);
	float W = 0.f, H = 0.f;
	GetTextSize(Text, W, H, Font, Scale);
	Y += H + 2.f;
}

void AR1BehaviourPanel::DrawHUD()
{
	Super::DrawHUD();
	if (!Canvas)
	{
		return;
	}
	const double Now = FPlatformTime::Seconds();
	if (Now >= NextPoll)
	{
		PollStatus();
		NextPoll = Now + 0.5;
	}
	if (bConfirmOpen && Now - ConfirmOpenedAt > 15.0)
	{
		bConfirmOpen = false;   // the confirmation expires
	}

	const bool bIdle = bServiceOnline && Job.IsEmpty();
	const float Pad = 12.f, BtnH = 34.f, PanelW = 640.f;
	const float X0 = 16.f, Y0 = Canvas->ClipY - 250.f;
	DrawRect(PanelBg, X0, Y0, PanelW, 234.f);

	float Y = Y0 + Pad;
	DrawLine(FString::Printf(TEXT("Behaviour: %s   +-%.0f deg at %.2f Hz for %.0f s   (only the head-yaw motor is powered)"),
		*Behaviour, AmplitudeDeg, FrequencyHz, DurationS), X0 + Pad, Y, TextCol);
	Y += 4.f;

	float X = X0 + Pad;
	TArray<FButton> Buttons = {
		{IdPreview, TEXT("Preview in twin"), {X, Y}, {140.f, BtnH}, bIdle, BtnCol},
		{IdCheck, TEXT("Check robot"), {X + 148.f, Y}, {120.f, BtnH}, bIdle, BtnCol},
		{IdRun, TEXT("Run on robot"), {X + 276.f, Y}, {130.f, BtnH}, bIdle && bCanRunRobot, RobotCol},
		{IdStop, TEXT("STOP"), {X + 414.f, Y}, {90.f, BtnH}, bServiceOnline, StopCol},
	};
	const float AmpX = X + 512.f;
	for (int32 i = 0; i < 3; ++i)
	{
		const float A = 10.f * (i + 1);
		const FName Id = i == 0 ? IdAmp10 : i == 1 ? IdAmp20 : IdAmp30;
		Buttons.Add({Id, FString::Printf(TEXT("%.0f"), A), {AmpX + i * 38.f, Y}, {34.f, BtnH}, bIdle,
			FMath::IsNearlyEqual(AmplitudeDeg, A) ? OkCol * 0.6f : BtnCol * 0.7f});
	}
	for (const FButton& B : Buttons)
	{
		DrawButton(B);
	}
	Y += BtnH + 8.f;

	if (!bServiceOnline)
	{
		DrawLine(TEXT("Behaviour service offline - start it:  python -m robotcontrol.behaviours.service"), X, Y, BadCol);
	}
	else
	{
		const FString State = Job.IsEmpty() ? TEXT("idle") : FString::Printf(TEXT("running: %s"), *Job);
		DrawLine(FString::Printf(TEXT("Status: %s   %s"), *State, *LastResult), X, Y,
			Job.IsEmpty() ? TextCol : WarnCol);
		if (Job == TEXT("run"))
		{
			DrawLine(FString::Printf(TEXT("Live mirror: twin shows the robot's measured angles (%d samples)"), MirrorSamples), X, Y, OkCol);
		}
		else if (!bCanRunRobot && !WhyNotRun.IsEmpty())
		{
			DrawLine(FString::Printf(TEXT("Run on robot: %s"), *WhyNotRun), X, Y, DimCol);
		}
	}
	if (!LastError.IsEmpty())
	{
		DrawLine(FString::Printf(TEXT("Refused: %s"), *LastError), X, Y, BadCol);
	}
	const int32 First = FMath::Max(0, Lines.Num() - 5);
	for (int32 i = First; i < Lines.Num(); ++i)
	{
		DrawLine(Lines[i].Left(110), X, Y, DimCol, 0.9f);
	}
	DrawLine(TEXT("Emergency stop: remote L2+B for 5 s"), X, Y, WarnCol);

	if (bConfirmOpen)
	{
		const float CW = 520.f, CH = 150.f;
		const float CX = (Canvas->ClipX - CW) * 0.5f, CY = (Canvas->ClipY - CH) * 0.5f;
		DrawRect(FLinearColor(0.05f, 0.03f, 0.02f, 0.95f), CX, CY, CW, CH);
		float CYl = CY + 14.f;
		DrawLine(TEXT("Run on the PHYSICAL robot?"), CX + 16.f, CYl, WarnCol, 1.3f);
		DrawLine(FString::Printf(TEXT("Only the head-yaw motor is powered (+-%.0f deg). Confirm that:"), AmplitudeDeg), CX + 16.f, CYl, TextCol);
		DrawLine(TEXT("the robot is on its stand, the head faces forward, the remote is in your hand."), CX + 16.f, CYl, TextCol);
		DrawButton({IdConfirm, TEXT("Confirm and run"), {CX + 16.f, CY + CH - 46.f}, {170.f, 34.f}, bCanRunRobot && bIdle, RobotCol});
		DrawButton({IdCancel, TEXT("Cancel"), {CX + 196.f, CY + CH - 46.f}, {100.f, 34.f}, true, OffCol * 1.6f});
	}
}
