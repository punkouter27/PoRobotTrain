// Behaviour panel: operator console tabs (docs/service_api.md sections 4-7).
//   Health   GET /health, /versions, /events, /power, /history/temps, /imu, the twin head camera (/camera.png)
//   Motions  poses (/poses/preview), motions (/motions, /motions/play, /player/stop, export/import packages),
//            teach mode (/motions/record/*), modes (/mode), session limits (/limits)
//   Setup    pre-flight checklist (/preflight), zero-offset calibration (/calibration/*), connection profiles (/network)
//   Help     docs/*.md (/docs) as wrapped text
// plus the E-STOP on every tab (POST /estop, End key).
//
// Safety: poses, motions and modes are only ever sent with target "urlab" (the simulated stand twin mirrored onto
// Unreal); the panel never offers "real" for them. The pre-flight checklist and the limits only add restrictions.
#include "R1BehaviourPanel.h"
#include "R1Twin.h"
#include "R1PanelStyle.h"

#include "Algo/Sort.h"
#include "Engine/Engine.h"
#include "Engine/Font.h"
#include "Engine/Texture2D.h"
#include "GameFramework/PlayerController.h"
#include "HAL/FileManager.h"
#include "HttpModule.h"
#include "IImageWrapper.h"
#include "IImageWrapperModule.h"
#include "Interfaces/IHttpResponse.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Modules/ModuleManager.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"
#include "Serialization/JsonWriter.h"

using namespace R1PanelStyle;

namespace
{
	const FLinearColor ConsoleCol(0.39f, 0.40f, 0.95f, 1.f);   // accent #6366f1
	const FLinearColor TwinCol(0.15f, 0.44f, 0.88f, 1.f);
	const FLinearColor EStopCol(0.85f, 0.06f, 0.05f, 1.f);
	const FLinearColor OffGrey(0.24f, 0.24f, 0.26f, 1.f);

	FName CId(const FString& S) { return FName(*(FString(TEXT("c_")) + S)); }

	FString JStr(const TSharedPtr<FJsonObject>& J, const TCHAR* F)
	{
		FString S;
		if (J.IsValid() && J->HasTypedField<EJson::String>(F))
		{
			J->TryGetStringField(F, S);
		}
		return S;
	}

	double JNum(const TSharedPtr<FJsonObject>& J, const TCHAR* F, double Def = 0.0)
	{
		double V = Def;
		return J.IsValid() && J->HasTypedField<EJson::Number>(F) && J->TryGetNumberField(F, V) ? V : Def;
	}

	bool JHasNum(const TSharedPtr<FJsonObject>& J, const TCHAR* F)
	{
		return J.IsValid() && J->HasTypedField<EJson::Number>(F);
	}

	bool JBool(const TSharedPtr<FJsonObject>& J, const TCHAR* F, bool Def = false)
	{
		bool B = Def;
		return J.IsValid() && J->TryGetBoolField(F, B) ? B : Def;
	}

	TSharedPtr<FJsonObject> JObj(const TSharedPtr<FJsonObject>& J, const TCHAR* F)
	{
		const TSharedPtr<FJsonObject>* P = nullptr;
		return J.IsValid() && J->TryGetObjectField(F, P) && P ? *P : nullptr;
	}

	TArray<TSharedPtr<FJsonValue>> JArr(const TSharedPtr<FJsonObject>& J, const TCHAR* F)
	{
		const TArray<TSharedPtr<FJsonValue>>* A = nullptr;
		return J.IsValid() && J->TryGetArrayField(F, A) && A ? *A : TArray<TSharedPtr<FJsonValue>>();
	}

	TSharedPtr<FJsonObject> AsObj(const TSharedPtr<FJsonValue>& V)
	{
		const TSharedPtr<FJsonObject>* P = nullptr;
		return V.IsValid() && V->TryGetObject(P) && P ? *P : nullptr;
	}

	FString ToJson(const TSharedPtr<FJsonObject>& J)
	{
		FString Out;
		if (J.IsValid())
		{
			const TSharedRef<TJsonWriter<TCHAR, TCondensedJsonPrintPolicy<TCHAR>>> W = TJsonWriterFactory<TCHAR, TCondensedJsonPrintPolicy<TCHAR>>::Create(&Out);
			FJsonSerializer::Serialize(J.ToSharedRef(), W);
		}
		return Out;
	}

	FString Esc(const FString& S) { return R1Panel::JsonEscape(S); }

	FString Ago(double At)
	{
		const double S = FMath::Max(0.0, static_cast<double>(FDateTime::UtcNow().ToUnixTimestamp()) - At);
		return S < 90.0 ? FString::Printf(TEXT("%.0f s ago"), S) : S < 5400.0 ? FString::Printf(TEXT("%.0f min ago"), S / 60.0)
			: FString::Printf(TEXT("%.1f h ago"), S / 3600.0);
	}

	/** Shorten to fit MaxW (adds "...") so long service strings never run off the panel. */
	FString Fit(const AHUD* Hud, const FString& S, float MaxW, float Scale)
	{
		UFont* Font = GEngine->GetMediumFont();
		float W = 0.f, H = 0.f;
		Hud->GetTextSize(S, W, H, Font, Scale);
		if (W <= MaxW)
		{
			return S;
		}
		FString T = S;
		while (T.Len() > 3)
		{
			T.LeftChopInline(1);
			Hud->GetTextSize(T + TEXT("..."), W, H, Font, Scale);
			if (W <= MaxW)
			{
				break;
			}
		}
		return T + TEXT("...");
	}

	const TCHAR* LimitKeys[] = {TEXT("amplitude_cap_deg"), TEXT("freq_cap_hz"), TEXT("joint_range_frac"), TEXT("walk_speed_scale"), TEXT("motion_speed_scale")};
	const float LimitSteps[] = {5.f, 0.05f, 0.1f, 0.1f, 0.1f};

	FString MotionStoreDir() { return FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("MotionStore")); }
}

// ------------------------------------------------------------------------------------------------ fetching
void AR1BehaviourPanel::TickConsole(double Now)
{
	APlayerController* PC = GetOwningPlayerController();
	if (PC)
	{
		const bool bDown = PC->IsInputKeyDown(EKeys::End);   // Esc stays "quit PIE"
		if (bDown && !bEStopKeyWasDown)
		{
			SendEStop();
		}
		bEStopKeyWasDown = bDown;
		if (Tab == ER1PanelTab::Help)
		{
			if (PC->WasInputKeyJustPressed(EKeys::PageDown))
			{
				DocScroll += 320.f;
			}
			if (PC->WasInputKeyJustPressed(EKeys::PageUp))
			{
				DocScroll -= 320.f;
			}
		}
	}
	if (Tab == ER1PanelTab::Help && WheelQueued != 0.f)
	{
		DocScroll -= WheelQueued * 64.f;
	}
	WheelQueued = 0.f;
	if (!bServiceOnline)
	{
		return;
	}
	switch (Tab)
	{
	case ER1PanelTab::Health:
		FetchConsole(TEXT("/health"), &HealthJ, Now, 4.0);
		FetchConsole(TEXT("/versions"), &VersionsJ, Now, 30.0);
		FetchConsole(FString::Printf(TEXT("/events?level=%s&limit=8"), *EventLevel), &EventsJ, Now, 2.0);
		FetchConsole(TEXT("/power"), &PowerJ, Now, 3.0);
		FetchConsole(TEXT("/history/temps?window=300"), &TempsJ, Now, 3.0);
		FetchConsole(TEXT("/imu"), &ImuJ, Now, 2.0);
		if (Now >= NextCameraFetch && !bCameraInFlight)
		{
			NextCameraFetch = Now + 0.25;   // about 4 Hz
			FetchCamera();
		}
		break;
	case ER1PanelTab::Motions:
		FetchConsole(TEXT("/poses"), &PosesJ, Now, 15.0);
		FetchConsole(TEXT("/motions"), &MotionsJ, Now, 5.0);
		FetchConsole(TEXT("/player"), &PlayerJ, Now, 0.7);
		FetchConsole(TEXT("/limits"), &LimitsJ, Now, 3.0);
		break;
	case ER1PanelTab::Setup:
		FetchConsole(TEXT("/preflight"), &PreflightJ, Now, 2.0);
		FetchConsole(TEXT("/network"), &NetworkJ, Now, 15.0);
		FetchConsole(TEXT("/calibration/offsets"), &CalibSavedJ, Now, 5.0);
		break;
	case ER1PanelTab::Help:
		FetchConsole(TEXT("/docs"), &DocsJ, Now, 60.0);
		break;
	default:
		break;
	}
}

void AR1BehaviourPanel::FetchConsole(const FString& Path, TSharedPtr<FJsonObject>* Slot, double Now, double Every)
{
	const double* Next = ConsoleNextFetch.Find(Path);
	if ((Next && Now < *Next) || ConsoleInFlight.Contains(Path))
	{
		return;
	}
	ConsoleNextFetch.Add(Path, Now + Every);
	ConsoleInFlight.Add(Path);
	GetJson(Path, [this, Path, Slot](int32 Code, const FString& Body)
	{
		ConsoleInFlight.Remove(Path);
		TSharedPtr<FJsonObject> J;
		if (Code == 200 && FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Body), J) && J.IsValid())
		{
			*Slot = J;
		}
		else if (Code == 404)
		{
			*Slot = MakeShared<FJsonObject>();
			(*Slot)->SetStringField(TEXT("error"), FString::Printf(TEXT("this service has no %s: restart it from the current RobotControl"), *Path));
		}
	});
}

void AR1BehaviourPanel::FetchCamera()
{
	bCameraInFlight = true;
	TSharedRef<IHttpRequest, ESPMode::ThreadSafe> Req = FHttpModule::Get().CreateRequest();
	Req->SetURL(ServiceUrl + TEXT("/camera.png?source=twin&mode=virtual"));
	Req->SetVerb(TEXT("GET"));
	Req->SetTimeout(3.f);
	Req->OnProcessRequestComplete().BindWeakLambda(this, [this](FHttpRequestPtr, FHttpResponsePtr Resp, bool bOk)
	{
		bCameraInFlight = false;
		if (!bOk || !Resp.IsValid())
		{
			CameraNote = TEXT("camera: service not reachable");
			return;
		}
		if (Resp->GetResponseCode() != 200)
		{
			CameraNote = FString::Printf(TEXT("camera: HTTP %d"), Resp->GetResponseCode());
			return;
		}
		IImageWrapperModule& Mod = FModuleManager::LoadModuleChecked<IImageWrapperModule>(FName(TEXT("ImageWrapper")));
		TSharedPtr<IImageWrapper> Png = Mod.CreateImageWrapper(EImageFormat::PNG);
		const TArray<uint8>& Bytes = Resp->GetContent();
		TArray64<uint8> Raw;
		if (!Png.IsValid() || !Png->SetCompressed(Bytes.GetData(), Bytes.Num()) || !Png->GetRaw(ERGBFormat::BGRA, 8, Raw))
		{
			CameraNote = TEXT("camera: unreadable PNG");
			return;
		}
		const int32 W = static_cast<int32>(Png->GetWidth()), H = static_cast<int32>(Png->GetHeight());
		if (!CameraTex || CameraTex->GetSizeX() != W || CameraTex->GetSizeY() != H)
		{
			CameraTex = UTexture2D::CreateTransient(W, H, PF_B8G8R8A8);
			if (!CameraTex)
			{
				return;
			}
			CameraTex->SRGB = true;
		}
		FTexture2DMipMap& Mip = CameraTex->GetPlatformData()->Mips[0];
		void* Dst = Mip.BulkData.Lock(LOCK_READ_WRITE);
		FMemory::Memcpy(Dst, Raw.GetData(), FMath::Min<int64>(Raw.Num(), static_cast<int64>(W) * H * 4));
		Mip.BulkData.Unlock();
		CameraTex->UpdateResource();
		CameraNote.Reset();
	});
	Req->ProcessRequest();
}

void AR1BehaviourPanel::ConsolePost(const FString& Path, const FString& Body, const FString& OkToast, TFunction<void(TSharedPtr<FJsonObject>)> OnOk)
{
	PostJson(Path, Body, [this, Path, OkToast, OnOk](int32 Code, TSharedPtr<FJsonObject> J)
	{
		const bool bOk = Code == 200 && JBool(J, TEXT("ok"));
		if (bOk)
		{
			if (!OkToast.IsEmpty())
			{
				ShowToast(OkToast, OkCol);
			}
			if (OnOk)
			{
				OnOk(J);
			}
		}
		else
		{
			FString Err = JStr(J, TEXT("error"));
			if (Err.IsEmpty())
			{
				Err = Code == 0 ? FString(TEXT("behaviour service not reachable"))
					: Code == 404 ? FString::Printf(TEXT("this service has no %s (restart it from the current RobotControl)"), *Path)
					: FString::Printf(TEXT("HTTP %d"), Code);
			}
			ShowToast(FString::Printf(TEXT("Refused: %s"), *Err), BadCol);
		}
		ConsoleNextFetch.Reset();   // show the effect right away
		NextPoll = 0.0;
	});
}

void AR1BehaviourPanel::SendEStop()
{
	VoiceWalkCmd = FVector::ZeroVector;
	PostJson(TEXT("/estop"), TEXT("{}"), [this](int32 Code, TSharedPtr<FJsonObject> J)
	{
		if (Code == 200)
		{
			TArray<FString> Stopped;
			for (const TSharedPtr<FJsonValue>& V : JArr(J, TEXT("stopped")))
			{
				Stopped.Add(V->AsString());
			}
			ShowToast(FString::Printf(TEXT("E-STOP sent: %s. On the robot the remote (L2+B, hold 5 s) is the real emergency stop."),
				Stopped.Num() ? *(TEXT("stopped ") + FString::Join(Stopped, TEXT(", "))) : TEXT("nothing was running")), BadCol);
		}
		else
		{
			Post(TEXT("/stop"));   // older service without /estop: the plain STOP
			ShowToast(Code == 0 ? TEXT("E-STOP: behaviour service not reachable. Use the remote (L2+B, hold 5 s).")
				: TEXT("E-STOP: this service has no /estop, STOP sent instead. Remote L2+B is the real emergency stop."), BadCol);
		}
		ConsoleNextFetch.Reset();
		NextPoll = 0.0;
	});
}

void AR1BehaviourPanel::OpenDoc(const FString& Name)
{
	DocName = Name;
	DocTitle = Name;
	DocMarkdown = TEXT("Loading...");
	DocWrapped.Reset();
	DocScroll = 0.f;
	GetJson(FString::Printf(TEXT("/docs/%s"), *Name), [this, Name](int32 Code, const FString& Body)
	{
		if (DocName != Name)
		{
			return;
		}
		TSharedPtr<FJsonObject> J;
		if (Code == 200 && FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Body), J) && J.IsValid())
		{
			DocTitle = JStr(J, TEXT("title"));
			DocMarkdown = JStr(J, TEXT("markdown"));
		}
		else
		{
			DocMarkdown = FString::Printf(TEXT("Could not load %s (HTTP %d)."), *Name, Code);
		}
		DocWrapped.Reset();
	});
}

// ------------------------------------------------------------------------------------------------ drawing helpers
void AR1BehaviourPanel::DrawEStop(float X, float Y, float W, float H)
{
	DrawButton({IdEStop, TEXT("E-STOP"), {X, Y}, {W, H}, true, EStopCol});
}

void AR1BehaviourPanel::DrawSection(const FString& Title, float X, float Inner, float& Y, const FLinearColor& Accent)
{
	Y += 4.f;
	DrawRect(Accent, X, Y + 3.f, 3.f, 14.f);
	DrawText(Title, Accent, X + 9.f, Y, GEngine->GetMediumFont(), 0.86f);
	Y += 20.f;
	DrawRect(DividerCol, X, Y, Inner, 1.f);
	Y += 5.f;
}

void AR1BehaviourPanel::DrawPendingPose(float X, float Inner, float& Y)
{
	if (PendingPose.IsEmpty() || FPlatformTime::Seconds() - PendingPoseAt > 30.0)
	{
		return;
	}
	const bool bIdle = bServiceOnline && Job.IsEmpty() && !WalkState.bRunning;
	DrawButton({IdPosePreview, FString::Printf(TEXT("Preview pose: %s (twin only)"), *PendingPoseLabel), {X, Y}, {Inner * 0.62f, 26.f}, bIdle, TwinCol});
	DrawText(TEXT("nothing starts until you click"), DimCol, X + Inner * 0.62f + 10.f, Y + 5.f, GEngine->GetMediumFont(), 0.78f);
	Y += 32.f;
}

// ------------------------------------------------------------------------------------------------ Health tab
void AR1BehaviourPanel::DrawHealthTab(float X, float Inner, float& Y)
{
	UFont* Font = GEngine->GetMediumFont();
	const float CamW = 216.f, CamH = 174.f;
	const float LeftW = Inner - CamW - 12.f;
	const float Top = Y;

	// ---- system health rows (left)
	{
		const FString Overall = JStr(HealthJ, TEXT("overall"));
		const int32 Failing = static_cast<int32>(JNum(HealthJ, TEXT("failing")));
		const FString Head = !HealthJ.IsValid() ? FString(TEXT("SYSTEM HEALTH  (probing...)"))
			: !JStr(HealthJ, TEXT("error")).IsEmpty() ? FString(TEXT("SYSTEM HEALTH  (unavailable)"))
			: FString::Printf(TEXT("SYSTEM HEALTH  %s%s"), Overall == TEXT("ok") ? TEXT("OK") : TEXT("DEGRADED"),
				Failing > 0 ? *FString::Printf(TEXT(" (%d failing)"), Failing) : TEXT(""));
		DrawSection(Head, X, LeftW, Y, Overall == TEXT("ok") ? OkCol : HealthJ.IsValid() ? WarnCol : ConsoleCol);
		if (!JStr(HealthJ, TEXT("error")).IsEmpty())
		{
			DrawWrapped(JStr(HealthJ, TEXT("error")), X, Y, LeftW, BadCol, 0.8f);
		}
		for (const TSharedPtr<FJsonValue>& V : JArr(HealthJ, TEXT("rows")))
		{
			const TSharedPtr<FJsonObject> R = AsObj(V);
			const bool bOk = JBool(R, TEXT("ok")), bOpt = JBool(R, TEXT("optional"));
			const FString Word = bOk ? TEXT("OK") : bOpt ? TEXT("OFF") : TEXT("FAIL");
			const FLinearColor C = bOk ? OkCol : bOpt ? DimCol : BadCol;
			DrawRect(FLinearColor(C.R, C.G, C.B, 0.25f), X, Y + 1.f, 38.f, 15.f);
			DrawText(Word, C, X + 4.f, Y, Font, 0.72f);
			const FString Line = FString::Printf(TEXT("%s  -  %s"), *JStr(R, TEXT("name")), *JStr(R, TEXT("detail")));
			DrawText(Fit(this, Line, LeftW - 46.f, 0.72f), bOk ? TextCol : bOpt ? DimCol : BadCol, X + 44.f, Y, Font, 0.72f);
			Y += 16.f;
		}
	}
	const float LeftBottom = Y;

	// ---- head camera + IMU (right)
	{
		const float CX = X + Inner - CamW;
		float CY = Top + 4.f;
		DrawText(TEXT("HEAD CAMERA (twin)"), ConsoleCol, CX, CY, Font, 0.8f);
		CY += 18.f;
		DrawRect(CardBg, CX, CY, CamW, CamH);
		if (CameraTex)
		{
			DrawTexture(CameraTex, CX, CY, CamW, CamH, 0.f, 0.f, 1.f, 1.f);
		}
		else
		{
			float Ty = CY + CamH * 0.4f;
			DrawWrapped(CameraNote.IsEmpty() ? FString(TEXT("loading the twin's head_rgbd view...")) : CameraNote, CX + 8.f, Ty, CamW - 16.f, DimCol, 0.75f);
		}
		CY += CamH + 3.f;
		DrawLine(TEXT("Twin view at the virtual pose. Robot camera: not streamed yet."), CX, CY, DimCol, 0.62f);
		CY += 4.f;
		const FString Verdict = JStr(ImuJ, TEXT("verdict"));
		DrawLine(FString::Printf(TEXT("IMU CHECK (%s): %s"), *JStr(ImuJ, TEXT("source")), Verdict.IsEmpty() ? TEXT("...") : *Verdict.ToUpper()),
			CX, CY, Verdict == TEXT("pass") ? OkCol : Verdict == TEXT("warn") ? WarnCol : DimCol, 0.8f);
		if (JBool(ImuJ, TEXT("available")))
		{
			const TArray<TSharedPtr<FJsonValue>> G = JArr(ImuJ, TEXT("gravity_body"));
			DrawLine(FString::Printf(TEXT("tilt %.2f deg   yaw drift %s deg/min"), JNum(ImuJ, TEXT("tilt_deg")),
				JHasNum(ImuJ, TEXT("yaw_drift_deg_per_min")) ? *FString::Printf(TEXT("%+.2f"), JNum(ImuJ, TEXT("yaw_drift_deg_per_min"))) : TEXT("--")), CX, CY, TextCol, 0.72f);
			if (G.Num() >= 3)
			{
				DrawLine(FString::Printf(TEXT("gravity (body)  %.3f  %.3f  %.3f"), G[0]->AsNumber(), G[1]->AsNumber(), G[2]->AsNumber()), CX, CY, DimCol, 0.72f);
			}
			DrawLine(FString::Printf(TEXT("roll/pitch std %.2f / %.2f deg, %d samples"), JNum(ImuJ, TEXT("roll_std_deg")), JNum(ImuJ, TEXT("pitch_std_deg")),
				static_cast<int32>(JNum(ImuJ, TEXT("samples")))), CX, CY, DimCol, 0.72f);
		}
		for (const TSharedPtr<FJsonValue>& N : JArr(ImuJ, TEXT("notes")))
		{
			DrawWrapped(N->AsString(), CX, CY, CamW, WarnCol, 0.68f);
		}
		Y = FMath::Max(LeftBottom, CY) + 4.f;
	}

	// ---- versions
	DrawSection(TEXT("VERSIONS"), X, Inner, Y, ConsoleCol);
	if (VersionsJ.IsValid())
	{
		const TSharedPtr<FJsonObject> Repo = JObj(VersionsJ, TEXT("repo"));
		const TSharedPtr<FJsonObject> Pk = JObj(VersionsJ, TEXT("packages"));
		DrawLine(Fit(this, FString::Printf(TEXT("service %s   commit %s (%d changed files)   mujoco %s   onnxruntime %s   sdk2 %s"),
			*JStr(VersionsJ, TEXT("service")), *JStr(Repo, TEXT("commit")), static_cast<int32>(JNum(Repo, TEXT("dirty_files"))),
			*JStr(Pk, TEXT("mujoco")), *JStr(Pk, TEXT("onnxruntime")), *JStr(Pk, TEXT("unitree_sdk2py"))), Inner, 0.72f), X, Y, TextCol, 0.72f);
		for (const TSharedPtr<FJsonValue>& V : JArr(VersionsJ, TEXT("policies")))
		{
			const TSharedPtr<FJsonObject> P = AsObj(V);
			const bool bOk = JBool(P, TEXT("ok"));
			DrawLine(Fit(this, FString::Printf(TEXT("%s  %s   sha %s   trained %s"), bOk ? TEXT("OK  ") : TEXT("FAIL"), *JStr(P, TEXT("name")),
				*JStr(P, TEXT("policy_sha256")).Left(12), *JStr(P, TEXT("trained_at"))), Inner, 0.72f), X, Y, bOk ? OkCol : BadCol, 0.72f);
			DrawLine(Fit(this, FString::Printf(TEXT("      gate: %s"), *JStr(P, TEXT("gate"))), Inner, 0.68f), X, Y, DimCol, 0.68f);
		}
		for (const TSharedPtr<FJsonValue>& V : JArr(VersionsJ, TEXT("warnings")))
		{
			DrawWrapped(FString::Printf(TEXT("! %s"), *V->AsString()), X, Y, Inner, WarnCol, 0.72f);
		}
		DrawLine(Fit(this, FString::Printf(TEXT("robot firmware: %s"), *JStr(JObj(VersionsJ, TEXT("robot")), TEXT("note"))), Inner, 0.68f), X, Y, DimCol, 0.68f);
	}
	else
	{
		DrawLine(TEXT("loading..."), X, Y, DimCol, 0.75f);
	}

	// ---- power + temperatures
	DrawSection(TEXT("POWER AND TEMPERATURES"), X, Inner, Y, ConsoleCol);
	if (JBool(PowerJ, TEXT("available")))
	{
		const FString Left = JHasNum(PowerJ, TEXT("minutes_left")) ? FString::Printf(TEXT("   about %.0f min left"), JNum(PowerJ, TEXT("minutes_left"))) : FString();
		DrawLine(FString::Printf(TEXT("Battery (%s): %.1f V   about %.0f %% (estimate)%s"), *JStr(PowerJ, TEXT("source")),
			JNum(PowerJ, TEXT("latest_v")), JNum(PowerJ, TEXT("soc_pct")), *Left), X, Y, JBool(PowerJ, TEXT("low")) ? WarnCol : TextCol, 0.8f);
		DrawLine(Fit(this, FString::Printf(TEXT("%s; current: %s"), *JStr(PowerJ, TEXT("assumption")), *JStr(PowerJ, TEXT("current_note"))), Inner, 0.66f), X, Y, DimCol, 0.66f);
	}
	else
	{
		DrawLine(TEXT("Battery: no bus voltage yet (Snapshot or Monitor reads it; the estimate needs a reading)."), X, Y, DimCol, 0.75f);
	}
	{
		const TArray<TSharedPtr<FJsonValue>> Names = JArr(TempsJ, TEXT("joint_names"));
		const TArray<TSharedPtr<FJsonValue>> Latest = JArr(TempsJ, TEXT("latest"));
		if (Latest.Num() == Names.Num() && Latest.Num() > 0)
		{
			TArray<int32> Order;
			for (int32 i = 0; i < Latest.Num(); ++i)
			{
				Order.Add(i);
			}
			Algo::Sort(Order, [&Latest](int32 A, int32 B) { return Latest[A]->AsNumber() > Latest[B]->AsNumber(); });
			TArray<FString> Top3;
			for (int32 k = 0; k < FMath::Min(3, Order.Num()); ++k)
			{
				Top3.Add(FString::Printf(TEXT("%s %.0f C"), *Names[Order[k]]->AsString(), Latest[Order[k]]->AsNumber()));
			}
			const float HotC = static_cast<float>(Latest[Order[0]]->AsNumber());
			const float RowY = Y;
			DrawLine(FString::Printf(TEXT("Hottest (%s): %s"), *JStr(TempsJ, TEXT("source")), *FString::Join(Top3, TEXT(",  "))), X, Y,
				HotC >= 65.f ? BadCol : HotC >= 55.f ? WarnCol : TextCol, 0.78f);
			// sparkline of the hottest joint over the window, with the 55 C warning line
			const TArray<TSharedPtr<FJsonValue>> Series = JArr(TempsJ, TEXT("series"));
			if (Series.IsValidIndex(Order[0]))
			{
				const TArray<TSharedPtr<FJsonValue>>& S = Series[Order[0]]->AsArray();
				const float SW = 180.f, SH = 30.f, SX = X + Inner - SW, SY = RowY;
				DrawRect(CardBg, SX, SY, SW, SH);
				const float Lo = 20.f, Hi = 80.f;
				auto ToY = [&](float C) { return SY + SH - (FMath::Clamp(C, Lo, Hi) - Lo) / (Hi - Lo) * SH; };
				AHUD::DrawLine(SX, ToY(55.f), SX + SW, ToY(55.f), FLinearColor(WarnCol.R, WarnCol.G, WarnCol.B, 0.5f), 1.f);
				for (int32 i = 1; i < S.Num(); ++i)
				{
					const float X0s = SX + (i - 1) * SW / FMath::Max(1, S.Num() - 1), X1s = SX + i * SW / FMath::Max(1, S.Num() - 1);
					AHUD::DrawLine(X0s, ToY(S[i - 1]->AsNumber()), X1s, ToY(S[i]->AsNumber()), CyanCol, 1.5f);
				}
				Y = FMath::Max(Y, SY + SH + 2.f);
			}
		}
		else
		{
			DrawLine(TEXT("Temperatures: no readings yet (Snapshot or Monitor)."), X, Y, DimCol, 0.75f);
		}
	}

	// ---- events
	DrawSection(TEXT("ALARMS AND FAULTS"), X, Inner, Y, ConsoleCol);
	{
		const TCHAR* Levels[] = {TEXT("info"), TEXT("warn"), TEXT("critical")};
		const TCHAR* Labels[] = {TEXT("All"), TEXT("Warn+"), TEXT("Critical")};
		for (int32 i = 0; i < 3; ++i)
		{
			DrawButton({CId(FString::Printf(TEXT("ev_%s"), Levels[i])), Labels[i], {X + i * 86.f, Y}, {80.f, 24.f}, true,
				EventLevel == Levels[i] ? ChipOnCol : ChipCol});
		}
		const TSharedPtr<FJsonObject> Counts = JObj(EventsJ, TEXT("counts"));
		DrawText(FString::Printf(TEXT("session: %d info, %d warn, %d critical"), static_cast<int32>(JNum(Counts, TEXT("info"))),
			static_cast<int32>(JNum(Counts, TEXT("warn"))), static_cast<int32>(JNum(Counts, TEXT("critical")))), DimCol, X + 270.f, Y + 4.f, Font, 0.72f);
		Y += 30.f;
		const TArray<TSharedPtr<FJsonValue>> Evs = JArr(EventsJ, TEXT("events"));
		if (Evs.Num() == 0)
		{
			DrawLine(TEXT("No events at this level."), X, Y, DimCol, 0.75f);
		}
		for (const TSharedPtr<FJsonValue>& V : Evs)
		{
			const TSharedPtr<FJsonObject> E = AsObj(V);
			const FString Lv = JStr(E, TEXT("level"));
			const FLinearColor C = Lv == TEXT("critical") ? BadCol : Lv == TEXT("warn") ? WarnCol : DimCol;
			DrawLine(Fit(this, FString::Printf(TEXT("%-8s %-9s %-11s %s"), *Ago(JNum(E, TEXT("at"))), *Lv.ToUpper(), *JStr(E, TEXT("kind")),
				*JStr(E, TEXT("message"))), Inner, 0.72f), X, Y, C, 0.72f);
		}
	}
}

// ------------------------------------------------------------------------------------------------ Motions tab
void AR1BehaviourPanel::DrawMotionsTab(float X, float Inner, float& Y)
{
	UFont* Font = GEngine->GetMediumFont();
	const bool bIdle = bServiceOnline && Job.IsEmpty() && !WalkState.bRunning;
	const bool bPlaying = Job == TEXT("motion") || Job == TEXT("pose");

	DrawWrapped(TEXT("Simulation only: poses, motions and modes run on the stand twin and are mirrored here. They never target the real robot."),
		X, Y, Inner, DimCol, 0.74f);

	// ---- poses
	DrawSection(TEXT("POSES"), X, Inner, Y, TwinCol * 1.35f);
	{
		const TArray<TSharedPtr<FJsonValue>> Poses = JArr(PosesJ, TEXT("poses"));
		const float Gap = 6.f, W = (Inner - 3.f * Gap) / 4.f;
		for (int32 i = 0; i < Poses.Num(); ++i)
		{
			const TSharedPtr<FJsonObject> P = AsObj(Poses[i]);
			DrawButton({CId(TEXT("pose_") + JStr(P, TEXT("name"))), JStr(P, TEXT("label")), {X + (i % 4) * (W + Gap), Y + (i / 4) * 30.f}, {W, 26.f}, bIdle, ChipCol});
		}
		Y += FMath::Max(1, (Poses.Num() + 3) / 4) * 30.f + 2.f;
		if (Poses.Num() == 0)
		{
			DrawLine(PosesJ.IsValid() ? JStr(PosesJ, TEXT("error")) : TEXT("loading..."), X, Y, DimCol, 0.75f);
		}
	}

	// ---- motions + store
	DrawSection(TEXT("MOTIONS  (built-in + yours; export/import = local action store)"), X, Inner, Y, TwinCol * 1.35f);
	{
		const TArray<TSharedPtr<FJsonValue>> Ms = JArr(MotionsJ, TEXT("motions"));
		for (int32 i = 0; i < FMath::Min(8, Ms.Num()); ++i)
		{
			const TSharedPtr<FJsonObject> M = AsObj(Ms[i]);
			const FString Name = JStr(M, TEXT("name"));
			const bool bValid = JBool(M, TEXT("valid"));
			const FString Line = FString::Printf(TEXT("%s  %s  (%s, %.1f s, %d keys)"), bValid ? TEXT("OK ") : TEXT("BAD"), *JStr(M, TEXT("title")),
				*JStr(M, TEXT("source")), JNum(M, TEXT("duration_s")), static_cast<int32>(JNum(M, TEXT("keyframes"))));
			DrawText(Fit(this, Line, Inner - 150.f, 0.78f), bValid ? TextCol : BadCol, X, Y + 4.f, Font, 0.78f);
			DrawButton({CId(TEXT("play_") + Name), TEXT("Play"), {X + Inner - 142.f, Y}, {64.f, 24.f}, bIdle && bValid, TwinCol});
			DrawButton({CId(TEXT("export_") + Name), TEXT("Export"), {X + Inner - 72.f, Y}, {72.f, 24.f}, bServiceOnline, ChipCol});
			Y += 28.f;
		}
		if (Ms.Num() == 0)
		{
			DrawLine(MotionsJ.IsValid() ? JStr(MotionsJ, TEXT("error")) : TEXT("loading..."), X, Y, DimCol, 0.75f);
		}
		DrawButton({CId(TEXT("loop")), bMotionLoop ? TEXT("Loop: on") : TEXT("Loop: off"), {X, Y}, {96.f, 26.f}, true, bMotionLoop ? ChipOnCol : ChipCol});
		DrawButton({CId(TEXT("player_stop")), TEXT("Stop playback"), {X + 102.f, Y}, {128.f, 26.f}, bPlaying, StopCol});
		DrawButton({CId(TEXT("import")), TEXT("Import from Saved/MotionStore"), {X + 236.f, Y}, {Inner - 236.f, 26.f}, bServiceOnline, ChipCol});
		Y += 32.f;
		if (JBool(PlayerJ, TEXT("running")))
		{
			DrawLine(FString::Printf(TEXT("Playing %s on the twin: %.1f s%s"), *JStr(PlayerJ, TEXT("name")), JNum(PlayerJ, TEXT("t")),
				JBool(PlayerJ, TEXT("loop")) ? TEXT(" (loop)") : TEXT("")), X, Y, CyanCol, 0.8f);
		}
		else if (!JStr(PlayerJ, TEXT("reason")).IsEmpty())
		{
			DrawLine(Fit(this, FString::Printf(TEXT("Last playback stopped: %s"), *JStr(PlayerJ, TEXT("reason"))), Inner, 0.75f), X, Y, BadCol, 0.75f);
		}
	}

	// ---- teach mode
	DrawSection(TEXT("TEACH  (record joint angles, read-only)"), X, Inner, Y, TwinCol * 1.35f);
	{
		const TSharedPtr<FJsonObject> Rec = JObj(PlayerJ, TEXT("recording"));
		const bool bRec = JBool(Rec, TEXT("active"));
		const TCHAR* Srcs[] = {TEXT("practice"), TEXT("virtual"), TEXT("real")};
		for (int32 i = 0; i < 3; ++i)
		{
			DrawButton({CId(FString::Printf(TEXT("recsrc_%s"), Srcs[i])), Srcs[i], {X + i * 86.f, Y}, {80.f, 26.f}, !bRec,
				RecordSource == Srcs[i] ? ChipOnCol : ChipCol});
		}
		DrawButton({CId(bRec ? TEXT("rec_stop") : TEXT("rec_start")), bRec ? TEXT("Stop and keep") : TEXT("Record"), {X + 264.f, Y}, {130.f, 26.f},
			bServiceOnline, bRec ? StopCol : TwinCol});
		Y += 30.f;
		if (bRec)
		{
			DrawLine(FString::Printf(TEXT("Recording %s: %d samples, %.1f s"), *JStr(Rec, TEXT("source")), static_cast<int32>(JNum(Rec, TEXT("samples"))),
				JNum(Rec, TEXT("elapsed_s"))), X, Y, CyanCol, 0.78f);
		}
		else
		{
			DrawLine(RecordSource == TEXT("real") ? TEXT("real: start Monitor (read-only) and move a limp arm by hand.")
				: RecordSource == TEXT("virtual") ? TEXT("virtual: move the twin (behaviour preview, sliders) while recording.")
				: TEXT("practice: play a pose or motion on the practice twin while recording."), X, Y, DimCol, 0.72f);
		}
		if (RecordedJ.IsValid())
		{
			const TSharedPtr<FJsonObject> Clip = JObj(RecordedJ, TEXT("clip"));
			const TSharedPtr<FJsonObject> Val = JObj(RecordedJ, TEXT("validation"));
			const bool bOk = JBool(Val, TEXT("ok"));
			DrawLine(FString::Printf(TEXT("Take \"%s\": %d keyframes, %.1f s, %s"), *JStr(Clip, TEXT("name")), JArr(Clip, TEXT("keyframes")).Num(),
				JNum(Val, TEXT("duration_s")), bOk ? TEXT("valid") : TEXT("INVALID")), X, Y, bOk ? OkCol : BadCol, 0.78f);
			DrawButton({CId(TEXT("rec_save")), TEXT("Save take"), {X, Y}, {110.f, 24.f}, bOk && bServiceOnline, TwinCol});
			DrawButton({CId(TEXT("rec_discard")), TEXT("Discard"), {X + 116.f, Y}, {90.f, 24.f}, true, ChipCol});
			Y += 28.f;
		}
	}

	// ---- modes
	DrawSection(TEXT("MODE  (twin only; only the joint you pick)"), X, Inner, Y, TwinCol * 1.35f);
	{
		const bool bModeOn = Job == TEXT("mode");
		DrawButton({CId(TEXT("mj_prev")), TEXT("<"), {X, Y}, {28.f, 26.f}, !bModeOn, ChipCol});
		const FString JL = JointMetas.IsValidIndex(ModeJointIdx) ? JointMetas[ModeJointIdx].Label : FString(TEXT("?"));
		DrawText(JL, TextCol, X + 36.f, Y + 4.f, Font, 0.82f);
		DrawButton({CId(TEXT("mj_next")), TEXT(">"), {X + 134.f, Y}, {28.f, 26.f}, !bModeOn, ChipCol});
		const TCHAR* Modes[] = {TEXT("damp"), TEXT("hold"), TEXT("zero_torque"), TEXT("off")};
		const TCHAR* ModeLabels[] = {TEXT("Damp"), TEXT("Hold"), TEXT("Zero torque"), TEXT("Off")};
		const float MW = (Inner - 172.f - 3.f * 6.f) / 4.f;
		for (int32 i = 0; i < 4; ++i)
		{
			const bool bEnabled = i == 3 ? bModeOn : bIdle;
			DrawButton({CId(FString::Printf(TEXT("mode_%s"), Modes[i])), ModeLabels[i], {X + 172.f + i * (MW + 6.f), Y}, {MW, 26.f}, bEnabled,
				i == 3 ? StopCol : ChipCol});
		}
		Y += 30.f;
		if (bModeOn)
		{
			DrawLine(FString::Printf(TEXT("Mode %s active on the twin%s"), *JStr(PlayerJ, TEXT("mode")),
				JArr(PlayerJ, TEXT("joints")).Num() ? *FString::Printf(TEXT(" (%s)"), *JArr(PlayerJ, TEXT("joints"))[0]->AsString()) : TEXT("")), X, Y, CyanCol, 0.78f);
		}
	}

	// ---- session limits
	DrawSection(TEXT("SESSION LIMITS  (can only tighten the built-in limits)"), X, Inner, Y, WarnCol);
	{
		const TSharedPtr<FJsonObject> Vals = JObj(LimitsJ, TEXT("values"));
		const TSharedPtr<FJsonObject> Specs = JObj(LimitsJ, TEXT("specs"));
		if (!Vals.IsValid())
		{
			DrawLine(LimitsJ.IsValid() ? JStr(LimitsJ, TEXT("error")) : TEXT("loading..."), X, Y, DimCol, 0.75f);
		}
		for (int32 i = 0; i < UE_ARRAY_COUNT(LimitKeys) && Vals.IsValid(); ++i)
		{
			const TSharedPtr<FJsonObject> Sp = JObj(Specs, LimitKeys[i]);
			const double V = JNum(Vals, LimitKeys[i]), Mx = JNum(Sp, TEXT("max"), V), Mn = JNum(Sp, TEXT("min"), 0.0);
			const bool bTight = V < Mx - 1e-6;
			DrawText(Fit(this, JStr(Sp, TEXT("label")), Inner - 220.f, 0.76f), bTight ? WarnCol : TextCol, X, Y + 3.f, Font, 0.76f);
			DrawText(FString::Printf(TEXT("%g %s"), V, *JStr(Sp, TEXT("unit"))), bTight ? WarnCol : TextCol, X + Inner - 206.f, Y + 3.f, Font, 0.76f);
			DrawButton({CId(FString::Printf(TEXT("lim_dec_%d"), i)), TEXT("-"), {X + Inner - 68.f, Y}, {30.f, 22.f}, bServiceOnline && V > Mn + 1e-6, ChipCol});
			DrawButton({CId(FString::Printf(TEXT("lim_inc_%d"), i)), TEXT("+"), {X + Inner - 32.f, Y}, {30.f, 22.f}, bServiceOnline && bTight, ChipCol});
			Y += 25.f;
		}
		DrawButton({CId(TEXT("lim_reset")), TEXT("Reset limits"), {X, Y}, {130.f, 24.f}, bServiceOnline, ChipCol});
		Y += 30.f;
	}
}

// ------------------------------------------------------------------------------------------------ Setup tab
void AR1BehaviourPanel::DrawSetupTab(float X, float Inner, float& Y)
{
	UFont* Font = GEngine->GetMediumFont();

	// ---- pre-flight
	{
		const bool bDone = JBool(PreflightJ, TEXT("complete"));
		const int32 Open = JArr(PreflightJ, TEXT("missing")).Num();
		const FString Head = !PreflightJ.IsValid() ? FString(TEXT("PRE-FLIGHT CHECKLIST  (loading...)"))
			: bDone ? FString::Printf(TEXT("PRE-FLIGHT CHECKLIST  COMPLETE (valid %.0f min)"), JNum(PreflightJ, TEXT("valid_for_s")) / 60.0)
			: FString::Printf(TEXT("PRE-FLIGHT CHECKLIST  %d open (Move REAL robot stays locked)"), Open);
		DrawSection(Head, X, Inner, Y, bDone ? OkCol : WarnCol);
		for (const TSharedPtr<FJsonValue>& V : JArr(PreflightJ, TEXT("items")))
		{
			const TSharedPtr<FJsonObject> It = AsObj(V);
			const bool bOk = JBool(It, TEXT("ok"));
			const bool bManual = JStr(It, TEXT("kind")) == TEXT("manual");
			if (bManual)
			{
				DrawButton({CId(TEXT("pf_") + JStr(It, TEXT("id"))), bOk ? TEXT("[x]") : TEXT("[ ]"), {X, Y}, {34.f, 20.f}, bServiceOnline, bOk ? OkCol * 0.7f : ChipCol});
			}
			else
			{
				DrawText(bOk ? TEXT("OK") : TEXT("NO"), bOk ? OkCol : BadCol, X + 6.f, Y + 2.f, Font, 0.72f);
			}
			const FString Det = JStr(It, TEXT("detail"));
			DrawText(Fit(this, JStr(It, TEXT("label")) + (Det.IsEmpty() ? FString() : FString(TEXT("  -  ")) + Det), Inner - 44.f, 0.74f),
				bOk ? TextCol : bManual ? TextCol : DimCol, X + 42.f, Y + 2.f, Font, 0.74f);
			Y += 22.f;
		}
		DrawButton({CId(TEXT("pf_reset")), TEXT("Reset checklist"), {X, Y}, {140.f, 24.f}, bServiceOnline, ChipCol});
		DrawText(TEXT("Manual ticks last 30 min. Practice runs never need it."), DimCol, X + 150.f, Y + 4.f, Font, 0.7f);
		Y += 30.f;
	}

	// ---- calibration
	DrawSection(TEXT("CALIBRATION  (joint zero offsets: stored for review, never applied)"), X, Inner, Y, ConsoleCol);
	{
		const TCHAR* Srcs[] = {TEXT("practice"), TEXT("real")};
		const TCHAR* Refs[] = {TEXT("zero"), TEXT("home")};
		for (int32 i = 0; i < 2; ++i)
		{
			DrawButton({CId(FString::Printf(TEXT("calsrc_%s"), Srcs[i])), Srcs[i], {X + i * 86.f, Y}, {80.f, 24.f}, true, CalibSource == Srcs[i] ? ChipOnCol : ChipCol});
			DrawButton({CId(FString::Printf(TEXT("calref_%s"), Refs[i])), FString::Printf(TEXT("ref: %s"), Refs[i]), {X + 186.f + i * 86.f, Y}, {80.f, 24.f}, true,
				CalibReference == Refs[i] ? ChipOnCol : ChipCol});
		}
		Y += 28.f;
		const bool bIdle = bServiceOnline && Job.IsEmpty() && !WalkState.bRunning;
		DrawButton({CId(TEXT("cal_read")), TEXT("1. Read pose (snapshot)"), {X, Y}, {184.f, 26.f}, bIdle, CalibSource == TEXT("real") ? FLinearColor(0.18f, 0.48f, 0.54f, 1.f) : TwinCol});
		DrawButton({CId(TEXT("cal_capture")), TEXT("2. Capture"), {X + 190.f, Y}, {110.f, 26.f}, bServiceOnline, ChipCol});
		DrawButton({CId(TEXT("cal_save")), bCalibConfirm ? TEXT("Confirm: store (not applied)") : TEXT("3. Store offsets..."), {X + 306.f, Y}, {Inner - 306.f, 26.f},
			bServiceOnline && CalibJ.IsValid(), bCalibConfirm ? WarnCol * 0.8f : ChipCol});
		Y += 30.f;
		DrawLine(CalibReference == TEXT("zero") ? TEXT("Reference zero: put each joint on its zero mark, then read and capture.")
			: TEXT("Reference home: the stand home pose (joint map defaults)."), X, Y, DimCol, 0.7f);
		TArray<TSharedPtr<FJsonObject>> Rows;
		for (const TSharedPtr<FJsonValue>& V : JArr(CalibJ, TEXT("rows")))
		{
			Rows.Add(AsObj(V));
		}
		Algo::Sort(Rows, [](const TSharedPtr<FJsonObject>& A, const TSharedPtr<FJsonObject>& B)
		{
			return FMath::Abs(JNum(A, TEXT("offset_deg"))) > FMath::Abs(JNum(B, TEXT("offset_deg")));
		});
		if (Rows.Num() > 0)
		{
			DrawLine(FString::Printf(TEXT("Captured from %s (ref %s), largest offsets first:"), *JStr(CalibJ, TEXT("source")), *JStr(CalibJ, TEXT("reference"))),
				X, Y, TextCol, 0.74f);
			for (int32 i = 0; i < FMath::Min(6, Rows.Num()); ++i)
			{
				const FString Vd = JStr(Rows[i], TEXT("verdict"));
				DrawLine(FString::Printf(TEXT("  %-28s measured %+7.2f   ref %+7.2f   offset %+6.2f deg   %s"), *R1Panel::PrettyJoint(JStr(Rows[i], TEXT("joint"))),
					JNum(Rows[i], TEXT("measured_deg")), JNum(Rows[i], TEXT("reference_deg")), JNum(Rows[i], TEXT("offset_deg")), *Vd.ToUpper()),
					X, Y, Vd == TEXT("ok") ? OkCol : Vd == TEXT("warn") ? WarnCol : BadCol, 0.7f);
			}
		}
		const TSharedPtr<FJsonObject> Saved = JObj(CalibSavedJ, TEXT("saved"));
		DrawLine(Saved.IsValid() ? FString::Printf(TEXT("Stored %s: %d joints (applied: no)"), *JStr(Saved, TEXT("saved_at")), JObj(Saved, TEXT("offsets_deg")).IsValid()
			? JObj(Saved, TEXT("offsets_deg"))->Values.Num() : 0) : FString(TEXT("No offsets stored yet.")), X, Y, DimCol, 0.72f);
	}

	// ---- network
	DrawSection(TEXT("CONNECTION PROFILES"), X, Inner, Y, ConsoleCol);
	{
		TArray<FString> Ifs;
		for (const TSharedPtr<FJsonValue>& V : JArr(NetworkJ, TEXT("interfaces")))
		{
			const TSharedPtr<FJsonObject> I = AsObj(V);
			TArray<FString> Ips;
			for (const TSharedPtr<FJsonValue>& Ip : JArr(I, TEXT("ipv4")))
			{
				Ips.Add(Ip->AsString());
			}
			Ifs.Add(Ips.Num() ? FString::Printf(TEXT("%s (%s)"), *JStr(I, TEXT("name")), *FString::Join(Ips, TEXT(", "))) : JStr(I, TEXT("name")));
		}
		DrawLine(Fit(this, FString::Printf(TEXT("Interfaces: %s"), Ifs.Num() ? *FString::Join(Ifs, TEXT("; ")) : TEXT("...")), Inner, 0.7f), X, Y, DimCol, 0.7f);
		const FString Active = JStr(NetworkJ, TEXT("active"));
		for (const TSharedPtr<FJsonValue>& V : JArr(NetworkJ, TEXT("profiles")))
		{
			const TSharedPtr<FJsonObject> P = AsObj(V);
			const FString Name = JStr(P, TEXT("name"));
			const FString Line = FString::Printf(TEXT("%s%s  (%s)  %s:%d  %s"), *Name, Name == Active ? TEXT(" *") : TEXT(""), *JStr(P, TEXT("target")),
				*JStr(P, TEXT("host")), static_cast<int32>(JNum(P, TEXT("port"))), *JStr(P, TEXT("note")));
			DrawText(Fit(this, Line, Inner - 80.f, 0.76f), TextCol, X, Y + 3.f, Font, 0.76f);
			DrawButton({CId(TEXT("nettest_") + Name), TEXT("Test"), {X + Inner - 70.f, Y}, {70.f, 22.f}, bServiceOnline, ChipCol});
			Y += 25.f;
			if (const FString* R = NetTestResult.Find(Name))
			{
				DrawLine(Fit(this, *R, Inner - 16.f, 0.7f), X + 16.f, Y, R->StartsWith(TEXT("reachable")) ? OkCol : WarnCol, 0.7f);
			}
		}
		DrawLine(TEXT("A profile is information (which interface to pass to --network); it does not reconfigure the service."), X, Y, DimCol, 0.66f);
	}
}

// ------------------------------------------------------------------------------------------------ Help tab
void AR1BehaviourPanel::DrawHelpTab(float X, float Inner, float& Y)
{
	UFont* Font = GEngine->GetMediumFont();
	const TArray<TSharedPtr<FJsonValue>> Docs = JArr(DocsJ, TEXT("docs"));
	if (DocName.IsEmpty() && Docs.Num() > 0)
	{
		// open the doc that belongs to the tab the operator came from; the service API by default
		OpenDoc(TEXT("service_api"));
	}
	const float Gap = 6.f, W = (Inner - 2.f * Gap) / 3.f;
	for (int32 i = 0; i < Docs.Num(); ++i)
	{
		const TSharedPtr<FJsonObject> D = AsObj(Docs[i]);
		const FString Name = JStr(D, TEXT("name"));
		DrawButton({CId(TEXT("doc_") + Name), Name, {X + (i % 3) * (W + Gap), Y + (i / 3) * 28.f}, {W, 24.f}, true, Name == DocName ? ChipOnCol : ChipCol});
	}
	Y += FMath::Max(1, (Docs.Num() + 2) / 3) * 28.f + 4.f;
	if (Docs.Num() == 0)
	{
		DrawLine(DocsJ.IsValid() ? JStr(DocsJ, TEXT("error")) : TEXT("loading the docs list..."), X, Y, DimCol, 0.75f);
	}

	const float Scale = 0.74f;
	if (DocWrapped.Num() == 0 && !DocMarkdown.IsEmpty())
	{
		TArray<FString> Src;
		DocMarkdown.Replace(TEXT("\r"), TEXT("")).ParseIntoArray(Src, TEXT("\n"), false);
		bool bCode = false;
		for (FString L : Src)
		{
			FLinearColor C = TextCol;
			if (L.StartsWith(TEXT("```")))
			{
				bCode = !bCode;
				continue;
			}
			if (bCode)
			{
				C = CyanCol * 0.85f;
			}
			else if (L.StartsWith(TEXT("#")))
			{
				while (L.StartsWith(TEXT("#")))
				{
					L.RightChopInline(1);
				}
				L.TrimStartInline();
				C = ConsoleCol * 1.3f;
			}
			else if (L.StartsWith(TEXT("|")))
			{
				C = DimCol;
			}
			if (L.IsEmpty())
			{
				DocWrapped.Emplace(FString(), C);
				continue;
			}
			// word wrap to the panel width
			TArray<FString> Words;
			L.ParseIntoArray(Words, TEXT(" "));
			FString Line;
			for (const FString& Word : Words)
			{
				const FString Try = Line.IsEmpty() ? Word : Line + TEXT(" ") + Word;
				float TW = 0.f, TH = 0.f;
				GetTextSize(Try, TW, TH, Font, Scale);
				if (TW > Inner - 12.f && !Line.IsEmpty())
				{
					DocWrapped.Emplace(Line, C);
					Line = Word;
				}
				else
				{
					Line = Try;
				}
			}
			DocWrapped.Emplace(Line, C);
		}
	}

	DrawSection(DocTitle.IsEmpty() ? FString(TEXT("DOCUMENT")) : DocTitle, X, Inner, Y, ConsoleCol);
	const float AreaH = 400.f, LineH = 15.f;
	const float Total = DocWrapped.Num() * LineH;
	DocScroll = FMath::Clamp(DocScroll, 0.f, FMath::Max(0.f, Total - AreaH));
	DrawRect(CardBg, X, Y, Inner, AreaH);
	const int32 First = FMath::FloorToInt(DocScroll / LineH);
	const int32 Count = FMath::CeilToInt(AreaH / LineH) - 1;
	for (int32 i = 0; i < Count && DocWrapped.IsValidIndex(First + i); ++i)
	{
		DrawText(DocWrapped[First + i].Key, DocWrapped[First + i].Value, X + 6.f, Y + 3.f + i * LineH, Font, Scale);
	}
	if (Total > AreaH)
	{
		const float BarH = FMath::Max(20.f, AreaH * AreaH / Total);
		DrawRect(FLinearColor(1.f, 1.f, 1.f, 0.25f), X + Inner - 4.f, Y + (AreaH - BarH) * DocScroll / FMath::Max(1.f, Total - AreaH), 4.f, BarH);
	}
	Y += AreaH + 4.f;
	DrawButton({CId(TEXT("doc_up")), TEXT("Page up"), {X, Y}, {96.f, 24.f}, DocScroll > 0.f, ChipCol});
	DrawButton({CId(TEXT("doc_down")), TEXT("Page down"), {X + 102.f, Y}, {96.f, 24.f}, DocScroll < Total - AreaH, ChipCol});
	DrawText(FString::Printf(TEXT("mouse wheel, PgUp / PgDn   line %d of %d"), First + 1, DocWrapped.Num()), DimCol, X + 210.f, Y + 4.f, Font, 0.7f);
	Y += 30.f;
}

// ------------------------------------------------------------------------------------------------ clicks
bool AR1BehaviourPanel::HandleConsoleClick(FName Id)
{
	if (Id == IdPosePreview)
	{
		const FString Name = PendingPose;
		PendingPose.Reset();
		ConsolePost(TEXT("/poses/preview"), FString::Printf(TEXT("{\"name\":\"%s\",\"target\":\"urlab\"}"), *Esc(Name)),
			FString::Printf(TEXT("Previewing pose %s on the twin."), *PendingPoseLabel));
		return true;
	}
	FString S = Id.ToString();
	if (!S.StartsWith(TEXT("c_")))
	{
		return false;
	}
	S.RightChopInline(2);
	auto After = [&S](const TCHAR* Prefix, FString& Out) { if (S.StartsWith(Prefix)) { Out = S.RightChop(FCString::Strlen(Prefix)); return true; } return false; };
	FString Arg;

	// Health
	if (After(TEXT("ev_"), Arg))
	{
		EventLevel = Arg;
		EventsJ.Reset();
	}
	// Motions
	else if (After(TEXT("pose_"), Arg))
	{
		ConsolePost(TEXT("/poses/preview"), FString::Printf(TEXT("{\"name\":\"%s\",\"target\":\"urlab\"}"), *Esc(Arg)), TEXT("Pose preview started on the twin."));
	}
	else if (After(TEXT("play_"), Arg))
	{
		ConsolePost(TEXT("/motions/play"), FString::Printf(TEXT("{\"name\":\"%s\",\"target\":\"urlab\",\"loop\":%s}"), *Esc(Arg), bMotionLoop ? TEXT("true") : TEXT("false")),
			FString::Printf(TEXT("Playing %s on the twin."), *Arg));
	}
	else if (After(TEXT("export_"), Arg))
	{
		const FString Name = Arg;
		GetJson(FString::Printf(TEXT("/motions/%s/export"), *Name), [this, Name](int32 Code, const FString& Body)
		{
			if (Code != 200)
			{
				ShowToast(FString::Printf(TEXT("Export of %s failed (HTTP %d)."), *Name, Code), BadCol);
				return;
			}
			const FString File = FPaths::Combine(MotionStoreDir(), Name + TEXT(".json"));
			if (FFileHelper::SaveStringToFile(Body, *File, FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM))
			{
				ShowToast(FString::Printf(TEXT("Exported %s to %s"), *Name, *FPaths::ConvertRelativePathToFull(File)), OkCol);
			}
			else
			{
				ShowToast(FString::Printf(TEXT("Could not write %s"), *File), BadCol);
			}
		});
	}
	else if (S == TEXT("import"))
	{
		TArray<FString> Files;
		IFileManager::Get().FindFiles(Files, *FPaths::Combine(MotionStoreDir(), TEXT("*.json")), true, false);
		if (Files.Num() == 0)
		{
			ShowToast(FString::Printf(TEXT("No packages in %s (Export one first)."), *FPaths::ConvertRelativePathToFull(MotionStoreDir())), WarnCol);
			return true;
		}
		TSharedRef<TArray<FString>> Results = MakeShared<TArray<FString>>();
		TSharedRef<int32> Pending = MakeShared<int32>(Files.Num());
		for (const FString& F : Files)
		{
			FString Content;
			if (!FFileHelper::LoadFileToString(Content, *FPaths::Combine(MotionStoreDir(), F)))
			{
				--(*Pending);
				continue;
			}
			PostJson(TEXT("/motions/import"), FString::Printf(TEXT("{\"package\":%s,\"overwrite\":false}"), *Content),
				[this, F, Results, Pending](int32 Code, TSharedPtr<FJsonObject> J)
			{
				Results->Add(Code == 200 && JBool(J, TEXT("ok")) ? FString::Printf(TEXT("%s imported"), *F)
					: FString::Printf(TEXT("%s: %s"), *F, JStr(J, TEXT("error")).IsEmpty() ? *FString::Printf(TEXT("HTTP %d"), Code) : *JStr(J, TEXT("error"))));
				if (--(*Pending) <= 0)
				{
					ShowToast(FString::Join(*Results, TEXT("; ")), TextCol);
					ConsoleNextFetch.Reset();
				}
			});
		}
	}
	else if (S == TEXT("loop"))
	{
		bMotionLoop = !bMotionLoop;
	}
	else if (S == TEXT("player_stop"))
	{
		ConsolePost(TEXT("/player/stop"), TEXT("{}"), TEXT("Playback stopped."));
	}
	else if (After(TEXT("recsrc_"), Arg))
	{
		RecordSource = Arg;
	}
	else if (S == TEXT("rec_start"))
	{
		RecordedJ.Reset();
		ConsolePost(TEXT("/motions/record/start"), FString::Printf(TEXT("{\"source\":\"%s\"}"), *RecordSource),
			FString::Printf(TEXT("Recording the %s joint angles (read-only)."), *RecordSource));
	}
	else if (S == TEXT("rec_stop"))
	{
		const FString Name = FString::Printf(TEXT("rec_%s"), *FDateTime::Now().ToString(TEXT("%H%M%S")));
		ConsolePost(TEXT("/motions/record/stop"), FString::Printf(TEXT("{\"name\":\"%s\"}"), *Name), TEXT("Take kept (not saved yet)."),
			[this](TSharedPtr<FJsonObject> J) { RecordedJ = J; });
	}
	else if (S == TEXT("rec_save"))
	{
		const TSharedPtr<FJsonObject> Clip = JObj(RecordedJ, TEXT("clip"));
		ConsolePost(TEXT("/motions/save"), FString::Printf(TEXT("{\"clip\":%s,\"overwrite\":false}"), *ToJson(Clip)),
			FString::Printf(TEXT("Saved %s to your motions."), *JStr(Clip, TEXT("name"))), [this](TSharedPtr<FJsonObject>) { RecordedJ.Reset(); });
	}
	else if (S == TEXT("rec_discard"))
	{
		RecordedJ.Reset();
	}
	else if (S == TEXT("mj_prev") || S == TEXT("mj_next"))
	{
		const int32 N = FMath::Max(1, JointMetas.Num());
		ModeJointIdx = (ModeJointIdx + (S == TEXT("mj_next") ? 1 : N - 1)) % N;
	}
	else if (After(TEXT("mode_"), Arg))
	{
		const FString Joint = JointMetas.IsValidIndex(ModeJointIdx) ? JointMetas[ModeJointIdx].MjName.ToString() : FString();
		const FString Joints = (Arg == TEXT("damp") || Arg == TEXT("hold")) ? FString::Printf(TEXT(",\"joints\":[\"%s\"]"), *Esc(Joint)) : FString();
		ConsolePost(TEXT("/mode"), FString::Printf(TEXT("{\"mode\":\"%s\",\"target\":\"urlab\"%s}"), *Arg, *Joints),
			Arg == TEXT("off") ? FString(TEXT("Mode ended.")) : FString::Printf(TEXT("Mode %s on the twin."), *Arg));
	}
	else if (After(TEXT("lim_dec_"), Arg) || After(TEXT("lim_inc_"), Arg))
	{
		const int32 i = FCString::Atoi(*Arg);
		const TSharedPtr<FJsonObject> Vals = JObj(LimitsJ, TEXT("values"));
		const TSharedPtr<FJsonObject> Sp = JObj(JObj(LimitsJ, TEXT("specs")), i >= 0 && i < UE_ARRAY_COUNT(LimitKeys) ? LimitKeys[i] : TEXT(""));
		if (i >= 0 && i < UE_ARRAY_COUNT(LimitKeys) && Vals.IsValid())
		{
			const bool bInc = S.StartsWith(TEXT("lim_inc_"));
			double V = JNum(Vals, LimitKeys[i]) + (bInc ? LimitSteps[i] : -LimitSteps[i]);
			V = FMath::Clamp(V, JNum(Sp, TEXT("min"), 0.0), JNum(Sp, TEXT("max"), V));
			V = FMath::RoundToDouble(V * 100.0) / 100.0;
			ConsolePost(TEXT("/limits"), FString::Printf(TEXT("{\"values\":{\"%s\":%g}}"), LimitKeys[i], V),
				FString::Printf(TEXT("%s set to %g."), *JStr(Sp, TEXT("label")), V), [this](TSharedPtr<FJsonObject> J) { LimitsJ = J; });
		}
	}
	else if (S == TEXT("lim_reset"))
	{
		ConsolePost(TEXT("/limits/reset"), TEXT("{}"), TEXT("Session limits back to the defaults."), [this](TSharedPtr<FJsonObject> J) { LimitsJ = J; });
	}
	// Setup
	else if (S == TEXT("pf_reset"))
	{
		ConsolePost(TEXT("/preflight/reset"), TEXT("{}"), TEXT("Checklist cleared."), [this](TSharedPtr<FJsonObject> J) { PreflightJ = J; });
	}
	else if (After(TEXT("pf_"), Arg))
	{
		bool bNow = false;
		for (const TSharedPtr<FJsonValue>& V : JArr(PreflightJ, TEXT("items")))
		{
			if (JStr(AsObj(V), TEXT("id")) == Arg)
			{
				bNow = JBool(AsObj(V), TEXT("ok"));
			}
		}
		ConsolePost(TEXT("/preflight"), FString::Printf(TEXT("{\"item\":\"%s\",\"checked\":%s}"), *Esc(Arg), bNow ? TEXT("false") : TEXT("true")), FString(),
			[this](TSharedPtr<FJsonObject> J) { PreflightJ = J; });
	}
	else if (After(TEXT("calsrc_"), Arg))
	{
		CalibSource = Arg;
		bCalibConfirm = false;
	}
	else if (After(TEXT("calref_"), Arg))
	{
		CalibReference = Arg;
		bCalibConfirm = false;
	}
	else if (S == TEXT("cal_read"))
	{
		// a snapshot reads the joint angles at 0 % motor power (practice: the practice twin)
		PostJson(TEXT("/snapshot"), CalibSource == TEXT("practice") ? TEXT("{\"practice\":true}") : TEXT("{}"));
		ShowToast(FString::Printf(TEXT("Reading the %s joint angles (0 %% motor power)..."), *CalibSource), TextCol);
		NextPoll = 0.0;
	}
	else if (S == TEXT("cal_capture"))
	{
		bCalibConfirm = false;
		ConsolePost(TEXT("/calibration/capture"), FString::Printf(TEXT("{\"source\":\"%s\",\"reference\":\"%s\"}"), *CalibSource, *CalibReference),
			TEXT("Offsets captured. Review them, then Store."), [this](TSharedPtr<FJsonObject> J) { CalibJ = J; });
	}
	else if (S == TEXT("cal_save"))
	{
		if (!bCalibConfirm)
		{
			bCalibConfirm = true;
			ShowToast(TEXT("Click again to store the offsets. They are NOT applied to any controller."), WarnCol);
		}
		else
		{
			bCalibConfirm = false;
			ConsolePost(TEXT("/calibration/save"), TEXT("{\"confirm\":true}"), TEXT("Offsets stored (not applied)."));
		}
	}
	else if (After(TEXT("nettest_"), Arg))
	{
		for (const TSharedPtr<FJsonValue>& V : JArr(NetworkJ, TEXT("profiles")))
		{
			const TSharedPtr<FJsonObject> P = AsObj(V);
			if (JStr(P, TEXT("name")) != Arg)
			{
				continue;
			}
			const FString Name = Arg;
			NetTestResult.Add(Name, TEXT("testing..."));
			PostJson(TEXT("/network/test"), FString::Printf(TEXT("{\"host\":\"%s\",\"port\":%d,\"icmp\":true}"), *Esc(JStr(P, TEXT("host"))),
				static_cast<int32>(JNum(P, TEXT("port"), 22.0))), [this, Name](int32 Code, TSharedPtr<FJsonObject> J)
			{
				if (Code != 200)
				{
					NetTestResult.Add(Name, FString::Printf(TEXT("test failed: %s"), JStr(J, TEXT("error")).IsEmpty() ? *FString::Printf(TEXT("HTTP %d"), Code) : *JStr(J, TEXT("error"))));
					return;
				}
				NetTestResult.Add(Name, FString::Printf(TEXT("%s   tcp %s %.1f ms   ping %s%s   %s"),
					JBool(J, TEXT("reachable")) ? TEXT("reachable") : TEXT("NOT reachable"), JBool(J, TEXT("tcp_ok")) ? TEXT("ok") : TEXT("no"),
					JNum(J, TEXT("tcp_ms")), JBool(J, TEXT("ping_ok")) ? TEXT("ok") : TEXT("no"),
					JHasNum(J, TEXT("ping_ms")) ? *FString::Printf(TEXT(" %.0f ms"), JNum(J, TEXT("ping_ms"))) : TEXT(""), *JStr(J, TEXT("detail"))));
			});
			break;
		}
	}
	// Help
	else if (S == TEXT("doc_up"))
	{
		DocScroll -= 380.f;
	}
	else if (S == TEXT("doc_down"))
	{
		DocScroll += 380.f;
	}
	else if (After(TEXT("doc_"), Arg))
	{
		OpenDoc(Arg);
	}
	else
	{
		return false;
	}
	return true;
}
