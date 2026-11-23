// Scenario lab panel: Level 2 walk sandbox tab (idea #20, docs/service_api.md section 3). SIMULATION ONLY.
//
// The behaviour service steps a headless MuJoCo ground scene and pushes every pose into this twin through URLab
// puppet mode (root free joint + 26 joints), so Unreal never simulates the robot; the in-engine Level 0 policy
// releases the actuators on its own while an external client steps URLab (UR1PolicyComponent::PolicyStep).
// This file: start / stop / reset, WASD-arrows-QE + gamepad -> POST /walk/command at 20 Hz (zeros on release; the
// service watchdog zeroes the command after 0.5 s without one), the HUD numbers from GET /walk/state, the course
// visuals (AR1WalkCourse), hiding the gantry stand and pointing the follow camera at the pelvis while it runs.
#include "R1BehaviourPanel.h"
#include "R1Twin.h"
#include "R1PanelStyle.h"
#include "R1FollowCamera.h"
#include "R1Showcase.h"
#include "R1WalkCourse.h"
#include "R1Room.h"
#include "EngineUtils.h"
#include "R1ArticulationBridge.h"

#include "Engine/Engine.h"
#include "Engine/World.h"
#include "GameFramework/PlayerController.h"
#if R1_WITH_URLAB && R1_URLAB_API_VERIFIED
#include "MuJoCo/Core/MjArticulation.h"
#include "MuJoCo/Components/Bodies/MjBody.h"
#endif

using namespace R1PanelStyle;

namespace
{
	const FName IdWalkStart(TEXT("walk_start")), IdWalkHold(TEXT("walk_hold")), IdWalkReset(TEXT("walk_reset")), IdWalkStop(TEXT("walk_stop"));
	const FName IdWalkFree(TEXT("walk_free"));
	const TCHAR* CoursePrefix = TEXT("walk_course_");
	const TCHAR* HoldLabel = TEXT("Hold pose: no walking policy (held by a virtual harness)");

	float Dead(float V, float Zone = 0.15f)
	{
		return FMath::Abs(V) < Zone ? 0.f : FMath::Sign(V) * (FMath::Abs(V) - Zone) / (1.f - Zone);
	}

	/** Stick value -1..1 -> command, using the asymmetric limits (e.g. vx -0.5..1.0). */
	float Span(float V, float Lo, float Hi)
	{
		return V >= 0.f ? V * Hi : -V * Lo;
	}

	FString Secs(float S)
	{
		return S < 0.f ? FString(TEXT("--")) : FString::Printf(TEXT("%.1f s"), S);
	}
}

void AR1BehaviourPanel::TickWalk(double Now)
{
	const double Interval = (Tab == ER1PanelTab::Walk || WalkState.bRunning) ? 0.1 : 2.0;
	if (Now >= NextWalkPoll && !bWalkPollInFlight && bServiceOnline)
	{
		NextWalkPoll = Now + Interval;
		PollWalk();
	}

	if (WalkState.bRunning != bWalkVisualsOn)
	{
		SetWalkRunningVisuals(WalkState.bRunning);
	}

	ReadWalkInput();
	if (WalkState.bRunning)
	{
		// someone is driving the sandbox: idle demo mode must not take over the camera and HUD
		if (UR1ShowcaseSubsystem* Showcase = UR1ShowcaseSubsystem::Get(this))
		{
			Showcase->NotifyUserInput();
		}
	}

	if (WalkState.bRunning && WalkState.Mode == TEXT("policy"))
	{
		const FVector Cmd = WalkState.ClampCmd(InputWalkCmd);
		const bool bDriving = !Cmd.IsNearlyZero(1e-3);
		if (bDriving && Now >= NextCmdSend)
		{
			SendWalkCommand(Cmd);
			NextCmdSend = Now + 0.05;   // ~20 Hz while held
		}
		else if (!bDriving && bWasDriving)
		{
			SendWalkCommand(FVector::ZeroVector);   // once on release
		}
		bWasDriving = bDriving;
	}
	else
	{
		bWasDriving = false;
	}
}

void AR1BehaviourPanel::PollWalk()
{
	bWalkPollInFlight = true;
	GetJson(TEXT("/walk/state"), [this](int32 Code, const FString& Body)
	{
		bWalkPollInFlight = false;
		if (Code == 404)
		{
			WalkState = FR1WalkState();
			WalkNote = TEXT("This behaviour service has no walk sandbox (update RobotControl).");
		}
		else if (Code == 200)
		{
			FR1WalkState S;
			if (R1Panel::ParseWalkState(Body, S))
			{
				WalkState = MoveTemp(S);
				WalkNote.Reset();
				if (!WalkState.Course.IsEmpty() && !WalkState.bRunning && WalkState.Courses.Num() > 0 && !WalkState.Courses.Contains(WalkCourse))
				{
					WalkCourse = WalkState.Course;
				}
			}
		}
		else if (Code == 0)
		{
			WalkState.bRunning = false;   // service gone: release the visuals
		}
		SyncCourse();
	});
}

void AR1BehaviourPanel::ReadWalkInput()
{
	InputWalkCmd = FVector::ZeroVector;
	APlayerController* PC = GetOwningPlayerController();
	if (!PC || Tab != ER1PanelTab::Walk || !WalkState.bRunning || WalkState.Replay.bActive)
	{
		return;
	}
	const FR1WalkState& S = WalkState;
	const bool bFast = PC->IsInputKeyDown(EKeys::LeftShift) || PC->IsInputKeyDown(EKeys::RightShift);
	auto Down = [PC](const FKey& A, const FKey& B) { return PC->IsInputKeyDown(A) || PC->IsInputKeyDown(B); };
	const float Fwd = (Down(EKeys::W, EKeys::Up) ? 1.f : 0.f) - (Down(EKeys::S, EKeys::Down) ? 1.f : 0.f);
	const float Left = (PC->IsInputKeyDown(EKeys::A) ? 1.f : 0.f) - (PC->IsInputKeyDown(EKeys::D) ? 1.f : 0.f);
	const float TurnLeft = (Down(EKeys::Q, EKeys::Left) ? 1.f : 0.f) - (Down(EKeys::E, EKeys::Right) ? 1.f : 0.f);
	if (Fwd != 0.f || Left != 0.f || TurnLeft != 0.f)
	{
		// keyboard: a comfortable walk; Shift = the policy's full range
		const float K = bFast ? 1.f : 0.5f;
		InputWalkCmd = FVector(Span(Fwd, S.VxMin, S.VxMax) * K, Span(Left, S.VyMin, S.VyMax) * K, Span(TurnLeft, S.WzMin, S.WzMax) * (bFast ? 1.f : 0.6f));
		return;
	}
	// gamepad: left stick = move (up = forward, right = sidestep right), right stick X = turn (right = turn right)
	const float LX = Dead(PC->GetInputAnalogKeyState(EKeys::Gamepad_LeftX));
	const float LY = Dead(PC->GetInputAnalogKeyState(EKeys::Gamepad_LeftY));
	const float RX = Dead(PC->GetInputAnalogKeyState(EKeys::Gamepad_RightX));
	if (LX != 0.f || LY != 0.f || RX != 0.f)
	{
		// MuJoCo y points LEFT and +wz turns left, so stick-right maps to negative vy / wz
		InputWalkCmd = FVector(Span(LY, S.VxMin, S.VxMax), Span(-LX, S.VyMin, S.VyMax), Span(-RX, S.WzMin, S.WzMax));
	}
}

void AR1BehaviourPanel::SendWalkCommand(const FVector& Cmd)
{
	PostJson(TEXT("/walk/command"), R1Panel::WalkCommandJson(Cmd), [this](int32 Code, TSharedPtr<FJsonObject> J)
	{
		if (Code != 200 && Code != 0 && J.IsValid())
		{
			J->TryGetStringField(TEXT("error"), WalkNote);
		}
	});
}

void AR1BehaviourPanel::SetWalkRunningVisuals(bool bOn)
{
	bWalkVisualsOn = bOn;
#if R1_WITH_URLAB && R1_URLAB_API_VERIFIED
	if (!StandBody.IsValid())
	{
		if (AMjArticulation* Art = Cast<AMjArticulation>(R1Urlab::FindArticulationActor(GetWorld(), TEXT("r1_stand"))))
		{
			for (UMjBody* B : Art->GetBodies())
			{
				const FString N = B ? B->GetMjName() : FString();
				if (N == TEXT("stand") || N.EndsWith(TEXT("_stand")))
				{
					StandBody = B;
					break;
				}
			}
		}
	}
#endif
	if (USceneComponent* Stand = StandBody.Get())
	{
		// The gantry stand is a MuJoCo body of the stand scene; the robot walks away from it, so hide it.
		Stand->SetVisibility(!bOn, /*bPropagateToChildren*/ true);
	}
	for (TActorIterator<AR1Room> It(GetWorld()); It; ++It)
	{
		It->SetActorHiddenInGame(bOn);   // the walk has its own course; the room comes back when it stops
	}
	APlayerController* PC = GetOwningPlayerController();
	if (AR1FollowCamera* Cam = PC ? Cast<AR1FollowCamera>(PC->GetPawn()) : nullptr)
	{
		if (bOn)
		{
			bPrevCamLock = Cam->bLockToStand;
			PrevCamDistance = Cam->Distance;
			Cam->SetLockToStand(false);   // follow the pelvis (puppet pose) instead of the fixed stand point
			Cam->Distance = FMath::Max(Cam->Distance, 460.f);
		}
		else
		{
			Cam->SetLockToStand(bPrevCamLock);
			Cam->Distance = PrevCamDistance;
		}
	}
	UE_LOG(LogR1Twin, Log, TEXT("R1BehaviourPanel: walk sandbox %s (stand %s, camera %s)"), bOn ? TEXT("running") : TEXT("stopped"),
		StandBody.IsValid() ? (bOn ? TEXT("hidden") : TEXT("shown")) : TEXT("not found"), bOn ? TEXT("follows pelvis") : TEXT("restored"));
}

void AR1BehaviourPanel::SyncCourse()
{
	const bool bHaveGeometry = WalkState.Boxes.Num() > 0 || WalkState.CheckpointsMj.Num() > 0 || WalkState.Props.Num() > 0;
	const bool bWant = WalkState.bValid && bHaveGeometry && (WalkState.bRunning || Tab == ER1PanelTab::Walk);
	AR1WalkCourse* Course = CourseActor.Get();
	if (!bWant)
	{
		if (Course)
		{
			Course->Destroy();
			CourseActor.Reset();
		}
		return;
	}
	if (!Course && GetWorld())
	{
		FActorSpawnParameters P;
		P.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
		P.ObjectFlags |= RF_Transient;
		Course = GetWorld()->SpawnActor<AR1WalkCourse>(AR1WalkCourse::StaticClass(), FTransform::Identity, P);
		CourseActor = Course;
	}
	if (Course)
	{
		Course->Sync(WalkState);
	}
}

void AR1BehaviourPanel::DrawWalkTab(float X, float Inner, float& Y)
{
	const FR1WalkState& S = WalkState;
	const bool bJobFree = bServiceOnline && Job.IsEmpty();
	UFont* Font = GEngine->GetMediumFont();

	DrawLine(TEXT("WALK SANDBOX  (Level 2, simulation only - never the real robot)"), X, Y, CyanCol, 1.05f);
	if (!WalkNote.IsEmpty())
	{
		DrawWrapped(WalkNote, X, Y, Inner, WarnCol, 0.88f);
	}
	Y += 4.f;

	// course chips
	{
		TArray<FString> Courses = S.Courses;
		if (Courses.Num() == 0)
		{
			Courses = {TEXT("flat"), TEXT("loop")};
		}
		Courses.AddUnique(TEXT("procedural"));   // docs/sim_features.md #3 (the service may list it already)
		DrawText(TEXT("Course"), DimCol, X, Y + 5.f, Font, 0.9f);
		float CX = X + 70.f;
		for (const FString& C : Courses)
		{
			const bool bSel = (S.bRunning && !S.Course.IsEmpty() ? S.Course : WalkCourse) == C;
			const FString Label = C.Left(1).ToUpper() + C.Mid(1);
			const float CW = C.Len() > 6 ? 110.f : 90.f;
			DrawButton({FName(*(FString(CoursePrefix) + C)), Label, {CX, Y}, {CW, 28.f}, !S.bRunning || bSel, bSel ? ChipOnCol : ChipCol});
			CX += CW + 6.f;
		}
		Y += 36.f;
	}

	// start / hold / reset / stop
	{
		const bool bCanStart = bJobFree && !S.bRunning && S.bPolicyAvailable;
		const bool bCanHold = bJobFree && !S.bRunning && S.bValid;
		// Let go = free_hold: every joint PD-held without the harness, so the robot tips over (fall analysis, replay)
		const float W1 = 160.f, W2 = 104.f, WF = 104.f, W3 = 76.f, G = 6.f;
		DrawButton({IdWalkStart, TEXT("Start walking"), {X, Y}, {W1, 42.f}, bCanStart, VirtualCol, true});
		if (!bCanStart && bJobFree && !S.bRunning && !S.bPolicyAvailable)
		{
			DisabledWhy.Add(IdWalkStart, TEXT("no validated Level 2 policy yet - use Hold pose to test the scene"));
		}
		DrawButton({IdWalkHold, TEXT("Hold pose"), {X + W1 + G, Y}, {W2, 42.f}, bCanHold, ChipCol});
		DrawButton({IdWalkFree, TEXT("Let go"), {X + W1 + W2 + 2.f * G, Y}, {WF, 42.f}, bCanHold, ChipCol});
		DrawButton({IdWalkReset, TEXT("Reset"), {X + W1 + W2 + WF + 3.f * G, Y}, {W3, 42.f}, S.bRunning, ChipCol});
		DrawButton({IdWalkStop, TEXT("Stop walk"), {X + W1 + W2 + WF + W3 + 4.f * G, Y}, {Inner - W1 - W2 - WF - W3 - 4.f * G, 42.f}, S.bRunning, StopCol * 0.85f});
		if (!S.bRunning)
		{
			DisabledWhy.Add(IdWalkReset, TEXT("the walk sandbox is not running"));
			DisabledWhy.Add(IdWalkStop, TEXT("the walk sandbox is not running"));
		}
		Y += 50.f;
	}
	DrawWalkSimControls(X, Inner, Y);   // R1PanelSim.cpp: terrain seed, throw, scenarios

	// state line
	if (!S.bValid)
	{
		DrawWrapped(bServiceOnline ? TEXT("Waiting for /walk/state...") : TEXT("Behaviour service offline."), X, Y, Inner, DimCol, 0.9f);
	}
	else if (S.bRunning && S.Mode == TEXT("free_hold"))
	{
		DrawWrapped(S.bFallen ? TEXT("Let go: the robot fell (no harness, no policy). Replay it or reset.")
			: TEXT("Let go: joints held, no harness and no policy - it will tip over in about a second."), X, Y, Inner, WarnCol, 0.9f);
	}
	else if (S.bRunning && S.Mode == TEXT("hold"))
	{
		DrawWrapped(HoldLabel, X, Y, Inner, WarnCol, 0.95f);
	}
	else if (S.bRunning)
	{
		DrawLine(S.bFallen ? TEXT("FALLEN - resetting to the start in 1 s") : TEXT("Walking. Drive with WASD / arrows or a gamepad."), X, Y, S.bFallen ? BadCol : OkCol, 0.95f);
	}
	else if (!S.bPolicyAvailable)
	{
		DrawWrapped(FString::Printf(TEXT("No validated Level 2 policy yet: %s"), S.PolicyReason.IsEmpty() ? TEXT("not reported") : *S.PolicyReason),
			X, Y, Inner, WarnCol, 0.88f);
		DrawWrapped(TEXT("Hold pose still works: it tests the course, rendering and the puppet path."), X, Y, Inner, DimCol, 0.85f);
	}
	else
	{
		DrawLine(TEXT("Validated walking policy ready. Start walking, then drive."), X, Y, OkCol, 0.92f);
	}
	Y += 4.f;

	// telemetry card
	if (S.bValid && S.bRunning)
	{
		const float CardY = Y, CardH = 172.f;
		DrawRect(CardBg, X, CardY, Inner, CardH);
		float Ty = Y + 8.f;
		const float Cx = X + 10.f, Cw = Inner - 20.f;
		// command bars: centre = 0, filled towards the applied command, scaled to the limits
		struct FBar { const TCHAR* Name; float V; float Lo; float Hi; const TCHAR* Unit; };
		const FBar Bars[] = {
			{TEXT("forward vx"), static_cast<float>(S.Cmd.X), S.VxMin, S.VxMax, TEXT("m/s")},
			{TEXT("sideways vy"), static_cast<float>(S.Cmd.Y), S.VyMin, S.VyMax, TEXT("m/s")},
			{TEXT("turn wz"), static_cast<float>(S.Cmd.Z), S.WzMin, S.WzMax, TEXT("rad/s")},
		};
		const float BarStartY = Ty;
		for (const FBar& B : Bars)
		{
			DrawText(FString::Printf(TEXT("%s  %+.2f %s"), B.Name, B.V, B.Unit), TextCol, Cx, Ty, Font, 0.85f);
			const float GX = Cx + 220.f, GW = Cw - 220.f, GY = Ty + 7.f;
			const float Range = FMath::Max(FMath::Max(FMath::Abs(B.Lo), FMath::Abs(B.Hi)), 1e-3f);
			const float Mid = GX + GW * 0.5f;
			DrawRect(FLinearColor(1.f, 1.f, 1.f, 0.10f), GX, GY, GW, 7.f);
			DrawRect(FLinearColor(1.f, 1.f, 1.f, 0.35f), Mid - 1.f, GY - 3.f, 2.f, 13.f);
			const float Px = GW * 0.5f * FMath::Clamp(B.V / Range, -1.f, 1.f);
			DrawRect(CyanCol, Px >= 0.f ? Mid : Mid + Px, GY, FMath::Max(2.f, FMath::Abs(Px)), 7.f);
			Ty += 20.f;
		}
		Ty = FMath::Max(Ty, BarStartY + 60.f) + 4.f;
		const float Speed = FVector2D(S.LinVelMj.X, S.LinVelMj.Y).Size();
		DrawLine(FString::Printf(TEXT("speed %.2f m/s   |   sim time %.1f s   |   falls %d"), Speed, S.T, S.Falls), Cx, Ty, TextCol, 0.88f);
		DrawLine(FString::Printf(TEXT("lap %s   |   last %s   |   best %s   |   laps %d"), *Secs(S.LapCurrentS), *Secs(S.LapLastS), *Secs(S.LapBestS), S.Laps),
			Cx, Ty, TextCol, 0.88f);
		DrawLine(S.CheckpointTotal > 0 ? FString::Printf(TEXT("checkpoint %d / %d  (next: the blue ring with the pole)"),
			FMath::Min(S.CheckpointIndex + 1, S.CheckpointTotal), S.CheckpointTotal) : FString(TEXT("no checkpoints on this course")), Cx, Ty, DimCol, 0.88f);
		DrawLine(S.bPuppetConnected ? TEXT("Unreal view: receiving poses (URLab puppet mode)")
			: TEXT("Unreal view: not receiving poses yet (the service retries every 3 s; start it without --no-mirror)"),
			Cx, Ty, S.bPuppetConnected ? OkCol : WarnCol, 0.85f);
		Y = FMath::Max(Ty, CardY + CardH) + 10.f;
	}

	DrawWrapped(TEXT("Keys (Walk tab): W/S or Up/Down forward/back, A/D sidestep, Q/E or Left/Right turn, Shift faster. "
		"Gamepad: left stick move, right stick turn. Needs the behaviour service (python -m robotcontrol.behaviours.service)."), X, Y, Inner, DimCol, 0.8f);
}

void AR1BehaviourPanel::StartWalk(const FString& Mode)
{
	FString Body = FString::Printf(TEXT("{\"course\":\"%s\",\"mode\":\"%s\""), *R1Panel::JsonEscape(WalkCourse), *R1Panel::JsonEscape(Mode));
	if (WalkCourse == TEXT("procedural"))
	{
		Body += FString::Printf(TEXT(",\"seed\":%d,\"difficulty\":%.2f"), WalkSeed, WalkDifficulty);
	}
	Body += TEXT("}");
	PostJson(TEXT("/walk/start"), Body, [this, Mode](int32 Code, TSharedPtr<FJsonObject> J)
	{
		FString Err, Reason;
		if (J.IsValid())
		{
			J->TryGetStringField(TEXT("error"), Err);
			J->TryGetStringField(TEXT("policy_reason"), Reason);
		}
		if (Code == 200)
		{
			ShowToast(Mode == TEXT("hold") ? TEXT("Walk sandbox started in hold pose.")
				: Mode == TEXT("free_hold") ? TEXT("Let go: no harness, no policy - watch it tip over.")
				: TEXT("Walk sandbox started: drive with WASD or a gamepad."), CyanCol);
		}
		else
		{
			const FString Why = Code == 0 ? FString(TEXT("service not reachable")) : !Reason.IsEmpty() ? Reason : !Err.IsEmpty() ? Err : FString::Printf(TEXT("HTTP %d"), Code);
			ShowToast(FString::Printf(TEXT("Walk start refused: %s"), *Why), BadCol);
		}
		NextWalkPoll = 0.0;
		NextPoll = 0.0;
	});
}

bool AR1BehaviourPanel::HandleWalkClick(FName Id)
{
	const FString S = Id.ToString();
	if (Id == IdWalkStart || Id == IdWalkHold || Id == IdWalkFree)
	{
		StartWalk(Id == IdWalkStart ? TEXT("policy") : Id == IdWalkHold ? TEXT("hold") : TEXT("free_hold"));
		return true;
	}
	if (Id == IdWalkReset || Id == IdWalkStop)
	{
		const bool bStop = Id == IdWalkStop;
		PostJson(bStop ? TEXT("/walk/stop") : TEXT("/walk/reset"), TEXT("{}"), [this, bStop](int32 Code, TSharedPtr<FJsonObject> J)
		{
			FString Err;
			if (J.IsValid())
			{
				J->TryGetStringField(TEXT("error"), Err);
			}
			ShowToast(Code == 200 ? FString(bStop ? TEXT("Walk sandbox stopped.") : TEXT("Back to the start, lap timer reset."))
				: FString::Printf(TEXT("Refused: %s"), Err.IsEmpty() ? *FString::Printf(TEXT("HTTP %d"), Code) : *Err), Code == 200 ? TextCol : BadCol);
			NextWalkPoll = 0.0;
		});
		return true;
	}
	if (S.StartsWith(CoursePrefix))
	{
		if (!WalkState.bRunning)
		{
			WalkCourse = S.Mid(FCString::Strlen(CoursePrefix));
			ShowToast(FString::Printf(TEXT("Course: %s"), *WalkCourse), TextCol);
		}
		return true;
	}
	return false;
}
