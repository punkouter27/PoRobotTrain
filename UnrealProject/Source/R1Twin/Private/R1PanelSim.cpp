// Scenario lab panel: simulation features (docs/sim_features.md). SIMULATION ONLY - nothing here reaches the robot.
//
//   #1 / #17  motor sound (UR1MotorSynth) fed from the lab brain's motor model or the walk state's motors block,
//             torque alert tone + HUD row when a torque fraction stays above torque_alert_fraction
//   #2        physics overlay: walk contacts + forces, CoM, capture point, support polygon; lab head-joint torque arrows
//   #3        procedural terrain: seed, difficulty, New terrain (POST /walk/start)
//   #5        heat + battery card (walk: thermal / battery blocks; lab: FR1MotorModel in the brain)
//   #7        Record dataset: head-camera PNG frames + labels.json (UR1SkillBrainComponent::StartRecording)
//   #9        Throw ball / box (POST /walk/throw); the props are drawn by AR1WalkCourse
//   #12       Replay (POST /walk/replay), auto replay on fall, replay banner, free-fly camera while it plays
//   #15       action view: lab raw vs EMA plot + jitter; walk action_filter bars + Apply filter (POST /walk/filter)
//   #18       scenario scripts: walk ones via POST /walk/scenario, lab ones read from RobotControl/scenarios
//   #19       fall timeline: cp margin + tilt history with the unstable / fall markers and the reason
#include "R1BehaviourPanel.h"
#include "R1Twin.h"
#include "R1PanelStyle.h"
#include "R1SkillBrain.h"
#include "R1MotorAudio.h"
#include "R1FollowCamera.h"
#include "MjUeConvert.h"

#include "Dom/JsonObject.h"
#include "Engine/Canvas.h"
#include "Engine/Engine.h"
#include "Engine/World.h"
#include "GameFramework/PlayerController.h"
#include "Misc/DateTime.h"
#include "Misc/Paths.h"

using namespace R1PanelStyle;

namespace
{
	const FName IdSound(TEXT("sim_sound")), IdPhysics(TEXT("sim_physics")), IdRecord(TEXT("sim_record"));
	const FName IdReplay(TEXT("sim_replay")), IdReplayStop(TEXT("sim_replay_stop")), IdAutoReplay(TEXT("sim_auto_replay"));
	const FName IdFilter(TEXT("sim_filter")), IdNewTerrain(TEXT("sim_new_terrain")), IdSeedDown(TEXT("sim_seed_down")), IdSeedUp(TEXT("sim_seed_up"));
	const FName IdThrowBall(TEXT("sim_throw_ball")), IdThrowBox(TEXT("sim_throw_box")), IdScenarioStop(TEXT("sim_scn_stop"));
	const TCHAR* DiffPrefix = TEXT("sim_diff_");
	const TCHAR* FromPrefix = TEXT("sim_from_");
	const TCHAR* LabScnPrefix = TEXT("sim_lscn_");
	const TCHAR* WalkScnPrefix = TEXT("sim_wscn_");
	const float Difficulties[] = {0.25f, 0.5f, 0.75f, 1.f};
	const TCHAR* FromLabels[] = {TEXT("Front"), TEXT("Left"), TEXT("Right"), TEXT("Back")};
	const float FromDeg[] = {0.f, 90.f, -90.f, 180.f};   // the direction the object comes FROM (90 = robot's left)
	const float Volumes[] = {0.f, 0.25f, 0.5f, 1.f};
	const int32 DatasetFrames = 60;                        // 12 s at 5 frames/s
	const FLinearColor SimCol(0.55f, 0.62f, 1.f, 1.f);
	const FLinearColor ForceCol(1.f, 0.55f, 0.15f, 1.f);

	FName SimChipId(const TCHAR* Prefix, int32 I) { return FName(*FString::Printf(TEXT("%s%d"), Prefix, I)); }   // own name: R1PanelLab.cpp has an Id() too (unity builds)

	FString Pretty(const FString& Joint)
	{
		FString S = Joint;
		S.RemoveFromEnd(TEXT("_joint"));
		S.ReplaceInline(TEXT("_"), TEXT(" "));
		return S;
	}

	FLinearColor TempColor(float C)
	{
		const FR1MotorModelConfig& M = FR1MotorModelConfig::Get();
		return C >= M.Limit ? BadCol : C >= M.Warn ? WarnCol : OkCol;
	}

	FLinearColor FracColor(float F)
	{
		return F >= FR1MotorModelConfig::Get().AlertFraction ? BadCol : F >= 0.5f ? WarnCol : OkCol;
	}

	bool IsFoot(const FString& Name)
	{
		return Name.Contains(TEXT("foot")) || Name.Contains(TEXT("ankle"));
	}

	/** {"ok": false} or a non-200 code -> the reason; empty when it worked. */
	FString Refusal(int32 Code, const TSharedPtr<FJsonObject>& J)
	{
		bool bOk = Code == 200;
		FString Err;
		if (J.IsValid())
		{
			bool B = true;
			if (J->TryGetBoolField(TEXT("ok"), B)) bOk = bOk && B;
			J->TryGetStringField(TEXT("error"), Err);
		}
		if (bOk) return FString();
		return Code == 0 ? FString(TEXT("service not reachable")) : !Err.IsEmpty() ? Err : Code == 404 ? FString(TEXT("this service has no such endpoint (update RobotControl)"))
			: FString::Printf(TEXT("HTTP %d"), Code);
	}
}

// =====================================================================================================
// tick: sound, scenario lists, replay camera
// =====================================================================================================

FString AR1BehaviourPanel::DatasetDir() const
{
	return FPaths::ConvertRelativePathToFull(FPaths::Combine(FPaths::ProjectDir(), TEXT(".."), TEXT("datasets"), TEXT("unreal_head_cam"),
		FDateTime::Now().ToString(TEXT("%Y%m%d_%H%M%S"))));
}

void AR1BehaviourPanel::TickSim(double Now, float DeltaSeconds)
{
	UWorld* World = GetWorld();
	if (!MotorSynth && World && World->IsGameWorld())
	{
		MotorSynth = NewObject<UR1MotorSynth>(this, TEXT("R1MotorSynth"));
		MotorSynth->RegisterComponent();
		MotorSynth->Start();
	}

	// ---------------------------------------------------------------- sound source: walk sandbox, else the lab brain
	UR1SkillBrainComponent* B = FindBrain();
	const bool bWalk = WalkState.bRunning && WalkState.Motors.bValid;
	if (bWalk)
	{
		FeedMotorSound(WalkState.Motors, true, 8.f);
	}
	else if (B && !B->GetReadout().bExternal && B->GetMotorReadout().bValid)
	{
		FeedMotorSound(B->GetMotorReadout(), false, 3.f);
	}
	else
	{
		FeedMotorSound(FR1MotorReadout(), false, 1.f);
	}

	// foot thuds: a foot contact appearing with a large normal force (checked once per new walk state)
	if (bWalk && WalkState.Physics.bValid && !FMath::IsNearlyEqual(WalkState.T, LastContactT))
	{
		LastContactT = WalkState.T;
		TSet<FString> Down;
		for (const FR1WalkContact& C : WalkState.Physics.Contacts)
		{
			const FString Foot = IsFoot(C.A) ? C.A : IsFoot(C.B) ? C.B : FString();
			const float Fn = FMath::Abs(static_cast<float>(C.ForceMj.Z));
			if (Foot.IsEmpty() || Fn < 60.f)
			{
				continue;
			}
			Down.Add(Foot);
			if (!FootDown.Contains(Foot))
			{
				FootDown.Add(Foot, Now);
				if (MotorSynth)
				{
					MotorSynth->Thud(FMath::Clamp(Fn / 700.f, 0.2f, 1.f));
				}
			}
		}
		for (auto It = FootDown.CreateIterator(); It; ++It)
		{
			if (!Down.Contains(It.Key()))
			{
				It.RemoveCurrent();   // lifted: the next landing thuds again
			}
		}
	}
	else if (!bWalk)
	{
		FootDown.Reset();
	}

	// ---------------------------------------------------------------- scenario lists (#18)
	if (!bLabScenariosLoaded)
	{
		bLabScenariosLoaded = true;
		LabScenarios = R1Panel::LoadLabScenarios();
	}
	if (Tab == ER1PanelTab::Walk && bServiceOnline && Now >= NextScenarioFetch && !bScenarioFetchInFlight)
	{
		NextScenarioFetch = Now + 15.0;
		bScenarioFetchInFlight = true;
		GetJson(TEXT("/scenarios"), [this](int32 Code, const FString& Body)
		{
			bScenarioFetchInFlight = false;
			TArray<FR1ScenarioInfo> All;
			if (Code == 200 && R1Panel::ParseScenarioList(Body, All))
			{
				WalkScenarios = All.FilterByPredicate([](const FR1ScenarioInfo& I) { return I.Target == TEXT("walk"); });
			}
			else if (Code != 0)
			{
				WalkScenarios.Reset();
			}
		});
	}

	// ---------------------------------------------------------------- replay: free-fly camera while it plays (#12)
	const bool bWantFly = WalkState.bRunning && WalkState.Replay.bActive;
	if (bWantFly != bReplayCam)
	{
		APlayerController* PC = GetOwningPlayerController();
		if (AR1FollowCamera* Cam = PC ? Cast<AR1FollowCamera>(PC->GetPawn()) : nullptr)
		{
			Cam->SetFreeFly(bWantFly);
		}
		bReplayCam = bWantFly;
	}
}

void AR1BehaviourPanel::FeedMotorSound(const FR1MotorReadout& M, bool bFromWalk, float SpeedRef)
{
	const double Now = FPlatformTime::Seconds();
	float MaxSpeed = 0.f;
	for (float V : M.Speed)
	{
		MaxSpeed = FMath::Max(MaxSpeed, FMath::Abs(V));
	}
	const int32 Peak = M.bValid ? M.MaxFracIndex() : INDEX_NONE;
	const float Frac = Peak != INDEX_NONE ? M.TorqueFrac[Peak] : 0.f;

	// #17 torque alert: above the alert fraction for 0.2 s (a single 20 ms spike does not beep)
	const bool bOver = Peak != INDEX_NONE && Frac > FR1MotorModelConfig::Get().AlertFraction;
	AlertSince = bOver ? (AlertSince < 0.0 ? Now : AlertSince) : -1.0;
	bTorqueAlert = bOver && Now - AlertSince >= 0.2;
	TorqueAlertText = bTorqueAlert ? FString::Printf(TEXT("TORQUE ALERT: %s at %.0f%% of its limit"),
		M.Names.IsValidIndex(Peak) ? *Pretty(M.Names[Peak]) : TEXT("a joint"), Frac * 100.f) : FString();

	// gear clicks: a joint's velocity changes sign while moving
	if (bFromWalk != bPrevSoundFromWalk || PrevJointSpeed.Num() != M.Speed.Num())
	{
		bPrevSoundFromWalk = bFromWalk;
	}
	else if (MotorSynth)
	{
		for (int32 i = 0; i < M.Speed.Num(); ++i)
		{
			const float A = PrevJointSpeed[i], V = M.Speed[i];
			if (A * V < 0.f && FMath::Abs(A) > 0.25f && FMath::Abs(V) > 0.25f && Now - LastClickAt > 0.05)
			{
				MotorSynth->Click();
				LastClickAt = Now;
			}
		}
	}
	PrevJointSpeed = M.Speed;
	if (MotorSynth)
	{
		MotorSynth->SetMasterVolume(SoundVolume);
		MotorSynth->SetDrive(MaxSpeed / FMath::Max(SpeedRef, 0.1f), Frac, bTorqueAlert);
	}
}

// =====================================================================================================
// clicks
// =====================================================================================================

bool AR1BehaviourPanel::HandleSimClick(FName ClickId)
{
	const FString S = ClickId.ToString();
	if (!S.StartsWith(TEXT("sim_")))
	{
		return false;
	}
	auto After = [this](const FString& Done)
	{
		return [this, Done](int32 Code, TSharedPtr<FJsonObject> J)
		{
			const FString Why = Refusal(Code, J);
			ShowToast(Why.IsEmpty() ? Done : FString::Printf(TEXT("Refused: %s"), *Why), Why.IsEmpty() ? CyanCol : BadCol);
			NextWalkPoll = 0.0;
		};
	};
	UR1SkillBrainComponent* B = FindBrain();
	if (ClickId == IdSound)
	{
		int32 I = 0;
		for (int32 k = 0; k < UE_ARRAY_COUNT(Volumes); ++k)
		{
			if (SoundVolume >= Volumes[k] - 1e-3f) I = k;
		}
		SoundVolume = Volumes[(I + 1) % UE_ARRAY_COUNT(Volumes)];
		SaveConfig();
		ShowToast(SoundVolume <= 0.f ? FString(TEXT("Motor sound muted.")) : FString::Printf(TEXT("Motor sound %.0f%%."), SoundVolume * 100.f), TextCol);
	}
	else if (ClickId == IdPhysics)
	{
		bPhysicsOverlay = !bPhysicsOverlay;
		SaveConfig();
	}
	else if (ClickId == IdRecord && B)
	{
		FString Why;
		if (B->GetRecording().bActive)
		{
			B->StopRecording();
			ShowToast(FString::Printf(TEXT("Recording stopped: %d frame(s) in %s"), B->GetRecording().Frames, *B->GetRecording().Dir), TextCol);
		}
		else if (B->StartRecording(DatasetDir(), DatasetFrames, Why))
		{
			ShowToast(FString::Printf(TEXT("Recording %d head-camera frames (simulation only) to %s"), DatasetFrames, *B->GetRecording().Dir), CyanCol);
		}
		else
		{
			ShowToast(FString::Printf(TEXT("Cannot record: %s"), *Why), BadCol);
		}
	}
	else if (ClickId == IdReplay)
	{
		PostJson(TEXT("/walk/replay"), TEXT("{\"seconds\":3,\"speed\":0.25}"), After(TEXT("Replaying the last 3 s at 0.25x: fly with WASD, Q/E, right mouse.")));
	}
	else if (ClickId == IdReplayStop)
	{
		PostJson(TEXT("/walk/replay"), TEXT("{\"stop\":true}"), After(TEXT("Replay stopped; the simulation continues.")));
	}
	else if (ClickId == IdAutoReplay)
	{
		const bool bOn = !WalkState.Replay.bAutoOnFall;
		PostJson(TEXT("/walk/replay"), FString::Printf(TEXT("{\"auto_on_fall\":%s}"), bOn ? TEXT("true") : TEXT("false")),
			After(bOn ? TEXT("Auto replay on: every fall replays 3 s at 0.25x.") : TEXT("Auto replay off.")));
	}
	else if (ClickId == IdFilter)
	{
		const FR1WalkActionFilter& A = WalkState.ActionFilter;
		const bool bOn = !A.bApply;
		PostJson(TEXT("/walk/filter"), FString::Printf(TEXT("{\"alpha\":%.3f,\"apply\":%s}"), A.bValid ? A.Alpha : FR1MotorModelConfig::Get().FilterAlpha,
			bOn ? TEXT("true") : TEXT("false")),
			After(bOn ? TEXT("Low-pass filter applied to the walking policy's actions (it was trained without it).") : TEXT("Filter off: raw actions again.")));
	}
	else if (ClickId == IdSeedDown || ClickId == IdSeedUp)
	{
		WalkSeed = FMath::Max(0, WalkSeed + (ClickId == IdSeedUp ? 1 : -1));
		SaveConfig();
	}
	else if (S.StartsWith(DiffPrefix))
	{
		WalkDifficulty = Difficulties[FMath::Clamp(FCString::Atoi(*S.Mid(FCString::Strlen(DiffPrefix))), 0, static_cast<int32>(UE_ARRAY_COUNT(Difficulties)) - 1)];
		SaveConfig();
	}
	else if (ClickId == IdNewTerrain)
	{
		// a fresh seed, then (re)start the sandbox on it in the current mode (stop first if it runs)
		WalkSeed = FMath::RandRange(1, 99999);
		WalkCourse = TEXT("procedural");
		SaveConfig();
		const FString Mode = WalkState.bRunning && !WalkState.Mode.IsEmpty() ? WalkState.Mode : WalkState.bPolicyAvailable ? FString(TEXT("policy")) : FString(TEXT("hold"));
		if (WalkState.bRunning)
		{
			PostJson(TEXT("/walk/stop"), TEXT("{}"), [this, Mode](int32, TSharedPtr<FJsonObject>) { StartWalk(Mode); });
		}
		else
		{
			StartWalk(Mode);
		}
		ShowToast(FString::Printf(TEXT("New terrain: seed %d, difficulty %.2f."), WalkSeed, WalkDifficulty), CyanCol);
	}
	else if (ClickId == IdThrowBall || ClickId == IdThrowBox)
	{
		const bool bBall = ClickId == IdThrowBall;
		PostJson(TEXT("/walk/throw"), FString::Printf(TEXT("{\"kind\":\"%s\",\"speed\":4.0,\"from_deg\":%.0f}"), bBall ? TEXT("ball") : TEXT("box"), FromDeg[ThrowFrom]),
			After(FString::Printf(TEXT("Threw a %s from the %s."), bBall ? TEXT("ball") : TEXT("box"), *FString(FromLabels[ThrowFrom]).ToLower())));
	}
	else if (S.StartsWith(FromPrefix))
	{
		ThrowFrom = FMath::Clamp(FCString::Atoi(*S.Mid(FCString::Strlen(FromPrefix))), 0, 3);
	}
	else if (S.StartsWith(LabScnPrefix) && B)
	{
		const int32 I = FCString::Atoi(*S.Mid(FCString::Strlen(LabScnPrefix)));
		FString Why;
		if (!LabScenarios.IsValidIndex(I))
		{
			return true;
		}
		if (B->PlayScenario(LabScenarios[I], Why))
		{
			LabSkill = B->FindSkill(LabScenarios[I].Skill);
			ShowToast(FString::Printf(TEXT("Scenario %s on the virtual robot."), *LabScenarios[I].Name), CyanCol);
		}
		else
		{
			ShowToast(FString::Printf(TEXT("Cannot play the scenario: %s"), *Why), BadCol);
		}
	}
	else if (S.StartsWith(WalkScnPrefix))
	{
		const int32 I = FCString::Atoi(*S.Mid(FCString::Strlen(WalkScnPrefix)));
		if (WalkScenarios.IsValidIndex(I))
		{
			PostJson(TEXT("/walk/scenario"), FString::Printf(TEXT("{\"id\":\"%s\"}"), *R1Panel::JsonEscape(WalkScenarios[I].Id)),
				After(FString::Printf(TEXT("Scenario %s: the sandbox restarts and plays it."), *WalkScenarios[I].Name)));
			NextPoll = 0.0;
		}
	}
	else if (ClickId == IdScenarioStop)
	{
		PostJson(TEXT("/walk/scenario"), TEXT("{\"stop\":true}"), After(TEXT("Scenario stopped (the sandbox keeps running).")));
	}
	return true;
}

// =====================================================================================================
// left panel: walk controls (terrain, throw, scenarios)
// =====================================================================================================

void AR1BehaviourPanel::DrawWalkSimControls(float X, float Inner, float& Y)
{
	const FR1WalkState& S = WalkState;
	UFont* Font = GEngine->GetMediumFont();
	const FString Course = S.bRunning && !S.Course.IsEmpty() ? S.Course : WalkCourse;
	if (Course == TEXT("procedural"))
	{
		// #3 procedural terrain (the same seeded generator the training scenes use)
		float CX = X;
		DrawText(TEXT("Seed"), DimCol, CX, Y + 6.f, Font, 0.85f);
		CX += 44.f;
		DrawText(FString::Printf(TEXT("%d"), S.bRunning && S.Seed >= 0 ? S.Seed : WalkSeed), TextCol, CX, Y + 6.f, Font, 0.85f);
		CX += 52.f;
		DrawButton({IdSeedDown, TEXT("-"), {CX, Y}, {30.f, 28.f}, !S.bRunning, ChipCol});
		DrawButton({IdSeedUp, TEXT("+"), {CX + 34.f, Y}, {30.f, 28.f}, !S.bRunning, ChipCol});
		CX += 72.f;
		DrawButton({IdNewTerrain, TEXT("New terrain"), {CX, Y}, {112.f, 28.f}, bServiceOnline && Job.IsEmpty(), VirtualCol});
		CX += 120.f;
		DrawText(TEXT("Diff"), DimCol, CX, Y + 6.f, Font, 0.85f);
		CX += 38.f;
		const float Cur = S.bRunning && S.Difficulty >= 0.f ? S.Difficulty : WalkDifficulty;
		const float ChipW = FMath::Max(36.f, (X + Inner - CX - 3.f * 4.f) / 4.f);
		for (int32 i = 0; i < UE_ARRAY_COUNT(Difficulties); ++i)
		{
			DrawButton({SimChipId(DiffPrefix, i), FString::Printf(TEXT("%.0f%%"), Difficulties[i] * 100.f), {CX, Y}, {ChipW, 28.f}, !S.bRunning,
				FMath::IsNearlyEqual(Cur, Difficulties[i], 0.01f) ? ChipOnCol : ChipCol});
			CX += ChipW + 4.f;
		}
		if (S.bRunning)
		{
			DisabledWhy.Add(IdSeedDown, TEXT("stop the walk first, or press New terrain"));
			DisabledWhy.Add(IdSeedUp, TEXT("stop the walk first, or press New terrain"));
		}
		Y += 34.f;
	}
	if (S.bRunning)
	{
		// #9 throwables: MuJoCo free bodies in the service; drawn here as plain meshes
		float CX = X;
		DrawText(TEXT("Throw"), DimCol, CX, Y + 6.f, Font, 0.85f);
		CX += 54.f;
		const bool bCan = !S.Replay.bActive;
		DrawButton({IdThrowBall, TEXT("Ball"), {CX, Y}, {70.f, 28.f}, bCan, ForceCol * 0.75f});
		DrawButton({IdThrowBox, TEXT("Box"), {CX + 74.f, Y}, {70.f, 28.f}, bCan, ForceCol * 0.75f});
		CX += 160.f;
		DrawText(TEXT("from"), DimCol, CX, Y + 6.f, Font, 0.85f);
		CX += 42.f;
		const float ChipW = FMath::Max(48.f, (X + Inner - CX - 3.f * 4.f) / 4.f);
		for (int32 i = 0; i < 4; ++i)
		{
			DrawButton({SimChipId(FromPrefix, i), FromLabels[i], {CX, Y}, {ChipW, 28.f}, true, ThrowFrom == i ? ChipOnCol : ChipCol});
			CX += ChipW + 4.f;
		}
		if (!bCan)
		{
			DisabledWhy.Add(IdThrowBall, TEXT("a replay is playing"));
			DisabledWhy.Add(IdThrowBox, TEXT("a replay is playing"));
		}
		Y += 34.f;
	}
	if (WalkScenarios.Num() > 0)
	{
		// #18 walk scenario scripts (RobotControl/scenarios, played by the service on sim time)
		const bool bPlaying = S.Scenario.bValid && !S.Scenario.bDone;
		const float LabelW = 76.f, StopW = bPlaying ? 64.f : 0.f;
		DrawText(TEXT("Scenario"), DimCol, X, Y + 6.f, Font, 0.85f);
		const int32 PerRow = FMath::Min(WalkScenarios.Num(), 3);
		const float ChipW = (Inner - LabelW - StopW - (PerRow - 1) * 4.f - (bPlaying ? 4.f : 0.f)) / PerRow;
		for (int32 i = 0; i < WalkScenarios.Num(); ++i)
		{
			if (i > 0 && i % PerRow == 0)
			{
				Y += 32.f;
			}
			const FR1ScenarioInfo& I = WalkScenarios[i];
			float TW = 0.f, TH = 0.f;
			GetTextSize(I.Name, TW, TH, Font, 0.9f);
			const bool bSel = S.Scenario.bValid && S.Scenario.Id == I.Id;
			DrawButton({SimChipId(WalkScnPrefix, i), TW + 12.f > ChipW ? I.Id : I.Name, {X + LabelW + (i % PerRow) * (ChipW + 4.f), Y}, {ChipW, 28.f},
				bServiceOnline && Job.IsEmpty(), bSel ? ChipOnCol : ChipCol});
		}
		if (bPlaying)
		{
			DrawButton({IdScenarioStop, TEXT("Stop"), {X + Inner - StopW, Y}, {StopW, 28.f}, true, StopCol * 0.85f});
		}
		Y += 34.f;
		if (S.Scenario.bValid)
		{
			DrawLine(FString::Printf(TEXT("scenario %s: step %d / %d   t %.1f s%s"), *S.Scenario.Id, S.Scenario.Step, S.Scenario.Steps, S.Scenario.T,
				S.Scenario.bDone ? TEXT("   (done)") : TEXT("")), X, Y, S.Scenario.bDone ? DimCol : CyanCol, 0.82f);
		}
	}
}

// =====================================================================================================
// sim card (right column, or under the panel when the window is narrow)
// =====================================================================================================

void AR1BehaviourPanel::DrawSimCard(float X, float& Y, float W)
{
	const float Y0 = Y;
	DrawRect(FLinearColor(0.03f, 0.03f, 0.04f, 0.92f), X - 10.f, Y - 8.f, W + 20.f, LastCardH + 16.f);
	DrawRect(SimCol, X - 10.f, Y - 8.f, W + 20.f, 3.f);
	if (Tab == ER1PanelTab::Walk)
	{
		DrawWalkSimCard(X, Y, W);
	}
	else
	{
		DrawLabSimCard(X, Y, W);
	}
	LastCardH = FMath::Max(40.f, Y - Y0);
}

void AR1BehaviourPanel::DrawMotorBlock(const FR1MotorReadout& M, float X, float& Y, float W)
{
	const FR1MotorModelConfig& C = FR1MotorModelConfig::Get();
	if (!M.bValid)
	{
		DrawLine(TEXT("MOTORS  no data yet"), X, Y, DimCol, 0.82f);
		return;
	}
	DrawLine(FString::Printf(TEXT("MOTORS  hottest: %s %.1f C   (warn %.0f, limit %.0f)"), M.Hottest.IsEmpty() ? TEXT("-") : *Pretty(M.Hottest), M.HottestC, C.Warn, C.Limit),
		X, Y, TempColor(M.HottestC), 0.82f);
	{
		// temperature bar: ambient .. limit + 10 C, ticks at warn (amber) and limit (red)
		const float Lo = C.Ambient, Hi = C.Limit + 10.f;
		auto At = [&](float T) { return X + W * FMath::Clamp((T - Lo) / FMath::Max(Hi - Lo, 1.f), 0.f, 1.f); };
		DrawRect(FLinearColor(1.f, 1.f, 1.f, 0.10f), X, Y + 1.f, W, 8.f);
		DrawRect(TempColor(M.HottestC), X, Y + 1.f, FMath::Max(2.f, At(M.HottestC) - X), 8.f);
		DrawRect(WarnCol, At(C.Warn) - 1.f, Y - 2.f, 2.f, 14.f);
		DrawRect(BadCol, At(C.Limit) - 1.f, Y - 2.f, 2.f, 14.f);
		Y += 15.f;
	}
	{
		// battery bar + numbers
		const float Soc = FMath::Clamp(M.SocPct / 100.f, 0.f, 1.f);
		const FLinearColor BCol = Soc > 0.3f ? OkCol : Soc > 0.15f ? WarnCol : BadCol;
		DrawRect(FLinearColor(1.f, 1.f, 1.f, 0.10f), X, Y + 4.f, 60.f, 10.f);
		DrawRect(BCol, X, Y + 4.f, FMath::Max(2.f, 60.f * Soc), 10.f);
		DrawRect(FLinearColor(1.f, 1.f, 1.f, 0.4f), X + 60.f, Y + 6.f, 3.f, 6.f);
		float Ty = Y;
		DrawLine(FString::Printf(TEXT("battery %.1f%%   %s left   %.0f W"), M.SocPct,
			M.MinutesLeft >= 0.f ? *FString::Printf(TEXT("%.0f min"), M.MinutesLeft) : TEXT("--"), M.PowerW), X + 70.f, Ty, TextCol, 0.82f);
		Y = Ty;
	}
	const int32 Peak = M.MaxFracIndex();
	if (Peak != INDEX_NONE && M.Torque.IsValidIndex(Peak))
	{
		DrawLine(FString::Printf(TEXT("peak torque: %s %.2f N m (%.0f%% of its limit)"), M.Names.IsValidIndex(Peak) ? *Pretty(M.Names[Peak]) : TEXT("?"),
			FMath::Abs(M.Torque[Peak]), M.TorqueFrac[Peak] * 100.f), X, Y, FracColor(M.TorqueFrac[Peak]), 0.82f);
	}
	if (bTorqueAlert)
	{
		DrawLine(TorqueAlertText, X, Y, BadCol, 0.85f);
	}
	if (M.Limit.Num() > 0)
	{
		DrawWrapped(FString::Printf(TEXT("over the limit: %s"), *FString::JoinBy(M.Limit, TEXT(", "), [](const FString& N) { return Pretty(N); })), X, Y, W, BadCol, 0.8f);
	}
	else if (M.Warn.Num() > 0)
	{
		DrawWrapped(FString::Printf(TEXT("warm: %s"), *FString::JoinBy(M.Warn, TEXT(", "), [](const FString& N) { return Pretty(N); })), X, Y, W, WarnCol, 0.8f);
	}
	DrawLine(TEXT("display only: never derates a motor; battery size is a placeholder"), X, Y, DimCol, 0.72f);
	Y += 2.f;
}

void AR1BehaviourPanel::DrawLabSimCard(float X, float& Y, float W)
{
	UR1SkillBrainComponent* B = FindBrain();
	DrawSection(TEXT("SIMULATION TOOLS"), X, W, Y, SimCol);
	if (!B)
	{
		return;
	}
	UFont* Font = GEngine->GetMediumFont();
	const FR1BrainReadout& R = B->GetReadout();

	// #18 lab scenario scripts (read from RobotControl/scenarios; played by the brain on its own clock)
	if (LabScenarios.Num() == 0)
	{
		DrawWrapped(TEXT("No lab scenarios in RobotControl/scenarios (Reload re-reads them)."), X, Y, W, DimCol, 0.8f);
	}
	else
	{
		DrawLine(TEXT("Scenario scripts"), X, Y, DimCol, 0.8f);
		const int32 PerRow = W < 400.f ? 2 : 3;
		const float ChipW = (W - (PerRow - 1) * 4.f) / PerRow;
		for (int32 i = 0; i < LabScenarios.Num(); ++i)
		{
			if (i > 0 && i % PerRow == 0)
			{
				Y += 32.f;
			}
			const FR1LabScenario& Sc = LabScenarios[i];
			float TW = 0.f, TH = 0.f;
			GetTextSize(Sc.Name, TW, TH, Font, 0.9f);
			const bool bOn = R.ScenarioName == Sc.Name;
			const FName Bid = SimChipId(LabScnPrefix, i);
			DrawButton({Bid, TW + 12.f > ChipW ? Sc.Id : Sc.Name, {X + (i % PerRow) * (ChipW + 4.f), Y}, {ChipW, 28.f}, !R.bExternal, bOn ? ChipOnCol : ChipCol});
			if (R.bExternal)
			{
				DisabledWhy.Add(Bid, TEXT("the web app is driving the virtual robot right now"));
			}
		}
		Y += 34.f;
		if (!R.ScenarioName.IsEmpty())
		{
			DrawLine(FString::Printf(TEXT("%s  %.1f / %.0f s  step %d / %d"), *R.ScenarioName, R.ScenarioT, R.ScenarioDurationS, R.ScenarioStep, R.ScenarioSteps),
				X, Y, CyanCol, 0.82f);
		}
	}

	// tools: physics overlay (#2), record dataset (#7)
	{
		const float G = 6.f, BW = (W - G) * 0.5f;
		DrawButton({IdPhysics, bPhysicsOverlay ? TEXT("Physics overlay: on") : TEXT("Physics overlay: off"), {X, Y}, {BW, 30.f}, true,
			bPhysicsOverlay ? ChipOnCol : ChipCol});
		const FR1DatasetRecording& Rec = B->GetRecording();
		const bool bCam = B->GetHeadCamTexture() != nullptr;
		DrawButton({IdRecord, Rec.bActive ? FString::Printf(TEXT("Recording %d / %d"), Rec.Frames, Rec.Wanted) : FString(TEXT("Record dataset")),
			{X + BW + G, Y}, {BW, 30.f}, bCam || Rec.bActive, Rec.bActive ? StopCol * 0.85f : VirtualCol});
		if (!bCam && !Rec.bActive)
		{
			DisabledWhy.Add(IdRecord, TEXT("needs the head camera (the URLab articulation in the level)"));
		}
		Y += 36.f;
		if (Rec.bActive || Rec.Frames > 0)
		{
			const FString Where = FPaths::GetCleanFilename(Rec.Dir);
			DrawWrapped(!Rec.LastError.IsEmpty() ? FString::Printf(TEXT("dataset %s stopped: %s"), *Where, *Rec.LastError)
				: FString::Printf(TEXT("dataset %s: %d frames + labels.json in datasets/unreal_head_cam (simulation only)"), *Where, Rec.Frames),
				X, Y, W, Rec.LastError.IsEmpty() ? DimCol : WarnCol, 0.78f);
		}
	}

	// #5 heat + battery (C++ motor model on the brain's joints; PD torque estimate)
	if (R.bExternal)
	{
		DrawLine(TEXT("MOTORS  the web app is driving: no lab estimate"), X, Y, DimCol, 0.82f);
	}
	else
	{
		DrawMotorBlock(B->GetMotorReadout(), X, Y, W);
	}

	// #15 raw vs filtered action
	DrawLabActionPlot(X, Y, W);
}

void AR1BehaviourPanel::DrawLabActionPlot(float X, float& Y, float W)
{
	UR1SkillBrainComponent* B = FindBrain();
	const FR1ActionFilter& F = B->GetActionFilter();
	if (F.NumJoints() == 0 || F.HistoryRaw[0].Num() < 2)
	{
		DrawLine(TEXT("ACTION  play a skill to see raw vs filtered"), X, Y, DimCol, 0.82f);
		return;
	}
	DrawLine(FString::Printf(TEXT("ACTION  raw (grey) vs low-pass a=%.2f (cyan)   jitter %.3f"), FR1MotorModelConfig::Get().FilterAlpha, F.Jitter()),
		X, Y, TextCol, 0.82f);
	const int32 K = F.NumJoints();
	const float Gap = 8.f, PW = (W - (K - 1) * Gap) / K, PH = 46.f;
	const TArray<FString>& Labels = B->GetReadout().JointLabels;
	for (int32 j = 0; j < K; ++j)
	{
		const float PX = X + j * (PW + Gap);
		DrawRect(CardBg, PX, Y, PW, PH);
		const float Mid = Y + PH * 0.5f;
		DrawRect(DividerCol, PX, Mid, PW, 1.f);
		const TArray<float>& Raw = F.HistoryRaw[j];
		const TArray<float>& Flt = F.HistoryFiltered[j];
		float Scale = 0.25f;
		for (float V : Raw) Scale = FMath::Max(Scale, FMath::Abs(V));
		const float Step = PW / FMath::Max(1, FR1ActionFilter::HistoryLen - 1);
		auto Py = [&](float V) { return Mid - V / Scale * (PH * 0.45f); };
		for (int32 k = 1; k < Raw.Num(); ++k)
		{
			const float X0 = PX + (k - 1) * Step, X1 = PX + k * Step;
			AHUD::DrawLine(X0, Py(Raw[k - 1]), X1, Py(Raw[k]), FLinearColor(0.6f, 0.62f, 0.67f, 0.8f), 1.f);
			AHUD::DrawLine(X0, Py(Flt[k - 1]), X1, Py(Flt[k]), CyanCol, 2.f);
		}
		DrawText(Labels.IsValidIndex(j) ? Labels[j] : FString(), DimCol, PX + 4.f, Y + 2.f, GEngine->GetSmallFont(), 0.9f);
	}
	Y += PH + 6.f;
}

void AR1BehaviourPanel::DrawWalkSimCard(float X, float& Y, float W)
{
	const FR1WalkState& S = WalkState;
	DrawSection(TEXT("SIMULATION  (walk sandbox)"), X, W, Y, SimCol);
	if (!S.bRunning)
	{
		DrawWrapped(TEXT("Start the sandbox (Hold pose and Let go work without a walking policy) to see forces, heat, battery, replay and fall analysis."),
			X, Y, W, DimCol, 0.82f);
		if (S.FallsLog.Num() > 0)
		{
			DrawLine(FString::Printf(TEXT("falls logged: %d"), S.FallsLog.Num()), X, Y, DimCol, 0.82f);
		}
		return;
	}
	const float G = 6.f, BW = (W - G) * 0.5f;

	// #12 replay
	if (S.Replay.bActive)
	{
		DrawButton({IdReplayStop, TEXT("Stop replay"), {X, Y}, {BW, 30.f}, true, StopCol * 0.85f});
	}
	else
	{
		DrawButton({IdReplay, TEXT("Replay last 3 s"), {X, Y}, {BW, 30.f}, S.Replay.bValid, VirtualCol});
		if (!S.Replay.bValid)
		{
			DisabledWhy.Add(IdReplay, TEXT("this service has no replay (update RobotControl)"));
		}
	}
	DrawButton({IdAutoReplay, S.Replay.bAutoOnFall ? TEXT("Auto replay on fall: on") : TEXT("Auto replay on fall: off"), {X + BW + G, Y}, {BW, 30.f},
		S.Replay.bValid, S.Replay.bAutoOnFall ? ChipOnCol : ChipCol});
	Y += 36.f;
	if (S.Replay.bActive)
	{
		DrawLine(FString::Printf(TEXT("REPLAY %.2gx  %.1f / %.1f s  - free camera: WASD, Q/E, right mouse"), S.Replay.Speed, S.Replay.T, S.Replay.DurationS),
			X, Y, WarnCol, 0.82f);
	}

	// #2 overlay + #15 apply filter
	DrawButton({IdPhysics, bPhysicsOverlay ? TEXT("Physics overlay: on") : TEXT("Physics overlay: off"), {X, Y}, {BW, 30.f}, true,
		bPhysicsOverlay ? ChipOnCol : ChipCol});
	DrawButton({IdFilter, S.ActionFilter.bApply ? TEXT("Apply filter: on") : TEXT("Apply filter: off"), {X + BW + G, Y}, {BW, 30.f},
		S.ActionFilter.bValid && S.Mode == TEXT("policy"), S.ActionFilter.bApply ? ChipOnCol : ChipCol});
	if (S.Mode != TEXT("policy"))
	{
		DisabledWhy.Add(IdFilter, TEXT("the filter acts on the walking policy's actions (start walking first)"));
	}
	Y += 36.f;
	if (bPhysicsOverlay)
	{
		const FR1WalkPhysics& P = S.Physics;
		DrawWrapped(TEXT("orange: contacts + forces   white: centre of mass   cyan: support polygon   capture point: green inside, red outside"),
			X, Y, W, DimCol, 0.76f);
		if (P.bValid)
		{
			DrawLine(P.bHasMargin ? FString::Printf(TEXT("capture point margin %+.1f cm (%s)"), P.CpMarginM * 100.f, P.CpMarginM >= 0.f ? TEXT("inside the feet") : TEXT("outside: falling"))
				: FString(TEXT("capture point: no support (no foot on the ground)")), X, Y, !P.bHasMargin ? WarnCol : P.CpMarginM >= 0.f ? OkCol : BadCol, 0.82f);
		}
	}

	// #5 heat + battery (from the service's motor model)
	DrawMotorBlock(S.Motors, X, Y, W);
	// #15 action filter view
	DrawWalkActionBars(X, Y, W);
	// #19 fall analysis
	DrawFallStrip(X, Y, W);
}

void AR1BehaviourPanel::DrawWalkActionBars(float X, float& Y, float W)
{
	const FR1WalkActionFilter& A = WalkState.ActionFilter;
	if (!A.bValid || A.Raw.Num() == 0)
	{
		DrawLine(TEXT("ACTION  no policy actions (hold modes have none)"), X, Y, DimCol, 0.82f);
		return;
	}
	DrawLine(FString::Printf(TEXT("ACTION  raw (grey) vs EMA a=%.2f (cyan)  jitter %.3f  %s"), A.Alpha, A.Jitter, A.bApply ? TEXT("APPLIED") : TEXT("view only")),
		X, Y, A.bApply ? CyanCol : TextCol, 0.8f);
	const int32 N = A.Raw.Num();
	const float Slot = W / N, H = 40.f, Mid = Y + H * 0.5f;
	DrawRect(CardBg, X, Y, W, H);
	DrawRect(DividerCol, X, Mid, W, 1.f);
	float Scale = 0.5f;
	for (float V : A.Raw) Scale = FMath::Max(Scale, FMath::Abs(V));
	for (int32 i = 0; i < N; ++i)
	{
		const float R = FMath::Clamp(A.Raw[i] / Scale, -1.f, 1.f) * H * 0.45f;
		const float F = A.Filtered.IsValidIndex(i) ? FMath::Clamp(A.Filtered[i] / Scale, -1.f, 1.f) * H * 0.45f : 0.f;
		const float BX = X + i * Slot + 1.f, BW = FMath::Max(1.f, Slot * 0.42f);
		DrawRect(FLinearColor(0.6f, 0.62f, 0.67f, 0.85f), BX, R >= 0.f ? Mid - R : Mid, BW, FMath::Max(1.f, FMath::Abs(R)));
		DrawRect(CyanCol, BX + BW + 1.f, F >= 0.f ? Mid - F : Mid, BW, FMath::Max(1.f, FMath::Abs(F)));
	}
	Y += H + 6.f;
}

void AR1BehaviourPanel::DrawFallStrip(float X, float& Y, float W)
{
	const FR1WalkState& S = WalkState;
	const FR1FallAnalysis& F = S.FallAnalysis;
	if (!F.bValid)
	{
		DrawLine(FString::Printf(TEXT("FALLS  %d logged - none analysed yet (Let go makes one)"), S.FallsLog.Num()), X, Y, DimCol, 0.82f);
		return;
	}
	DrawLine(FString::Printf(TEXT("LAST FALL  at %.1f s, warning %.2f s earlier   (%d logged)"), F.TFall, F.LeadS, S.FallsLog.Num()), X, Y, BadCol, 0.82f);
	const float H = 60.f, Top = Y, Mid = Y + H * 0.5f, Bottom = Y + H;
	DrawRect(CardBg, X, Top, W, H);
	DrawRect(DividerCol, X, Mid, W, 1.f);
	const float T0 = F.History.Num() > 0 ? F.History[0].T : F.TFall - 3.f;
	const float T1 = FMath::Max(F.TFall, F.History.Num() > 0 ? F.History.Last().T : F.TFall) + 0.1f;
	auto Tx = [&](float T) { return X + W * FMath::Clamp((T - T0) / FMath::Max(T1 - T0, 0.1f), 0.f, 1.f); };
	auto MarginY = [&](float M) { return Mid - FMath::Clamp(M / 0.15f, -1.f, 1.f) * H * 0.45f; };     // +-15 cm
	auto TiltY = [&](float D) { return Bottom - FMath::Clamp(D / 90.f, 0.f, 1.f) * H * 0.9f; };
	for (int32 i = 1; i < F.History.Num(); ++i)
	{
		const FR1FallSample& A = F.History[i - 1];
		const FR1FallSample& B = F.History[i];
		AHUD::DrawLine(Tx(A.T), TiltY(A.TiltDeg), Tx(B.T), TiltY(B.TiltDeg), WarnCol, 1.5f);
		if (A.bHasMargin && B.bHasMargin)
		{
			AHUD::DrawLine(Tx(A.T), MarginY(A.CpMarginM), Tx(B.T), MarginY(B.CpMarginM), B.CpMarginM >= 0.f ? OkCol : BadCol, 2.f);
		}
	}
	UFont* Small = GEngine->GetSmallFont();
	const float XU = Tx(F.TUnstable), XF = Tx(F.TFall);
	DrawRect(WarnCol, XU - 1.f, Top, 2.f, H);
	DrawRect(BadCol, XF - 1.f, Top, 2.f, H);
	DrawText(TEXT("unstable"), WarnCol, FMath::Min(XU + 3.f, X + W - 60.f), Top + 2.f, Small, 0.9f);
	DrawText(TEXT("fall"), BadCol, FMath::Min(XF + 3.f, X + W - 26.f), Top + 16.f, Small, 0.9f);
	Y += H + 3.f;
	DrawLine(TEXT("green/red: capture-point margin (+-15 cm)   amber: tilt (0-90 deg)"), X, Y, DimCol, 0.72f);
	if (!F.Reason.IsEmpty())
	{
		DrawWrapped(FString::Printf(TEXT("why: %s%s"), *F.Reason, F.Cause.IsEmpty() ? TEXT("") : *FString::Printf(TEXT(" [%s]"), *F.Cause)), X, Y, W, TextCol, 0.8f);
	}
}

// =====================================================================================================
// replay banner (panel space) and the 3D overlay (screen pixels, drawn before the panel scale)
// =====================================================================================================

void AR1BehaviourPanel::DrawReplayBanner()
{
	const FR1WalkReplay& R = WalkState.Replay;
	if (!WalkState.bRunning || !R.bActive)
	{
		return;
	}
	// between the left panel (ends at 620) and the right edge, near the bottom; skipped when the window is narrow
	const float Left = 640.f, Avail = ViewW - Left - 20.f;
	if (Avail < 320.f)
	{
		return;
	}
	const float W = FMath::Min(560.f, Avail), H = 64.f;
	const float X = Left + (Avail - W) * 0.5f, Y = ViewH - H - 40.f;
	DrawRect(FLinearColor(0.f, 0.f, 0.f, 0.78f), X, Y, W, H);
	DrawRect(WarnCol, X, Y, W, 3.f);
	DrawText(FString::Printf(TEXT("REPLAY  %.2gx   %.1f / %.1f s"), R.Speed, R.T, R.DurationS), WarnCol, X + 14.f, Y + 10.f, GEngine->GetMediumFont(), 1.15f);
	DrawText(TEXT("Free camera: WASD move, Q/E down/up, right mouse look, Shift faster"), DimCol, X + 14.f, Y + 38.f, GEngine->GetMediumFont(), 0.78f);
	// progress
	DrawRect(FLinearColor(1.f, 1.f, 1.f, 0.12f), X, Y + H - 4.f, W, 4.f);
	DrawRect(WarnCol, X, Y + H - 4.f, W * FMath::Clamp(R.T / FMath::Max(R.DurationS, 0.1f), 0.f, 1.f), 4.f);
	DrawButton({IdReplayStop, TEXT("Stop"), {X + W - 86.f, Y + 12.f}, {74.f, 30.f}, true, StopCol * 0.85f});
}

void AR1BehaviourPanel::DrawPhysicsOverlay()
{
	if (!bPhysicsOverlay || !Canvas)
	{
		return;
	}
	APlayerController* PC = GetOwningPlayerController();
	if (!PC)
	{
		return;
	}
	auto ToScreen = [PC](const FVector& W, FVector2D& S) { return PC->ProjectWorldLocationToScreen(W, S, /*bPlayerViewportRelative*/ false); };
	auto Line = [&](const FVector& A, const FVector& B, const FLinearColor& C, float T)
	{
		FVector2D Sa, Sb;
		if (ToScreen(A, Sa) && ToScreen(B, Sb))
		{
			AHUD::DrawLine(float(Sa.X), float(Sa.Y), float(Sb.X), float(Sb.Y), C, T);
		}
	};
	auto Arrow = [&](const FVector& A, const FVector& B, const FLinearColor& C, float T)
	{
		FVector2D Sa, Sb;
		if (!ToScreen(A, Sa) || !ToScreen(B, Sb))
		{
			return;
		}
		AHUD::DrawLine(float(Sa.X), float(Sa.Y), float(Sb.X), float(Sb.Y), C, T);
		const FVector2D D = (Sb - Sa).GetSafeNormal(), P(-D.Y, D.X);
		if (!D.IsNearlyZero())
		{
			const FVector2D H1 = Sb - D * 10.f + P * 5.f, H2 = Sb - D * 10.f - P * 5.f;
			AHUD::DrawLine(float(Sb.X), float(Sb.Y), float(H1.X), float(H1.Y), C, T);
			AHUD::DrawLine(float(Sb.X), float(Sb.Y), float(H2.X), float(H2.Y), C, T);
		}
	};
	auto Ring = [&](const FVector& Ctr, const FVector& AxisA, const FVector& AxisB, float R, const FLinearColor& C, float T)
	{
		constexpr int32 Seg = 20;
		for (int32 i = 0; i < Seg; ++i)
		{
			const float A0 = 2.f * PI * i / Seg, A1 = 2.f * PI * (i + 1) / Seg;
			Line(Ctr + (AxisA * FMath::Cos(A0) + AxisB * FMath::Sin(A0)) * R, Ctr + (AxisA * FMath::Cos(A1) + AxisB * FMath::Sin(A1)) * R, C, T);
		}
	};
	auto Label = [&](const FVector& W, const FString& Text, const FLinearColor& C)
	{
		FVector2D S;
		if (ToScreen(W, S))
		{
			DrawText(Text, C, float(S.X) + 8.f, float(S.Y) - 8.f, GEngine->GetSmallFont(), 1.f);
		}
	};

	if (Tab == ER1PanelTab::Walk && WalkState.bRunning && WalkState.Physics.bValid)
	{
		const FR1WalkPhysics& P = WalkState.Physics;
		// ground height under the robot: the lowest foot contact (stairs / ramps on procedural terrain), else the floor
		double GroundZ = 0.0;
		bool bAny = false;
		for (const FR1WalkContact& C : P.Contacts)
		{
			if (IsFoot(C.A) || IsFoot(C.B))
			{
				GroundZ = bAny ? FMath::Min(GroundZ, C.PosMj.Z) : C.PosMj.Z;
				bAny = true;
			}
		}
		const double Lift = 0.004;   // just above the surface
		// support polygon
		for (int32 i = 0; i < P.SupportMj.Num() && P.SupportMj.Num() >= 2; ++i)
		{
			const FVector2D& A = P.SupportMj[i];
			const FVector2D& B = P.SupportMj[(i + 1) % P.SupportMj.Num()];
			Line(MjUe::PositionToUe(FVector(A.X, A.Y, GroundZ + Lift)), MjUe::PositionToUe(FVector(B.X, B.Y, GroundZ + Lift)), CyanCol, 2.f);
		}
		// contacts + forces (0.1 cm per N, capped at 80 cm)
		for (const FR1WalkContact& C : P.Contacts)
		{
			const FVector At = MjUe::PositionToUe(C.PosMj);
			Ring(At, FVector::ForwardVector, FVector::RightVector, 2.5f, ForceCol, 1.5f);
			FVector F = MjUe::DirectionToUe(C.ForceMj) * 0.1;
			if (F.Size() > 80.0) F = F.GetSafeNormal() * 80.0;
			if (F.Size() > 1.0) Arrow(At, At + F, ForceCol, 2.f);
		}
		// centre of mass and its ground projection
		const FVector Com = MjUe::PositionToUe(P.ComMj);
		const FVector ComGround = MjUe::PositionToUe(FVector(P.ComMj.X, P.ComMj.Y, GroundZ + Lift));
		Ring(Com, FVector::ForwardVector, FVector::RightVector, 6.f, FLinearColor::White, 2.f);
		Ring(Com, FVector::ForwardVector, FVector::UpVector, 6.f, FLinearColor::White, 1.f);
		Line(Com, ComGround, FLinearColor(1.f, 1.f, 1.f, 0.5f), 1.f);
		Ring(ComGround, FVector::ForwardVector, FVector::RightVector, 3.f, FLinearColor::White, 1.5f);
		Label(Com, TEXT("CoM"), FLinearColor::White);
		// capture point: green inside the support polygon, red outside, amber without support
		const FLinearColor CpCol = !P.bHasMargin ? WarnCol : P.CpMarginM >= 0.f ? OkCol : BadCol;
		const FVector Cp = MjUe::PositionToUe(FVector(P.CapturePointMj.X, P.CapturePointMj.Y, GroundZ + Lift));
		Arrow(ComGround, Cp, CpCol, 2.f);
		Line(Cp + FVector(-6, -6, 0), Cp + FVector(6, 6, 0), CpCol, 3.f);
		Line(Cp + FVector(-6, 6, 0), Cp + FVector(6, -6, 0), CpCol, 3.f);
		Label(Cp, P.bHasMargin ? FString::Printf(TEXT("capture point %+.1f cm"), P.CpMarginM * 100.f) : FString(TEXT("capture point")), CpCol);
	}
	else if (Tab == ER1PanelTab::Lab)
	{
		// head joints: a glow ring sized by the larger torque fraction, an arrow per joint along the way it pushes
		UR1SkillBrainComponent* B = FindBrain();
		FVector HeadPos;
		FQuat HeadRot;
		if (!B || B->GetReadout().bExternal || !B->GetBodyPoseUe(TEXT("head_pitch_link"), HeadPos, HeadRot))
		{
			return;
		}
		const FR1MotorReadout& M = B->GetMotorReadout();
		const int32 IP = M.Names.IndexOfByKey(TEXT("head_pitch_joint")), IY = M.Names.IndexOfByKey(TEXT("head_yaw_joint"));
		if (!M.bValid || IP == INDEX_NONE || IY == INDEX_NONE)
		{
			return;
		}
		const FVector Fwd = HeadRot.RotateVector(FVector::ForwardVector);
		const FVector Left = HeadRot.RotateVector(FVector(0.0, -1.0, 0.0));   // MuJoCo +y (left) is UE -y
		const FVector Up = HeadRot.RotateVector(FVector::UpVector);
		const FVector Base = HeadPos + Fwd * 16.f;
		const float FY = M.TorqueFrac[IY], FP = M.TorqueFrac[IP];
		const float Worst = FMath::Max(FY, FP);
		FLinearColor Glow = FracColor(Worst);
		Glow.A = 0.5f + 0.5f * FMath::Clamp(Worst, 0.f, 1.f);
		Ring(HeadPos, Fwd, Left, 12.f + 10.f * FMath::Clamp(Worst, 0.f, 1.f), Glow, 1.f + 3.f * FMath::Clamp(Worst, 0.f, 1.f));
		// yaw torque + turns the head left; pitch torque + tips it down
		const FVector YawEnd = Base + Left * FMath::Sign(M.Torque[IY]) * (4.f + 30.f * FMath::Clamp(FY, 0.f, 1.f));
		const FVector PitchEnd = Base - Up * FMath::Sign(M.Torque[IP]) * (4.f + 30.f * FMath::Clamp(FP, 0.f, 1.f));
		Arrow(Base, YawEnd, FracColor(FY), 3.f);
		Arrow(Base, PitchEnd, FracColor(FP), 3.f);
		Label(YawEnd, FString::Printf(TEXT("yaw %+.2f N m (%.0f%%)"), M.Torque[IY], FY * 100.f), FracColor(FY));
		Label(PitchEnd, FString::Printf(TEXT("pitch %+.1f N m (%.0f%%)"), M.Torque[IP], FP * 100.f), FracColor(FP));
	}
}
