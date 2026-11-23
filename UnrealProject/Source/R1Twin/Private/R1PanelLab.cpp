// Scenario lab tab: hatch the brains the web app trained and test them on the virtual R1. SIMULATION ONLY.
//
// Two modes. Watch (default): pick a skill (and a size where it applies), Play, read one result line; the head camera
// sits on the right. Experiment adds the tools: the look-at target (Script / Manual / Orbit), disturbance presets
// (Calm, Noisy sensors, Tired motors, Rough day) and a Push button, the skill timeline, the run length and the full
// result card (error vs. the validated error; whether the real robot's safety envelope would stop the run).
// The last skill, size, run length and mode are remembered between sessions. The brain is UR1SkillBrainComponent.
#include "R1BehaviourPanel.h"
#include "R1PanelStyle.h"
#include "R1SkillBrain.h"

#include "Dom/JsonObject.h"
#include "Engine/Engine.h"
#include "Misc/DateTime.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"
#include "Engine/World.h"
#include "EngineUtils.h"

using namespace R1PanelStyle;

namespace
{
	const FName IdReload(TEXT("lab_reload")), IdPlay(TEXT("lab_play")), IdMode(TEXT("lab_mode")), IdPush(TEXT("lab_push"));
	const FName IdTlAdd(TEXT("tl_add")), IdTlClear(TEXT("tl_clear")), IdTlPlay(TEXT("tl_play")), IdTlLoop(TEXT("tl_loop"));
	const TCHAR* SkillPrefix = TEXT("lab_skill_");
	const TCHAR* TlDelPrefix = TEXT("tl_del_");
	const float Durations[] = {6.f, 12.f, 30.f};
	const FLinearColor LabCol(0.24f, 0.78f, 0.95f, 1.f);
	const FLinearColor DisturbCol(1.f, 0.62f, 0.25f, 1.f);

	struct FPreset { const TCHAR* Label; int32 DelaySteps; float NoiseDeg; float StrengthPct; };
	const FPreset Presets[] = {
		{TEXT("Calm"), 0, 0.f, 100.f},
		{TEXT("Noisy sensors"), 0, 2.f, 100.f},
		{TEXT("Tired motors"), 0, 0.f, 50.f},
		{TEXT("Rough day"), 4, 1.f, 75.f},   // 80 ms delay, 1 deg noise, 75 % motors
	};

	FName Id(const TCHAR* Prefix, int32 I) { return FName(*FString::Printf(TEXT("%s%d"), Prefix, I)); }

	/** 10 / 20 / 30 up to the skill's max, plus its max; none for Hold Centre (size does not apply). */
	TArray<float> SizesFor(const FR1SkillDef& S)
	{
		TArray<float> Out;
		if (S.Id == TEXT("hold_centre"))
		{
			return Out;
		}
		for (float A : {10.f, 20.f, 30.f})
		{
			if (A <= S.MaxDeg + 1e-3f) Out.Add(A);
		}
		if (!Out.ContainsByPredicate([&S](float A) { return FMath::IsNearlyEqual(A, S.MaxDeg); }))
		{
			Out.Add(S.MaxDeg);
		}
		return Out;
	}

	/** One plain word for how close the head stays to what the skill should do. */
	const TCHAR* Grade(float ErrDeg)
	{
		return ErrDeg < 1.f ? TEXT("excellent") : ErrDeg < 2.f ? TEXT("good") : ErrDeg < 4.f ? TEXT("fair") : TEXT("poor");
	}
}

UR1SkillBrainComponent* AR1BehaviourPanel::FindBrain()
{
	if (UR1SkillBrainComponent* B = Brain.Get())
	{
		return B;
	}
	if (UWorld* World = GetWorld())
	{
		for (TActorIterator<AActor> It(World); It; ++It)
		{
			if (UR1SkillBrainComponent* B = It->FindComponentByClass<UR1SkillBrainComponent>())
			{
				Brain = B;
				return B;
			}
		}
	}
	return nullptr;
}

void AR1BehaviourPanel::TickLab(double Now)
{
	// The web app's Library "View in Unreal" posts a play request to the behaviour service; poll it once a second.
	// The poll also tells the web app that this lab is running (GET /lab/poll records when it was last seen).
	if (Now < NextLabPoll || bLabPollInFlight)
	{
		return;
	}
	NextLabPoll = Now + 1.0;
	bLabPollInFlight = true;
	GetJson(FString::Printf(TEXT("/lab/poll?seen=%d"), FMath::Max(LabSeq, 0)), [this](int32 Code, const FString& Body)
	{
		bLabPollInFlight = false;
		TSharedPtr<FJsonObject> J;
		const TSharedPtr<FJsonObject>* Req = nullptr;
		if (Code != 200 || !FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Body), J) || !J.IsValid()
			|| !J->TryGetObjectField(TEXT("request"), Req) || !Req || !Req->IsValid())
		{
			if (LabSeq < 0 && Code == 200)
			{
				LabSeq = 0;   // the service has no request yet
			}
			return;
		}
		double Seq = 0.0, Size = 10.0, At = 0.0;
		FString Id;
		(*Req)->TryGetNumberField(TEXT("seq"), Seq);
		(*Req)->TryGetNumberField(TEXT("size"), Size);
		(*Req)->TryGetNumberField(TEXT("at"), At);
		(*Req)->TryGetStringField(TEXT("behaviour"), Id);
		const int32 S = static_cast<int32>(Seq);
		const bool bFresh = FDateTime::UtcNow().ToUnixTimestamp() - static_cast<int64>(At) < 60;   // asked in the last minute
		if (S <= LabSeq || (LabSeq < 0 && !bFresh))
		{
			LabSeq = FMath::Max(LabSeq, S);
			return;
		}
		LabSeq = S;
		UR1SkillBrainComponent* B = FindBrain();
		if (!B)
		{
			return;
		}
		B->ReloadSkills();   // the web app may have trained or activated a new version
		const int32 Idx = B->FindSkill(Id);
		FString Why;
		SetTab(ER1PanelTab::Lab);
		if (Idx != INDEX_NONE && B->Play(Idx, static_cast<float>(Size), LabDurationS, Why))
		{
			LabSkill = Idx;
			LabSizeDeg = static_cast<float>(Size);
			ShowToast(FString::Printf(TEXT("Playing %s (sent from the web app's Library)."), *B->GetSkills()[Idx].Label), CyanCol);
		}
		else
		{
			ShowToast(FString::Printf(TEXT("The web app asked for %s, but: %s"), *Id, Idx == INDEX_NONE ? TEXT("no such trained skill here") : *Why), BadCol);
		}
	});
}

bool AR1BehaviourPanel::LabShowsCamera()
{
	UR1SkillBrainComponent* B = FindBrain();
	return B && B->GetSkills().IsValidIndex(LabSkill) && B->GetSkills()[LabSkill].Id == TEXT("look_at");
}

void AR1BehaviourPanel::RememberLab()
{
	if (UR1SkillBrainComponent* B = FindBrain())
	{
		if (B->GetSkills().IsValidIndex(LabSkill))
		{
			LabSkillId = B->GetSkills()[LabSkill].Id;
		}
	}
	SaveConfig();
}

void AR1BehaviourPanel::DrawLabTab(float X, float Inner, float& Y)
{
	UR1SkillBrainComponent* B = FindBrain();
	if (!B)
	{
		DrawWrapped(TEXT("No scenario brain in this level (AR1TwinGameMode spawns it)."), X, Y, Inner, BadCol, 0.9f);
		return;
	}
	const TArray<FR1SkillDef>& Skills = B->GetSkills();
	const FR1BrainReadout& R = B->GetReadout();
	const bool bFree = !R.bExternal;
	UFont* Font = GEngine->GetMediumFont();
	if (!Skills.IsValidIndex(LabSkill))
	{
		LabSkill = FMath::Max(0, B->FindSkill(LabSkillId));   // the remembered skill, else the first
	}

	// ---------------------------------------------------------------- 1. behaviour
	DrawSection(TEXT("TRY A BEHAVIOUR"), X, Inner, Y, LabCol);
	TArray<int32> Shown;   // simple view: only the skills that are ready; More tools also lists the locked ones
	for (int32 i = 0; i < Skills.Num(); ++i)
	{
		if (Skills[i].bReady || bExperiment)
		{
			Shown.Add(i);
		}
	}
	if (Shown.Num() == 0)
	{
		DrawWrapped(TEXT("No trained behaviours yet. Train them in the web app, then press Reload."), X, Y, Inner, WarnCol, 0.88f);
		DrawButton({IdReload, TEXT("Reload"), {X, Y}, {90.f, 30.f}, true, ChipCol});
		Y += 36.f;
		return;
	}
	{
		const float Gap = 6.f, W = (Inner - (Shown.Num() - 1) * Gap) / Shown.Num();
		if (!Skills[LabSkill].bReady && !bExperiment)
		{
			LabSkill = Shown[0];
		}
		for (int32 k = 0; k < Shown.Num(); ++k)
		{
			const int32 i = Shown[k];
			const FR1SkillDef& S = Skills[i];
			const bool bSel = i == LabSkill;
			DrawButton({Id(SkillPrefix, i), S.Label, {X + k * (W + Gap), Y}, {W, 30.f}, S.bReady && (!B->IsRunning() || bSel),
				bSel ? ChipOnCol : ChipCol});
			if (!S.bReady)
			{
				DisabledWhy.Add(Id(SkillPrefix, i), FString::Printf(TEXT("needs training in the web app (%s)"), *S.Reason));
			}
			else if (B->IsRunning() && !bSel)
			{
				DisabledWhy.Add(Id(SkillPrefix, i), TEXT("a skill is running (Stop first)"));
			}
		}
		Y += 36.f;
	}
	const FR1SkillDef& Sel = Skills[LabSkill];
	const TArray<float> Sizes = SizesFor(Sel);
	if (Sizes.Num() > 0 && !Sizes.ContainsByPredicate([this](float A) { return FMath::IsNearlyEqual(A, LabSizeDeg); }))
	{
		LabSizeDeg = Sizes[0];
	}
	{
		float CX = X;
		if (Sizes.Num() > 0)
		{
			DrawText(TEXT("Size"), DimCol, CX, Y + 6.f, Font, 0.85f);
			CX += 44.f;
			for (float S : Sizes)
			{
				DrawButton({FName(*FString::Printf(TEXT("lsize_%.0f"), S)), FString::Printf(TEXT("%.0f deg"), S), {CX, Y}, {62.f, 30.f}, !B->IsRunning(),
					FMath::IsNearlyEqual(LabSizeDeg, S) ? ChipOnCol : ChipCol});
				CX += 66.f;
			}
			CX += 12.f;
		}
		if (bExperiment)
		{
			DrawText(TEXT("Run"), DimCol, CX, Y + 6.f, Font, 0.85f);
			CX += 36.f;
			for (float D : Durations)
			{
				DrawButton({FName(*FString::Printf(TEXT("ldur_%.0f"), D)), FString::Printf(TEXT("%.0f s"), D), {CX, Y}, {52.f, 30.f}, !B->IsRunning(),
					FMath::IsNearlyEqual(LabDurationS, D) ? ChipOnCol : ChipCol});
				CX += 56.f;
			}
			DrawButton({IdReload, TEXT("Reload"), {X + Inner - 76.f, Y}, {76.f, 30.f}, !B->IsRunning(), ChipCol});
		}
		if (Sizes.Num() > 0 || bExperiment)
		{
			Y += 36.f;
		}
	}
	{
		const bool bCan = Sel.bReady && bFree;
		const FString Label = B->IsRunning() ? FString::Printf(TEXT("Stop %s"), *R.SkillLabel) : FString::Printf(TEXT("Play %s"), *Sel.Label);
		DrawButton({IdPlay, Label, {X, Y}, {Inner, 42.f}, B->IsRunning() || bCan, B->IsRunning() ? StopCol * 0.85f : VirtualCol, true});
		if (!bFree)
		{
			DisabledWhy.Add(IdPlay, TEXT("the web app is driving the virtual robot right now (preview or live mirror)"));
		}
		Y += 48.f;
	}

	// ---------------------------------------------------------------- result (one line in Watch)
	if (R.bExternal)
	{
		DrawWrapped(TEXT("The web app is driving the virtual robot right now (virtual robot preview, live mirror or walk); the lab waits."), X, Y, Inner, WarnCol, 0.85f);
	}
	if (R.QDeg.Num() == 0)
	{
		DrawLine(TEXT("Press Play and watch the robot (and its head camera, right)."), X, Y, DimCol, 0.85f);
	}
	else
	{
		const FString State = R.bRunning ? (R.bSettling ? TEXT("finishing") : TEXT("playing")) : TEXT("done");
		DrawLine(FString::Printf(TEXT("%s %s  %.0f / %.0f s   accuracy: %s (%.1f deg)"), *R.SkillLabel, *State, R.T, R.DurationS,
			Grade(R.ErrMeanDeg), R.ErrMeanDeg), X, Y, R.bRunning ? TextCol : DimCol, 0.88f);
		if (R.RobotWouldStop.IsEmpty())
		{
			DrawLine(TEXT("On the real robot this run would be allowed to continue."), X, Y, OkCol, 0.85f);
		}
		else
		{
			DrawWrapped(FString::Printf(TEXT("On the real robot: %s"), *R.RobotWouldStop), X, Y, Inner, BadCol, 0.85f);
		}
	}
	Y += 4.f;
	DrawButton({IdMode, bExperiment ? TEXT("Fewer tools") : TEXT("More tools: target ball, pushes, timeline, walk"), {X, Y}, {Inner, 32.f}, true,
		bExperiment ? ChipOnCol : ChipCol});
	Y += 38.f;
	if (!bExperiment)
	{
		return;
	}

	// ---------------------------------------------------------------- 2. look-at target
	DrawSection(TEXT("TARGET  (the ball Look At follows)"), X, Inner, Y, LabCol);
	{
		struct FMode { const TCHAR* Label; ER1TargetMode M; const TCHAR* IdS; };
		const FMode Modes[] = {{TEXT("Script"), ER1TargetMode::Script, TEXT("tm_script")}, {TEXT("Manual"), ER1TargetMode::Manual, TEXT("tm_manual")},
			{TEXT("Orbit"), ER1TargetMode::Orbit, TEXT("tm_orbit")}};
		float CX = X;
		for (const FMode& M : Modes)
		{
			DrawButton({FName(M.IdS), M.Label, {CX, Y}, {86.f, 30.f}, true, B->TargetMode == M.M ? ChipOnCol : ChipCol});
			CX += 90.f;
		}
		CX += 10.f;
		const TCHAR* Nudges[] = {TEXT("Left"), TEXT("Right"), TEXT("Up"), TEXT("Down")};
		for (int32 i = 0; i < 4; ++i)
		{
			DrawButton({Id(TEXT("tn_"), i), Nudges[i], {CX, Y}, {58.f, 30.f}, true, ChipCol});
			CX += 62.f;
		}
		Y += 36.f;
		DrawLine(FString::Printf(TEXT("ball: yaw %+.0f deg (left +), pitch %+.0f deg (down +)%s"), B->TargetYawDeg, B->TargetPitchDeg,
			B->TargetMode == ER1TargetMode::Manual ? TEXT("   arrow keys move it") : TEXT("")), X, Y, DimCol, 0.8f);
		Y += 4.f;
	}

	// ---------------------------------------------------------------- 3. disturbances: presets + push
	DrawSection(TEXT("DISTURB  (does the brain cope?)"), X, Inner, Y, DisturbCol);
	{
		const float Gap = 6.f, PushW = 92.f;
		const float W = (Inner - PushW - UE_ARRAY_COUNT(Presets) * Gap) / UE_ARRAY_COUNT(Presets);
		for (int32 i = 0; i < UE_ARRAY_COUNT(Presets); ++i)
		{
			const FPreset& P = Presets[i];
			const bool bOn = B->DelaySteps == P.DelaySteps && FMath::IsNearlyEqual(B->NoiseDeg, P.NoiseDeg) && FMath::IsNearlyEqual(B->StrengthPct, P.StrengthPct);
			DrawButton({Id(TEXT("preset_"), i), P.Label, {X + i * (W + Gap), Y}, {W, 30.f}, true, bOn ? ChipOnCol : ChipCol});
		}
		DrawButton({IdPush, TEXT("Push head"), {X + Inner - PushW, Y}, {PushW, 30.f}, R.bConnected, DisturbCol * 0.7f});
		if (!R.bConnected)
		{
			DisabledWhy.Add(IdPush, TEXT("needs the URLab articulation (a push is a real MuJoCo force)"));
		}
		Y += 36.f;
		DrawLine(FString::Printf(TEXT("delay %d ms   sensor noise %g deg   motors %.0f%%"), B->DelaySteps * 20, B->NoiseDeg, B->StrengthPct), X, Y, DimCol, 0.8f);
		Y += 2.f;
	}

	// ---------------------------------------------------------------- 4. timeline
	DrawSection(TEXT("TIMELINE  (chain skills into a scene)"), X, Inner, Y, LabCol);
	{
		if (B->Timeline.Num() == 0)
		{
			DrawLine(TEXT("Empty: pick a skill and size above, then + Add."), X, Y, DimCol, 0.8f);
		}
		for (int32 i = 0; i < B->Timeline.Num(); ++i)
		{
			const FR1TimelineStep& S = B->Timeline[i];
			const bool bNow = R.bRunning && R.TimelineStep == i;
			const FString Name = Skills.IsValidIndex(S.Skill) ? Skills[S.Skill].Label : FString(TEXT("?"));
			DrawText(FString::Printf(TEXT("%s%d.  %s   %.0f deg   %.0f s"), bNow ? TEXT("> ") : TEXT("   "), i + 1, *Name, S.SizeDeg, S.DurationS),
				bNow ? OkCol : TextCol, X, Y + 2.f, Font, 0.82f);
			DrawButton({Id(TlDelPrefix, i), TEXT("x"), {X + Inner - 28.f, Y}, {28.f, 22.f}, !B->IsRunning(), ChipCol});
			Y += 25.f;
		}
		Y += 2.f;
		const bool bCanAdd = !B->IsRunning() && Sel.bReady && B->Timeline.Num() < 8;
		DrawButton({IdTlAdd, TEXT("+ Add"), {X, Y}, {90.f, 30.f}, bCanAdd, ChipCol});
		DrawButton({IdTlClear, TEXT("Clear"), {X + 96.f, Y}, {70.f, 30.f}, !B->IsRunning() && B->Timeline.Num() > 0, ChipCol});
		DrawButton({IdTlLoop, B->bLoopTimeline ? TEXT("Loop: on") : TEXT("Loop: off"), {X + 172.f, Y}, {96.f, 30.f}, true,
			B->bLoopTimeline ? ChipOnCol : ChipCol});
		DrawButton({IdTlPlay, TEXT("Play timeline"), {X + 274.f, Y}, {Inner - 274.f, 30.f}, !B->IsRunning() && B->Timeline.Num() > 0 && bFree, VirtualCol});
		Y += 36.f;
	}

	// ---------------------------------------------------------------- 5. full result
	if (R.QDeg.Num() > 0)
	{
		DrawSection(TEXT("DETAILS"), X, Inner, Y, OkCol);
		for (int32 i = 0; i < R.QDeg.Num(); ++i)
		{
			DrawLine(FString::Printf(TEXT("%-11s  is %+6.1f   should be %+6.1f   brain cmd %+6.1f deg"),
				R.JointLabels.IsValidIndex(i) ? *R.JointLabels[i] : TEXT("joint"), R.QDeg[i], R.RefDeg[i], R.TargetDeg[i]), X, Y, TextCol, 0.82f);
		}
		const FString Valid = R.ValidatedErrDeg >= 0.f ? FString::Printf(TEXT(" (validated %.1f)"), R.ValidatedErrDeg) : FString();
		const bool bWorse = R.ValidatedErrDeg >= 0.f && R.ErrMeanDeg > 2.f * R.ValidatedErrDeg + 0.5f;
		DrawLine(FString::Printf(TEXT("error now %.1f   mean %.1f%s   max %.1f deg"), R.ErrNowDeg, R.ErrMeanDeg, *Valid, R.ErrMaxDeg), X, Y,
			bWorse ? WarnCol : OkCol, 0.85f);
		DrawLine(FString::Printf(TEXT("other joints moved %.1f deg (robot limit 5)   brain %.2f ms / step%s"), R.OtherMaxDeg, R.InferMs,
			R.bLookClamped ? TEXT("   ball outside the trained reach: clamped") : TEXT("")), X, Y, DimCol, 0.8f);
	}
	if (!R.bConnected)
	{
		DrawWrapped(TEXT("No URLab articulation found: the brain runs on mock joints (no physics, no camera)."), X, Y, Inner, WarnCol, 0.85f);
	}
}

bool AR1BehaviourPanel::HandleLabClick(FName ClickId)
{
	UR1SkillBrainComponent* B = FindBrain();
	if (!B)
	{
		return false;
	}
	const FString S = ClickId.ToString();
	FString Why;
	if (S.StartsWith(SkillPrefix))
	{
		LabSkill = FCString::Atoi(*S.Mid(FCString::Strlen(SkillPrefix)));
		RememberLab();
		return true;
	}
	if (S.StartsWith(TEXT("lsize_")))
	{
		LabSizeDeg = FCString::Atof(*S.Mid(6));
		RememberLab();
		return true;
	}
	if (S.StartsWith(TEXT("ldur_")))
	{
		LabDurationS = FCString::Atof(*S.Mid(5));
		RememberLab();
		return true;
	}
	if (ClickId == IdMode)
	{
		bExperiment = !bExperiment;
		RememberLab();
		return true;
	}
	if (ClickId == IdReload)
	{
		B->ReloadSkills();
		LabSkill = -1;
		ShowToast(FString::Printf(TEXT("Reloaded %d trained skill(s) from the web app."), B->GetSkills().Num()), TextCol);
		return true;
	}
	if (ClickId == IdPlay)
	{
		if (B->IsRunning())
		{
			B->Stop();
			ShowToast(TEXT("Stopped."), TextCol);
		}
		else if (B->Play(LabSkill, LabSizeDeg, LabDurationS, Why))
		{
			ShowToast(TEXT("Playing on the virtual robot. Nothing physical moves."), CyanCol);
		}
		else
		{
			ShowToast(FString::Printf(TEXT("Cannot play: %s"), *Why), BadCol);
		}
		return true;
	}
	if (S.StartsWith(TEXT("tm_")))
	{
		B->SetTargetMode(S == TEXT("tm_manual") ? ER1TargetMode::Manual : S == TEXT("tm_orbit") ? ER1TargetMode::Orbit : ER1TargetMode::Script);
		return true;
	}
	if (S.StartsWith(TEXT("tn_")))
	{
		const int32 I = FCString::Atoi(*S.Mid(3));
		B->NudgeTarget(I == 0 ? 5.f : I == 1 ? -5.f : 0.f, I == 2 ? -5.f : I == 3 ? 5.f : 0.f);
		return true;
	}
	if (S.StartsWith(TEXT("preset_")))
	{
		const FPreset& P = Presets[FMath::Clamp(FCString::Atoi(*S.Mid(7)), 0, static_cast<int32>(UE_ARRAY_COUNT(Presets)) - 1)];
		B->DelaySteps = P.DelaySteps;
		B->NoiseDeg = P.NoiseDeg;
		B->StrengthPct = P.StrengthPct;
		ShowToast(FString::Printf(TEXT("Conditions: %s."), P.Label), DisturbCol);
		return true;
	}
	if (ClickId == IdPush)
	{
		const FVector2D Dirs[] = {{1.f, 0.f}, {-1.f, 0.f}, {0.f, -1.f}, {0.f, 1.f}};
		const TCHAR* Names[] = {TEXT("left"), TEXT("right"), TEXT("up"), TEXT("down")};
		const int32 I = PushCount++ % 4;
		B->Push(Dirs[I]);
		ShowToast(FString::Printf(TEXT("Shoved the head %s."), Names[I]), DisturbCol);
		return true;
	}
	if (ClickId == IdTlAdd)
	{
		B->Timeline.Add({LabSkill, LabSizeDeg, LabDurationS});
		return true;
	}
	if (ClickId == IdTlClear)
	{
		B->Timeline.Reset();
		return true;
	}
	if (ClickId == IdTlLoop)
	{
		B->bLoopTimeline = !B->bLoopTimeline;
		return true;
	}
	if (ClickId == IdTlPlay)
	{
		if (B->PlayTimeline(Why))
		{
			ShowToast(FString::Printf(TEXT("Playing the %d-step timeline on the virtual robot."), B->Timeline.Num()), CyanCol);
		}
		else
		{
			ShowToast(FString::Printf(TEXT("Cannot play: %s"), *Why), BadCol);
		}
		return true;
	}
	if (S.StartsWith(TlDelPrefix))
	{
		const int32 I = FCString::Atoi(*S.Mid(FCString::Strlen(TlDelPrefix)));
		if (B->Timeline.IsValidIndex(I))
		{
			B->Timeline.RemoveAt(I);
		}
		return true;
	}
	return false;
}
