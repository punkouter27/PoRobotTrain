// Behaviour panel: robustness explorer tab (idea #19, docs/service_api.md section 2).
// GET /robustness -> one-factor-at-a-time heatmap (factor rows x levels) + the gain x latency grid.
// Every cell carries its verdict as a glyph and a word as well as the status colour, so it never relies on colour.
#include "R1BehaviourPanel.h"
#include "R1Twin.h"
#include "R1PanelStyle.h"

#include "Engine/Engine.h"
#include "Engine/Font.h"

using namespace R1PanelStyle;

namespace
{
	const FName IdRunFull(TEXT("rob_full")), IdRunQuick(TEXT("rob_quick"));

	FName CellId(int32 Kind, int32 Row, int32 Col)
	{
		return FName(*FString::Printf(TEXT("rcell_%d_%d_%d"), Kind, Row, Col));
	}

	FString VerdictWord(const FString& V)
	{
		return V == TEXT("pass") ? TEXT("PASS") : V == TEXT("warn") ? TEXT("WARN") : TEXT("FAIL");
	}

	/** The number that explains the verdict: acceleration when it broke the limit, else mean tracking error. */
	FString CellMetric(const FR1RobustCell& C, const FR1RobustReport& R)
	{
		if (!C.bFinite || !C.bValid)
		{
			return TEXT("non-finite");
		}
		if (C.MaxAcc > R.WarnAcc && (C.MaxAcc > R.FailAcc || C.MeanErrDeg <= R.WarnMeanErr))
		{
			return FString::Printf(TEXT("acc %.0f"), C.MaxAcc);
		}
		return FString::Printf(TEXT("err %.1f°"), C.MeanErrDeg);
	}
}

void AR1BehaviourPanel::TickRobustness(double Now)
{
	const bool bRunning = Job == TEXT("robustness");
	if (bRobustJobWasRunning && !bRunning)
	{
		NextRobustFetch = 0.0;   // the sweep just finished: pick up the new report
		ShowToast(TEXT("Robustness sweep finished."), OkCol);
	}
	bRobustJobWasRunning = bRunning;
	if ((Tab == ER1PanelTab::Robustness || !PanelCaptureDir.IsEmpty()) && Now >= NextRobustFetch && !bRobustFetchInFlight && bServiceOnline)
	{
		FetchRobustness();
		NextRobustFetch = Now + (bRunning ? 2.0 : 15.0);
	}
}

void AR1BehaviourPanel::FetchRobustness()
{
	bRobustFetchInFlight = true;
	GetJson(TEXT("/robustness"), [this](int32 Code, const FString& Body)
	{
		bRobustFetchInFlight = false;
		FR1RobustReport R;
		if (Code == 200 && R1Panel::ParseRobustness(Body, R))
		{
			Robust = MoveTemp(R);
		}
		else if (Code == 404)
		{
			Robust = FR1RobustReport();
			Robust.bValid = false;
		}
	});
}

void AR1BehaviourPanel::DrawRobustCell(FName Id, const FR1RobustCell& C, const FString& Top, float X, float Y, float W, float H,
	bool bNominal, bool bSelected, bool bThreeLines)
{
	FLinearColor Fill = R1Panel::VerdictColor(C.Verdict);
	float Mx = 0.f, My = 0.f;
	if (GetPanelMouse(Mx, My) && Mx >= X && Mx <= X + W && My >= Y && My <= Y + H)
	{
		Fill = FMath::Lerp(Fill, FLinearColor::White, 0.15f);
	}
	DrawRect(Fill, X, Y, W, H);
	if (bNominal)
	{
		const FLinearColor E = FLinearColor::White;
		DrawRect(E, X, Y, W, 2.f);
		DrawRect(E, X, Y + H - 2.f, W, 2.f);
		DrawRect(E, X, Y, 2.f, H);
		DrawRect(E, X + W - 2.f, Y, 2.f, H);
	}
	if (bSelected)
	{
		const float T = 3.f;
		DrawRect(CyanCol, X - T, Y - T, W + 2.f * T, T);
		DrawRect(CyanCol, X - T, Y + H, W + 2.f * T, T);
		DrawRect(CyanCol, X - T, Y, T, H);
		DrawRect(CyanCol, X + W, Y, T, H);
	}
	// label inside a status fill: white, except dark ink on the warning yellow
	const FLinearColor Ink = C.Verdict == TEXT("warn") ? DarkInk : FLinearColor::White;
	UFont* Font = GEngine->GetSmallFont();
	const float S = 0.95f;
	TArray<FString> Rows;
	if (bThreeLines && !Top.IsEmpty())
	{
		Rows.Add(Top);
	}
	Rows.Add(FString::Printf(TEXT("%s %s"), R1Panel::VerdictGlyph(C.Verdict), *VerdictWord(C.Verdict)));
	Rows.Add(CellMetric(C, Robust));
	float LineH = 0.f, Tw = 0.f;
	GetTextSize(TEXT("Ag"), Tw, LineH, Font, S);
	float Ty = Y + (H - Rows.Num() * LineH) * 0.5f;
	for (int32 i = 0; i < Rows.Num(); ++i)
	{
		float W2 = 0.f, H2 = 0.f;
		GetTextSize(Rows[i], W2, H2, Font, S);
		FLinearColor Col = Ink;
		if (i == 0 && Rows.Num() == 3)
		{
			Col.A = 0.8f;
		}
		DrawText(Rows[i], Col, X + (W - W2) * 0.5f, Ty, Font, S);
		Ty += LineH;
	}
	NextRects.Emplace(Id, FBox2D(FVector2D(X, Y), FVector2D(X + W, Y + H)));
}

void AR1BehaviourPanel::DrawRobustnessTab(float X, float Inner, float& Y)
{
	const bool bIdle = bServiceOnline && Job.IsEmpty() && !WalkState.bRunning;
	const bool bRunning = Job == TEXT("robustness");
	UFont* Font = GEngine->GetMediumFont();
	UFont* Small = GEngine->GetSmallFont();

	DrawLine(TEXT("ROBUSTNESS EXPLORER  (simulation only)"), X, Y, CyanCol, 1.05f);
	if (!Robust.bValid)
	{
		DrawWrapped(bServiceOnline ? TEXT("No report from the service yet (this service may predate /robustness).")
			: TEXT("Behaviour service offline."), X, Y, Inner, DimCol, 0.88f);
	}
	else if (!Robust.bAvailable)
	{
		DrawWrapped(TEXT("No sweep yet. A sweep replays the Level 0 policy on many perturbed copies of the stand twin "
			"(mass, joint friction, motor gains, latency, sensor noise, torque limit) and grades each one."), X, Y, Inner, DimCol, 0.88f);
	}
	else
	{
		DrawWrapped(FString::Printf(TEXT("Level 0 policy %s  |  %s sweep, %s  |  %.0f s per twin, done in %.1f s"),
			*Robust.PolicySha.Left(8), Robust.bQuick ? TEXT("quick") : TEXT("full"), *Robust.CreatedAt.Replace(TEXT("T"), TEXT(" ")),
			Robust.RolloutS, Robust.DurationS), X, Y, Inner, DimCol, 0.85f);
	}
	Y += 4.f;

	// run buttons + progress
	DrawButton({IdRunFull, bRunning ? TEXT("Sweep running...") : TEXT("Run full sweep"), {X, Y}, {170.f, 36.f}, bIdle, VirtualCol});
	DrawButton({IdRunQuick, TEXT("Run quick sweep"), {X + 178.f, Y}, {150.f, 36.f}, bIdle, ChipCol});
	if (bRunning)
	{
		const float BX = X + 340.f, BW = Inner - 340.f;
		const float Frac = ProgressTotal > 0 ? FMath::Clamp(static_cast<float>(ProgressDone) / ProgressTotal, 0.f, 1.f) : 0.f;
		DrawRect(FLinearColor(1.f, 1.f, 1.f, 0.12f), BX, Y + 22.f, BW, 8.f);
		DrawRect(CyanCol, BX, Y + 22.f, BW * Frac, 8.f);
		DrawText(ProgressTotal > 0 ? FString::Printf(TEXT("%d / %d twins"), ProgressDone, ProgressTotal) : FString(TEXT("starting...")),
			TextCol, BX, Y + 2.f, Font, 0.85f);
	}
	Y += 44.f;

	if (!Robust.bValid || !Robust.bAvailable)
	{
		return;
	}

	// summary: counts with glyph + word, then the nominal verdict in plain words
	{
		float SX = X;
		auto Part = [&](const TCHAR* Verdict, int32 N)
		{
			const FString S = FString::Printf(TEXT("%s %s %d"), R1Panel::VerdictGlyph(Verdict), *VerdictWord(Verdict).ToLower(), N);
			float W = 0.f, H = 0.f;
			GetTextSize(S, W, H, Font, 0.92f);
			DrawText(S, FMath::Lerp(R1Panel::VerdictColor(Verdict), FLinearColor::White, 0.25f), SX, Y, Font, 0.92f);
			SX += W + 22.f;
		};
		Part(TEXT("pass"), Robust.NumPass);
		Part(TEXT("warn"), Robust.NumWarn);
		Part(TEXT("fail"), Robust.NumFail);
		float W = 0.f, H = 0.f;
		GetTextSize(TEXT("Ag"), W, H, Font, 0.92f);
		Y += H + 4.f;
	}
	{
		const FR1RobustCell& N = Robust.Nominal;
		const FString R = N.Reason(Robust.FailMeanErr, Robust.FailAcc, Robust.WarnMeanErr, Robust.WarnAcc);
		DrawWrapped(FString::Printf(TEXT("Nominal twin: %s %s%s%s"), R1Panel::VerdictGlyph(N.Verdict), *VerdictWord(N.Verdict),
			R.IsEmpty() ? TEXT("") : TEXT(" - "), *R), X, Y, Inner, N.Verdict == TEXT("pass") ? OkCol : N.Verdict == TEXT("warn") ? WarnCol : BadCol, 0.88f);
	}
	// dominant failure cause, so "almost everything fails on head yaw acceleration" reads at a glance
	{
		TMap<FString, int32> AccFails;
		int32 FailCount = 0;
		auto Count = [&](const FR1RobustCell& C)
		{
			if (C.Verdict != TEXT("fail"))
			{
				return;
			}
			++FailCount;
			if (C.bValid && C.MaxAcc > Robust.FailAcc && !C.AccJoint.IsEmpty())
			{
				AccFails.FindOrAdd(C.AccJoint)++;
			}
		};
		for (const FR1RobustFactor& F : Robust.Sensitivity)
		{
			for (const FR1RobustCell& C : F.Cells)
			{
				Count(C);
			}
		}
		for (const TArray<FR1RobustCell>& Row : Robust.Interaction)
		{
			for (const FR1RobustCell& C : Row)
			{
				Count(C);
			}
		}
		FString TopJoint;
		int32 TopN = 0;
		for (const TPair<FString, int32>& P : AccFails)
		{
			if (P.Value > TopN)
			{
				TopN = P.Value;
				TopJoint = P.Key;
			}
		}
		if (FailCount > 0 && TopN * 2 >= FailCount)
		{
			DrawWrapped(FString::Printf(TEXT("Main cause: %d of %d failing twins exceed %.0f rad/s² acceleration on %s (the policy chatters on that joint)."),
				TopN, FailCount, Robust.FailAcc, *R1Panel::PrettyJoint(TopJoint)), X, Y, Inner, WarnCol, 0.85f);
		}
		else if (!Robust.Worst.IsEmpty())
		{
			DrawWrapped(FString::Printf(TEXT("Worst: %s"), *Robust.Worst), X, Y, Inner, DimCol, 0.85f);
		}
	}
	Y += 6.f;

	const float LabelW = 128.f, Gap = 3.f;

	// one factor at a time
	DrawText(TEXT("ONE FACTOR AT A TIME  (everything else nominal; white outline = nominal setting)"), DimCol, X, Y, Small, 0.95f);
	Y += 16.f;
	int32 MaxLevels = 1;
	for (const FR1RobustFactor& F : Robust.Sensitivity)
	{
		MaxLevels = FMath::Max(MaxLevels, F.Axis.Levels.Num());
	}
	const float CellW = (Inner - LabelW - (MaxLevels - 1) * Gap) / MaxLevels;
	const float RowH = 44.f;
	for (int32 r = 0; r < Robust.Sensitivity.Num(); ++r)
	{
		const FR1RobustFactor& F = Robust.Sensitivity[r];
		float Ly = Y + 8.f;
		DrawText(F.Axis.Label, TextCol, X, Ly, Font, 0.82f);
		const FString UnitLabel = F.Axis.Unit == TEXT("×") ? FString(TEXT("scale"))
			: F.Axis.Unit.StartsWith(TEXT("×")) ? FString(TEXT("x")) + F.Axis.Unit.Mid(1) : F.Axis.Unit;
		DrawText(UnitLabel, DimCol, X, Ly + 16.f, Small, 0.9f);
		const int32 Nom = F.Axis.NominalIndex();
		for (int32 c = 0; c < F.Cells.Num(); ++c)
		{
			DrawRobustCell(CellId(0, r, c), F.Cells[c], F.Axis.LevelText(c), X + LabelW + c * (CellW + Gap), Y, CellW, RowH,
				c == Nom, SelKind == 0 && SelRow == r && SelCol == c, true);
		}
		Y += RowH + Gap;
	}
	Y += 8.f;

	// gain x latency interaction grid
	if (Robust.Interaction.Num() > 0 && Robust.InterX.Levels.Num() > 0)
	{
		DrawText(FString::Printf(TEXT("%s (columns)  x  %s (rows)"), *Robust.InterX.Label, *Robust.InterY.Label), DimCol, X, Y, Small, 0.95f);
		Y += 16.f;
		const int32 NX = Robust.InterX.Levels.Num();
		const float IW = (Inner - LabelW - (NX - 1) * Gap) / NX;
		for (int32 c = 0; c < NX; ++c)
		{
			const FString L = Robust.InterX.LevelText(c);
			float W = 0.f, H = 0.f;
			GetTextSize(L, W, H, Small, 0.95f);
			DrawText(L, DimCol, X + LabelW + c * (IW + Gap) + (IW - W) * 0.5f, Y, Small, 0.95f);
		}
		Y += 15.f;
		const int32 NomX = Robust.InterX.NominalIndex(), NomY = Robust.InterY.NominalIndex();
		const float IH = 32.f;
		for (int32 r = 0; r < Robust.Interaction.Num(); ++r)
		{
			DrawText(Robust.InterY.LevelText(r), DimCol, X + 8.f, Y + 9.f, Font, 0.82f);
			for (int32 c = 0; c < Robust.Interaction[r].Num(); ++c)
			{
				DrawRobustCell(CellId(1, r, c), Robust.Interaction[r][c], FString(), X + LabelW + c * (IW + Gap), Y, IW, IH,
					c == NomX && r == NomY, SelKind == 1 && SelRow == r && SelCol == c, false);
			}
			Y += IH + Gap;
		}
		Y += 6.f;
	}

	// selected cell details
	const FR1RobustCell* Sel = nullptr;
	FString Where;
	if (SelKind == 0 && Robust.Sensitivity.IsValidIndex(SelRow) && Robust.Sensitivity[SelRow].Cells.IsValidIndex(SelCol))
	{
		const FR1RobustFactor& F = Robust.Sensitivity[SelRow];
		Sel = &F.Cells[SelCol];
		Where = FString::Printf(TEXT("%s %s"), *F.Axis.Label, *F.Axis.LevelText(SelCol));
	}
	else if (SelKind == 1 && Robust.Interaction.IsValidIndex(SelRow) && Robust.Interaction[SelRow].IsValidIndex(SelCol))
	{
		Sel = &Robust.Interaction[SelRow][SelCol];
		Where = FString::Printf(TEXT("%s %s, %s %s"), *Robust.InterX.Label, *Robust.InterX.LevelText(SelCol),
			*Robust.InterY.Label, *Robust.InterY.LevelText(SelRow));
	}
	if (Sel)
	{
		const FString R = Sel->Reason(Robust.FailMeanErr, Robust.FailAcc, Robust.WarnMeanErr, Robust.WarnAcc);
		DrawWrapped(FString::Printf(TEXT("%s: %s %s%s%s"), *Where, R1Panel::VerdictGlyph(Sel->Verdict), *VerdictWord(Sel->Verdict),
			R.IsEmpty() ? TEXT("") : TEXT(" - "), *R), X, Y, Inner, TextCol, 0.9f);
		if (Sel->bValid)
		{
			DrawWrapped(FString::Printf(TEXT("mean error %.2f°, max error %.2f°, max acceleration %.0f rad/s² (%s), jitter %.3f /step, trips %d"),
				Sel->MeanErrDeg, Sel->MaxErrDeg, Sel->MaxAcc, Sel->AccJoint.IsEmpty() ? TEXT("-") : *R1Panel::PrettyJoint(Sel->AccJoint),
				Sel->Jitter, Sel->Trips), X, Y, Inner, DimCol, 0.85f);
		}
	}
	else
	{
		DrawLine(FString::Printf(TEXT("Click a cell for its numbers.  Gate: mean error <= %.0f°, acceleration <= %.0f rad/s², no trips."),
			Robust.FailMeanErr, Robust.FailAcc), X, Y, DimCol, 0.85f);
	}
}

bool AR1BehaviourPanel::HandleRobustClick(FName Id)
{
	if (Id == IdRunFull || Id == IdRunQuick)
	{
		const bool bQuick = Id == IdRunQuick;
		PostJson(TEXT("/robustness/run"), bQuick ? TEXT("{\"quick\":true}") : TEXT("{\"quick\":false}"),
			[this, bQuick](int32 Code, TSharedPtr<FJsonObject> J)
			{
				FString Err;
				if (J.IsValid())
				{
					J->TryGetStringField(TEXT("error"), Err);
				}
				if (Code == 200)
				{
					ShowToast(bQuick ? TEXT("Quick robustness sweep started.") : TEXT("Full robustness sweep started."), CyanCol);
				}
				else
				{
					ShowToast(FString::Printf(TEXT("Sweep refused: %s"), Code == 0 ? TEXT("service not reachable") : Err.IsEmpty() ? *FString::Printf(TEXT("HTTP %d"), Code) : *Err), BadCol);
				}
				NextPoll = 0.0;
			});
		return true;
	}
	const FString S = Id.ToString();
	if (S.StartsWith(TEXT("rcell_")))
	{
		TArray<FString> Parts;
		S.ParseIntoArray(Parts, TEXT("_"));
		if (Parts.Num() == 4)
		{
			const int32 K = FCString::Atoi(*Parts[1]), R = FCString::Atoi(*Parts[2]), C = FCString::Atoi(*Parts[3]);
			const bool bSame = SelKind == K && SelRow == R && SelCol == C;
			SelKind = bSame ? -1 : K;
			SelRow = bSame ? -1 : R;
			SelCol = bSame ? -1 : C;
		}
		return true;
	}
	return false;
}
