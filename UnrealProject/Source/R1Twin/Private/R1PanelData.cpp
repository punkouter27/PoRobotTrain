#include "R1PanelData.h"

#include "Dom/JsonObject.h"
#include "Dom/JsonValue.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"

namespace R1PanelJsonDetail
{
	TSharedPtr<FJsonObject> ParseObject(const FString& Json)
	{
		TSharedPtr<FJsonObject> J;
		if (!FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Json), J) || !J.IsValid())
		{
			return nullptr;
		}
		return J;
	}

	float Num(const TSharedPtr<FJsonObject>& J, const TCHAR* Field, float Default)
	{
		double V = Default;
		return J.IsValid() && J->TryGetNumberField(Field, V) ? static_cast<float>(V) : Default;
	}

	bool OptNum(const TSharedPtr<FJsonObject>& J, const TCHAR* Field, float& Out)
	{
		double V = 0.0;
		if (J.IsValid() && J->HasTypedField<EJson::Number>(Field) && J->TryGetNumberField(Field, V))
		{
			Out = static_cast<float>(V);
			return true;
		}
		return false;
	}

	FString Str(const TSharedPtr<FJsonObject>& J, const TCHAR* Field)
	{
		FString S;
		if (J.IsValid() && J->HasTypedField<EJson::String>(Field))
		{
			J->TryGetStringField(Field, S);
		}
		return S;
	}

	bool Bool(const TSharedPtr<FJsonObject>& J, const TCHAR* Field, bool Default = false)
	{
		bool B = Default;
		return J.IsValid() && J->TryGetBoolField(Field, B) ? B : Default;
	}

	TSharedPtr<FJsonObject> Obj(const TSharedPtr<FJsonObject>& J, const TCHAR* Field)
	{
		const TSharedPtr<FJsonObject>* P = nullptr;
		return J.IsValid() && J->TryGetObjectField(Field, P) && P ? *P : nullptr;
	}

	const TArray<TSharedPtr<FJsonValue>>* Arr(const TSharedPtr<FJsonObject>& J, const TCHAR* Field)
	{
		const TArray<TSharedPtr<FJsonValue>>* A = nullptr;
		return J.IsValid() && J->TryGetArrayField(Field, A) ? A : nullptr;
	}

	bool ReadVec(const TArray<TSharedPtr<FJsonValue>>* A, FVector& Out)
	{
		if (!A || A->Num() < 3)
		{
			return false;
		}
		Out = FVector((*A)[0]->AsNumber(), (*A)[1]->AsNumber(), (*A)[2]->AsNumber());
		return true;
	}

	void ReadRange(const TSharedPtr<FJsonObject>& Lim, const TCHAR* Field, float& Lo, float& Hi)
	{
		const TArray<TSharedPtr<FJsonValue>>* A = Arr(Lim, Field);
		if (A && A->Num() >= 2)
		{
			Lo = static_cast<float>((*A)[0]->AsNumber());
			Hi = static_cast<float>((*A)[1]->AsNumber());
		}
	}

	FR1RobustCell ReadCell(const TSharedPtr<FJsonValue>& V)
	{
		FR1RobustCell C;
		const TSharedPtr<FJsonObject>* P = nullptr;
		if (!V.IsValid() || !V->TryGetObject(P) || !P || !P->IsValid())
		{
			C.Verdict = TEXT("fail");
			return C;
		}
		const TSharedPtr<FJsonObject>& J = *P;
		const bool bHaveMean = OptNum(J, TEXT("mean_err_deg"), C.MeanErrDeg);
		OptNum(J, TEXT("max_err_deg"), C.MaxErrDeg);
		OptNum(J, TEXT("max_acc"), C.MaxAcc);
		C.Trips = static_cast<int32>(Num(J, TEXT("trips"), 0.f));
		OptNum(J, TEXT("jitter"), C.Jitter);
		C.AccJoint = Str(J, TEXT("acc_joint"));
		C.bFinite = Bool(J, TEXT("finite"), true);
		C.Verdict = Str(J, TEXT("verdict")).ToLower();
		if (C.Verdict != TEXT("pass") && C.Verdict != TEXT("warn"))
		{
			C.Verdict = TEXT("fail");   // unknown or non-finite results are treated as failures
		}
		C.bValid = bHaveMean && C.bFinite;
		return C;
	}

	FR1RobustAxis ReadAxis(const TSharedPtr<FJsonObject>& J)
	{
		FR1RobustAxis A;
		A.Id = Str(J, TEXT("id"));
		A.Label = Str(J, TEXT("label"));
		A.Unit = Str(J, TEXT("unit"));
		A.bHasNominal = OptNum(J, TEXT("nominal"), A.Nominal);
		if (const TArray<TSharedPtr<FJsonValue>>* L = Arr(J, TEXT("levels")))
		{
			for (const TSharedPtr<FJsonValue>& V : *L)
			{
				A.Levels.Add(static_cast<float>(V->AsNumber()));
			}
		}
		if (!A.bHasNominal)
		{
			// Interaction axes carry no "nominal": gains 1.0x, latency 0 ms are the nominal settings.
			const float Guess = A.Unit == TEXT("ms") ? 0.f : 1.f;
			if (A.Levels.Contains(Guess))
			{
				A.bHasNominal = true;
				A.Nominal = Guess;
			}
		}
		return A;
	}
}
using namespace R1PanelJsonDetail;

FString FR1RobustCell::Reason(float FailMean, float FailAcc, float WarnMean, float WarnAcc) const
{
	if (Verdict == TEXT("pass"))
	{
		return FString();
	}
	if (!bFinite || !bValid)
	{
		return TEXT("simulation blew up (non-finite state)");
	}
	TArray<FString> Parts;
	const FString Joint = AccJoint.IsEmpty() ? FString(TEXT("a joint")) : R1Panel::PrettyJoint(AccJoint);
	if (MaxAcc > FailAcc)
	{
		Parts.Add(FString::Printf(TEXT("acceleration %.0f rad/s² on %s (limit %.0f)"), MaxAcc, *Joint, FailAcc));
	}
	if (MeanErrDeg > FailMean)
	{
		Parts.Add(FString::Printf(TEXT("mean tracking error %.1f deg (limit %.0f)"), MeanErrDeg, FailMean));
	}
	if (Trips > 0)
	{
		Parts.Add(FString::Printf(TEXT("%d safety trip(s)"), Trips));
	}
	if (Parts.Num() == 0)
	{
		if (MaxAcc > WarnAcc)
		{
			Parts.Add(FString::Printf(TEXT("acceleration %.0f rad/s² on %s (warn above %.0f)"), MaxAcc, *Joint, WarnAcc));
		}
		if (MeanErrDeg > WarnMean)
		{
			Parts.Add(FString::Printf(TEXT("mean tracking error %.1f deg (warn above %.0f)"), MeanErrDeg, WarnMean));
		}
	}
	return Parts.Num() ? FString::Join(Parts, TEXT(", ")) : FString(TEXT("outside the gate"));
}

FString R1Panel::PrettyJoint(const FString& Joint)
{
	FString S = Joint;
	S.RemoveFromEnd(TEXT("_joint"));
	return S.Replace(TEXT("_"), TEXT(" "));
}

FString FR1RobustAxis::LevelText(int32 Index) const
{
	if (!Levels.IsValidIndex(Index))
	{
		return FString();
	}
	const float V = Levels[Index];
	FString N = FMath::IsNearlyEqual(V, FMath::RoundToFloat(V)) ? FString::Printf(TEXT("%.0f"), V)
		: FMath::IsNearlyEqual(V * 10.f, FMath::RoundToFloat(V * 10.f)) ? FString::Printf(TEXT("%.1f"), V)
		: FString::Printf(TEXT("%.2f"), V);
	if (Unit.StartsWith(TEXT("×")) || Unit == TEXT("x"))
	{
		return N + TEXT("x");   // "×" and "× training" both read as a scale factor; the row label carries the unit
	}
	return Unit.IsEmpty() ? N : N + TEXT(" ") + Unit;
}

int32 FR1RobustAxis::NominalIndex() const
{
	if (!bHasNominal)
	{
		return INDEX_NONE;
	}
	for (int32 i = 0; i < Levels.Num(); ++i)
	{
		if (FMath::IsNearlyEqual(Levels[i], Nominal, 1e-4f))
		{
			return i;
		}
	}
	return INDEX_NONE;
}

uint32 FR1WalkState::GeometryHash() const
{
	uint32 H = GetTypeHash(Course);
	H = HashCombine(H, GetTypeHash(CheckpointsMj.Num()));
	for (const FVector& C : CheckpointsMj)
	{
		H = HashCombine(H, GetTypeHash(C));
	}
	for (const FR1WalkBox& B : Boxes)
	{
		H = HashCombine(H, GetTypeHash(B.Name));
		H = HashCombine(H, GetTypeHash(B.PosMj));
		H = HashCombine(H, GetTypeHash(B.HalfSizeMj));
	}
	return H;
}

FVector FR1WalkState::ClampCmd(const FVector& In) const
{
	return FVector(FMath::Clamp<double>(In.X, VxMin, VxMax), FMath::Clamp<double>(In.Y, VyMin, VyMax), FMath::Clamp<double>(In.Z, WzMin, WzMax));
}

bool R1Panel::ParseIntent(const FString& Json, FR1Intent& Out)
{
	Out = FR1Intent();
	const TSharedPtr<FJsonObject> J = ParseObject(Json);
	if (!J.IsValid())
	{
		return false;
	}
	Out.Kind = Str(J, TEXT("kind"));
	Out.Id = Str(J, TEXT("id"));
	Out.Label = Str(J, TEXT("label"));
	Out.Joint = Str(J, TEXT("joint"));
	Out.bHasAmplitude = OptNum(J, TEXT("amplitude"), Out.Amplitude);
	Out.bHasDeg = OptNum(J, TEXT("deg"), Out.Deg);
	Out.bBlocked = Bool(J, TEXT("blocked"));
	Out.Score = Num(J, TEXT("score"), 0.f);
	Out.Note = Str(J, TEXT("note"));
	Out.Say = Str(J, TEXT("say"));
	Out.Answer = Str(J, TEXT("answer"));
	Out.Pose = Str(J, TEXT("pose"));
	if (const TSharedPtr<FJsonObject> W = Obj(J, TEXT("walk")))
	{
		Out.bHasWalk = true;
		Out.bWalkStop = Bool(W, TEXT("stop"));
		Out.WalkCmd = FVector(Num(W, TEXT("vx"), 0.f), Num(W, TEXT("vy"), 0.f), Num(W, TEXT("wz"), 0.f));
	}
	if (Out.Kind.IsEmpty())
	{
		Out.Kind = TEXT("unknown");
	}
	Out.bValid = true;
	return true;
}

bool R1Panel::ParseRobustness(const FString& Json, FR1RobustReport& Out)
{
	Out = FR1RobustReport();
	const TSharedPtr<FJsonObject> J = ParseObject(Json);
	if (!J.IsValid())
	{
		return false;
	}
	Out.bValid = true;
	Out.bAvailable = Bool(J, TEXT("available"), J->HasField(TEXT("sensitivity")));
	if (!Out.bAvailable)
	{
		return true;
	}
	Out.CreatedAt = Str(J, TEXT("created_at"));
	Out.Policy = Str(J, TEXT("policy"));
	Out.PolicySha = Str(J, TEXT("policy_sha256"));
	Out.bQuick = Bool(J, TEXT("quick"));
	Out.DurationS = Num(J, TEXT("duration_s"), 0.f);
	Out.RolloutS = Num(J, TEXT("rollout_s"), 0.f);
	if (const TSharedPtr<FJsonObject> T = Obj(J, TEXT("thresholds")))
	{
		Out.FailMeanErr = Num(T, TEXT("mean_err_deg"), Out.FailMeanErr);
		Out.FailAcc = Num(T, TEXT("max_acc"), Out.FailAcc);
		Out.WarnMeanErr = Num(T, TEXT("warn_mean_err_deg"), Out.WarnMeanErr);
		Out.WarnAcc = Num(T, TEXT("warn_acc"), Out.WarnAcc);
	}
	if (J->HasField(TEXT("nominal")))
	{
		Out.Nominal = ReadCell(J->TryGetField(TEXT("nominal")));
	}
	if (const TArray<TSharedPtr<FJsonValue>>* S = Arr(J, TEXT("sensitivity")))
	{
		for (const TSharedPtr<FJsonValue>& V : *S)
		{
			const TSharedPtr<FJsonObject>* P = nullptr;
			if (!V->TryGetObject(P) || !P || !P->IsValid())
			{
				continue;
			}
			FR1RobustFactor F;
			F.Axis = ReadAxis(*P);
			if (const TArray<TSharedPtr<FJsonValue>>* Cells = Arr(*P, TEXT("cells")))
			{
				for (const TSharedPtr<FJsonValue>& C : *Cells)
				{
					F.Cells.Add(ReadCell(C));
				}
			}
			F.Cells.SetNum(F.Axis.Levels.Num());
			Out.Sensitivity.Add(MoveTemp(F));
		}
	}
	if (const TSharedPtr<FJsonObject> I = Obj(J, TEXT("interaction")))
	{
		Out.InterX = ReadAxis(Obj(I, TEXT("x")));
		Out.InterY = ReadAxis(Obj(I, TEXT("y")));
		if (const TArray<TSharedPtr<FJsonValue>>* Rows = Arr(I, TEXT("cells")))
		{
			for (const TSharedPtr<FJsonValue>& RowV : *Rows)
			{
				TArray<FR1RobustCell> Row;
				const TArray<TSharedPtr<FJsonValue>>* RowA = nullptr;
				if (RowV->TryGetArray(RowA) && RowA)
				{
					for (const TSharedPtr<FJsonValue>& C : *RowA)
					{
						Row.Add(ReadCell(C));
					}
				}
				Row.SetNum(Out.InterX.Levels.Num());
				Out.Interaction.Add(MoveTemp(Row));
			}
		}
		Out.Interaction.SetNum(Out.InterY.Levels.Num());
		for (TArray<FR1RobustCell>& Row : Out.Interaction)
		{
			Row.SetNum(Out.InterX.Levels.Num());
		}
	}
	if (const TSharedPtr<FJsonObject> S = Obj(J, TEXT("summary")))
	{
		Out.NumPass = static_cast<int32>(Num(S, TEXT("pass"), 0.f));
		Out.NumWarn = static_cast<int32>(Num(S, TEXT("warn"), 0.f));
		Out.NumFail = static_cast<int32>(Num(S, TEXT("fail"), 0.f));
		Out.Worst = Str(S, TEXT("worst"));
	}
	return true;
}

bool R1Panel::ParseWalkState(const FString& Json, FR1WalkState& Out)
{
	const TSharedPtr<FJsonObject> J = ParseObject(Json);
	if (!J.IsValid())
	{
		return false;
	}
	FR1WalkState S;
	S.bValid = true;
	S.bPolicyAvailable = Bool(J, TEXT("policy_available"));
	S.PolicyReason = Str(J, TEXT("policy_reason"));
	S.bRunning = Bool(J, TEXT("running"));
	S.Mode = Str(J, TEXT("mode"));
	S.Course = Str(J, TEXT("course"));
	if (const TArray<TSharedPtr<FJsonValue>>* Cs = Arr(J, TEXT("courses")))
	{
		for (const TSharedPtr<FJsonValue>& V : *Cs)
		{
			S.Courses.Add(V->AsString());
		}
	}
	S.T = Num(J, TEXT("t"), 0.f);
	ReadVec(Arr(J, TEXT("cmd")), S.Cmd);
	if (const TSharedPtr<FJsonObject> Lim = Obj(J, TEXT("cmd_limits")))
	{
		ReadRange(Lim, TEXT("vx"), S.VxMin, S.VxMax);
		ReadRange(Lim, TEXT("vy"), S.VyMin, S.VyMax);
		ReadRange(Lim, TEXT("wz"), S.WzMin, S.WzMax);
	}
	if (const TSharedPtr<FJsonObject> B = Obj(J, TEXT("base")))
	{
		ReadVec(Arr(B, TEXT("pos")), S.BasePosMj);
		ReadVec(Arr(B, TEXT("lin_vel")), S.LinVelMj);
		S.Yaw = Num(B, TEXT("yaw"), 0.f);
	}
	S.bFallen = Bool(J, TEXT("fallen"));
	S.Falls = static_cast<int32>(Num(J, TEXT("falls"), 0.f));
	if (const TSharedPtr<FJsonObject> C = Obj(J, TEXT("checkpoint")))
	{
		S.CheckpointIndex = static_cast<int32>(Num(C, TEXT("index"), 0.f));
		S.CheckpointTotal = static_cast<int32>(Num(C, TEXT("total"), 0.f));
	}
	if (const TSharedPtr<FJsonObject> L = Obj(J, TEXT("lap")))
	{
		OptNum(L, TEXT("current_s"), S.LapCurrentS);
		OptNum(L, TEXT("last_s"), S.LapLastS);
		OptNum(L, TEXT("best_s"), S.LapBestS);
		S.Laps = static_cast<int32>(Num(L, TEXT("laps"), 0.f));
	}
	if (const TSharedPtr<FJsonObject> G = Obj(J, TEXT("geometry")))
	{
		if (const TArray<TSharedPtr<FJsonValue>>* Cps = Arr(G, TEXT("checkpoints")))
		{
			for (const TSharedPtr<FJsonValue>& V : *Cps)
			{
				const TArray<TSharedPtr<FJsonValue>>* P = nullptr;
				FVector C;
				if (V->TryGetArray(P) && ReadVec(P, C))
				{
					S.CheckpointsMj.Add(C);
				}
			}
		}
		S.CheckpointRadius = Num(G, TEXT("checkpoint_radius"), S.CheckpointRadius);
		if (const TArray<TSharedPtr<FJsonValue>>* Boxes = Arr(G, TEXT("boxes")))
		{
			for (const TSharedPtr<FJsonValue>& V : *Boxes)
			{
				const TSharedPtr<FJsonObject>* P = nullptr;
				if (!V->TryGetObject(P) || !P || !P->IsValid())
				{
					continue;
				}
				FR1WalkBox Box;
				Box.Name = Str(*P, TEXT("name"));
				ReadVec(Arr(*P, TEXT("pos")), Box.PosMj);
				ReadVec(Arr(*P, TEXT("size")), Box.HalfSizeMj);
				if (const TArray<TSharedPtr<FJsonValue>>* Q = Arr(*P, TEXT("quat")); Q && Q->Num() >= 4)
				{
					Box.QuatW = (*Q)[0]->AsNumber();
					Box.QuatX = (*Q)[1]->AsNumber();
					Box.QuatY = (*Q)[2]->AsNumber();
					Box.QuatZ = (*Q)[3]->AsNumber();
				}
				if (const TArray<TSharedPtr<FJsonValue>>* Rgba = Arr(*P, TEXT("rgba")); Rgba && Rgba->Num() >= 3)
				{
					Box.Color = FLinearColor(static_cast<float>((*Rgba)[0]->AsNumber()), static_cast<float>((*Rgba)[1]->AsNumber()),
						static_cast<float>((*Rgba)[2]->AsNumber()), Rgba->Num() >= 4 ? static_cast<float>((*Rgba)[3]->AsNumber()) : 1.f);
				}
				S.Boxes.Add(MoveTemp(Box));
			}
		}
	}
	if (S.CheckpointTotal <= 0)
	{
		S.CheckpointTotal = S.CheckpointsMj.Num();
	}
	if (const TSharedPtr<FJsonObject> P = Obj(J, TEXT("puppet")))
	{
		S.bPuppetConnected = Bool(P, TEXT("connected"));
	}
	Out = MoveTemp(S);
	return true;
}

bool R1Panel::ParseProgress(const TArray<FString>& Lines, int32& OutDone, int32& OutTotal)
{
	for (int32 i = Lines.Num() - 1; i >= 0; --i)
	{
		const int32 At = Lines[i].Find(TEXT("@progress"));
		if (At == INDEX_NONE)
		{
			continue;
		}
		FString Rest = Lines[i].Mid(At + 9).TrimStartAndEnd();
		FString A, B;
		if (Rest.Split(TEXT("/"), &A, &B))
		{
			int32 Space = INDEX_NONE;
			if (B.FindChar(TEXT(' '), Space))
			{
				B = B.Left(Space);
			}
			if (A.IsNumeric() && B.IsNumeric())
			{
				OutDone = FCString::Atoi(*A);
				OutTotal = FCString::Atoi(*B);
				return OutTotal > 0;
			}
		}
	}
	return false;
}

FLinearColor R1Panel::VerdictColor(const FString& Verdict)
{
	if (Verdict == TEXT("pass"))
	{
		return FLinearColor::FromSRGBColor(FColor::FromHex(TEXT("0ca30c")));
	}
	if (Verdict == TEXT("warn"))
	{
		return FLinearColor::FromSRGBColor(FColor::FromHex(TEXT("fab219")));
	}
	return FLinearColor::FromSRGBColor(FColor::FromHex(TEXT("d03b3b")));
}

const TCHAR* R1Panel::VerdictGlyph(const FString& Verdict)
{
	return Verdict == TEXT("pass") ? TEXT("o") : Verdict == TEXT("warn") ? TEXT("!") : TEXT("x");
}

FString R1Panel::WalkCommandJson(const FVector& Cmd)
{
	return FString::Printf(TEXT("{\"vx\":%.3f,\"vy\":%.3f,\"wz\":%.3f}"), Cmd.X, Cmd.Y, Cmd.Z);
}

FString R1Panel::JsonEscape(const FString& In)
{
	FString Out;
	Out.Reserve(In.Len() + 8);
	for (const TCHAR C : In)
	{
		switch (C)
		{
		case TEXT('"'): Out += TEXT("\\\""); break;
		case TEXT('\\'): Out += TEXT("\\\\"); break;
		case TEXT('\n'): Out += TEXT("\\n"); break;
		case TEXT('\r'): Out += TEXT("\\r"); break;
		case TEXT('\t'): Out += TEXT("\\t"); break;
		default:
			if (C < 0x20)
			{
				Out += FString::Printf(TEXT("\\u%04x"), static_cast<uint32>(C));
			}
			else
			{
				Out.AppendChar(C);
			}
		}
	}
	return Out;
}
