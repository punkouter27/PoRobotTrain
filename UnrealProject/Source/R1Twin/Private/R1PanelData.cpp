#include "R1PanelData.h"

#include "Dom/JsonObject.h"
#include "Dom/JsonValue.h"
#include "HAL/FileManager.h"
#include "Misc/CommandLine.h"
#include "Misc/FileHelper.h"
#include "Misc/Parse.h"
#include "Misc/Paths.h"
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

	bool ReadVec2(const TArray<TSharedPtr<FJsonValue>>* A, FVector2D& Out)
	{
		if (!A || A->Num() < 2)
		{
			return false;
		}
		Out = FVector2D((*A)[0]->AsNumber(), (*A)[1]->AsNumber());
		return true;
	}

	/** Numbers of an array field (null entries become 0). */
	void Floats(const TSharedPtr<FJsonObject>& J, const TCHAR* Field, TArray<float>& Out)
	{
		Out.Reset();
		if (const TArray<TSharedPtr<FJsonValue>>* A = Arr(J, Field))
		{
			for (const TSharedPtr<FJsonValue>& V : *A)
			{
				double D = 0.0;
				Out.Add(V.IsValid() && V->TryGetNumber(D) ? static_cast<float>(D) : 0.f);
			}
		}
	}

	void Strings(const TSharedPtr<FJsonObject>& J, const TCHAR* Field, TArray<FString>& Out)
	{
		Out.Reset();
		if (const TArray<TSharedPtr<FJsonValue>>* A = Arr(J, Field))
		{
			for (const TSharedPtr<FJsonValue>& V : *A)
			{
				FString S;
				if (V.IsValid() && V->TryGetString(S)) Out.Add(S);
			}
		}
	}

	const TSharedPtr<FJsonObject>* ObjValue(const TSharedPtr<FJsonValue>& V)
	{
		const TSharedPtr<FJsonObject>* P = nullptr;
		return V.IsValid() && V->TryGetObject(P) && P && P->IsValid() ? P : nullptr;
	}

	void ReadFall(const TSharedPtr<FJsonObject>& F, FR1FallAnalysis& Out)
	{
		Out.bValid = true;
		Out.TFall = Num(F, TEXT("t_fall"), 0.f);
		Out.TUnstable = Num(F, TEXT("t_unstable"), Out.TFall);
		Out.LeadS = Num(F, TEXT("lead_s"), Out.TFall - Out.TUnstable);
		Out.Reason = Str(F, TEXT("reason"));
		Out.Cause = Str(F, TEXT("cause"));
		if (const TArray<TSharedPtr<FJsonValue>>* H = Arr(F, TEXT("history")))
		{
			for (const TSharedPtr<FJsonValue>& V : *H)
			{
				if (const TSharedPtr<FJsonObject>* P = ObjValue(V))
				{
					FR1FallSample Smp;
					Smp.T = Num(*P, TEXT("t"), 0.f);
					Smp.bHasMargin = OptNum(*P, TEXT("cp_margin_m"), Smp.CpMarginM);
					Smp.TiltDeg = Num(*P, TEXT("tilt_deg"), 0.f);
					Smp.Height = Num(*P, TEXT("height"), 0.f);
					Out.History.Add(Smp);
				}
			}
		}
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

}
using namespace R1PanelJsonDetail;

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
		H = HashCombine(H, GetTypeHash(FVector4(B.QuatW, B.QuatX, B.QuatY, B.QuatZ)));
	}
	return H;
}

FVector FR1WalkState::ClampCmd(const FVector& In) const
{
	return FVector(FMath::Clamp<double>(In.X, VxMin, VxMax), FMath::Clamp<double>(In.Y, VyMin, VyMax), FMath::Clamp<double>(In.Z, WzMin, WzMax));
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

	// ---------------------------------------------------------------- docs/sim_features.md additions
	float Tmp = 0.f;
	if (OptNum(J, TEXT("seed"), Tmp)) S.Seed = FMath::RoundToInt(Tmp);
	OptNum(J, TEXT("difficulty"), S.Difficulty);
	if (const TSharedPtr<FJsonObject> Ph = Obj(J, TEXT("physics")))
	{
		FR1WalkPhysics& P = S.Physics;
		P.bValid = true;
		ReadVec(Arr(Ph, TEXT("com")), P.ComMj);
		ReadVec(Arr(Ph, TEXT("com_vel")), P.ComVelMj);
		ReadVec2(Arr(Ph, TEXT("capture_point")), P.CapturePointMj);
		if (const TArray<TSharedPtr<FJsonValue>>* Sup = Arr(Ph, TEXT("support")))
		{
			for (const TSharedPtr<FJsonValue>& V : *Sup)
			{
				const TArray<TSharedPtr<FJsonValue>>* Pt = nullptr;
				FVector2D XY;
				if (V.IsValid() && V->TryGetArray(Pt) && ReadVec2(Pt, XY)) P.SupportMj.Add(XY);
			}
		}
		P.bHasMargin = OptNum(Ph, TEXT("cp_margin_m"), P.CpMarginM);
		if (const TArray<TSharedPtr<FJsonValue>>* Cs = Arr(Ph, TEXT("contacts")))
		{
			for (const TSharedPtr<FJsonValue>& V : *Cs)
			{
				if (const TSharedPtr<FJsonObject>* C = ObjValue(V))
				{
					FR1WalkContact Ct;
					ReadVec(Arr(*C, TEXT("pos")), Ct.PosMj);
					ReadVec(Arr(*C, TEXT("force")), Ct.ForceMj);
					Ct.A = Str(*C, TEXT("a"));
					Ct.B = Str(*C, TEXT("b"));
					P.Contacts.Add(Ct);
				}
			}
		}
	}
	if (const TSharedPtr<FJsonObject> M = Obj(J, TEXT("motors")))
	{
		FR1MotorReadout& R = S.Motors;
		R.bValid = true;
		Strings(M, TEXT("names"), R.Names);
		Floats(M, TEXT("speed"), R.Speed);
		Floats(M, TEXT("torque"), R.Torque);
		Floats(M, TEXT("torque_frac"), R.TorqueFrac);
		const int32 N = R.Names.Num();
		R.Speed.SetNumZeroed(N);
		R.Torque.SetNumZeroed(N);
		R.TorqueFrac.SetNumZeroed(N);
	}
	if (const TSharedPtr<FJsonObject> Th = Obj(J, TEXT("thermal")))
	{
		FR1MotorReadout& R = S.Motors;
		Floats(Th, TEXT("temp_c"), R.TempC);
		R.Hottest = Str(Th, TEXT("hottest"));
		R.HottestC = Num(Th, TEXT("hottest_c"), R.HottestC);
		Strings(Th, TEXT("warn"), R.Warn);
		Strings(Th, TEXT("limit"), R.Limit);
	}
	if (const TSharedPtr<FJsonObject> Bt = Obj(J, TEXT("battery")))
	{
		FR1MotorReadout& R = S.Motors;
		R.SocPct = Num(Bt, TEXT("soc_pct"), R.SocPct);
		R.PowerW = Num(Bt, TEXT("power_w"), R.PowerW);
		R.MinutesLeft = Num(Bt, TEXT("minutes_left"), -1.f);
	}
	if (const TSharedPtr<FJsonObject> Af = Obj(J, TEXT("action_filter")))
	{
		FR1WalkActionFilter& A = S.ActionFilter;
		A.bValid = true;
		A.Alpha = Num(Af, TEXT("alpha"), A.Alpha);
		A.bApply = Bool(Af, TEXT("apply"));
		Floats(Af, TEXT("raw"), A.Raw);
		Floats(Af, TEXT("filtered"), A.Filtered);
		A.Jitter = Num(Af, TEXT("jitter"), 0.f);
	}
	if (const TArray<TSharedPtr<FJsonValue>>* Props = Arr(J, TEXT("props")))
	{
		for (const TSharedPtr<FJsonValue>& V : *Props)
		{
			if (const TSharedPtr<FJsonObject>* P = ObjValue(V))
			{
				FR1WalkProp Pr;
				Pr.Id = FMath::RoundToInt(Num(*P, TEXT("id"), 0.f));
				Pr.Kind = Str(*P, TEXT("kind"));
				ReadVec(Arr(*P, TEXT("pos")), Pr.PosMj);
				if (const TArray<TSharedPtr<FJsonValue>>* Q = Arr(*P, TEXT("quat")); Q && Q->Num() >= 4)
				{
					Pr.QuatW = (*Q)[0]->AsNumber();
					Pr.QuatX = (*Q)[1]->AsNumber();
					Pr.QuatY = (*Q)[2]->AsNumber();
					Pr.QuatZ = (*Q)[3]->AsNumber();
				}
				Floats(*P, TEXT("size"), Pr.Size);
				S.Props.Add(MoveTemp(Pr));
			}
		}
	}
	if (const TSharedPtr<FJsonObject> F = Obj(J, TEXT("fall_analysis")))
	{
		ReadFall(F, S.FallAnalysis);
	}
	if (const TArray<TSharedPtr<FJsonValue>>* Log = Arr(J, TEXT("falls_log")))
	{
		for (const TSharedPtr<FJsonValue>& V : *Log)
		{
			if (const TSharedPtr<FJsonObject>* P = ObjValue(V))
			{
				FR1FallAnalysis E;
				ReadFall(*P, E);
				S.FallsLog.Add(MoveTemp(E));
			}
		}
	}
	if (const TSharedPtr<FJsonObject> R = Obj(J, TEXT("replay")))
	{
		S.Replay.bValid = true;
		S.Replay.bActive = Bool(R, TEXT("active"));
		S.Replay.Speed = Num(R, TEXT("speed"), 0.25f);
		S.Replay.T = Num(R, TEXT("t"), 0.f);
		S.Replay.DurationS = Num(R, TEXT("duration_s"), 3.f);
		S.Replay.bAutoOnFall = Bool(R, TEXT("auto_on_fall"));
	}
	if (const TSharedPtr<FJsonObject> Sc = Obj(J, TEXT("scenario")))
	{
		S.Scenario.bValid = true;
		S.Scenario.Id = Str(Sc, TEXT("id"));
		S.Scenario.T = Num(Sc, TEXT("t"), 0.f);
		S.Scenario.Step = FMath::RoundToInt(Num(Sc, TEXT("step"), 0.f));
		S.Scenario.Steps = FMath::RoundToInt(Num(Sc, TEXT("steps"), 0.f));
		S.Scenario.bDone = Bool(Sc, TEXT("done"));
	}
	Out = MoveTemp(S);
	return true;
}

bool R1Panel::ParseScenarioList(const FString& Json, TArray<FR1ScenarioInfo>& Out)
{
	const TSharedPtr<FJsonObject> J = ParseObject(Json);
	const TArray<TSharedPtr<FJsonValue>>* List = Arr(J, TEXT("scenarios"));
	if (!List)
	{
		return false;
	}
	Out.Reset();
	for (const TSharedPtr<FJsonValue>& V : *List)
	{
		if (const TSharedPtr<FJsonObject>* P = ObjValue(V))
		{
			FR1ScenarioInfo I;
			I.Id = Str(*P, TEXT("id"));
			I.Name = Str(*P, TEXT("name"));
			I.Target = Str(*P, TEXT("target"));
			I.Description = Str(*P, TEXT("description"));
			I.DurationS = Num(*P, TEXT("duration_s"), 0.f);
			if (!I.Id.IsEmpty())
			{
				if (I.Name.IsEmpty()) I.Name = I.Id;
				Out.Add(MoveTemp(I));
			}
		}
	}
	return true;
}

bool R1Panel::ParseLabScenario(const FString& Json, FR1LabScenario& Out, FString& OutWhy)
{
	const TSharedPtr<FJsonObject> J = ParseObject(Json);
	if (!J.IsValid())
	{
		OutWhy = TEXT("not valid JSON");
		return false;
	}
	if (Str(J, TEXT("target")) != TEXT("lab"))
	{
		OutWhy = TEXT("not a lab scenario (target must be lab)");
		return false;
	}
	FR1LabScenario S;
	S.Id = Str(J, TEXT("id"));
	S.Name = Str(J, TEXT("name"));
	S.Description = Str(J, TEXT("description"));
	S.Skill = Str(J, TEXT("skill"));
	S.SizeDeg = Num(J, TEXT("size_deg"), S.SizeDeg);
	S.DurationS = FMath::Max(1.f, Num(J, TEXT("duration_s"), S.DurationS));
	S.Seed = FMath::RoundToInt(Num(J, TEXT("seed"), 0.f));
	if (S.Name.IsEmpty()) S.Name = S.Id;
	if (S.Skill.IsEmpty())
	{
		OutWhy = TEXT("the scenario names no skill");
		return false;
	}
	if (const TArray<TSharedPtr<FJsonValue>>* Steps = Arr(J, TEXT("steps")))
	{
		for (const TSharedPtr<FJsonValue>& V : *Steps)
		{
			if (const TSharedPtr<FJsonObject>* P = ObjValue(V))
			{
				FR1LabScenarioStep St;
				St.T = Num(*P, TEXT("t"), 0.f);
				St.Do = Str(*P, TEXT("do"));
				St.YawDeg = Num(*P, TEXT("yaw_deg"), 0.f);
				St.PitchDeg = Num(*P, TEXT("pitch_deg"), 0.f);
				St.Yaw = Num(*P, TEXT("yaw"), 0.f);
				St.Pitch = Num(*P, TEXT("pitch"), 0.f);
				St.Deg = Num(*P, TEXT("deg"), 0.f);
				St.Steps = FMath::RoundToInt(Num(*P, TEXT("steps"), 0.f));
				St.Pct = Num(*P, TEXT("pct"), 100.f);
				S.Steps.Add(MoveTemp(St));
			}
		}
	}
	S.Steps.StableSort([](const FR1LabScenarioStep& A, const FR1LabScenarioStep& B) { return A.T < B.T; });
	Out = MoveTemp(S);
	return true;
}

FString R1Panel::ScenariosDir()
{
	FString Dir;
	if (FParse::Value(FCommandLine::Get(), TEXT("R1ScenariosDir="), Dir))
	{
		return FPaths::ConvertRelativePathToFull(Dir);
	}
	return FPaths::ConvertRelativePathToFull(FPaths::Combine(FPaths::ProjectDir(), TEXT(".."), TEXT("RobotControl"), TEXT("scenarios")));
}

TArray<FR1LabScenario> R1Panel::LoadLabScenarios()
{
	TArray<FR1LabScenario> Out;
	const FString Dir = ScenariosDir();
	TArray<FString> Files;
	IFileManager::Get().FindFiles(Files, *FPaths::Combine(Dir, TEXT("*.json")), /*Files*/ true, /*Dirs*/ false);
	Files.Sort();
	for (const FString& F : Files)
	{
		FString Text, Why;
		FR1LabScenario S;
		if (FFileHelper::LoadFileToString(Text, *FPaths::Combine(Dir, F)) && ParseLabScenario(Text, S, Why))
		{
			if (S.Id.IsEmpty()) S.Id = FPaths::GetBaseFilename(F);
			Out.Add(MoveTemp(S));
		}
	}
	return Out;
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
