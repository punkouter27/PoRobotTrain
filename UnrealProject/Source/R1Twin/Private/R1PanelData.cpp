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
	Out = MoveTemp(S);
	return true;
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
