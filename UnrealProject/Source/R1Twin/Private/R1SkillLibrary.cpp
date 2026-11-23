#include "R1SkillLibrary.h"
#include "R1Twin.h"

#include "Dom/JsonObject.h"
#include "Dom/JsonValue.h"
#include "HAL/FileManager.h"
#include "Misc/CommandLine.h"
#include "Misc/FileHelper.h"
#include "Misc/Parse.h"
#include "Misc/Paths.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"

namespace
{
	// ------------------------------------------------------------------ SHA-256 (FIPS 180-4); the engine has none
	constexpr uint32 K256[64] = {
		0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
		0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
		0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
		0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
		0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
		0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
		0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
		0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2,
	};

	FORCEINLINE uint32 Rotr(uint32 X, uint32 N) { return (X >> N) | (X << (32 - N)); }

	void Sha256Block(uint32 H[8], const uint8* P)
	{
		uint32 W[64];
		for (int32 i = 0; i < 16; ++i)
		{
			W[i] = (uint32(P[4 * i]) << 24) | (uint32(P[4 * i + 1]) << 16) | (uint32(P[4 * i + 2]) << 8) | uint32(P[4 * i + 3]);
		}
		for (int32 i = 16; i < 64; ++i)
		{
			const uint32 S0 = Rotr(W[i - 15], 7) ^ Rotr(W[i - 15], 18) ^ (W[i - 15] >> 3);
			const uint32 S1 = Rotr(W[i - 2], 17) ^ Rotr(W[i - 2], 19) ^ (W[i - 2] >> 10);
			W[i] = W[i - 16] + S0 + W[i - 7] + S1;
		}
		uint32 A = H[0], B = H[1], C = H[2], D = H[3], E = H[4], F = H[5], G = H[6], Hh = H[7];
		for (int32 i = 0; i < 64; ++i)
		{
			const uint32 T1 = Hh + (Rotr(E, 6) ^ Rotr(E, 11) ^ Rotr(E, 25)) + ((E & F) ^ (~E & G)) + K256[i] + W[i];
			const uint32 T2 = (Rotr(A, 2) ^ Rotr(A, 13) ^ Rotr(A, 22)) + ((A & B) ^ (A & C) ^ (B & C));
			Hh = G; G = F; F = E; E = D + T1; D = C; C = B; B = A; A = T1 + T2;
		}
		H[0] += A; H[1] += B; H[2] += C; H[3] += D; H[4] += E; H[5] += F; H[6] += G; H[7] += Hh;
	}

	// ------------------------------------------------------------------ deploy.json helpers
	bool FloatArray(const TSharedPtr<FJsonObject>& J, const TCHAR* Field, TArray<float>& Out)
	{
		const TArray<TSharedPtr<FJsonValue>>* Arr = nullptr;
		if (!J->TryGetArrayField(Field, Arr) || !Arr)
		{
			return false;
		}
		Out.Reset(Arr->Num());
		for (const TSharedPtr<FJsonValue>& V : *Arr)
		{
			Out.Add(static_cast<float>(V->AsNumber()));
		}
		return true;
	}

	bool NameArray(const TSharedPtr<FJsonObject>& J, const TCHAR* Field, TArray<FName>& Out)
	{
		const TArray<TSharedPtr<FJsonValue>>* Arr = nullptr;
		if (!J->TryGetArrayField(Field, Arr) || !Arr)
		{
			return false;
		}
		Out.Reset(Arr->Num());
		for (const TSharedPtr<FJsonValue>& V : *Arr)
		{
			Out.Add(FName(*V->AsString()));
		}
		return true;
	}

	/** "head_yaw_joint" -> "head_yaw" (robotcontrol.configs.joint_map.actuator_name_for). */
	FName ActuatorFor(FName Joint)
	{
		FString S = Joint.ToString();
		S.RemoveFromEnd(TEXT("_joint"));
		return FName(*S);
	}

	bool LoadOne(const FString& SkillDir, FR1SkillDef& S)
	{
		S.Dir = SkillDir;
		S.Id = FPaths::GetCleanFilename(SkillDir);
		S.Label = S.Id;
		S.OnnxPath = FPaths::Combine(SkillDir, TEXT("policy.onnx"));
		FString Text;
		if (!FFileHelper::LoadFileToString(Text, *FPaths::Combine(SkillDir, TEXT("deploy.json"))))
		{
			S.Reason = TEXT("no deploy.json (re-run validation in the web app)");
			return true;
		}
		TSharedPtr<FJsonObject> J;
		if (!FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Text), J) || !J.IsValid())
		{
			S.Reason = TEXT("deploy.json is not valid JSON");
			return true;
		}
		J->TryGetStringField(TEXT("label"), S.Label);
		J->TryGetStringField(TEXT("desc"), S.Desc);
		double D = 0.0;
		if (J->TryGetNumberField(TEXT("num_obs"), D)) S.NumObs = static_cast<int32>(D);
		if (J->TryGetNumberField(TEXT("period_s"), D)) S.PeriodS = static_cast<float>(D);
		if (J->TryGetNumberField(TEXT("max_deg"), D)) S.MaxDeg = static_cast<float>(D);
		if (J->TryGetNumberField(TEXT("yaw_range"), D)) S.YawRange = static_cast<float>(D);
		if (J->TryGetNumberField(TEXT("pitch_range"), D)) S.PitchRange = static_cast<float>(D);
		if (J->TryGetNumberField(TEXT("glance_move"), D)) S.GlanceMove = static_cast<float>(D);
		const bool bShape = NameArray(J, TEXT("joint_names"), S.Joints)
			&& FloatArray(J, TEXT("default_joint_pos"), S.Default) && FloatArray(J, TEXT("action_scale"), S.ActionScale)
			&& FloatArray(J, TEXT("kp"), S.Kp) && FloatArray(J, TEXT("kd"), S.Kd) && FloatArray(J, TEXT("effort"), S.Effort)
			&& FloatArray(J, TEXT("soft_limit_lo"), S.SoftLo) && FloatArray(J, TEXT("soft_limit_hi"), S.SoftHi)
			&& NameArray(J, TEXT("hold_joints"), S.HoldJoints) && FloatArray(J, TEXT("hold_default"), S.HoldDefault)
			&& FloatArray(J, TEXT("hold_kp"), S.HoldKp) && FloatArray(J, TEXT("hold_kd"), S.HoldKd);
		const int32 K = S.Joints.Num();
		if (!bShape || K == 0 || S.Default.Num() != K || S.ActionScale.Num() != K || S.Kp.Num() != K || S.Effort.Num() != K
			|| S.SoftLo.Num() != K || S.SoftHi.Num() != K || S.HoldDefault.Num() != S.HoldJoints.Num() || S.NumObs != 3 * K + R1Skills::NumCmd)
		{
			S.Reason = TEXT("deploy.json does not match the head-skill contract");
			return true;
		}
		for (const FName& N : S.Joints) S.Actuators.Add(ActuatorFor(N));
		for (const FName& N : S.HoldJoints) S.HoldActuators.Add(ActuatorFor(N));
		if (S.Id.StartsWith(TEXT("pose_")))
		{
			// taught pose: where to hold the head (the contract field names, plus the likely alternatives)
			S.bPose = true;
			double Yaw = 0.0, Pitch = 0.0;
			bool bYaw = J->TryGetNumberField(TEXT("pose_yaw_deg"), Yaw), bPitch = J->TryGetNumberField(TEXT("pose_pitch_deg"), Pitch);
			auto FromObj = [&](const TCHAR* Field, const TCHAR* YawKey, const TCHAR* PitchKey)
			{
				const TSharedPtr<FJsonObject>* O = nullptr;
				if (J->TryGetObjectField(Field, O) && O && O->IsValid())
				{
					if (!bYaw) bYaw = (*O)->TryGetNumberField(YawKey, Yaw);
					if (!bPitch) bPitch = (*O)->TryGetNumberField(PitchKey, Pitch);
				}
			};
			FromObj(TEXT("metadata"), TEXT("pose_yaw_deg"), TEXT("pose_pitch_deg"));
			FromObj(TEXT("metadata"), TEXT("target_yaw_deg"), TEXT("target_pitch_deg"));
			FromObj(TEXT("pose"), TEXT("yaw_deg"), TEXT("pitch_deg"));
			if (!bYaw || !bPitch)
			{
				S.Reason = TEXT("taught pose without pose_yaw_deg / pose_pitch_deg in deploy.json");
				return true;
			}
			S.PoseYawRad = FMath::DegreesToRadians(static_cast<float>(Yaw));
			S.PosePitchRad = FMath::DegreesToRadians(static_cast<float>(Pitch));
			TArray<float> Cmd;
			if (FloatArray(J, TEXT("pose_command"), Cmd) && Cmd.Num() == R1Skills::NumCmd)
			{
				for (int32 c = 0; c < R1Skills::NumCmd; ++c) S.PoseCommand[c] = Cmd[c];
			}
			if (S.Desc.IsEmpty())
			{
				S.Desc = FString::Printf(TEXT("Taught pose: holds the head at yaw %+.0f, pitch %+.0f deg."), Yaw, Pitch);
			}
		}
		const TArray<TSharedPtr<FJsonValue>>* Keys = nullptr;
		if (J->TryGetArrayField(TEXT("glance_keys"), Keys) && Keys)
		{
			for (const TSharedPtr<FJsonValue>& V : *Keys)
			{
				const TArray<TSharedPtr<FJsonValue>>& P = V->AsArray();
				if (P.Num() == 2) S.GlanceKeys.Add(FVector2D(P[0]->AsNumber(), P[1]->AsNumber()));
			}
		}

		// the validation gate
		const TSharedPtr<FJsonObject>* ValObj = nullptr;
		if (!J->TryGetObjectField(TEXT("validation"), ValObj) || !ValObj || !ValObj->IsValid())
		{
			S.Reason = TEXT("trained but not validated");
			return true;
		}
		const TSharedPtr<FJsonObject>& Val = *ValObj;
		bool bPassed = false;
		Val->TryGetBoolField(TEXT("passed"), bPassed);
		Val->TryGetStringField(TEXT("validated_at"), S.ValidatedAt);
		const TSharedPtr<FJsonObject>* MetObj = nullptr;
		if (Val->TryGetObjectField(TEXT("metrics"), MetObj) && MetObj && MetObj->IsValid() && (*MetObj)->TryGetNumberField(TEXT("track_err_deg"), D))
		{
			S.TrackErrDeg = static_cast<float>(D);
		}
		if (!bPassed)
		{
			S.Reason = TEXT("failed its validation gate (retrain in the web app)");
			return true;
		}
		FString Want;
		Val->TryGetStringField(TEXT("policy_sha256"), Want);
		const FString Have = R1Skills::Sha256File(S.OnnxPath);
		if (Have.IsEmpty())
		{
			S.Reason = TEXT("policy.onnx missing");
			return true;
		}
		if (!Want.Equals(Have, ESearchCase::IgnoreCase))
		{
			S.Reason = TEXT("policy.onnx changed after validation (sha256 mismatch)");
			return true;
		}
		S.bReady = true;
		return true;
	}
}

FString R1Skills::Sha256Hex(const uint8* Data, int64 Len)
{
	uint32 H[8] = {0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a, 0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19};
	int64 i = 0;
	for (; i + 64 <= Len; i += 64)
	{
		Sha256Block(H, Data + i);
	}
	uint8 Tail[128] = {};
	const int64 Rem = Len - i;
	FMemory::Memcpy(Tail, Data + i, Rem);
	Tail[Rem] = 0x80;
	const int32 TailLen = Rem + 1 + 8 <= 64 ? 64 : 128;
	const uint64 Bits = static_cast<uint64>(Len) * 8ull;
	for (int32 b = 0; b < 8; ++b)
	{
		Tail[TailLen - 1 - b] = static_cast<uint8>(Bits >> (8 * b));
	}
	Sha256Block(H, Tail);
	if (TailLen == 128)
	{
		Sha256Block(H, Tail + 64);
	}
	FString Out;
	for (uint32 W : H)
	{
		Out += FString::Printf(TEXT("%08x"), W);
	}
	return Out;
}

FString R1Skills::Sha256File(const FString& Path)
{
	TArray64<uint8> Bytes;
	if (!FFileHelper::LoadFileToArray(Bytes, *Path))
	{
		return FString();
	}
	return Sha256Hex(Bytes.GetData(), Bytes.Num());
}

FString R1Skills::DefaultDir()
{
	FString Dir;
	if (FParse::Value(FCommandLine::Get(), TEXT("R1SkillsDir="), Dir))
	{
		return FPaths::ConvertRelativePathToFull(Dir);
	}
	return FPaths::ConvertRelativePathToFull(FPaths::Combine(FPaths::ProjectDir(), TEXT(".."), TEXT("RobotControl"), TEXT("models"), TEXT("head_skills")));
}

TArray<FR1SkillDef> R1Skills::LoadAll(const FString& Dir)
{
	TArray<FString> Folders;
	IFileManager::Get().FindFiles(Folders, *FPaths::Combine(Dir, TEXT("*")), /*Files*/ false, /*Dirs*/ true);
	static const TCHAR* Order[] = {TEXT("hold_centre"), TEXT("look_at"), TEXT("nod_yes"), TEXT("shake_no"), TEXT("glance_around")};
	Folders.Sort([](const FString& A, const FString& B)
	{
		auto Rank = [](const FString& S) { for (int32 i = 0; i < UE_ARRAY_COUNT(Order); ++i) { if (S == Order[i]) return i; } return 100; };
		const int32 Ra = Rank(A), Rb = Rank(B);
		return Ra != Rb ? Ra < Rb : A < B;
	});
	TArray<FR1SkillDef> Out;
	for (const FString& F : Folders)
	{
		const FString Path = FPaths::Combine(Dir, F);
		if (!FPaths::FileExists(FPaths::Combine(Path, TEXT("policy.onnx"))))
		{
			continue;   // staging leftovers etc.
		}
		FR1SkillDef S;
		LoadOne(Path, S);
		UE_LOG(LogR1Twin, Log, TEXT("R1Skills: %s %s%s"), *S.Id, S.bReady ? TEXT("ready") : TEXT("LOCKED: "), *S.Reason);
		Out.Add(MoveTemp(S));
	}
	return Out;
}

void R1Skills::PhaseCommand(const FR1SkillDef& Skill, float T, float Size, float OutCmd[NumCmd])
{
	const float Ph = 2.f * PI * T / FMath::Max(Skill.PeriodS, 1e-3f);
	OutCmd[0] = FMath::Sin(Ph);
	OutCmd[1] = FMath::Cos(Ph);
	OutCmd[2] = Size;
}

void R1Skills::Reference(const FR1SkillDef& Skill, const float Cmd[NumCmd], float Phase, TArray<float>& Out)
{
	const int32 K = Skill.NumJoints();
	Out.SetNumZeroed(K);
	const int32 IP = Skill.JointIndex(HeadPitch), IY = Skill.JointIndex(HeadYaw);
	if (Skill.Id == TEXT("hold_centre"))
	{
		return;
	}
	if (Skill.bPose)
	{
		if (IY >= 0) Out[IY] = Skill.PoseYawRad;
		if (IP >= 0) Out[IP] = Skill.PosePitchRad;
		return;
	}
	if (Skill.Id == TEXT("look_at"))
	{
		if (IY >= 0) Out[IY] = Cmd[0];
		if (IP >= 0) Out[IP] = Cmd[1];
		return;
	}
	if (Phase < 0.f)
	{
		Phase = FMath::Fmod(FMath::Atan2(Cmd[0], Cmd[1]) / (2.f * PI) + 1.f, 1.f);
	}
	const float Amp = FMath::DegreesToRadians(Skill.MaxDeg) * Cmd[2];
	if (Skill.Id == TEXT("nod_yes") || Skill.Id == TEXT("shake_no"))
	{
		Out[0] = Amp * FMath::Sin(2.f * PI * Phase);
		return;
	}
	if (Skill.Id == TEXT("glance_around") && Skill.GlanceKeys.Num() > 0)
	{
		const int32 N = Skill.GlanceKeys.Num();
		const float SegF = Phase * N;
		const int32 Seg = FMath::FloorToInt(SegF) % N;
		float U = FMath::Clamp((SegF - FMath::FloorToFloat(SegF) - (1.f - Skill.GlanceMove)) / Skill.GlanceMove, 0.f, 1.f);
		U = U * U * (3.f - 2.f * U);
		const FVector2D A = Skill.GlanceKeys[Seg], B = Skill.GlanceKeys[(Seg + 1) % N];
		const FVector2D YP = (A + (B - A) * U) * Amp;
		if (IY >= 0) Out[IY] = YP.X;
		if (IP >= 0) Out[IP] = YP.Y;
	}
}
