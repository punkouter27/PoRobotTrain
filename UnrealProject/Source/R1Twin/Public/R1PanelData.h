// Data model + JSON parsing for the behaviour service endpoints added in docs/service_api.md:
//   POST /assistant/interpret  -> FR1Intent          (voice, idea #11)
//   GET  /robustness           -> FR1RobustReport    (robustness explorer, idea #19)
//   GET  /walk/state           -> FR1WalkState       (Level 2 walk sandbox, idea #20)
// Plain structs with no UObject dependency so they can be unit-tested and shared by the panel and the course actor.
#pragma once

#include "CoreMinimal.h"

/** POST /assistant/interpret response. */
struct R1TWIN_API FR1Intent
{
	bool bValid = false;
	FString Kind;                 // "behaviour" | "joint" | "walk" | "pose" | "answer" | "unknown" (/assistant/chat)
	FString Answer;               // kind "answer": status answer from the live service state
	FString Pose;                 // kind "pose": pose name to offer for a twin preview (never auto-started)
	FString Id;                   // behaviour id
	FString Label;
	FString Joint;
	bool bHasAmplitude = false;
	float Amplitude = 0.f;
	bool bHasDeg = false;
	float Deg = 0.f;
	bool bBlocked = false;
	bool bHasWalk = false;
	bool bWalkStop = false;
	FVector WalkCmd = FVector::ZeroVector;   // vx, vy, wz
	float Score = 0.f;
	FString Note;
	FString Say;
};

/** One robustness cell (one perturbed copy of the stand twin). */
struct R1TWIN_API FR1RobustCell
{
	bool bValid = false;
	float MeanErrDeg = 0.f;
	float MaxErrDeg = 0.f;
	float MaxAcc = 0.f;
	int32 Trips = 0;
	float Jitter = 0.f;           // mean |delta action| per step (informational)
	FString AccJoint;             // joint with the largest acceleration
	bool bFinite = true;
	FString Verdict;              // "pass" | "warn" | "fail" (missing or unknown -> "fail")

	/** Plain-words reason for a warn/fail verdict, e.g. "acceleration 289 rad/s² on head yaw (limit 80)". Empty for pass. */
	FString Reason(float FailMean, float FailAcc, float WarnMean, float WarnAcc) const;
};

struct R1TWIN_API FR1RobustAxis
{
	FString Id;
	FString Label;
	FString Unit;
	TArray<float> Levels;
	bool bHasNominal = false;
	float Nominal = 0.f;

	/** "0.7×", "40 ms" */
	FString LevelText(int32 Index) const;
	/** Index of the nominal level, or INDEX_NONE. */
	int32 NominalIndex() const;
};

struct R1TWIN_API FR1RobustFactor
{
	FR1RobustAxis Axis;
	TArray<FR1RobustCell> Cells;  // one per level
};

/** GET /robustness response. */
struct R1TWIN_API FR1RobustReport
{
	bool bValid = false;          // parsed OK
	bool bAvailable = false;      // a sweep has run
	FString CreatedAt;
	FString Policy;
	FString PolicySha;
	bool bQuick = false;
	float DurationS = 0.f;
	float RolloutS = 0.f;
	float FailMeanErr = 3.f, FailAcc = 80.f, WarnMeanErr = 2.f, WarnAcc = 60.f;
	FR1RobustCell Nominal;
	TArray<FR1RobustFactor> Sensitivity;
	FR1RobustAxis InterX, InterY;
	TArray<TArray<FR1RobustCell>> Interaction;   // [y][x]
	int32 NumPass = 0, NumWarn = 0, NumFail = 0;
	FString Worst;
};

struct R1TWIN_API FR1WalkBox
{
	FString Name;
	FVector PosMj = FVector::ZeroVector;      // metres, MuJoCo world
	FVector HalfSizeMj = FVector::ZeroVector; // metres, half extents
	double QuatW = 1.0, QuatX = 0.0, QuatY = 0.0, QuatZ = 0.0;
	FLinearColor Color = FLinearColor(0.4f, 0.45f, 0.55f, 1.f);
};

/** GET /walk/state response. */
struct R1TWIN_API FR1WalkState
{
	bool bValid = false;
	bool bPolicyAvailable = false;
	FString PolicyReason;
	bool bRunning = false;
	FString Mode;                 // "policy" | "hold"
	FString Course;
	TArray<FString> Courses;      // selectable courses ("flat", "loop")
	float T = 0.f;
	FVector Cmd = FVector::ZeroVector;
	float VxMin = -0.5f, VxMax = 1.f, VyMin = -0.5f, VyMax = 0.5f, WzMin = -1.f, WzMax = 1.f;
	FVector BasePosMj = FVector::ZeroVector;
	FVector LinVelMj = FVector::ZeroVector;
	float Yaw = 0.f;
	bool bFallen = false;
	int32 Falls = 0;
	int32 CheckpointIndex = 0;
	int32 CheckpointTotal = 0;
	float LapCurrentS = -1.f, LapLastS = -1.f, LapBestS = -1.f;
	int32 Laps = 0;
	TArray<FVector> CheckpointsMj;
	float CheckpointRadius = 0.5f;
	TArray<FR1WalkBox> Boxes;
	bool bPuppetConnected = false;

	/** Changes when the static course geometry changes (rebuild the course visuals). */
	uint32 GeometryHash() const;
	/** Clamp a velocity command to cmd_limits. */
	FVector ClampCmd(const FVector& In) const;
};

namespace R1Panel
{
	R1TWIN_API bool ParseIntent(const FString& Json, FR1Intent& Out);
	R1TWIN_API bool ParseRobustness(const FString& Json, FR1RobustReport& Out);
	R1TWIN_API bool ParseWalkState(const FString& Json, FR1WalkState& Out);
	/** Last "@progress <done>/<total>" among the service output lines; false if none (fallback for /status.progress). */
	R1TWIN_API bool ParseProgress(const TArray<FString>& Lines, int32& OutDone, int32& OutTotal);
	/** "head_yaw_joint" -> "head yaw". */
	R1TWIN_API FString PrettyJoint(const FString& Joint);
	/** Verdict -> status colour (good #0ca30c, warning #fab219, critical #d03b3b). */
	R1TWIN_API FLinearColor VerdictColor(const FString& Verdict);
	/** Verdict -> glyph shown with the colour so the verdict never relies on colour alone. */
	R1TWIN_API const TCHAR* VerdictGlyph(const FString& Verdict);
	/** Body of POST /walk/command. */
	R1TWIN_API FString WalkCommandJson(const FVector& Cmd);
	/** Escape a string for a JSON string literal. */
	R1TWIN_API FString JsonEscape(const FString& In);
}
