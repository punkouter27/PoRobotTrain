// Data model + JSON parsing for the walk sandbox endpoint (docs/service_api.md section 3):
//   GET  /walk/state           -> FR1WalkState       (Level 2 walk sandbox, simulation only)
// Plain structs with no UObject dependency so they can be unit-tested and shared by the panel and the course actor.
#pragma once

#include "CoreMinimal.h"

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
	R1TWIN_API bool ParseWalkState(const FString& Json, FR1WalkState& Out);
	/** Body of POST /walk/command. */
	R1TWIN_API FString WalkCommandJson(const FVector& Cmd);
	/** Escape a string for a JSON string literal. */
	R1TWIN_API FString JsonEscape(const FString& In);
}
