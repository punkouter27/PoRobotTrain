// Data model + JSON parsing for the walk sandbox endpoint (docs/service_api.md section 3):
//   GET  /walk/state           -> FR1WalkState       (Level 2 walk sandbox, simulation only; docs/sim_features.md blocks)
//   GET  /scenarios            -> FR1ScenarioInfo[]  (scenario scripts; lab ones are read from disk)
// Plain structs with no UObject dependency so they can be unit-tested and shared by the panel and the course actor.
#pragma once

#include "CoreMinimal.h"
#include "R1MotorModel.h"

struct R1TWIN_API FR1WalkBox
{
	FString Name;
	FVector PosMj = FVector::ZeroVector;      // metres, MuJoCo world
	FVector HalfSizeMj = FVector::ZeroVector; // metres, half extents
	double QuatW = 1.0, QuatX = 0.0, QuatY = 0.0, QuatZ = 0.0;
	FLinearColor Color = FLinearColor(0.4f, 0.45f, 0.55f, 1.f);
};

/** physics.contacts[] (MuJoCo world, N). */
struct R1TWIN_API FR1WalkContact
{
	FVector PosMj = FVector::ZeroVector;
	FVector ForceMj = FVector::ZeroVector;
	FString A, B;
};

/** /walk/state "physics" block (docs/sim_features.md #2). */
struct R1TWIN_API FR1WalkPhysics
{
	bool bValid = false;
	FVector ComMj = FVector::ZeroVector;
	FVector ComVelMj = FVector::ZeroVector;
	FVector2D CapturePointMj = FVector2D::ZeroVector;
	TArray<FVector2D> SupportMj;
	bool bHasMargin = false;
	float CpMarginM = 0.f;
	TArray<FR1WalkContact> Contacts;
};

/** /walk/state "action_filter" block (#15). */
struct R1TWIN_API FR1WalkActionFilter
{
	bool bValid = false;
	float Alpha = 0.5f;
	bool bApply = false;
	TArray<float> Raw, Filtered;
	float Jitter = 0.f;
};

/** /walk/state "props" entry (#9): visual only in Unreal. */
struct R1TWIN_API FR1WalkProp
{
	int32 Id = 0;
	FString Kind;              // "ball" | "box"
	FVector PosMj = FVector::ZeroVector;
	double QuatW = 1.0, QuatX = 0.0, QuatY = 0.0, QuatZ = 0.0;
	TArray<float> Size;        // ball [r], box [hx, hy, hz] (m)
};

/** fall_analysis.history entry. */
struct R1TWIN_API FR1FallSample
{
	float T = 0.f;
	bool bHasMargin = false;
	float CpMarginM = 0.f;
	float TiltDeg = 0.f;
	float Height = 0.f;
};

/** /walk/state "fall_analysis" (#19), and the falls_log entries (History empty). */
struct R1TWIN_API FR1FallAnalysis
{
	bool bValid = false;
	float TFall = 0.f, TUnstable = 0.f, LeadS = 0.f;
	FString Reason, Cause;
	TArray<FR1FallSample> History;
};

/** /walk/state "replay" (#12). */
struct R1TWIN_API FR1WalkReplay
{
	bool bValid = false;
	bool bActive = false;
	float Speed = 0.25f, T = 0.f, DurationS = 3.f;
	bool bAutoOnFall = false;
};

/** /walk/state "scenario" (#18). */
struct R1TWIN_API FR1WalkScenarioState
{
	bool bValid = false;
	FString Id;
	float T = 0.f;
	int32 Step = 0, Steps = 0;
	bool bDone = false;
};

/** GET /scenarios entry. */
struct R1TWIN_API FR1ScenarioInfo
{
	FString Id, Name, Target, Description;
	float DurationS = 0.f;
};

/** RobotControl/scenarios/<id>.json with "target": "lab" (played by UR1SkillBrainComponent). */
struct R1TWIN_API FR1LabScenarioStep
{
	float T = 0.f;
	FString Do;                    // target | push | noise | delay | strength
	float YawDeg = 0.f, PitchDeg = 0.f;   // target
	float Yaw = 0.f, Pitch = 0.f;         // push direction
	float Deg = 0.f;                      // noise
	int32 Steps = 0;                      // delay
	float Pct = 100.f;                    // strength
};

struct R1TWIN_API FR1LabScenario
{
	FString Id, Name, Description, Skill;
	float SizeDeg = 10.f;
	float DurationS = 12.f;
	int32 Seed = 0;
	TArray<FR1LabScenarioStep> Steps;   // sorted by T
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

	// docs/sim_features.md additions (absent / null -> bValid false or defaults)
	int32 Seed = -1;              // procedural only
	float Difficulty = -1.f;
	FR1WalkPhysics Physics;
	FR1MotorReadout Motors;       // motors + thermal + battery blocks
	FR1WalkActionFilter ActionFilter;
	TArray<FR1WalkProp> Props;
	FR1FallAnalysis FallAnalysis;
	TArray<FR1FallAnalysis> FallsLog;
	FR1WalkReplay Replay;
	FR1WalkScenarioState Scenario;

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
	/** GET /scenarios -> entries (false if not parseable). */
	R1TWIN_API bool ParseScenarioList(const FString& Json, TArray<FR1ScenarioInfo>& Out);
	/** A lab scenario file (target must be "lab"); OutWhy on failure. */
	R1TWIN_API bool ParseLabScenario(const FString& Json, FR1LabScenario& Out, FString& OutWhy);
	/** RobotControl/scenarios next to UnrealProject/ (or -R1ScenariosDir=<path>). */
	R1TWIN_API FString ScenariosDir();
	/** Every lab scenario in ScenariosDir() (sorted by id). */
	R1TWIN_API TArray<FR1LabScenario> LoadLabScenarios();
	/** Escape a string for a JSON string literal. */
	R1TWIN_API FString JsonEscape(const FString& In);
}
