// Scenario lab panel: the Unreal app's on-screen HUD. SIMULATION ONLY - nothing here reaches the physical robot.
//
// Ownership (AGENTS.md): the web app trains, validates and runs skills on the real R1 on its stand, and owns
// health, setup and logs. This app hatches those trained brains on the virtual R1 and tests them in scenarios.
//
// HUD anchors (Lab tab):
//   1. BRAIN     the trained head skills (validated ONNX from the web app), size, duration, [Play] / [Stop]
//   2. TARGET    look-at ball: Script / Manual (arrow keys) / Orbit, nudges
//   3. DISTURB   push the head, action delay, sensor noise, motor strength
//   4. [STOP]    stops the brain and the walk sandbox (every tab)
//   5. [Details] toggles URLab's debug panels (every tab)
// plus the skill timeline, the result card (tracking error vs. the validated error, and whether the real robot's
// safety envelope would have stopped the run) and the head-camera picture-in-picture (or the 26-joint matrix).
//
// Tabs: Lab (R1PanelLab.cpp, brain: UR1SkillBrainComponent) and Walk (R1PanelWalk.cpp: Level 2 walk sandbox,
// simulation only, stepped by the behaviour service).
//
// Command line:
//   -R1Service=<url>       behaviour service URL for the walk sandbox (default ServiceUrl, http://127.0.0.1:8765)
//   -R1SkillsDir=<path>    trained skills folder (default ../RobotControl/models/head_skills)
//   -PanelTab=lab|walk     start on a tab
//   -PanelCapture=<dir>    scripted scenario screenshots (Look At orbit, push, timeline, disturbances, walk), then quit
#pragma once

#include "CoreMinimal.h"
#include "GameFramework/HUD.h"
#include "InputCoreTypes.h"
#include "Interfaces/IHttpRequest.h"
#include "Dom/JsonObject.h"
#include "Templates/Function.h"
#include "R1PanelData.h"
#include "R1BehaviourPanel.generated.h"

class AR1WalkCourse;
class UR1SkillBrainComponent;

UENUM()
enum class ER1PanelTab : uint8
{
	Lab,
	Walk,
};

UCLASS(Config = Game)
class R1TWIN_API AR1BehaviourPanel : public AHUD
{
	GENERATED_BODY()

public:
	AR1BehaviourPanel();

	/** Behaviour service base URL (walk sandbox only; RobotControl/behaviours/service.py). */
	UPROPERTY(Config, EditAnywhere, Category = "R1|Walk")
	FString ServiceUrl = TEXT("http://127.0.0.1:8765");

	UPROPERTY(Config, EditAnywhere, Category = "R1|Walk")
	FString WalkCourse = TEXT("loop");

	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type Reason) override;
	virtual void Tick(float DeltaSeconds) override;
	virtual void DrawHUD() override;
	/** Called by the Slate input pre-processor on every left mouse press (before game / UMG input routing). */
	void QueueClick() { bClickQueued = true; }
	ER1PanelTab GetTab() const { return Tab; }

private:
	struct FButton { FName Id; FString Label; FVector2D Pos; FVector2D Size; bool bEnabled; FLinearColor Color; bool bLarge = false; };
	struct FJointMeta
	{
		FName MjName;
		FString Label;
		float MinDeg = -180.f;
		float MaxDeg = 180.f;
		float DefaultDeg = 0.f;
	};
	struct FTwinReadout
	{
		bool bValid = false;
		TArray<float> QDeg;
		float ImuRoll = 0.f, ImuPitch = 0.f, ImuYaw = 0.f;
	};

	void HandleClick(FName BoxName);
	void PollStatus();
	void OnStatus(FHttpRequestPtr Req, FHttpResponsePtr Resp, bool bOk);
	void DrawButton(const FButton& B);
	void DrawLine(const FString& Text, float X, float& Y, const FLinearColor& Color, float Scale = 1.f);
	void DrawWrapped(const FString& Text, float X, float& Y, float MaxW, const FLinearColor& Color, float Scale = 0.9f);
	void DrawSection(const FString& Title, float X, float Inner, float& Y, const FLinearColor& Accent);
	void SetUrlabDebugVisible(bool bVisible);
	void ResolveTwinJoints();
	bool SampleLiveTwin(FTwinReadout& Out) const;
	void DrawJointMatrix(float X, float Y, float W);
	void DrawHeadCamera(float X, float Y, float W);
	void ShowToast(const FString& Text, const FLinearColor& Color);
	bool GetPanelMouse(float& OutX, float& OutY) const;
	void SetTab(ER1PanelTab NewTab);
	void DrawTabStrip(float X0, float X, float Inner, float PanelW, float& Y);
	/** POST JSON to the service; OnDone gets the HTTP code (0 = unreachable) and the parsed body (may be null). */
	void PostJson(const FString& Path, const FString& Body, TFunction<void(int32, TSharedPtr<FJsonObject>)> OnDone = nullptr);
	void GetJson(const FString& Path, TFunction<void(int32, const FString&)> OnDone);
	void StopAll();

	// behaviour service (walk sandbox)
	bool bServiceOnline = false;
	FString Job;
	double NextPoll = 0.0;
	bool bPollInFlight = false;

	// twin readout (joint matrix)
	TArray<FJointMeta> JointMetas;
	bool bTwinResolved = false;
	TArray<TWeakObjectPtr<UObject>> TwinJointObjs;
	TWeakObjectPtr<UObject> TwinRootBody;
	bool bShowDetails = false;
	bool bShowJoints = false;   // right column: head camera (default) or the 26-joint matrix

	// Clicks: HUD hit boxes depend on PlayerController click routing, which the follow camera and URLab's widget
	// change at runtime, so the panel hit-tests its own button rectangles from a Slate input pre-processor.
	TSharedPtr<class IInputProcessor> ClickProcessor;
	bool bClickQueued = false;
	TArray<TPair<FName, FBox2D>> ButtonRects;   // drawn last frame, in canvas pixels (topmost last)
	TArray<TPair<FName, FBox2D>> NextRects;     // being drawn this frame
	FName PressedId;
	double PressedAt = -10.0;
	FString Toast;
	FLinearColor ToastColor = FLinearColor::White;
	double ToastAt = -10.0;
	TMap<FName, FString> DisabledWhy;   // buttons drawn disabled this frame -> why (shown when clicked)
	TMap<FName, FString> ButtonLabel;
	bool bUrlabDebugHidden = false;

	// layout
	ER1PanelTab Tab = ER1PanelTab::Lab;
	float PanelScale = 1.f;        // the whole panel is drawn scaled (small windows); mouse is mapped back
	float LastPanelH = 640.f;      // measured content height (panel background uses last frame's)
	float ViewW = 1600.f, ViewH = 900.f;   // canvas size in panel units

	// ---------------------------------------------------------------- scenario lab (R1PanelLab.cpp)
	TWeakObjectPtr<UR1SkillBrainComponent> Brain;
	int32 LabSkill = -1;           // resolved from LabSkillId once the skills are loaded
	/** Remembered between sessions (Saved/Config/.../Game.ini): last skill, size and run length, Watch vs Experiment. */
	UPROPERTY(Config) FString LabSkillId = TEXT("look_at");
	UPROPERTY(Config) float LabSizeDeg = 10.f;
	UPROPERTY(Config) float LabDurationS = 12.f;
	UPROPERTY(Config) bool bExperiment = false;
	int32 PushCount = 0;           // the Push button cycles left, right, up, down
	void RememberLab();
	/** Simple view shows the head camera only for skills that look at something (Look At). */
	bool LabShowsCamera();
	// web app -> lab: "View in Unreal" (GET /lab/poll every second; a new request seq plays that skill)
	int32 LabSeq = -1;             // last request handled (-1 = not polled yet: an old request is not replayed)
	double NextLabPoll = 0.0;
	bool bLabPollInFlight = false;
	void TickLab(double Now);
	UR1SkillBrainComponent* FindBrain();
	void DrawLabTab(float X, float Inner, float& Y);
	bool HandleLabClick(FName Id);

	// ---------------------------------------------------------------- walk sandbox (R1PanelWalk.cpp)
	FR1WalkState WalkState;
	bool bWalkPollInFlight = false;
	double NextWalkPoll = 0.0;
	FVector InputWalkCmd = FVector::ZeroVector;
	bool bWasDriving = false;
	double NextCmdSend = 0.0;
	bool bWalkVisualsOn = false;
	bool bPrevCamLock = true;
	float PrevCamDistance = 320.f;
	TWeakObjectPtr<AR1WalkCourse> CourseActor;
	TWeakObjectPtr<USceneComponent> StandBody;
	FString WalkNote;
	void TickWalk(double Now);
	void PollWalk();
	void ReadWalkInput();
	void SendWalkCommand(const FVector& Cmd);
	void SetWalkRunningVisuals(bool bOn);
	void SyncCourse();
	void DrawWalkTab(float X, float Inner, float& Y);
	bool HandleWalkClick(FName Id);

	// ---------------------------------------------------------------- unattended capture (-PanelCapture=<dir>)
	FString PanelCaptureDir;
	double CaptureStartAt = -1.0;
	int32 CaptureStep = 0;
	void TickPanelCapture(double Now);
	void CaptureShot(const TCHAR* Name);
};
