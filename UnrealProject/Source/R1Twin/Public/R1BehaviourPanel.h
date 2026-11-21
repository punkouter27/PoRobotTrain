// Behaviour panel: on-screen buttons and stand-safe telemetry HUD that drive RobotControl's
// behaviour service (Python, 127.0.0.1:8765).
//
// Five HUD anchors:
//   1. VIRTUAL ROBOT (on screen)     [Move VIRTUAL robot]                     <- blue, safe
//   2. REAL ROBOT    (Check/Monitor) [Check] [Monitor]                        <- orange, read-only
//   3. REAL ROBOT    (Move REAL)     [Move REAL robot]                        <- orange, gated on 1+2+confirm
//   4. [STOP]                        remote L2+B 5 s = emergency stop
//   5. [Details]                     toggles URLab debug panels and raw service output
// The anchors live on the Behaviours tab; STOP and Details stay visible on every tab.
//
// Tabs (docs/service_api.md):
//   Behaviours   the five anchors above
//   Robustness   idea #19: GET /robustness heatmaps, POST /robustness/run   (R1PanelRobustness.cpp)
//   Walk         idea #20: Level 2 walk sandbox, simulation only            (R1PanelWalk.cpp, AR1WalkCourse)
//   Health       operator console: health, versions, events, power, temperatures, IMU, head camera (R1PanelConsole.cpp)
//   Motions      poses, motions (play / record / export / import), modes, session limits (twin only)
//   Setup        pre-flight checklist, zero-offset calibration, connection profiles
//   Help         docs/*.md as wrapped text (mouse wheel / PgUp / PgDn)
// E-STOP (every tab, End key): POST /estop stops every job, the walk sandbox and the simulation player.
// Voice (idea #11, every tab): [Mic] or hold V -> Windows SAPI dictation -> POST /assistant/chat (superset of
// /assistant/interpret) -> the suggestion fills in the selection like a click (it never starts anything), status
// questions are answered, a pose is offered as a "Preview pose" button; the reply is spoken (R1PanelVoice.cpp).
//
// Command line:
//   -R1Service=<url>                       behaviour service URL (default ServiceUrl, http://127.0.0.1:8765)
//   -PanelTab=behaviours|robustness|walk|health|motions|setup|help   start on a tab
//   -PanelCapture=<dir>                    screenshot each tab (walk: hold mode via the service), then quit
//   -VoiceTest=<phrase_with_underscores>   with -PanelCapture: inject the phrase as heard speech (no microphone)
#pragma once

#include "CoreMinimal.h"
#include "GameFramework/HUD.h"
#include "InputCoreTypes.h"
#include "Interfaces/IHttpRequest.h"
#include "Dom/JsonObject.h"
#include "Templates/Function.h"
#include "Templates/UniquePtr.h"
#include "R1PanelData.h"
#include "R1BehaviourPanel.generated.h"

class FR1VoiceSapi;
class AR1WalkCourse;
class UTexture2D;

UENUM()
enum class ER1PanelTab : uint8
{
	Behaviours,
	Robustness,
	Walk,
	Health,
	Motions,
	Setup,
	Help,
};

UCLASS(Config = Game)
class R1TWIN_API AR1BehaviourPanel : public AHUD
{
	GENERATED_BODY()

public:
	AR1BehaviourPanel();

	/** Behaviour service base URL (RobotControl/behaviours/service.py). */
	UPROPERTY(Config, EditAnywhere, Category = "R1|Behaviour")
	FString ServiceUrl = TEXT("http://127.0.0.1:8765");

	UPROPERTY(Config, EditAnywhere, Category = "R1|Behaviour")
	FString Behaviour = TEXT("neck_turn");

	UPROPERTY(EditAnywhere, Category = "R1|Behaviour")
	float AmplitudeDeg = 10.f;

	UPROPERTY(EditAnywhere, Category = "R1|Behaviour")
	float FrequencyHz = 0.2f;

	UPROPERTY(EditAnywhere, Category = "R1|Behaviour")
	float DurationS = 20.f;

	/** Push-to-talk key (hold to listen). Tab = overlay, C = command mode, WASD/QE/arrows = walk sandbox. */
	UPROPERTY(EditAnywhere, Category = "R1|Voice")
	FKey PushToTalkKey;

	/** Speak the assistant's reply (SAPI text to speech). */
	UPROPERTY(Config, EditAnywhere, Category = "R1|Voice")
	bool bSpeakReplies = true;

	UPROPERTY(Config, EditAnywhere, Category = "R1|Walk")
	FString WalkCourse = TEXT("loop");

	AR1BehaviourPanel(FVTableHelper& Helper);
	virtual ~AR1BehaviourPanel() override;
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type Reason) override;
	virtual void Tick(float DeltaSeconds) override;
	virtual void DrawHUD() override;
	/** Called by the Slate input pre-processor on every left mouse press (before game / UMG input routing). */
	void QueueClick() { bClickQueued = true; }
	/** Called by the Slate input pre-processor on every mouse wheel tick (Help tab scrolling). Never consumed. */
	void QueueWheel(float Delta) { WheelQueued += Delta; }
	ER1PanelTab GetTab() const { return Tab; }

private:
	struct FButton { FName Id; FString Label; FVector2D Pos; FVector2D Size; bool bEnabled; FLinearColor Color; bool bLarge = false; };
	struct FJobResult { bool bValid = false; bool bOk = false; float Amplitude = -1.f; FString Reason; };
	struct FBehaviourInfo { FString Id; FString Label; FString Desc; FString PrimaryLabel; float MaxDeg = 30.f; };
	struct FJointMeta
	{
		FName MjName;
		FString Label;
		float MinDeg = -180.f;
		float MaxDeg = 180.f;
		float DefaultDeg = 0.f;
	};
	struct FTelemetryReadout
	{
		bool bValid = false, bHasTarget = false, bHasImu = false, bHasTemp = false, bHasBusV = false, bHasActive = false;
		float ActualDeg = 0.f, TargetDeg = 0.f, OtherDeg = 0.f, AgeS = 0.f;
		float ImuRoll = 0.f, ImuPitch = 0.f, ImuYaw = 0.f;
		int32 MaxTempC = 0, ActiveMotors = 0, ModeMachine = 0;
		float BusV = 0.f;
		FString PrimaryLabel, HotJoint, Source, Phase;
		TArray<float> QDeg;
		TArray<int32> TempsC;
	};

	void HandleClick(FName BoxName);
	void PollStatus();
	void Post(const FString& Path, bool bConfirm = false);
	void OnStatus(FHttpRequestPtr Req, FHttpResponsePtr Resp, bool bOk);
	void OnPost(FHttpRequestPtr Req, FHttpResponsePtr Resp, bool bOk);
	FString ParamsJson(bool bConfirm) const;
	void DrawButton(const FButton& B);
	void DrawLine(const FString& Text, float X, float& Y, const FLinearColor& Color, float Scale = 1.f);
	void DrawStep(bool bDone, const FString& Text, float X, float& Y);
	void SetUrlabDebugVisible(bool bVisible);
	void ResolveTwinJoints();
	bool SampleLiveTwin(FTelemetryReadout& Out) const;
	static FJobResult ReadResult(const TSharedPtr<FJsonObject>& Results, const TCHAR* Job);
	static float ReadAmplitude(const TSharedPtr<FJsonObject>& J, const TCHAR* Field);
	static FString ReadBehaviour(const TSharedPtr<FJsonObject>& J, const TCHAR* Field);
	static FTelemetryReadout ReadTelemetry(const TSharedPtr<FJsonObject>& Telemetry, const TSharedPtr<FJsonObject>& Head, const TCHAR* Who);
	void DrawTelemetry(const FTelemetryReadout& T, bool bLive, const FString& NoData, const FLinearColor& Accent, float X, float W, float& Y);
	void DrawJointMatrix(float X, float Y, float W);
	const FBehaviourInfo* ActiveBehaviourInfo() const;

	// latest service status
	bool bServiceOnline = false;
	FString Job;
	FString LastResult;
	FString WhyNotRun;
	bool bCanRunRobot = false;
	int32 MirrorSamples = 0;
	float PreviewedAmp = -1.f;
	float CheckedAmp = -1.f;
	FString PreviewedBehaviour;
	FString CheckedBehaviour;
	float CheckAgeS = -1.f;
	FJobResult PreviewRes, CheckRes, RunRes, MonitorRes;
	FTelemetryReadout VirtualTel, RealTel;
	TArray<FBehaviourInfo> Behaviours;
	TArray<FJointMeta> JointMetas;
	bool bTwinResolved = false;
	TArray<TWeakObjectPtr<UObject>> TwinJointObjs;
	TWeakObjectPtr<UObject> TwinRootBody;
	bool bShowDetails = false;
	bool bShowJoints = true;
	// Clicks: HUD hit boxes depend on PlayerController click routing, which the follow camera and URLab's widget
	// change at runtime, so the panel hit-tests its own button rectangles from a Slate input pre-processor.
	TSharedPtr<class IInputProcessor> ClickProcessor;
	bool bClickQueued = false;
	TArray<TPair<FName, FBox2D>> ButtonRects;   // drawn last frame, in canvas pixels (topmost last)
	TArray<TPair<FName, FBox2D>> NextRects;     // being drawn this frame
	// click feedback: the clicked button flashes, a message bar confirms what happened
	FName PressedId;
	double PressedAt = -10.0;
	FString Toast;
	FLinearColor ToastColor = FLinearColor::White;
	double ToastAt = -10.0;
	TMap<FName, FString> DisabledWhy;   // buttons drawn disabled this frame -> why (shown when clicked)
	TMap<FName, FString> ButtonLabel;
	FString RunLockedWhy;               // why "Move REAL robot" is locked (set each frame in DrawHUD)
	void ShowToast(const FString& Text, const FLinearColor& Color);
	bool bUrlabDebugHidden = false;
	TArray<FString> Lines;
	FString LastError;

	bool bConfirmOpen = false;
	double ConfirmOpenedAt = 0.0;
	double NextPoll = 0.0;
	bool bPollInFlight = false;

	// ---------------------------------------------------------------- tabs + layout
	ER1PanelTab Tab = ER1PanelTab::Behaviours;
	float PanelScale = 1.f;        // the whole panel is drawn scaled (portrait windows); mouse is mapped back
	float LastPanelH = 640.f;      // measured content height (panel background uses last frame's)
	float ViewW = 1600.f, ViewH = 900.f;   // canvas size in panel units
	bool GetPanelMouse(float& OutX, float& OutY) const;
	void SetTab(ER1PanelTab NewTab);
	void DrawTabStrip(float X0, float X, float Inner, float PanelW, float& Y);
	void DrawWrapped(const FString& Text, float X, float& Y, float MaxW, const FLinearColor& Color, float Scale = 0.9f);
	/** POST JSON to the service; OnDone gets the HTTP code (0 = unreachable) and the parsed body (may be null). */
	void PostJson(const FString& Path, const FString& Body, TFunction<void(int32, TSharedPtr<FJsonObject>)> OnDone = nullptr);
	void GetJson(const FString& Path, TFunction<void(int32, const FString&)> OnDone);
	bool IsWalkRunning() const { return WalkState.bRunning; }
	FString StatusProgressJob;
	int32 ProgressDone = 0, ProgressTotal = 0;

	// ---------------------------------------------------------------- voice (R1PanelVoice.cpp)
	TUniquePtr<FR1VoiceSapi> Voice;
	bool bMicOn = false;
	bool bPushToTalkHeld = false;
	FString HeardText;
	double HeardAt = -100.0;
	FR1Intent LastIntent;
	double IntentAt = -100.0;
	FString IntentError;
	int32 IntentInFlight = 0;
	FName HighlightJoint;
	double HighlightAt = -100.0;
	void TickVoice();
	void SendInterpret(const FString& Text);
	void SendInterpretTo(const FString& Text, const FString& Path);
	void ApplyIntent(const FR1Intent& Intent);
	void DrawVoiceLine(float X, float Inner, float& Y);
	bool HandleVoiceClick(FName Id);

	// ---------------------------------------------------------------- robustness (R1PanelRobustness.cpp)
	FR1RobustReport Robust;
	bool bRobustFetchInFlight = false;
	double NextRobustFetch = 0.0;
	bool bRobustJobWasRunning = false;
	int32 SelKind = -1, SelRow = -1, SelCol = -1;   // selected cell: 0 = sensitivity, 1 = interaction
	void TickRobustness(double Now);
	void FetchRobustness();
	void DrawRobustnessTab(float X, float Inner, float& Y);
	void DrawRobustCell(FName Id, const FR1RobustCell& C, const FString& Top, float X, float Y, float W, float H, bool bNominal, bool bSelected, bool bThreeLines);
	bool HandleRobustClick(FName Id);

	// ---------------------------------------------------------------- walk sandbox (R1PanelWalk.cpp)
	FR1WalkState WalkState;
	bool bWalkPollInFlight = false;
	double NextWalkPoll = 0.0;
	FVector VoiceWalkCmd = FVector::ZeroVector;
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

	// ---------------------------------------------------------------- operator console (R1PanelConsole.cpp)
	// Health / Motions / Setup / Help tabs and the global E-STOP (docs/service_api.md sections 4-7). The replies are
	// kept as parsed JSON objects and drawn directly; poses, motions and modes only ever target the twin ("urlab").
	TSharedPtr<FJsonObject> HealthJ, VersionsJ, EventsJ, PowerJ, TempsJ, ImuJ;
	TSharedPtr<FJsonObject> PosesJ, MotionsJ, PlayerJ, LimitsJ, RecordedJ;
	TSharedPtr<FJsonObject> PreflightJ, CalibJ, CalibSavedJ, NetworkJ, DocsJ;
	TSet<FString> ConsoleInFlight;
	TMap<FString, double> ConsoleNextFetch;
	FString EventLevel = TEXT("info");
	FString RecordSource = TEXT("practice");
	FString CalibSource = TEXT("practice");
	FString CalibReference = TEXT("zero");
	bool bCalibConfirm = false;
	int32 ModeJointIdx = 22;   // right_elbow_joint
	bool bMotionLoop = false;
	TMap<FString, FString> NetTestResult;   // profile name -> one line
	FString DocName, DocTitle, DocMarkdown;
	TArray<TPair<FString, FLinearColor>> DocWrapped;
	float DocScroll = 0.f;
	float WheelQueued = 0.f;
	FString PendingPose, PendingPoseLabel;   // offered by the assistant; previewed only on click
	double PendingPoseAt = -100.0;
	bool bEStopKeyWasDown = false;
	UPROPERTY(Transient)
	TObjectPtr<UTexture2D> CameraTex;
	bool bCameraInFlight = false;
	double NextCameraFetch = 0.0;
	FString CameraNote;
	void TickConsole(double Now);
	void FetchConsole(const FString& Path, TSharedPtr<FJsonObject>* Slot, double Now, double Every);
	void FetchCamera();
	void ConsolePost(const FString& Path, const FString& Body, const FString& OkToast, TFunction<void(TSharedPtr<FJsonObject>)> OnOk = nullptr);
	void SendEStop();
	void OpenDoc(const FString& Name);
	void DrawEStop(float X, float Y, float W, float H);
	void DrawHealthTab(float X, float Inner, float& Y);
	void DrawMotionsTab(float X, float Inner, float& Y);
	void DrawSetupTab(float X, float Inner, float& Y);
	void DrawHelpTab(float X, float Inner, float& Y);
	void DrawSection(const FString& Title, float X, float Inner, float& Y, const FLinearColor& Accent);
	bool HandleConsoleClick(FName Id);
	void DrawPendingPose(float X, float Inner, float& Y);

	// ---------------------------------------------------------------- unattended capture (-PanelCapture=<dir>)
	FString PanelCaptureDir;
	FString PanelCaptureTag;
	FString VoiceTestText;   // -VoiceTest=<phrase with underscores>: injected as heard speech during -PanelCapture
	double CaptureStartAt = -1.0;
	int32 CaptureStep = 0;
	void TickPanelCapture(double Now);
	void CaptureShot(const TCHAR* Name);
};
