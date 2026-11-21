// Behaviour panel: on-screen buttons and stand-safe telemetry HUD that drive RobotControl's
// behaviour service (Python, 127.0.0.1:8765).
//
// Five HUD anchors:
//   1. VIRTUAL ROBOT (on screen)     [Move VIRTUAL robot]                     <- blue, safe
//   2. REAL ROBOT    (Check/Monitor) [Check] [Monitor]                        <- orange, read-only
//   3. REAL ROBOT    (Move REAL)     [Move REAL robot]                        <- orange, gated on 1+2+confirm
//   4. [STOP]                        remote L2+B 5 s = emergency stop
//   5. [Details]                     toggles URLab debug panels and raw service output
#pragma once

#include "CoreMinimal.h"
#include "GameFramework/HUD.h"
#include "Interfaces/IHttpRequest.h"
#include "Dom/JsonObject.h"
#include "R1BehaviourPanel.generated.h"

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

	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type Reason) override;
	virtual void DrawHUD() override;
	/** Called by the Slate input pre-processor on every left mouse press (before game / UMG input routing). */
	void QueueClick() { bClickQueued = true; }

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
};
