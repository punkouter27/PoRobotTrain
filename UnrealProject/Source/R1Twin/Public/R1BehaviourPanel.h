// Behaviour panel: on-screen buttons that drive RobotControl's behaviour service (Python, 127.0.0.1:8765).
//
//   HEAD TURN                     turn size [10] [20] [30] deg
//   VIRTUAL ROBOT (on screen)     [Move VIRTUAL robot]                       <- blue, safe
//   REAL ROBOT (Unitree R1)       [Check connection] [Move REAL robot]       <- orange, needs steps 1 + 2
//   [STOP]   remote L2+B 5 s = emergency stop                          [Details]
//
// URLab's own debug panels are hidden on start (Details or Tab shows them, plus the raw service output).
//
// The service does the work: Preview runs the behaviour on this Unreal twin (URLab direct mode); Check runs
// it read-only on the robot; Run powers ONLY the behaviour's joints on the robot and mirrors the measured
// angles back onto this twin. "Run on robot" is enabled only when the service says so (same settings
// previewed + fresh read-only check), and needs a second click on a confirmation box.
// STOP is a convenience over the network; the remote (L2+B for 5 s) is the emergency stop.
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
	struct FHeadReadout
	{
		bool bValid = false, bHasPitch = false, bHasTarget = false;
		float Yaw = 0.f, Pitch = 0.f, Target = 0.f, Other = 0.f, AgeS = 0.f;
		FString Source, Phase;
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
	static FJobResult ReadResult(const TSharedPtr<FJsonObject>& Results, const TCHAR* Job);
	static float ReadAmplitude(const TSharedPtr<FJsonObject>& J, const TCHAR* Field);
	static FHeadReadout ReadHead(const TSharedPtr<FJsonObject>& Head, const TCHAR* Who);
	void DrawHead(const FHeadReadout& H, bool bLive, const FString& NoData, const FLinearColor& Accent, float X, float W, float& Y);

	// latest service status
	bool bServiceOnline = false;
	FString Job;
	FString LastResult;
	FString WhyNotRun;
	bool bCanRunRobot = false;
	int32 MirrorSamples = 0;
	float PreviewedAmp = -1.f;     // amplitude of the last clean twin preview (-1: none)
	float CheckedAmp = -1.f;       // amplitude of the last passed read-only check (-1: none)
	float CheckAgeS = -1.f;
	FJobResult PreviewRes, CheckRes, RunRes;
	FHeadReadout VirtualHead, RealHead;
	bool bShowDetails = false;
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
