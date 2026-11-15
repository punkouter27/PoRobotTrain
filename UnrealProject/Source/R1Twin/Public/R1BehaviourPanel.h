// Behaviour panel: on-screen buttons that drive RobotControl's behaviour service (Python, 127.0.0.1:8765).
//
//   [Preview in twin]  [Check robot (read-only)]  [Run on robot]  [STOP]      amplitude: [10] [20] [30] deg
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
	virtual void DrawHUD() override;
	virtual void NotifyHitBoxClick(FName BoxName) override;

private:
	struct FButton { FName Id; FString Label; FVector2D Pos; FVector2D Size; bool bEnabled; FLinearColor Color; };

	void PollStatus();
	void Post(const FString& Path, bool bConfirm = false);
	void OnStatus(FHttpRequestPtr Req, FHttpResponsePtr Resp, bool bOk);
	void OnPost(FHttpRequestPtr Req, FHttpResponsePtr Resp, bool bOk);
	FString ParamsJson(bool bConfirm) const;
	void DrawButton(const FButton& B);
	void DrawLine(const FString& Text, float X, float& Y, const FLinearColor& Color, float Scale = 1.f);

	// latest service status
	bool bServiceOnline = false;
	FString Job;
	FString LastResult;
	FString WhyNotRun;
	bool bCanRunRobot = false;
	int32 MirrorSamples = 0;
	TArray<FString> Lines;
	FString LastError;

	bool bConfirmOpen = false;
	double ConfirmOpenedAt = 0.0;
	double NextPoll = 0.0;
	bool bPollInFlight = false;
};
