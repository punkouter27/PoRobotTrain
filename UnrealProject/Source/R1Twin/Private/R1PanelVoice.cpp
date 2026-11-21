// Behaviour panel: voice commands + spoken replies (idea #11, docs/service_api.md section 1).
// Windows SAPI turns speech into text (FR1VoiceSapi); POST /assistant/interpret turns text into a suggestion;
// the suggestion fills in the selection exactly as a click would. Nothing is started, checked or run by voice.
// The one exception is the walk sandbox (simulation only), where a walk phrase sets the velocity command.
#include "R1BehaviourPanel.h"
#include "R1Twin.h"
#include "R1PanelStyle.h"
#include "R1VoiceSapi.h"

#include "GameFramework/PlayerController.h"
#include "HttpModule.h"
#include "Interfaces/IHttpResponse.h"

using namespace R1PanelStyle;

void AR1BehaviourPanel::TickVoice()
{
	if (!Voice)
	{
		return;
	}
	APlayerController* PC = GetOwningPlayerController();
	bPushToTalkHeld = PC && PushToTalkKey.IsValid() && PC->IsInputKeyDown(PushToTalkKey);
	Voice->SetListening(bMicOn || bPushToTalkHeld);
	FString Phrase;
	while (Voice->PopHeard(Phrase))
	{
		HeardText = Phrase;
		HeardAt = FPlatformTime::Seconds();
		UE_LOG(LogR1Twin, Log, TEXT("R1BehaviourPanel: heard \"%s\""), *Phrase);
		SendInterpret(Phrase);
	}
}

void AR1BehaviourPanel::SendInterpret(const FString& Text)
{
	SendInterpretTo(Text, TEXT("/assistant/chat"));
}

void AR1BehaviourPanel::SendInterpretTo(const FString& Text, const FString& Path)
{
	// /assistant/chat is a superset of /assistant/interpret (status answers, poses, follow-ups with memory);
	// an older service without it gets the same request on /assistant/interpret.
	const FString Body = FString::Printf(TEXT("{\"text\":\"%s\",\"session\":\"unreal\",\"arm_unlocked\":true,\"walk_active\":%s}"),
		*R1Panel::JsonEscape(Text), Tab == ER1PanelTab::Walk ? TEXT("true") : TEXT("false"));
	TSharedRef<IHttpRequest, ESPMode::ThreadSafe> Req = FHttpModule::Get().CreateRequest();
	Req->SetURL(ServiceUrl + Path);
	Req->SetVerb(TEXT("POST"));
	Req->SetHeader(TEXT("Content-Type"), TEXT("application/json"));
	Req->SetContentAsString(Body);
	Req->SetTimeout(5.f);
	++IntentInFlight;
	IntentError.Reset();
	Req->OnProcessRequestComplete().BindWeakLambda(this, [this, Text, Path](FHttpRequestPtr, FHttpResponsePtr Resp, bool bOk)
	{
		IntentInFlight = FMath::Max(0, IntentInFlight - 1);
		FR1Intent Intent;
		if (!bOk || !Resp.IsValid())
		{
			IntentError = TEXT("behaviour service not reachable");
		}
		else if (Resp->GetResponseCode() == 404 && Path == TEXT("/assistant/chat"))
		{
			SendInterpretTo(Text, TEXT("/assistant/interpret"));   // older service
			return;
		}
		else if (Resp->GetResponseCode() == 404)
		{
			IntentError = TEXT("this behaviour service has no /assistant/interpret (update RobotControl)");
		}
		else if (!R1Panel::ParseIntent(Resp->GetContentAsString(), Intent))
		{
			IntentError = FString::Printf(TEXT("unreadable reply (HTTP %d)"), Resp->GetResponseCode());
		}
		else
		{
			ApplyIntent(Intent);
			return;
		}
		ShowToast(FString::Printf(TEXT("Voice: %s"), *IntentError), BadCol);
	});
	Req->ProcessRequest();
}

void AR1BehaviourPanel::ApplyIntent(const FR1Intent& Intent)
{
	LastIntent = Intent;
	IntentAt = FPlatformTime::Seconds();
	const bool bIdle = bServiceOnline && Job.IsEmpty() && !WalkState.bRunning;
	FString Say = Intent.Say;

	if (Intent.Kind == TEXT("behaviour") && !Intent.Id.IsEmpty())
	{
		if (Intent.bBlocked)
		{
			ShowToast(Intent.Note, WarnCol);
		}
		else if (!bIdle)
		{
			ShowToast(FString::Printf(TEXT("Heard %s, but the selection is locked while something runs. Nothing changed."), *Intent.Label), WarnCol);
			Say = TEXT("Something is running. I did not change the selection.");
		}
		else
		{
			// exactly what clicking the behaviour chip and the size chip does: select, never start
			Behaviour = Intent.Id;
			if (Intent.bHasAmplitude && Intent.Amplitude > 0.f)
			{
				AmplitudeDeg = FMath::Clamp(FMath::RoundToFloat(Intent.Amplitude / 10.f) * 10.f, 10.f, 30.f);
			}
			SetTab(ER1PanelTab::Behaviours);
			ShowToast(Intent.Note, TextCol);
		}
	}
	else if (Intent.Kind == TEXT("joint") && !Intent.Joint.IsEmpty())
	{
		HighlightJoint = FName(*Intent.Joint);
		HighlightAt = FPlatformTime::Seconds();
		if (ViewW >= 20.f + 600.f + 20.f + 540.f)
		{
			bShowJoints = true;
			if (bShowDetails)
			{
				bShowDetails = false;
				SetUrlabDebugVisible(false);
			}
		}
		ShowToast(FString::Printf(TEXT("%s (highlighted in the joint view)"), *Intent.Note), CyanCol);
	}
	else if (Intent.Kind == TEXT("walk") && Intent.bHasWalk)
	{
		if (!WalkState.bRunning)
		{
			ShowToast(TEXT("Start the walk sandbox first (Walk tab)."), WarnCol);
			Say = TEXT("Start the walk sandbox first.");
		}
		else if (WalkState.Mode == TEXT("hold"))
		{
			ShowToast(TEXT("Hold pose ignores velocity commands (no walking policy)."), WarnCol);
			Say = TEXT("Hold pose cannot walk.");
		}
		else
		{
			VoiceWalkCmd = Intent.bWalkStop ? FVector::ZeroVector : WalkState.ClampCmd(Intent.WalkCmd);
			ShowToast(Intent.Note, CyanCol);
		}
	}
	else if (Intent.Kind == TEXT("answer"))
	{
		// a status question ("how hot are the motors?"): show and speak the answer, change nothing
		ShowToast(Intent.Answer.IsEmpty() ? Intent.Note : Intent.Answer, CyanCol);
	}
	else if (Intent.Kind == TEXT("pose") && !Intent.Pose.IsEmpty())
	{
		// offer, never start: the "Preview pose" button under the voice line previews it on the twin only
		PendingPose = Intent.Pose;
		PendingPoseLabel = Intent.Label.IsEmpty() ? R1Panel::PrettyJoint(Intent.Pose) : Intent.Label;
		PendingPoseAt = FPlatformTime::Seconds();
		ShowToast(FString::Printf(TEXT("%s  Click \"Preview pose\" to show it on the twin."), *Intent.Note), CyanCol);
	}
	else
	{
		ShowToast(Intent.Note.IsEmpty() ? FString(TEXT("No stand-safe motion matches that.")) : Intent.Note, WarnCol);
	}
	if (bSpeakReplies && Voice && !Say.IsEmpty())
	{
		Voice->Speak(Say);
	}
}

void AR1BehaviourPanel::DrawVoiceLine(float X, float Inner, float& Y)
{
	const double Now = FPlatformTime::Seconds();
	const FString Example = Tab == ER1PanelTab::Walk ? TEXT("\"walk forward\", \"turn left\", \"stop\"")
		: TEXT("\"nod the head twenty degrees\"");
	if (Voice && Voice->IsListening())
	{
		DrawWrapped(FString::Printf(TEXT("Listening... say e.g. %s"), *Example), X, Y, Inner, CyanCol, 0.88f);
	}
	else if (Now - HeardAt < 15.0)
	{
		DrawWrapped(FString::Printf(TEXT("Heard: \"%s\""), *HeardText), X, Y, Inner, TextCol, 0.88f);
		if (IntentInFlight > 0)
		{
			DrawLine(TEXT("Interpreting..."), X, Y, DimCol, 0.85f);
		}
		else if (!IntentError.IsEmpty())
		{
			DrawWrapped(IntentError, X, Y, Inner, BadCol, 0.85f);
		}
		else if (IntentAt >= HeardAt)
		{
			const bool bWarn = LastIntent.Kind == TEXT("unknown") || LastIntent.bBlocked;
			DrawWrapped(FString::Printf(TEXT("-> %s"), *LastIntent.Note), X, Y, Inner, bWarn ? WarnCol : OkCol, 0.85f);
		}
	}
	else if (Voice && !Voice->IsRecognizerReady() && !Voice->GetRecognizerError().IsEmpty())
	{
		DrawWrapped(FString::Printf(TEXT("Voice input unavailable: %s"), *Voice->GetRecognizerError()), X, Y, Inner, WarnCol, 0.85f);
	}
	else
	{
		DrawWrapped(FString::Printf(TEXT("Voice: click Mic or hold %s and say e.g. %s. It only suggests; it never starts a run."),
			*PushToTalkKey.GetDisplayName().ToString(), *Example), X, Y, Inner, DimCol, 0.85f);
	}
}

bool AR1BehaviourPanel::HandleVoiceClick(FName Id)
{
	if (Id == IdMic)
	{
		bMicOn = !bMicOn;
		ShowToast(bMicOn ? TEXT("Microphone on: speak a command.") : TEXT("Microphone off."), bMicOn ? CyanCol : DimCol);
		return true;
	}
	if (Id == IdMute)
	{
		bSpeakReplies = !bSpeakReplies;
		ShowToast(bSpeakReplies ? TEXT("Spoken replies on.") : TEXT("Spoken replies muted."), DimCol);
		return true;
	}
	return false;
}
