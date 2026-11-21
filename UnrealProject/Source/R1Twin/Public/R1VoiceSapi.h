// Offline voice I/O for the behaviour panel (idea #11): Windows SAPI speech recognition + text to speech.
//
// One background thread owns every COM object (MTA): an in-process recogniser with a dictation grammar on the
// default microphone, and an ISpVoice for the spoken replies. The game thread only flips SetListening(), drains
// recognised phrases with PopHeard() and queues replies with Speak(). Phrases heard while the voice is speaking
// are dropped so the twin does not answer itself. Non-Windows builds compile to a stub that reports "unavailable".
//
// Voice never starts anything by itself: the panel turns a phrase into a suggestion via the behaviour service
// (POST /assistant/interpret) and fills in the selection exactly as a click would.
#pragma once

#include "CoreMinimal.h"
#include "HAL/Runnable.h"
#include <atomic>

class FRunnableThread;

class R1TWIN_API FR1VoiceSapi : public FRunnable
{
public:
	FR1VoiceSapi() = default;
	virtual ~FR1VoiceSapi() override;

	/** Starts the COM thread (idempotent). */
	void Start();
	/** Stops and joins the thread. */
	void Shutdown();

	/** Microphone on (recogniser active) or off. */
	void SetListening(bool bListen) { bWantListen = bListen; }
	bool IsListening() const { return bListening; }
	bool IsRecognizerReady() const { return bRecoReady; }
	bool IsVoiceReady() const { return bVoiceReady; }
	bool IsSpeaking() const { return bSpeaking; }
	bool IsStarted() const { return Thread != nullptr; }

	/** Next recognised phrase, oldest first. */
	bool PopHeard(FString& Out);
	/** Queue a spoken reply (async, interrupts the previous one). */
	void Speak(const FString& Text);

	/** Recogniser problem in plain words ("no microphone", ...), empty when fine. */
	FString GetRecognizerError() const;
	FString GetVoiceError() const;

	// FRunnable
	virtual uint32 Run() override;
	virtual void Stop() override { bStopRequested = true; }

private:
	FRunnableThread* Thread = nullptr;
	std::atomic<bool> bStopRequested{false};
	std::atomic<bool> bWantListen{false};
	std::atomic<bool> bListening{false};
	std::atomic<bool> bRecoReady{false};
	std::atomic<bool> bVoiceReady{false};
	std::atomic<bool> bSpeaking{false};

	mutable FCriticalSection Lock;
	TArray<FString> Heard;
	TArray<FString> ToSay;
	FString RecoError;
	FString VoiceError;
};
