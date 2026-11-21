#include "R1VoiceSapi.h"
#include "R1Twin.h"

#include "HAL/PlatformProcess.h"
#include "HAL/PlatformTime.h"
#include "HAL/RunnableThread.h"
#include "Misc/ScopeLock.h"

#if PLATFORM_WINDOWS
#include "Windows/AllowWindowsPlatformTypes.h"
THIRD_PARTY_INCLUDES_START
#include <sapi.h>
#include <sperror.h>
THIRD_PARTY_INCLUDES_END
#include "Windows/HideWindowsPlatformTypes.h"
#endif

FR1VoiceSapi::~FR1VoiceSapi()
{
	Shutdown();
}

void FR1VoiceSapi::Start()
{
	if (Thread)
	{
		return;
	}
	bStopRequested = false;
	Thread = FRunnableThread::Create(this, TEXT("R1VoiceSapi"), 0, TPri_BelowNormal);
}

void FR1VoiceSapi::Shutdown()
{
	if (Thread)
	{
		Stop();
		Thread->WaitForCompletion();
		delete Thread;
		Thread = nullptr;
	}
}

bool FR1VoiceSapi::PopHeard(FString& Out)
{
	FScopeLock L(&Lock);
	if (Heard.Num() == 0)
	{
		return false;
	}
	Out = Heard[0];
	Heard.RemoveAt(0);
	return true;
}

void FR1VoiceSapi::Speak(const FString& Text)
{
	if (Text.IsEmpty())
	{
		return;
	}
	FScopeLock L(&Lock);
	ToSay.Reset();
	ToSay.Add(Text);
}

FString FR1VoiceSapi::GetRecognizerError() const
{
	FScopeLock L(&Lock);
	return RecoError;
}

FString FR1VoiceSapi::GetVoiceError() const
{
	FScopeLock L(&Lock);
	return VoiceError;
}

#if PLATFORM_WINDOWS
namespace
{
	template <typename T>
	void SafeRelease(T*& P)
	{
		if (P)
		{
			P->Release();
			P = nullptr;
		}
	}

	/** Default token of a SAPI category (microphone, recogniser) without sphelper.h / ATL. */
	ISpObjectToken* DefaultToken(const WCHAR* CategoryId)
	{
		ISpObjectTokenCategory* Cat = nullptr;
		ISpObjectToken* Tok = nullptr;
		LPWSTR Id = nullptr;
		if (SUCCEEDED(CoCreateInstance(__uuidof(SpObjectTokenCategory), nullptr, CLSCTX_ALL, __uuidof(ISpObjectTokenCategory), reinterpret_cast<void**>(&Cat)))
			&& SUCCEEDED(Cat->SetId(CategoryId, 0))
			&& SUCCEEDED(Cat->GetDefaultTokenId(&Id)) && Id
			&& SUCCEEDED(CoCreateInstance(__uuidof(SpObjectToken), nullptr, CLSCTX_ALL, __uuidof(ISpObjectToken), reinterpret_cast<void**>(&Tok))))
		{
			if (FAILED(Tok->SetId(nullptr, Id, 0)))
			{
				SafeRelease(Tok);
			}
		}
		if (Id)
		{
			CoTaskMemFree(Id);
		}
		SafeRelease(Cat);
		return Tok;
	}

	/** SAPI events own their lParam (COM object or CoTaskMem string/pointer). */
	void FreeEvent(SPEVENT& Ev)
	{
		switch (Ev.elParamType)
		{
		case SPET_LPARAM_IS_OBJECT:
		case SPET_LPARAM_IS_TOKEN:
			if (Ev.lParam)
			{
				reinterpret_cast<IUnknown*>(Ev.lParam)->Release();
			}
			break;
		case SPET_LPARAM_IS_POINTER:
		case SPET_LPARAM_IS_STRING:
			if (Ev.lParam)
			{
				CoTaskMemFree(reinterpret_cast<void*>(Ev.lParam));
			}
			break;
		default:
			break;
		}
		Ev.lParam = 0;
	}

	FString Explain(const TCHAR* Step, HRESULT Hr)
	{
		if (Hr == SPERR_NOT_FOUND || Hr == REGDB_E_CLASSNOTREG)
		{
			return FString::Printf(TEXT("%s: not installed (Windows Settings > Time & language > Speech)"), Step);
		}
		return FString::Printf(TEXT("%s failed (0x%08x)"), Step, static_cast<uint32>(Hr));
	}
}
#endif

uint32 FR1VoiceSapi::Run()
{
#if PLATFORM_WINDOWS
	const HRESULT HrInit = CoInitializeEx(nullptr, COINIT_MULTITHREADED);
	ISpVoice* Voice = nullptr;
	ISpRecognizer* Rec = nullptr;
	ISpRecoContext* Ctx = nullptr;
	ISpRecoGrammar* Gram = nullptr;
	FString RErr, VErr;

	HRESULT Hr = CoCreateInstance(__uuidof(SpVoice), nullptr, CLSCTX_ALL, __uuidof(ISpVoice), reinterpret_cast<void**>(&Voice));
	if (FAILED(Hr))
	{
		VErr = Explain(TEXT("text to speech"), Hr);
		SafeRelease(Voice);
	}
	else
	{
		bVoiceReady = true;
	}

	Hr = CoCreateInstance(__uuidof(SpInprocRecognizer), nullptr, CLSCTX_ALL, __uuidof(ISpRecognizer), reinterpret_cast<void**>(&Rec));
	if (FAILED(Hr))
	{
		RErr = Explain(TEXT("speech recogniser"), Hr);
	}
	else
	{
		if (ISpObjectToken* EngineTok = DefaultToken(SPCAT_RECOGNIZERS))
		{
			Hr = Rec->SetRecognizer(EngineTok);
			EngineTok->Release();
			if (FAILED(Hr))
			{
				RErr = Explain(TEXT("speech recogniser engine"), Hr);
			}
		}
		else
		{
			RErr = TEXT("no speech recogniser installed (Windows Settings > Time & language > Speech)");
		}
		if (RErr.IsEmpty())
		{
			if (ISpObjectToken* AudioTok = DefaultToken(SPCAT_AUDIOIN))
			{
				Hr = Rec->SetInput(AudioTok, 1);
				AudioTok->Release();
				if (FAILED(Hr))
				{
					RErr = Explain(TEXT("microphone"), Hr);
				}
			}
			else
			{
				RErr = TEXT("no microphone found");
			}
		}
		if (RErr.IsEmpty() && FAILED(Hr = Rec->CreateRecoContext(&Ctx)))
		{
			RErr = Explain(TEXT("recognition context"), Hr);
		}
		if (RErr.IsEmpty() && (FAILED(Hr = Ctx->SetNotifyWin32Event())
			|| FAILED(Hr = Ctx->SetInterest(SPFEI(SPEI_RECOGNITION), SPFEI(SPEI_RECOGNITION)))))
		{
			RErr = Explain(TEXT("recognition events"), Hr);
		}
		if (RErr.IsEmpty() && (FAILED(Hr = Ctx->CreateGrammar(1, &Gram))
			|| FAILED(Hr = Gram->LoadDictation(nullptr, SPLO_STATIC))
			|| FAILED(Hr = Gram->SetDictationState(SPRS_ACTIVE))))
		{
			RErr = Explain(TEXT("dictation grammar"), Hr);
		}
		if (RErr.IsEmpty())
		{
			Rec->SetRecoState(SPRST_INACTIVE);
			bRecoReady = true;
		}
	}
	{
		FScopeLock L(&Lock);
		RecoError = RErr;
		VoiceError = VErr;
	}
	UE_LOG(LogR1Twin, Log, TEXT("R1VoiceSapi: recogniser %s, voice %s"),
		RErr.IsEmpty() ? TEXT("ready") : *RErr, VErr.IsEmpty() ? TEXT("ready") : *VErr);

	double QuietUntil = 0.0;
	while (!bStopRequested)
	{
		if (bRecoReady && bWantListen.load() != bListening.load())
		{
			const bool bOn = bWantListen.load();
			if (SUCCEEDED(Rec->SetRecoState(bOn ? SPRST_ACTIVE : SPRST_INACTIVE)))
			{
				bListening = bOn;
			}
		}

		TArray<FString> Say;
		{
			FScopeLock L(&Lock);
			Swap(Say, ToSay);
		}
		if (Voice)
		{
			for (const FString& S : Say)
			{
				Voice->Speak(*S, SPF_ASYNC | SPF_PURGEBEFORESPEAK | SPF_IS_NOT_XML, nullptr);
			}
			SPVOICESTATUS St = {};
			if (SUCCEEDED(Voice->GetStatus(&St, nullptr)))
			{
				const bool bNow = St.dwRunningState == SPRS_IS_SPEAKING || Say.Num() > 0;
				if (bNow)
				{
					QuietUntil = FPlatformTime::Seconds() + 0.4;   // ignore the tail of our own reply
				}
				bSpeaking = bNow;
			}
		}

		if (Ctx)
		{
			Ctx->WaitForNotifyEvent(50);
			SPEVENT Ev = {};
			ULONG Fetched = 0;
			while (SUCCEEDED(Ctx->GetEvents(1, &Ev, &Fetched)) && Fetched == 1)
			{
				if (Ev.eEventId == SPEI_RECOGNITION && Ev.elParamType == SPET_LPARAM_IS_OBJECT && Ev.lParam)
				{
					ISpRecoResult* Result = reinterpret_cast<ISpRecoResult*>(Ev.lParam);
					LPWSTR Text = nullptr;
					if (SUCCEEDED(Result->GetText(SP_GETWHOLEPHRASE, SP_GETWHOLEPHRASE, 1, &Text, nullptr)) && Text)
					{
						const FString Phrase = FString(Text).TrimStartAndEnd();
						CoTaskMemFree(Text);
						if (!Phrase.IsEmpty() && bListening && FPlatformTime::Seconds() > QuietUntil)
						{
							FScopeLock L(&Lock);
							Heard.Add(Phrase);
						}
					}
				}
				FreeEvent(Ev);
				Fetched = 0;
			}
		}
		else
		{
			FPlatformProcess::Sleep(0.05f);
		}
	}

	if (Rec)
	{
		Rec->SetRecoState(SPRST_INACTIVE);
	}
	if (Voice)
	{
		Voice->Speak(nullptr, SPF_PURGEBEFORESPEAK, nullptr);
	}
	SafeRelease(Gram);
	SafeRelease(Ctx);
	SafeRelease(Rec);
	SafeRelease(Voice);
	bListening = false;
	bRecoReady = false;
	bVoiceReady = false;
	if (SUCCEEDED(HrInit))
	{
		CoUninitialize();
	}
#else
	{
		FScopeLock L(&Lock);
		RecoError = TEXT("voice input needs Windows (SAPI)");
		VoiceError = TEXT("voice output needs Windows (SAPI)");
	}
	while (!bStopRequested)
	{
		FPlatformProcess::Sleep(0.1f);
	}
#endif
	return 0;
}
