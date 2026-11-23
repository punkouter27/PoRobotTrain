#include "R1MotorAudio.h"

#include "Sound/SoundGenerator.h"

namespace
{
	/** Runs on the audio render thread only. Reads the atomics once per buffer; no allocation, no locks. */
	class FR1MotorSynthGenerator final : public ISoundGenerator
	{
	public:
		FR1MotorSynthGenerator(const FSoundGeneratorInitParams& In, TSharedRef<FR1MotorSynthParams, ESPMode::ThreadSafe> InParams)
			: Params(MoveTemp(InParams))
			, SampleRate(In.SampleRate > 0.f ? In.SampleRate : 48000.f)
			, Channels(FMath::Max(1, In.NumChannels))
		{
			SeenClicks = Params->Clicks.load(std::memory_order_relaxed);
			SeenThuds = Params->Thuds.load(std::memory_order_relaxed);
		}

		virtual int32 OnGenerateAudio(float* Out, int32 NumSamples) override
		{
			const float Volume = FMath::Clamp(Params->Volume.load(std::memory_order_relaxed), 0.f, 1.f);
			const float SpeedT = FMath::Clamp(Params->Speed.load(std::memory_order_relaxed), 0.f, 1.f);
			const float LoadT = FMath::Clamp(Params->Load.load(std::memory_order_relaxed), 0.f, 1.f);
			const bool bAlert = Params->bAlert.load(std::memory_order_relaxed);
			const uint32 Clicks = Params->Clicks.load(std::memory_order_acquire);
			const uint32 Thuds = Params->Thuds.load(std::memory_order_acquire);
			if (Clicks != SeenClicks)
			{
				SeenClicks = Clicks;
				ClickEnv = 1.f;
			}
			if (Thuds != SeenThuds)
			{
				SeenThuds = Thuds;
				ThudEnv = FMath::Clamp(Params->ThudStrength.load(std::memory_order_relaxed), 0.1f, 1.f);
				ThudPhase = 0.f;
			}

			const float Dt = 1.f / SampleRate;
			const float Smooth = 1.f - FMath::Exp(-Dt / 0.04f);        // 40 ms parameter glide (no zipper noise)
			const float ClickDecay = FMath::Exp(-Dt / 0.004f);
			const float ThudDecay = FMath::Exp(-Dt / 0.12f);
			const int32 Frames = NumSamples / Channels;
			for (int32 f = 0; f < Frames; ++f)
			{
				Speed += Smooth * (SpeedT - Speed);
				Load += Smooth * (LoadT - Load);
				Gain += Smooth * (Volume - Gain);

				// whine: 180 Hz idle .. ~1.6 kHz at full speed, with a load-dependent second harmonic
				const float Freq = 180.f + 1400.f * FMath::Pow(Speed, 0.7f);
				WhinePhase = FMath::Fmod(WhinePhase + Freq * Dt, 1.f);
				const float Ph = 2.f * PI * WhinePhase;
				const float Whine = (FMath::Sin(Ph) + 0.35f * FMath::Sin(2.f * Ph) + 0.15f * FMath::Sin(3.f * Ph)) * (0.16f * Speed + 0.02f * Load);

				// load: a 100 Hz buzz that grows with the torque fraction (a motor holding hard)
				HumPhase = FMath::Fmod(HumPhase + 100.f * Dt, 1.f);
				const float Hum = (HumPhase < 0.5f ? 1.f : -1.f) * 0.05f * Load * Load + FMath::Sin(2.f * PI * HumPhase) * 0.04f * Load;

				// click: a 4 ms burst of noise
				const float Click = ClickEnv > 1e-3f ? Noise() * 0.35f * ClickEnv : 0.f;
				ClickEnv *= ClickDecay;

				// thud: a falling 70 -> 40 Hz sine with a little noise, 120 ms decay
				float Thud = 0.f;
				if (ThudEnv > 1e-3f)
				{
					ThudPhase += (40.f + 30.f * ThudEnv) * Dt;
					Thud = (FMath::Sin(2.f * PI * ThudPhase) * 0.6f + Noise() * 0.08f) * ThudEnv;
					ThudEnv *= ThudDecay;
				}

				// torque alert: 880 / 1320 Hz alternating every 0.15 s
				float Alert = 0.f;
				AlertClock = FMath::Fmod(AlertClock + Dt, 0.3f);
				if (bAlert)
				{
					const bool bHigh = AlertClock < 0.15f;
					AlertPhase = FMath::Fmod(AlertPhase + (bHigh ? 1320.f : 880.f) * Dt, 1.f);
					Alert = FMath::Sin(2.f * PI * AlertPhase) * 0.14f;
				}

				float S = (Whine + Hum + Click + Thud + Alert) * Gain;
				S = S / (1.f + FMath::Abs(S));   // soft clip
				for (int32 c = 0; c < Channels; ++c)
				{
					Out[f * Channels + c] = S;
				}
			}
			for (int32 i = Frames * Channels; i < NumSamples; ++i)
			{
				Out[i] = 0.f;
			}
			return NumSamples;
		}

	private:
		float Noise()
		{
			Rng = Rng * 1664525u + 1013904223u;
			return static_cast<float>(Rng >> 8) / static_cast<float>(1u << 24) * 2.f - 1.f;
		}

		TSharedRef<FR1MotorSynthParams, ESPMode::ThreadSafe> Params;
		float SampleRate;
		int32 Channels;
		uint32 SeenClicks = 0, SeenThuds = 0;
		uint32 Rng = 0x12345u;
		float Speed = 0.f, Load = 0.f, Gain = 0.f;
		float WhinePhase = 0.f, HumPhase = 0.f, AlertPhase = 0.f, AlertClock = 0.f;
		float ClickEnv = 0.f, ThudEnv = 0.f, ThudPhase = 0.f;
	};
}

UR1MotorSynth::UR1MotorSynth(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
	NumChannels = 1;
	bAllowSpatialization = false;   // a UI sound: the same wherever the camera is
	bIsUISound = true;
	bAutoActivate = false;
}

bool UR1MotorSynth::Init(int32& /*SampleRate*/)
{
	NumChannels = 1;
	return true;
}

ISoundGeneratorPtr UR1MotorSynth::CreateSoundGenerator(const FSoundGeneratorInitParams& InParams)
{
	return MakeShared<FR1MotorSynthGenerator, ESPMode::ThreadSafe>(InParams, Params);
}

void UR1MotorSynth::SetDrive(float Speed01, float Load01, bool bAlert)
{
	Params->Speed.store(FMath::Clamp(Speed01, 0.f, 1.f), std::memory_order_relaxed);
	Params->Load.store(FMath::Clamp(Load01, 0.f, 1.f), std::memory_order_relaxed);
	Params->bAlert.store(bAlert, std::memory_order_relaxed);
}

void UR1MotorSynth::SetMasterVolume(float Volume01)
{
	Params->Volume.store(FMath::Clamp(Volume01, 0.f, 1.f), std::memory_order_relaxed);
}

void UR1MotorSynth::Click()
{
	Params->Clicks.fetch_add(1, std::memory_order_release);
}

void UR1MotorSynth::Thud(float Strength01)
{
	Params->ThudStrength.store(FMath::Clamp(Strength01, 0.f, 1.f), std::memory_order_relaxed);
	Params->Thuds.fetch_add(1, std::memory_order_release);
}
