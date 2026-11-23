// Procedural motor sound for the scenario lab (docs/sim_features.md #1 and #17). SIMULATION ONLY.
//
// No sound assets: a USynthComponent whose ISoundGenerator synthesises every sample on the audio render thread.
//   whine   pitch + loudness from the fastest joint speed
//   load    a low buzz under the whine from the largest torque fraction
//   clicks  short noise ticks when a joint's velocity changes sign (gear backlash)
//   thuds   low decaying thumps when a foot lands (walk contacts appearing with a large normal force)
//   alert   a distinct two-tone beep while any torque fraction is above torque_alert_fraction
// The game thread only writes the small lock-free parameter block (atomics); the audio thread never touches UObjects.
#pragma once

#include "CoreMinimal.h"
#include "Components/SynthComponent.h"
#include <atomic>
#include "R1MotorAudio.generated.h"

/** Parameters shared by the game thread (writer) and the audio render thread (reader). Lock-free. */
struct FR1MotorSynthParams
{
	std::atomic<float> Volume{0.5f};      // master 0..1 (0 = muted)
	std::atomic<float> Speed{0.f};        // 0..1, fastest joint relative to its "fast" speed
	std::atomic<float> Load{0.f};         // 0..1, largest torque fraction
	std::atomic<bool> bAlert{false};
	std::atomic<uint32> Clicks{0};        // incremented per click event
	std::atomic<uint32> Thuds{0};         // incremented per thud event
	std::atomic<float> ThudStrength{0.5f};
};

UCLASS(ClassGroup = (R1), meta = (BlueprintSpawnableComponent))
class R1TWIN_API UR1MotorSynth : public USynthComponent
{
	GENERATED_BODY()

public:
	UR1MotorSynth(const FObjectInitializer& ObjectInitializer);

	/** Game thread: the continuous sound (Speed and Load 0..1). */
	void SetDrive(float Speed01, float Load01, bool bAlert);
	void SetMasterVolume(float Volume01);
	void Click();
	void Thud(float Strength01);

protected:
	virtual bool Init(int32& SampleRate) override;
	virtual ISoundGeneratorPtr CreateSoundGenerator(const FSoundGeneratorInitParams& InParams) override;

private:
	TSharedRef<FR1MotorSynthParams, ESPMode::ThreadSafe> Params = MakeShared<FR1MotorSynthParams, ESPMode::ThreadSafe>();
};
