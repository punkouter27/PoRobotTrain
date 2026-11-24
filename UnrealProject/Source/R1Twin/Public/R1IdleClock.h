// Idle clock for demo mode (Po Game Standard C13): counts real seconds without input, but ignores any single frame
// longer than MaxFrameSeconds (shader warm-up, level load, a debugger break), so a hitch never starts demo mode.
// Plain struct, no UObject: the showcase subsystem owns one and the automation test feeds it frames directly.
#pragma once

#include "CoreMinimal.h"

struct FR1IdleClock
{
	/** A frame longer than this is not idle time. */
	static constexpr float MaxFrameSeconds = 0.25f;

	float Seconds = 0.f;

	/** Adds one frame; returns true when the frame counted. */
	bool Advance(float DeltaSeconds)
	{
		if (DeltaSeconds <= 0.f || DeltaSeconds > MaxFrameSeconds) return false;
		Seconds += DeltaSeconds;
		return true;
	}

	void Reset() { Seconds = 0.f; }
	bool Reached(float Threshold) const { return Seconds >= Threshold; }
};
