// Colours shared by the behaviour panel's tab files (R1PanelVoice/Robustness/Walk.cpp). Same values as the
// anonymous-namespace constants at the top of R1BehaviourPanel.cpp, which predates this header.
#pragma once

#include "CoreMinimal.h"

namespace R1PanelStyle
{
	inline const FLinearColor TextCol(0.95f, 0.95f, 0.96f, 1.f);
	inline const FLinearColor DimCol(0.60f, 0.62f, 0.67f, 1.f);
	inline const FLinearColor OkCol(0.32f, 0.85f, 0.52f, 1.f);
	inline const FLinearColor WarnCol(1.f, 0.74f, 0.25f, 1.f);
	inline const FLinearColor BadCol(1.f, 0.36f, 0.34f, 1.f);
	inline const FLinearColor CyanCol(0.24f, 0.78f, 0.95f, 1.f);
	inline const FLinearColor VirtualCol(0.15f, 0.44f, 0.88f, 1.f);
	inline const FLinearColor StopCol(0.78f, 0.1f, 0.08f, 1.f);
	inline const FLinearColor ChipCol(0.20f, 0.21f, 0.24f, 1.f);
	inline const FLinearColor ChipOnCol(0.86f, 0.88f, 0.92f, 1.f);
	inline const FLinearColor CardBg(0.07f, 0.08f, 0.10f, 0.92f);
	inline const FLinearColor DividerCol(1.f, 1.f, 1.f, 0.08f);
	inline const FLinearColor DarkInk(0.05f, 0.05f, 0.06f, 1.f);

	// button ids used by more than one file
	inline const FName IdMic(TEXT("voice_mic"));
	inline const FName IdMute(TEXT("voice_mute"));
	inline const FName IdTabHealth(TEXT("tab_health"));
	inline const FName IdTabMotions(TEXT("tab_motions"));
	inline const FName IdTabSetup(TEXT("tab_setup"));
	inline const FName IdTabHelp(TEXT("tab_help"));
	inline const FName IdEStop(TEXT("estop"));
	inline const FName IdPosePreview(TEXT("voice_pose_preview"));
}
