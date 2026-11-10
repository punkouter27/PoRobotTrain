// R1Twin primary game module. Written for UE 5.7; NOT YET COMPILED (see docs/ue5_setup.md).
#include "R1Twin.h"
#include "MjUeConvert.h"
#include "Modules/ModuleManager.h"

DEFINE_LOG_CATEGORY(LogR1Twin);

void FR1TwinModule::StartupModule()
{
	UE_LOG(LogR1Twin, Log, TEXT("R1Twin module starting (URLab compiled in: %d, URLab API verified: %d)"),
		static_cast<int32>(R1_WITH_URLAB), static_cast<int32>(R1_URLAB_API_VERIFIED));

	// Cheap runtime sanity check of the MuJoCo -> UE frame conversion (identity + 90 deg yaw).
	// Compile-time checks live in MjUeConvert.h as static_asserts.
	if (!MjUe::RunSelfTest())
	{
		UE_LOG(LogR1Twin, Error, TEXT("MjUeConvert self-test FAILED - frame conversion is wrong, do not trust poses."));
	}
}

void FR1TwinModule::ShutdownModule()
{
	UE_LOG(LogR1Twin, Log, TEXT("R1Twin module shutting down"));
}

IMPLEMENT_PRIMARY_GAME_MODULE(FR1TwinModule, R1Twin, "R1Twin");
