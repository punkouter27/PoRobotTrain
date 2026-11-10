// R1Twin primary game module. Written for UE 5.7; NOT YET COMPILED (see docs/ue5_setup.md).
#pragma once

#include "CoreMinimal.h"
#include "Modules/ModuleManager.h"

/** Single log category for the whole digital-twin module. Usage: UE_LOG(LogR1Twin, Log, TEXT("...")); */
R1TWIN_API DECLARE_LOG_CATEGORY_EXTERN(LogR1Twin, Log, All);

class FR1TwinModule : public IModuleInterface
{
public:
	virtual void StartupModule() override;
	virtual void ShutdownModule() override;
};
