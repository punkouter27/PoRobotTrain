// Written for UE 5.7; NOT YET COMPILED (see docs/ue5_setup.md).
#include "R1TwinGameMode.h"
#include "R1Twin.h"
#include "R1FollowCamera.h"
#include "R1PolicyComponent.h"
#include "R1Level0SweepCommand.h"
#include "R1ArticulationBridge.h"
#include "R1DeployConfig.h"
#include "NNEModelData.h"

#include "Engine/World.h"
#include "EngineUtils.h"
#include "Components/PrimitiveComponent.h"
#include "GameFramework/PlayerController.h"

AR1TwinGameMode::AR1TwinGameMode()
{
	DefaultPawnClass = AR1FollowCamera::StaticClass();
	PlayerControllerClass = APlayerController::StaticClass();
	bStartPlayersAsSpectators = false;
}

void AR1TwinGameMode::StartPlay()
{
	// Runs before BeginPlay is dispatched to the level's actors, so anything we spawn here receives
	// BeginPlay in the normal start-up sweep (UR1PolicyComponent::BeginPlay then finds the placed
	// Articulation Blueprint instance).
	if (bForceDisableChaosSimulation)
	{
		DisableChaosOnAllActors();
	}
	if (bAutoSpawnPolicyHost)
	{
		SpawnPolicyHostIfNeeded();
	}

	Super::StartPlay();

	if (AActor* Art = R1Urlab::FindArticulationActor(GetWorld(), ArticulationActorTag))
	{
		UE_LOG(LogR1Twin, Log, TEXT("R1TwinGameMode: articulation actor '%s' present."), *Art->GetName());
	}
	else
	{
		UE_LOG(LogR1Twin, Warning, TEXT("R1TwinGameMode: no URLab articulation actor in level (tag '%s'). Policy will use the mock bridge."), *ArticulationActorTag.ToString());
	}
}

UR1PolicyComponent* AR1TwinGameMode::FindExistingPolicyComponent() const
{
	UWorld* World = GetWorld();
	if (!World)
	{
		return nullptr;
	}
	for (TActorIterator<AActor> It(World); It; ++It)
	{
		if (UR1PolicyComponent* Comp = It->FindComponentByClass<UR1PolicyComponent>())
		{
			return Comp;
		}
	}
	return nullptr;
}

void AR1TwinGameMode::SpawnPolicyHostIfNeeded()
{
	UWorld* World = GetWorld();
	if (!World)
	{
		return;
	}

	if (UR1PolicyComponent* Existing = FindExistingPolicyComponent())
	{
		PolicyComponent = Existing;
		PolicyHost = Existing->GetOwner();
		UE_LOG(LogR1Twin, Log, TEXT("R1TwinGameMode: using level-placed policy component on '%s'."), *GetNameSafe(PolicyHost));
		return;
	}

	FActorSpawnParameters Params;
	Params.Name = TEXT("R1PolicyHost");
	Params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
	PolicyHost = World->SpawnActor<AActor>(AActor::StaticClass(), FTransform::Identity, Params);
	if (!PolicyHost)
	{
		UE_LOG(LogR1Twin, Error, TEXT("R1TwinGameMode: failed to spawn policy host."));
		return;
	}
#if WITH_EDITOR
	PolicyHost->SetActorLabel(TEXT("R1PolicyHost"));
#endif

	// Command source first so the policy component can pick it up in BeginPlay.
	UR1Level0SweepCommand* Cmd = NewObject<UR1Level0SweepCommand>(PolicyHost, TEXT("Level0Command"));
	Cmd->Mode = DefaultCommandMode;
	PolicyHost->AddInstanceComponent(Cmd);
	Cmd->RegisterComponent();

	UR1PolicyComponent* Policy = NewObject<UR1PolicyComponent>(PolicyHost, TEXT("Policy"));
	Policy->DeployConfigAsset = DefaultDeployConfig;
	if (!DefaultPolicyModel && DefaultPolicyModelPath.IsValid())
	{
		DefaultPolicyModel = Cast<UNNEModelData>(DefaultPolicyModelPath.TryLoad());
		UE_LOG(LogR1Twin, Log, TEXT("R1TwinGameMode: policy model from config %s -> %s"),
			*DefaultPolicyModelPath.ToString(), *GetNameSafe(DefaultPolicyModel.Get()));
	}
	Policy->PolicyModelOverride = DefaultPolicyModel;
	Policy->ArticulationActorTag = ArticulationActorTag;
	Policy->CommandSource = Cmd;
	PolicyHost->AddInstanceComponent(Policy);
	Policy->RegisterComponent();

	PolicyComponent = Policy;
	UE_LOG(LogR1Twin, Log, TEXT("R1TwinGameMode: spawned policy host (config: %s, model: %s)."),
		*GetNameSafe(DefaultDeployConfig.Get()), *GetNameSafe(DefaultPolicyModel.Get()));
}

void AR1TwinGameMode::DisableChaosOnAllActors()
{
	UWorld* World = GetWorld();
	if (!World)
	{
		return;
	}

	int32 NumDisabled = 0;
	for (TActorIterator<AActor> It(World); It; ++It)
	{
		TInlineComponentArray<UPrimitiveComponent*> Prims(*It);
		for (UPrimitiveComponent* Prim : Prims)
		{
			if (Prim && Prim->IsSimulatingPhysics())
			{
				Prim->SetSimulatePhysics(false);
				++NumDisabled;
			}
		}
	}
	UE_LOG(LogR1Twin, Log, TEXT("R1TwinGameMode: Chaos simulation disabled on %d primitive component(s); MuJoCo (URLab) is the physics authority."), NumDisabled);
}
