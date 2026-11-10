// Game mode for the R1 digital twin levels. Written for UE 5.7; NOT YET COMPILED (see docs/ue5_setup.md).
//
// Responsibilities
//   - default pawn = AR1FollowCamera (orbit camera following the pelvis)
//   - optionally spawns a "policy host" actor carrying UR1PolicyComponent + UR1Level0SweepCommand so a
//     level only needs the URLab Articulation Blueprint placed in it
//   - makes sure Chaos never simulates anything (MuJoCo via URLab is the single physics authority)
#pragma once

#include "CoreMinimal.h"
#include "GameFramework/GameModeBase.h"
#include "R1Level0SweepCommand.h"
#include "R1TwinGameMode.generated.h"

class UR1PolicyComponent;
class UR1DeployConfigAsset;
class UNNEModelData;

UCLASS(Config = Game)
class R1TWIN_API AR1TwinGameMode : public AGameModeBase
{
	GENERATED_BODY()

public:
	AR1TwinGameMode();

	/** Spawn an actor with UR1PolicyComponent + UR1Level0SweepCommand if the level has none. */
	UPROPERTY(EditDefaultsOnly, BlueprintReadOnly, Category = "R1")
	bool bAutoSpawnPolicyHost = true;

	/** Deploy config handed to the auto-spawned policy component (null -> built-in Level 0 defaults). */
	UPROPERTY(EditDefaultsOnly, BlueprintReadOnly, Category = "R1")
	TObjectPtr<UR1DeployConfigAsset> DefaultDeployConfig;

	/** ONNX model handed to the auto-spawned policy component (overrides the asset's model if set). */
	UPROPERTY(EditDefaultsOnly, BlueprintReadOnly, Category = "R1")
	TObjectPtr<UNNEModelData> DefaultPolicyModel;

	/** Used when DefaultPolicyModel is unset. Set in Config/DefaultGame.ini:
	 *  [/Script/R1Twin.R1TwinGameMode] DefaultPolicyModelPath=/Game/Humanoid/Policies/r1_level0_upperbody.r1_level0_upperbody */
	UPROPERTY(Config, EditDefaultsOnly, BlueprintReadOnly, Category = "R1")
	FSoftObjectPath DefaultPolicyModelPath;

	UPROPERTY(EditDefaultsOnly, BlueprintReadOnly, Category = "R1")
	ER1CommandMode DefaultCommandMode = ER1CommandMode::Sinusoid;

	/** Tag the placed Articulation Blueprint instance with this so it is found unambiguously. */
	UPROPERTY(EditDefaultsOnly, BlueprintReadOnly, Category = "R1")
	FName ArticulationActorTag = TEXT("R1Articulation");

	/** Level 0: camera stays on the gantry stand focus point instead of tracking the pelvis. */
	UPROPERTY(EditDefaultsOnly, BlueprintReadOnly, Category = "R1")
	bool bLockCameraToStand = true;

	/** Walk every primitive component at start-up and call SetSimulatePhysics(false). */
	UPROPERTY(EditDefaultsOnly, BlueprintReadOnly, Category = "R1|Physics")
	bool bForceDisableChaosSimulation = true;

	UFUNCTION(BlueprintPure, Category = "R1")
	UR1PolicyComponent* GetPolicyComponent() const { return PolicyComponent; }

	UFUNCTION(BlueprintPure, Category = "R1")
	AActor* GetPolicyHost() const { return PolicyHost; }

	virtual void StartPlay() override;

private:
	void SpawnPolicyHostIfNeeded();
	void DisableChaosOnAllActors();
	UR1PolicyComponent* FindExistingPolicyComponent() const;

	UPROPERTY(Transient)
	TObjectPtr<AActor> PolicyHost;

	UPROPERTY(Transient)
	TObjectPtr<UR1PolicyComponent> PolicyComponent;
};
