// Visual-only course for the Level 2 walk sandbox (idea #20). Built from /walk/state "geometry": boxes
// (engine BasicShapes cube) and checkpoint discs (BasicShapes cylinder), converted with MjUeConvert. The next
// checkpoint gets a tall marker pole and the accent colour, passed ones turn green.
// Nothing here collides or simulates: the robot's physics (and the course's) runs in MuJoCo in the behaviour
// service, and the robot pose reaches Unreal through URLab puppet mode.
#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "R1PanelData.h"
#include "R1WalkCourse.generated.h"

class UStaticMesh;
class UStaticMeshComponent;
class UMaterialInterface;
class UMaterialInstanceDynamic;

UCLASS(NotPlaceable, Transient)
class R1TWIN_API AR1WalkCourse : public AActor
{
	GENERATED_BODY()

public:
	AR1WalkCourse();

	/** Rebuilds the course when the geometry changed; updates checkpoint colours and the thrown props every call. */
	void Sync(const FR1WalkState& State);

	uint32 GetGeometryHash() const { return BuiltHash; }

private:
	UStaticMeshComponent* AddMesh(UStaticMesh* Mesh, const FTransform& Xf, const FLinearColor& Color, UMaterialInstanceDynamic*& OutMid);
	void Clear();
	/** Thrown objects (docs/sim_features.md #9): engine sphere / cube posed from /walk/state props, visual only. */
	void SyncProps(const TArray<FR1WalkProp>& Props);

	UPROPERTY(Transient) TObjectPtr<USceneComponent> Root;
	UPROPERTY(Transient) TObjectPtr<UStaticMesh> CubeMesh;
	UPROPERTY(Transient) TObjectPtr<UStaticMesh> CylinderMesh;
	UPROPERTY(Transient) TObjectPtr<UStaticMesh> SphereMesh;
	UPROPERTY(Transient) TMap<int32, TObjectPtr<UStaticMeshComponent>> PropParts;   // prop id -> mesh
	TMap<int32, FString> PropKinds;
	UPROPERTY(Transient) TObjectPtr<UMaterialInterface> BaseMaterial;
	UPROPERTY(Transient) TArray<TObjectPtr<UStaticMeshComponent>> Parts;
	UPROPERTY(Transient) TArray<TObjectPtr<UMaterialInstanceDynamic>> CheckpointMids;
	UPROPERTY(Transient) TObjectPtr<UStaticMeshComponent> NextPole;

	uint32 BuiltHash = 0;
	bool bBuilt = false;
};
