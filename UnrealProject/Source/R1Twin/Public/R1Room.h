// A simple empty room for the virtual robot to live in: four walls around the stand, open to the sky so the daylight
// still reaches the robot. Spawned once at startup by AR1TwinGameMode (no runtime spawns); visual only - MuJoCo (URLab)
// owns the physics and nothing here collides. Hidden while the walk sandbox runs (the robot walks a course instead).
#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "R1Room.generated.h"

class UStaticMeshComponent;
class UMaterialInstanceDynamic;

UCLASS(NotPlaceable)
class R1TWIN_API AR1Room : public AActor
{
	GENERATED_BODY()

public:
	AR1Room();

	/** Inner size of the room (cm) and wall height; the robot's stand sits in the middle. */
	UPROPERTY(EditAnywhere, Category = "R1|Room")
	FVector2D SizeCm = FVector2D(800.f, 700.f);

	UPROPERTY(EditAnywhere, Category = "R1|Room")
	float WallHeightCm = 260.f;

	UPROPERTY(EditAnywhere, Category = "R1|Room")
	FLinearColor WallColor = FLinearColor(0.62f, 0.60f, 0.56f, 1.f);

	UPROPERTY(EditAnywhere, Category = "R1|Room")
	FLinearColor TrimColor = FLinearColor(0.20f, 0.21f, 0.24f, 1.f);

	virtual void BeginPlay() override;

private:
	UPROPERTY(VisibleAnywhere)
	TObjectPtr<USceneComponent> Root;

	UPROPERTY(Transient)
	TArray<TObjectPtr<UStaticMeshComponent>> Parts;

	UStaticMeshComponent* AddBox(const FName& Name, const FVector& CentreCm, const FVector& SizeCmXYZ, UMaterialInstanceDynamic* Mat);
};
