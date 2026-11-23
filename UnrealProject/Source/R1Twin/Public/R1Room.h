// The virtual robot's surroundings: a simple empty room around the stand, or outside in an open area with a concrete
// floor (expansion joints every 4 m). Spawned once at startup by AR1TwinGameMode (no runtime spawns); visual only -
// MuJoCo (URLab) owns the physics and nothing here collides. The robot stays on its stand in both places. The panel
// switches the place (SetEnvironment); hidden while the walk sandbox runs (the robot walks a course instead).
#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "R1Room.generated.h"

class UStaticMeshComponent;
class UMaterialInstanceDynamic;

UENUM()
enum class ER1Environment : uint8
{
	Room,      // on the stand, in an empty room
	Outside,   // on the stand, outside on a concrete floor
};

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

	/** Outside: a square concrete apron (cm) with expansion joints. */
	UPROPERTY(EditAnywhere, Category = "R1|Outside")
	float ConcreteSizeCm = 40000.f;   // 400 m: the edge sits at the horizon

	UPROPERTY(EditAnywhere, Category = "R1|Outside")
	float JointSpacingCm = 400.f;

	/** Expansion joints are drawn in this square around the robot (cm). */
	UPROPERTY(EditAnywhere, Category = "R1|Outside")
	float JointAreaCm = 8000.f;

	UPROPERTY(EditAnywhere, Category = "R1|Outside")
	FLinearColor ConcreteColor = FLinearColor(0.42f, 0.41f, 0.39f, 1.f);

	UFUNCTION(BlueprintCallable, Category = "R1|Room")
	void SetEnvironment(ER1Environment InEnvironment);

	ER1Environment GetEnvironment() const { return Environment; }

	virtual void BeginPlay() override;

private:
	UPROPERTY(VisibleAnywhere)
	TObjectPtr<USceneComponent> Root;

	UPROPERTY(Transient)
	TArray<TObjectPtr<UStaticMeshComponent>> RoomParts;

	UPROPERTY(Transient)
	TArray<TObjectPtr<UStaticMeshComponent>> OutsideParts;

	ER1Environment Environment = ER1Environment::Room;

	UStaticMeshComponent* AddBox(const FName& Name, const FVector& CentreCm, const FVector& SizeCmXYZ, UMaterialInstanceDynamic* Mat,
		TArray<TObjectPtr<UStaticMeshComponent>>& Group);
};
