#include "R1Room.h"
#include "R1Twin.h"

#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "Materials/MaterialInstanceDynamic.h"

AR1Room::AR1Room()
{
	Root = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
	SetRootComponent(Root);
}

UStaticMeshComponent* AR1Room::AddBox(const FName& Name, const FVector& CentreCm, const FVector& SizeCmXYZ, UMaterialInstanceDynamic* Mat,
	TArray<TObjectPtr<UStaticMeshComponent>>& Group)
{
	static UStaticMesh* Cube = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cube.Cube"));   // 100 cm cube
	UStaticMeshComponent* C = NewObject<UStaticMeshComponent>(this, Name);
	C->SetupAttachment(Root);
	C->SetStaticMesh(Cube);
	C->SetCollisionEnabled(ECollisionEnabled::NoCollision);   // visual only: MuJoCo owns the physics
	C->SetRelativeLocation(CentreCm);
	C->SetRelativeScale3D(SizeCmXYZ / 100.f);
	if (Mat)
	{
		C->SetMaterial(0, Mat);
	}
	C->RegisterComponent();
	Group.Add(C);
	return C;
}

void AR1Room::BeginPlay()
{
	Super::BeginPlay();
	UMaterialInterface* Base = LoadObject<UMaterialInterface>(nullptr, TEXT("/Engine/BasicShapes/BasicShapeMaterial.BasicShapeMaterial"));
	auto Mat = [this, Base](const FLinearColor& Col)
	{
		UMaterialInstanceDynamic* M = Base ? UMaterialInstanceDynamic::Create(Base, this) : nullptr;
		if (M) M->SetVectorParameterValue(TEXT("Color"), Col);
		return M;
	};

	// ---- room: four walls (the floor is the level's own), each with a dark skirting board
	UMaterialInstanceDynamic* Wall = Mat(WallColor);
	UMaterialInstanceDynamic* Trim = Mat(TrimColor);
	const float HX = SizeCm.X * 0.5f, HY = SizeCm.Y * 0.5f, T = 12.f, H = WallHeightCm, Sk = 14.f;
	AddBox(TEXT("WallFront"), FVector(HX + T * 0.5f, 0.f, H * 0.5f), FVector(T, SizeCm.Y + 2.f * T, H), Wall, RoomParts);
	AddBox(TEXT("WallBack"), FVector(-HX - T * 0.5f, 0.f, H * 0.5f), FVector(T, SizeCm.Y + 2.f * T, H), Wall, RoomParts);
	AddBox(TEXT("WallLeft"), FVector(0.f, HY + T * 0.5f, H * 0.5f), FVector(SizeCm.X, T, H), Wall, RoomParts);
	AddBox(TEXT("WallRight"), FVector(0.f, -HY - T * 0.5f, H * 0.5f), FVector(SizeCm.X, T, H), Wall, RoomParts);
	AddBox(TEXT("SkirtFront"), FVector(HX - 1.f, 0.f, Sk * 0.5f), FVector(2.f, SizeCm.Y, Sk), Trim, RoomParts);
	AddBox(TEXT("SkirtBack"), FVector(-HX + 1.f, 0.f, Sk * 0.5f), FVector(2.f, SizeCm.Y, Sk), Trim, RoomParts);
	AddBox(TEXT("SkirtLeft"), FVector(0.f, HY - 1.f, Sk * 0.5f), FVector(SizeCm.X, 2.f, Sk), Trim, RoomParts);
	AddBox(TEXT("SkirtRight"), FVector(0.f, -HY + 1.f, Sk * 0.5f), FVector(SizeCm.X, 2.f, Sk), Trim, RoomParts);

	// ---- outside: a concrete apron just above the level floor, cut into 4 m slabs by darker expansion joints
	UMaterialInstanceDynamic* Concrete = Mat(ConcreteColor);
	UMaterialInstanceDynamic* Joint = Mat(ConcreteColor * 0.55f);
	const float S = ConcreteSizeCm, Half = S * 0.5f;
	AddBox(TEXT("Concrete"), FVector(0.f, 0.f, -0.7f), FVector(S, S, 2.f), Concrete, OutsideParts);   // top at z +0.3 cm
	// joints only around the robot (they fade out with distance anyway); a few dozen pieces keep Lumen's cache small
	const float JHalf = FMath::Min(JointAreaCm, S) * 0.5f;
	int32 N = 0;
	for (float P = -JHalf + JointSpacingCm; P < JHalf - 1.f; P += JointSpacingCm, ++N)
	{
		AddBox(*FString::Printf(TEXT("JointX%d"), N), FVector(P, 0.f, 0.35f), FVector(2.f, 2.f * JHalf, 0.2f), Joint, OutsideParts);
		AddBox(*FString::Printf(TEXT("JointY%d"), N), FVector(0.f, P, 0.35f), FVector(2.f * JHalf, 2.f, 0.2f), Joint, OutsideParts);
	}
	for (UStaticMeshComponent* C : OutsideParts)
	{
		C->SetCastShadow(false);   // flat floor pieces: nothing to shadow
		C->SetAffectDistanceFieldLighting(false);
	}
	SetEnvironment(Environment);
	UE_LOG(LogR1Twin, Log, TEXT("R1Room: room %.0f x %.0f cm and a %.0f m concrete apron (visual only)"), SizeCm.X, SizeCm.Y, S / 100.f);
}

void AR1Room::SetEnvironment(ER1Environment InEnvironment)
{
	Environment = InEnvironment;
	const bool bRoom = Environment == ER1Environment::Room;
	for (UStaticMeshComponent* C : RoomParts)
	{
		if (C) C->SetVisibility(bRoom);
	}
	for (UStaticMeshComponent* C : OutsideParts)
	{
		if (C) C->SetVisibility(!bRoom);
	}
}
