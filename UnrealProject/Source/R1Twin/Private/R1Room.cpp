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

UStaticMeshComponent* AR1Room::AddBox(const FName& Name, const FVector& CentreCm, const FVector& SizeCmXYZ, UMaterialInstanceDynamic* Mat)
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
	Parts.Add(C);
	return C;
}

void AR1Room::BeginPlay()
{
	Super::BeginPlay();
	UMaterialInterface* Base = LoadObject<UMaterialInterface>(nullptr, TEXT("/Engine/BasicShapes/BasicShapeMaterial.BasicShapeMaterial"));
	UMaterialInstanceDynamic* Wall = Base ? UMaterialInstanceDynamic::Create(Base, this) : nullptr;
	UMaterialInstanceDynamic* Trim = Base ? UMaterialInstanceDynamic::Create(Base, this) : nullptr;
	if (Wall) Wall->SetVectorParameterValue(TEXT("Color"), WallColor);
	if (Trim) Trim->SetVectorParameterValue(TEXT("Color"), TrimColor);

	const float HX = SizeCm.X * 0.5f, HY = SizeCm.Y * 0.5f, T = 12.f, H = WallHeightCm, Base_ = 14.f;
	// four walls (the floor is the level's own), each with a dark skirting board
	AddBox(TEXT("WallFront"), FVector(HX + T * 0.5f, 0.f, H * 0.5f), FVector(T, SizeCm.Y + 2.f * T, H), Wall);
	AddBox(TEXT("WallBack"), FVector(-HX - T * 0.5f, 0.f, H * 0.5f), FVector(T, SizeCm.Y + 2.f * T, H), Wall);
	AddBox(TEXT("WallLeft"), FVector(0.f, HY + T * 0.5f, H * 0.5f), FVector(SizeCm.X, T, H), Wall);
	AddBox(TEXT("WallRight"), FVector(0.f, -HY - T * 0.5f, H * 0.5f), FVector(SizeCm.X, T, H), Wall);
	AddBox(TEXT("SkirtFront"), FVector(HX - 1.f, 0.f, Base_ * 0.5f), FVector(2.f, SizeCm.Y, Base_), Trim);
	AddBox(TEXT("SkirtBack"), FVector(-HX + 1.f, 0.f, Base_ * 0.5f), FVector(2.f, SizeCm.Y, Base_), Trim);
	AddBox(TEXT("SkirtLeft"), FVector(0.f, HY - 1.f, Base_ * 0.5f), FVector(SizeCm.X, 2.f, Base_), Trim);
	AddBox(TEXT("SkirtRight"), FVector(0.f, -HY + 1.f, Base_ * 0.5f), FVector(SizeCm.X, 2.f, Base_), Trim);
	UE_LOG(LogR1Twin, Log, TEXT("R1Room: %.0f x %.0f cm room, walls %.0f cm high (visual only)"), SizeCm.X, SizeCm.Y, H);
}
