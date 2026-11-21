#include "R1WalkCourse.h"
#include "R1Twin.h"
#include "MjUeConvert.h"

#include "Components/SceneComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Materials/MaterialInterface.h"
#include "UObject/ConstructorHelpers.h"

namespace
{
	const FName ColorParam(TEXT("Color"));   // /Engine/BasicShapes/BasicShapeMaterial exposes a "Color" vector
	FLinearColor Hex(const TCHAR* H) { return FLinearColor::FromSRGBColor(FColor::FromHex(H)); }
}

AR1WalkCourse::AR1WalkCourse()
{
	PrimaryActorTick.bCanEverTick = false;
	Root = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
	SetRootComponent(Root);

	static ConstructorHelpers::FObjectFinder<UStaticMesh> Cube(TEXT("/Engine/BasicShapes/Cube.Cube"));
	static ConstructorHelpers::FObjectFinder<UStaticMesh> Cyl(TEXT("/Engine/BasicShapes/Cylinder.Cylinder"));
	static ConstructorHelpers::FObjectFinder<UMaterialInterface> Mat(TEXT("/Engine/BasicShapes/BasicShapeMaterial.BasicShapeMaterial"));
	CubeMesh = Cube.Object;
	CylinderMesh = Cyl.Object;
	BaseMaterial = Mat.Object;
}

UStaticMeshComponent* AR1WalkCourse::AddMesh(UStaticMesh* Mesh, const FTransform& Xf, const FLinearColor& Color, UMaterialInstanceDynamic*& OutMid)
{
	OutMid = nullptr;
	if (!Mesh)
	{
		return nullptr;
	}
	UStaticMeshComponent* C = NewObject<UStaticMeshComponent>(this);
	C->SetStaticMesh(Mesh);
	C->SetMobility(EComponentMobility::Movable);
	C->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	C->SetSimulatePhysics(false);   // MuJoCo is the only physics authority
	C->SetCastShadow(true);
	C->SetupAttachment(Root);
	C->SetRelativeTransform(Xf);
	C->RegisterComponent();
	if (BaseMaterial)
	{
		OutMid = UMaterialInstanceDynamic::Create(BaseMaterial, this);
		OutMid->SetVectorParameterValue(ColorParam, Color);
		C->SetMaterial(0, OutMid);
	}
	Parts.Add(C);
	return C;
}

void AR1WalkCourse::Clear()
{
	for (UStaticMeshComponent* C : Parts)
	{
		if (C)
		{
			C->DestroyComponent();
		}
	}
	Parts.Reset();
	CheckpointMids.Reset();
	NextPole = nullptr;
	bBuilt = false;
}

void AR1WalkCourse::Sync(const FR1WalkState& State)
{
	const uint32 Hash = State.GeometryHash();
	if (!bBuilt || Hash != BuiltHash)
	{
		Clear();
		BuiltHash = Hash;
		bBuilt = true;
		UMaterialInstanceDynamic* Mid = nullptr;
		// Boxes: the engine cube is 100 cm per side, pivot at its centre -> scale = 2 * half extent (m).
		for (const FR1WalkBox& B : State.Boxes)
		{
			const FQuat Rot = MjUe::QuatToUe(B.QuatW, B.QuatX, B.QuatY, B.QuatZ);
			const FTransform Xf(Rot, MjUe::PositionToUe(B.PosMj), B.HalfSizeMj * 2.0);
			AddMesh(CubeMesh, Xf, B.Color, Mid);
		}
		// Checkpoints: a 2 cm disc of the trigger radius on the floor (the cylinder is 100 cm wide and tall).
		const double D = FMath::Max(0.1, static_cast<double>(State.CheckpointRadius) * 2.0);
		for (const FVector& Cp : State.CheckpointsMj)
		{
			const FVector P = MjUe::PositionToUe(Cp) + FVector(0.0, 0.0, 1.0);
			AddMesh(CylinderMesh, FTransform(FQuat::Identity, P, FVector(D, D, 0.02)), Hex(TEXT("3a4560")), Mid);
			CheckpointMids.Add(Mid);
		}
		NextPole = AddMesh(CylinderMesh, FTransform(FQuat::Identity, FVector::ZeroVector, FVector(0.06, 0.06, 1.6)), Hex(TEXT("6366f1")), Mid);
		UE_LOG(LogR1Twin, Log, TEXT("R1WalkCourse: built course '%s' (%d boxes, %d checkpoints)"),
			*State.Course, State.Boxes.Num(), State.CheckpointsMj.Num());
	}

	// Checkpoint colours: next = accent, passed this lap = green, upcoming = slate. The pole marks the next one.
	const int32 Next = State.bRunning ? State.CheckpointIndex : INDEX_NONE;
	for (int32 i = 0; i < CheckpointMids.Num(); ++i)
	{
		if (UMaterialInstanceDynamic* M = CheckpointMids[i])
		{
			const FLinearColor C = i == Next ? Hex(TEXT("6366f1")) : (Next != INDEX_NONE && i < Next) ? Hex(TEXT("0ca30c")) : Hex(TEXT("3a4560"));
			M->SetVectorParameterValue(ColorParam, C);
		}
	}
	if (NextPole)
	{
		const bool bShow = State.CheckpointsMj.IsValidIndex(Next);
		NextPole->SetVisibility(bShow);
		if (bShow)
		{
			NextPole->SetRelativeLocation(MjUe::PositionToUe(State.CheckpointsMj[Next]) + FVector(0.0, 0.0, 80.0));
		}
	}
}
