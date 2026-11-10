// MuJoCo <-> Unreal Engine frame conversion for the R1 digital twin.
// Header-only. Written for UE 5.7; NOT YET COMPILED (see docs/ue5_setup.md).
//
// Conventions
// -----------
// MuJoCo : right-handed, Z up, X forward, Y LEFT, metres, quaternion stored (w, x, y, z).
// Unreal : left-handed,  Z up, X forward, Y RIGHT, centimetres, FQuat stored (X, Y, Z, W).
//
// The two frames differ by a reflection of the Y axis, M = diag(1, -1, 1), plus a 100x scale.
//
//   position_ue   = (x, -y, z) * 100                       (metres -> cm, flip Y)
//   quaternion    : MuJoCo (w, x, y, z) -> UE FQuat(X = x, Y = -y, Z = z, W = -w)
//
// Why the quaternion looks like that: conjugating a rotation R by a reflection M gives
// R' = M R M whose quaternion is (w, det(M) * M v) = (w, -x, y, -z). Negating the whole
// quaternion leaves the rotation unchanged, giving the equivalent (-w, x, -y, z), i.e.
// FQuat(X = x, Y = -y, Z = z, W = -w). Both spellings are the same rotation.
//
// Axial (pseudo-)vectors such as angular velocity transform as  w' = det(M) * M w = (-wx, wy, -wz).
// Polar vectors (positions, linear velocities, directions, accelerations) transform as (x, -y, z).
//
// Worked examples (also enforced by the static_asserts at the bottom of this file):
//   identity       (1, 0, 0, 0)              -> FQuat(0, 0, 0, -1)   == identity (q and -q are the same rotation)
//   yaw +90 deg    (c, 0, 0, s), c=s=1/sqrt2 -> FQuat(0, 0, s, -c)   == yaw -90 deg in UE
//                  MuJoCo +90 deg about Z takes +X to +Y_mj; +Y_mj is -Y_ue, and in UE's left-handed
//                  frame taking +X to -Y is a NEGATIVE yaw. The sign flip is expected and correct.
//   pelvis z=1.0 m (0, 0, 1.0)                -> FVector(0, 0, 100) cm
#pragma once

#include "CoreMinimal.h"
#include "Math/Vector.h"
#include "Math/Quat.h"
#include "Math/Transform.h"

namespace MjUe
{
	constexpr double MetersToCm = 100.0;
	constexpr double CmToMeters = 0.01;

	// ------------------------------------------------------------------ positions / polar vectors

	/** MuJoCo position (metres) -> UE position (cm). */
	FORCEINLINE FVector PositionToUe(const FVector& PosMjMeters)
	{
		return FVector(PosMjMeters.X, -PosMjMeters.Y, PosMjMeters.Z) * MetersToCm;
	}

	/** UE position (cm) -> MuJoCo position (metres). */
	FORCEINLINE FVector PositionToMj(const FVector& PosUeCm)
	{
		return FVector(PosUeCm.X, -PosUeCm.Y, PosUeCm.Z) * CmToMeters;
	}

	/** Unit-less polar vector (direction, normal) -> UE. No scaling. */
	FORCEINLINE FVector DirectionToUe(const FVector& DirMj)
	{
		return FVector(DirMj.X, -DirMj.Y, DirMj.Z);
	}

	/** Linear velocity or acceleration (m/s, m/s^2) -> UE (cm/s, cm/s^2). */
	FORCEINLINE FVector LinearVelocityToUe(const FVector& VelMj)
	{
		return FVector(VelMj.X, -VelMj.Y, VelMj.Z) * MetersToCm;
	}

	/** Angular velocity (axial vector, rad/s) -> UE (rad/s). Note the different sign pattern. */
	FORCEINLINE FVector AngularVelocityToUe(const FVector& OmegaMj)
	{
		return FVector(-OmegaMj.X, OmegaMj.Y, -OmegaMj.Z);
	}

	// ------------------------------------------------------------------ orientations

	/** MuJoCo quaternion components (w, x, y, z) -> UE FQuat. */
	FORCEINLINE FQuat QuatToUe(double W, double X, double Y, double Z)
	{
		return FQuat(X, -Y, Z, -W);
	}

	/**
	 * MuJoCo quaternion that has been packed into an FQuat WITHOUT conversion, i.e.
	 * Raw.X = x, Raw.Y = y, Raw.Z = z, Raw.W = w (this is what IR1ArticulationBridge returns).
	 */
	FORCEINLINE FQuat QuatToUe(const FQuat& RawMj)
	{
		return QuatToUe(RawMj.W, RawMj.X, RawMj.Y, RawMj.Z);
	}

	/** UE FQuat -> MuJoCo quaternion returned as a raw FQuat (X = x, Y = y, Z = z, W = w). */
	FORCEINLINE FQuat QuatToMj(const FQuat& QUe)
	{
		// Inverse of (x, -y, z, -w): x = X, y = -Y, z = Z, w = -W.
		return FQuat(QUe.X, -QUe.Y, QUe.Z, -QUe.W);
	}

	/** Full pose (MuJoCo metres + raw quaternion) -> UE FTransform (cm). Scale = 1. */
	FORCEINLINE FTransform PoseToUe(const FVector& PosMjMeters, const FQuat& RawQuatMj)
	{
		return FTransform(QuatToUe(RawQuatMj), PositionToUe(PosMjMeters), FVector::OneVector);
	}

	/** Projected gravity in body frame is a polar vector: same rule as directions. */
	FORCEINLINE FVector ProjectedGravityToUe(const FVector& GravMjBody)
	{
		return DirectionToUe(GravMjBody);
	}

	// ------------------------------------------------------------------ compile-time checks
	//
	// FQuat/FVector are not constexpr-constructible, so the identities are checked on plain
	// doubles that mirror the exact arithmetic used above.
	namespace Test
	{
		struct FRawQuat { double X, Y, Z, W; };
		struct FRawVec  { double X, Y, Z; };

		constexpr FRawQuat QuatToUeRaw(double W, double X, double Y, double Z) { return { X, -Y, Z, -W }; }
		constexpr FRawVec  PosToUeRaw(double X, double Y, double Z)            { return { X * MetersToCm, -Y * MetersToCm, Z * MetersToCm }; }
		constexpr FRawVec  AngVelToUeRaw(double X, double Y, double Z)         { return { -X, Y, -Z }; }

		constexpr double InvSqrt2 = 0.70710678118654752440;

		// identity (1,0,0,0) -> (0,0,0,-1)  (== identity rotation)
		static_assert(QuatToUeRaw(1.0, 0.0, 0.0, 0.0).X == 0.0,  "identity X");
		static_assert(QuatToUeRaw(1.0, 0.0, 0.0, 0.0).Y == 0.0,  "identity Y");
		static_assert(QuatToUeRaw(1.0, 0.0, 0.0, 0.0).Z == 0.0,  "identity Z");
		static_assert(QuatToUeRaw(1.0, 0.0, 0.0, 0.0).W == -1.0, "identity W (sign-flipped identity is still identity)");

		// yaw +90 deg about Z in MuJoCo (c,0,0,s) -> UE (0,0,s,-c) == yaw -90 deg in UE
		static_assert(QuatToUeRaw(InvSqrt2, 0.0, 0.0, InvSqrt2).X == 0.0,       "yaw90 X");
		static_assert(QuatToUeRaw(InvSqrt2, 0.0, 0.0, InvSqrt2).Y == 0.0,       "yaw90 Y");
		static_assert(QuatToUeRaw(InvSqrt2, 0.0, 0.0, InvSqrt2).Z == InvSqrt2,  "yaw90 Z");
		static_assert(QuatToUeRaw(InvSqrt2, 0.0, 0.0, InvSqrt2).W == -InvSqrt2, "yaw90 W");

		// pelvis on the Level 0 stand: (0, 0, 1.0 m) -> (0, 0, 100 cm); Y flips sign
		static_assert(PosToUeRaw(0.0, 0.0, 1.0).Z == 100.0, "stand height 1 m -> 100 cm");
		static_assert(PosToUeRaw(0.5, 0.25, 0.0).X == 50.0, "x scales");
		static_assert(PosToUeRaw(0.5, 0.25, 0.0).Y == -25.0, "y flips and scales");

		// angular velocity is an axial vector: (1,2,3) -> (-1,2,-3)
		static_assert(AngVelToUeRaw(1.0, 2.0, 3.0).X == -1.0, "gyro X flips");
		static_assert(AngVelToUeRaw(1.0, 2.0, 3.0).Y == 2.0,  "gyro Y keeps sign");
		static_assert(AngVelToUeRaw(1.0, 2.0, 3.0).Z == -3.0, "gyro Z flips");
	}

	// ------------------------------------------------------------------ runtime self-test
	//
	// Exercises the real FQuat path (which the static_asserts cannot). Called from
	// FR1TwinModule::StartupModule; the caller logs on failure.
	// To turn this into an Automation test, wrap it in IMPLEMENT_SIMPLE_AUTOMATION_TEST in a
	// Private/Tests/MjUeConvertTest.cpp (WITH_DEV_AUTOMATION_TESTS) and TestTrue(RunSelfTest()).
	inline bool RunSelfTest()
	{
		constexpr double Tol = 1e-6;
		bool bOk = true;

		// Identity
		{
			const FQuat Q = QuatToUe(1.0, 0.0, 0.0, 0.0);
			bOk &= Q.Equals(FQuat::Identity, Tol) || Q.Equals(FQuat(0.0, 0.0, 0.0, -1.0), Tol);
			bOk &= FMath::IsNearlyZero(Q.Rotator().Yaw, 1e-4) && FMath::IsNearlyZero(Q.Rotator().Pitch, 1e-4);
		}

		// +90 deg yaw in MuJoCo must come out as -90 deg yaw in UE
		{
			const double C = Test::InvSqrt2;
			const FQuat Q = QuatToUe(C, 0.0, 0.0, C);
			const double Yaw = Q.Rotator().Yaw;
			bOk &= FMath::IsNearlyEqual(Yaw, -90.0, 1e-3);

			// Rotating UE +X by Q must give UE -Y (which is MuJoCo +Y).
			const FVector Rotated = Q.RotateVector(FVector::ForwardVector);
			bOk &= Rotated.Equals(FVector(0.0, -1.0, 0.0), 1e-5);
		}

		// Positions
		{
			bOk &= PositionToUe(FVector(0.0, 0.0, 1.0)).Equals(FVector(0.0, 0.0, 100.0), 1e-9);
			bOk &= PositionToMj(PositionToUe(FVector(0.3, -0.2, 0.7332))).Equals(FVector(0.3, -0.2, 0.7332), 1e-9);
		}

		// Round-trip quaternion
		{
			const FQuat RawMj(0.1, 0.2, 0.3, 0.9); // arbitrary (unnormalised is fine for a round-trip)
			bOk &= QuatToMj(QuatToUe(RawMj)).Equals(RawMj, 1e-12);
		}

		// Consistency: rotate-then-convert equals convert-then-rotate.
		{
			const double C = Test::InvSqrt2;
			const FQuat QUe = QuatToUe(C, 0.0, 0.0, C); // yaw +90 (mj)
			const FVector PMj(1.0, 0.0, 0.0);             // +X in mujoco
			const FVector RotatedMj(0.0, 1.0, 0.0);       // yaw90 * +X = +Y in mujoco
			bOk &= QUe.RotateVector(PositionToUe(PMj)).Equals(PositionToUe(RotatedMj), 1e-5);
		}

		return bOk;
	}
}
