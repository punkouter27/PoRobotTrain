// Thin abstraction over the URLab articulation so that the policy/camera code never calls URLab
// directly. Verified against URLab (Aug 2026) on UE 5.8.4: AMjArticulation::GetJoint/GetActuator/GetBody/GetSensors.
//
// All quantities crossing this interface are in the MuJoCo frame and MuJoCo units
// (metres, radians, rad/s, quaternion packed as raw FQuat(X=x, Y=y, Z=z, W=w)). Convert with
// MjUeConvert.h on the consumer side.
#pragma once

#include "CoreMinimal.h"
#include "Templates/UniquePtr.h"
#include "UObject/WeakObjectPtr.h"

class AActor;
class UWorld;

/** Everything a bridge needs to pre-resolve names to indices (no per-tick string work). */
struct R1TWIN_API FR1BridgeInitParams
{
	/** Full MJCF joint names, e.g. "right_shoulder_pitch_joint" (policy order). */
	TArray<FName> JointNames;
	/** MJCF actuator names, e.g. "right_shoulder_pitch" (same order as JointNames). */
	TArray<FName> ActuatorNames;
	/** Optional initial joint positions (rad) used by the mock bridge only. */
	TArray<double> InitialJointPos;
	FName ImuSiteName = TEXT("imu");
	FName RootBodyName = TEXT("pelvis");
	/** Candidate articulation actor (URLab AMjArticulation-derived Blueprint instance). May be null. */
	TWeakObjectPtr<AActor> ArticulationActor;
};

/**
 * Minimal articulation API consumed by UR1PolicyComponent and AR1FollowCamera.
 * Implementations: FR1UrlabBridge (real), FR1MockBridge (no URLab actor present / API unverified).
 */
class R1TWIN_API IR1ArticulationBridge
{
public:
	virtual ~IR1ArticulationBridge() = default;

	/** Resolve names, grab component pointers. Returns false if the bridge cannot serve data. */
	virtual bool Initialize(const FR1BridgeInitParams& Params) = 0;
	virtual bool IsConnected() const = 0;
	virtual FString GetDescription() const = 0;

	/** Joint angle, rad (MuJoCo qpos of a hinge joint). */
	virtual double GetJointPos(FName JointName) const = 0;
	/** Joint velocity, rad/s (MuJoCo qvel). */
	virtual double GetJointVel(FName JointName) const = 0;

	/** Position target handed to the position actuator / UMjPDController, rad. */
	virtual void SetActuatorTarget(FName ActuatorName, double TargetRad) = 0;
	/** PD gains for one actuator (Nm/rad, Nm s/rad). */
	virtual void SetActuatorGains(FName ActuatorName, double Kp, double Kd) = 0;

	/** IMU orientation (framequat on site "imu") as raw MuJoCo components packed in FQuat(X=x,Y=y,Z=z,W=w). */
	virtual FQuat GetImuQuat() const = 0;
	/** IMU gyro (rad/s, body frame, MuJoCo axes). */
	virtual FVector GetImuGyro() const = 0;

	/** World pose of a body (metres + raw MuJoCo quaternion). False if unknown. */
	virtual bool GetBodyPoseMj(FName BodyName, FVector& OutPosMeters, FQuat& OutQuatRaw) const = 0;

	/** Lets the mock integrate its fake dynamics. No-op for the real bridge (physics is async in URLab). */
	virtual void Advance(float DeltaSeconds) {}

	/** True while an external client (Python runner over URLab Direct/Puppet) owns physics stepping;
	 *  in-engine consumers must not write actuator targets then. */
	virtual bool IsExternallyStepped() const { return false; }

	/** Claim (true) or release (false) the actuators for in-engine control. URLab: articulation
	 *  ControlSource UI vs ZMQ; Python direct-mode RPCs write the ZMQ (network) channel. */
	virtual void SetInEngineControl(bool bClaim) {}

	/** Start a standalone in-engine session: reset to the 'home' keyframe and unpause physics. */
	virtual void StartInEngineSession() {}
};

/**
 * Stand-in used when no URLab articulation is in the level (or the URLab API is not yet verified).
 * First-order lag towards the commanded target so the policy loop has something plausible to chew on.
 */
class R1TWIN_API FR1MockBridge final : public IR1ArticulationBridge
{
public:
	virtual bool Initialize(const FR1BridgeInitParams& Params) override;
	virtual bool IsConnected() const override { return bInitialized; }
	virtual FString GetDescription() const override { return TEXT("FR1MockBridge (no URLab articulation; synthetic first-order joints)"); }

	virtual double GetJointPos(FName JointName) const override;
	virtual double GetJointVel(FName JointName) const override;
	virtual void SetActuatorTarget(FName ActuatorName, double TargetRad) override;
	virtual void SetActuatorGains(FName ActuatorName, double Kp, double Kd) override;
	virtual FQuat GetImuQuat() const override;
	virtual FVector GetImuGyro() const override;
	virtual bool GetBodyPoseMj(FName BodyName, FVector& OutPosMeters, FQuat& OutQuatRaw) const override;
	virtual void Advance(float DeltaSeconds) override;

	/** Where the mock pretends the pelvis is (metres, MuJoCo). Defaults to the Level 0 stand height. */
	FVector RootPosMeters = FVector(0.0, 0.0, 1.0);

private:
	bool bInitialized = false;
	FName RootBodyName;
	TMap<FName, int32> JointIndex;
	TMap<FName, int32> ActuatorIndex;
	TArray<double> Q;
	TArray<double> V;
	TArray<double> Target;
	TArray<double> Kp;
	TArray<double> Kd;
};

/**
 * Real bridge onto a URLab articulation actor (an imported MJCF becomes an Articulation Blueprint
 * whose actor derives from AMjArticulation and owns a UMjComponent tree; a UMjPDController turns
 * position targets into torques on the async MuJoCo thread).
 *
 * Every URLab call in the .cpp is tagged "TODO verify against UMjComponent API" and compiled only
 * when R1_WITH_URLAB && R1_URLAB_API_VERIFIED (see R1Twin.Build.cs). Otherwise Initialize() fails
 * and UR1PolicyComponent falls back to FR1MockBridge.
 */
class R1TWIN_API FR1UrlabBridge final : public IR1ArticulationBridge
{
public:
	virtual bool Initialize(const FR1BridgeInitParams& Params) override;
	virtual bool IsConnected() const override { return bConnected; }
	virtual FString GetDescription() const override;

	virtual double GetJointPos(FName JointName) const override;
	virtual double GetJointVel(FName JointName) const override;
	virtual void SetActuatorTarget(FName ActuatorName, double TargetRad) override;
	virtual void SetActuatorGains(FName ActuatorName, double Kp, double Kd) override;
	virtual FQuat GetImuQuat() const override;
	virtual FVector GetImuGyro() const override;
	virtual bool GetBodyPoseMj(FName BodyName, FVector& OutPosMeters, FQuat& OutQuatRaw) const override;
	virtual bool IsExternallyStepped() const override;
	virtual void SetInEngineControl(bool bClaim) override;
	virtual void StartInEngineSession() override;

private:
	bool bConnected = false;
	TWeakObjectPtr<AActor> ArticulationActor;
	FName ImuSiteName;
	FName RootBodyName;
	/** Component caches resolved once in Initialize (UMjJoint / UMjActuator / UMjBody / UMjSensor),
	 *  so the hot path does no string work and no allocation. Typed as UObject to keep URLab headers
	 *  out of this public header. */
	TMap<FName, TWeakObjectPtr<UObject>> Joints;
	TMap<FName, TWeakObjectPtr<UObject>> Actuators;
	TWeakObjectPtr<UObject> RootBody;
	TWeakObjectPtr<UObject> GyroSensor;
	mutable bool bWarnedGains = false;
	int32 ClaimedState = -1; // -1 unknown, 0 released, 1 claimed
};

namespace R1Urlab
{
	/**
	 * Find the URLab articulation actor in a world without a compile-time dependency on its header:
	 * prefers an actor tagged PreferredTag, else the first actor whose class derives from
	 * "/Script/UnrealRoboticsLab.MjArticulation" (TODO verify script path once the plugin is present).
	 */
	R1TWIN_API AActor* FindArticulationActor(UWorld* World, FName PreferredTag);

	/** Builds the best available bridge: URLab if an actor is found and the API is verified, else mock. */
	R1TWIN_API TUniquePtr<IR1ArticulationBridge> CreateBridge(const FR1BridgeInitParams& Params, bool bAllowMock);
}
