// Deploy-time policy description (the UE-side twin of RobotControl's deploy.yaml sidecar).
// Written for UE 5.7; NOT YET COMPILED (see docs/ue5_setup.md).
#pragma once

#include "CoreMinimal.h"
#include "Engine/DataAsset.h"
#include "R1DeployConfig.generated.h"

class UNNEModelData;

/**
 * Mirrors deploy.yaml:  joint_names, default_joint_pos, kp, kd, action_scale (+ sizes / rates).
 *
 * Joint naming: entries in JointNames are the SHORT actuator-style names used by the MJCF
 * (`right_shoulder_pitch`, `head_yaw`, ...). The MJCF joint is always `<short>_joint`; the MJCF
 * actuator and the sensors `<short>_pos/_vel/_frc` use the short name. Use GetJointName() /
 * GetActuatorName() rather than string-building at the call site; both tolerate a `_joint` suffix
 * already being present in the yaml.
 *
 * Level 0 (7 DoF right arm + head) values come straight from RobotControl/configs/r1_joint_map.yaml.
 */
USTRUCT(BlueprintType)
struct R1TWIN_API FR1DeployConfig
{
	GENERATED_BODY()

	/** Policy order. Level 0: right_shoulder_pitch, right_shoulder_roll, right_shoulder_yaw, right_elbow, right_wrist_roll, head_pitch, head_yaw. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Policy")
	TArray<FName> JointNames;

	/** Default joint positions q_default, rad. Same length/order as JointNames. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Policy")
	TArray<float> DefaultJointPos;

	/** PD stiffness, Nm/rad. Same length/order as JointNames. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Policy")
	TArray<float> Kp;

	/** PD damping, Nm s/rad. Same length/order as JointNames. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Policy")
	TArray<float> Kd;

	/** Per-joint action scale: q_target = q_default + ActionScale * action. A scalar in yaml is broadcast to all joints. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Policy")
	TArray<float> ActionScale;

	/** Per-joint soft limits (0.9 of the joint range about its centre), applied to targets exactly as in
	 *  training and the Python runner. Empty = no clamp. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "R1|Action")
	TArray<float> SoftLimitLo;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "R1|Action")
	TArray<float> SoftLimitHi;

	/** ONNX input "obs" is float32 (1, NumObs). Level 0: 23 = 7 + 7 + 7 + 2. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Policy|Tensors", meta = (ClampMin = 1))
	int32 NumObs = 23;

	/** ONNX output "actions" is float32 (1, NumActions). Level 0: 7. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Policy|Tensors", meta = (ClampMin = 1))
	int32 NumActions = 7;

	/** Command vector length appended to the observation. Level 0: 2 (head yaw target, shoulder pitch target). */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Policy|Tensors", meta = (ClampMin = 0))
	int32 NumCommands = 2;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Policy|Tensors")
	FString InputTensorName = TEXT("obs");

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Policy|Tensors")
	FString OutputTensorName = TEXT("actions");

	/** Policy rate. Level 0: 50 Hz (decimation 4 at the 0.005 s MuJoCo step). */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Policy|Timing", meta = (ClampMin = 1.0, ClampMax = 1000.0))
	float PolicyHz = 50.f;

	/** Observation scaling; must match the training env (mjlab default: 1.0 for joint_pos_rel, 1.0 for joint_vel - TODO confirm). */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Policy|Observation")
	float ObsJointPosScale = 1.f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Policy|Observation")
	float ObsJointVelScale = 1.f;

	/** If > 0, raw actions are clamped to [-ActionClipAbs, +ActionClipAbs] before scaling (0 = no clip). */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Policy|Action", meta = (ClampMin = 0.0))
	float ActionClipAbs = 0.f;

	/** Consistency checks; OutError describes the first problem found. */
	bool Validate(FString& OutError) const;

	/** `right_shoulder_pitch_joint` (MJCF <joint name>). */
	FName GetJointName(int32 Index) const;
	/** `right_shoulder_pitch` (MJCF <actuator name>, also sensor prefix). */
	FName GetActuatorName(int32 Index) const;

	static FName ToJointName(FName ShortOrFull);
	static FName ToActuatorName(FName ShortOrFull);

	/** Level 0 layout size helper: 3 * NumActions + NumCommands. */
	static int32 Level0ObsSize(int32 InNumActions, int32 InNumCommands) { return 3 * InNumActions + InNumCommands; }

	/**
	 * Built-in Level 0 config (values from RobotControl/configs/r1_joint_map.yaml) so the component can
	 * run against the mock bridge before any UR1DeployConfigAsset exists.
	 */
	static FR1DeployConfig MakeLevel0Default();
};

/**
 * Editor-authored data asset holding one deploy config + the imported ONNX model.
 *
 * Filling it from RobotControl's deploy.yaml (planned helper `scripts/deploy_yaml_to_uasset.py`,
 * not written yet) is a UE-Python job, e.g. run with
 *   UnrealEditor-Cmd.exe R1Twin.uproject -run=pythonscript -script="scripts/deploy_yaml_to_uasset.py <deploy.yaml> /Game/Humanoid/Policies/Level0"
 * and inside the script roughly:
 *   cfg = unreal.R1DeployConfig()
 *   cfg.set_editor_property('joint_names', [unreal.Name(n) for n in y['joint_names']])
 *   cfg.set_editor_property('default_joint_pos', y['default_joint_pos'])          # list[float]
 *   cfg.set_editor_property('kp', y['kp']); cfg.set_editor_property('kd', y['kd'])
 *   scale = y['action_scale']; cfg.set_editor_property('action_scale', scale if isinstance(scale, list) else [scale]*len(y['joint_names']))
 *   cfg.set_editor_property('num_obs', y['num_obs']); cfg.set_editor_property('num_actions', y['num_actions'])
 *   asset = unreal.AssetToolsHelpers.get_asset_tools().create_asset('DA_Level0Deploy', '/Game/Humanoid/Policies', unreal.R1DeployConfigAsset, unreal.DataAssetFactory())
 *   asset.set_editor_property('config', cfg)
 *   asset.set_editor_property('policy_model', unreal.load_asset('/Game/Humanoid/Policies/level0_policy'))  # UNNEModelData from the imported .onnx
 *   unreal.EditorAssetLibrary.save_loaded_asset(asset)
 * (Requires the PythonScriptPlugin enabled in R1Twin.uproject - it is.)
 */
UCLASS(BlueprintType)
class R1TWIN_API UR1DeployConfigAsset : public UDataAsset
{
	GENERATED_BODY()

public:
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Policy")
	FR1DeployConfig Config;

	/** The imported .onnx (drag it into the Content Browser; NNE creates a UNNEModelData). */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Policy")
	TObjectPtr<UNNEModelData> PolicyModel;

	/** Provenance, filled by the import script. */
	UPROPERTY(EditAnywhere, Category = "Provenance")
	FString SourceDeployYaml;

	UPROPERTY(EditAnywhere, Category = "Provenance")
	FString SourceOnnxSha256;

	UPROPERTY(EditAnywhere, Category = "Provenance")
	FString TrainingRunId;

#if WITH_EDITOR
	virtual EDataValidationResult IsDataValid(class FDataValidationContext& Context) const override;
#endif
};
