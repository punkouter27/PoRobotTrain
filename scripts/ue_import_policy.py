"""Runs inside the editor: import the trained ONNX as a UNNEModelData asset at /Game/Project/RL_Policies."""

import pathlib

import unreal

# The project dir is <repo>/UnrealProject/; the policy lives in <repo>/RobotControl/models/.
PROJECT_DIR = pathlib.Path(unreal.Paths.convert_relative_path_to_full(unreal.Paths.project_dir()))
SRC = str(PROJECT_DIR.parent / "RobotControl" / "models" / "r1_level0_upperbody" / "policy.onnx")
DEST = "/Game/Project/RL_Policies"
task = unreal.AssetImportTask()
task.filename = SRC
task.destination_path = DEST
task.destination_name = "r1_level0_upperbody"
task.automated = True
task.replace_existing = True
task.save = True
unreal.AssetToolsHelpers.get_asset_tools().import_asset_tasks([task])
print("imported:", list(task.imported_object_paths))
a = unreal.load_asset(DEST + "/r1_level0_upperbody")
print("asset:", a, type(a).__name__ if a else None)
