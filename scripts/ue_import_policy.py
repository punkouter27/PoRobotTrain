"""Runs inside the editor: import the trained ONNX as a UNNEModelData asset at /Game/Humanoid/Policies."""
import unreal, os
SRC = r"C:/Users/punko/Downloads/CODE_MATT/PoRobotTrain/RobotControl/models/r1_level0_upperbody/policy.onnx"
DEST = "/Game/Humanoid/Policies"
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
