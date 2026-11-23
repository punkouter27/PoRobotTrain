"""Runs INSIDE the Unreal editor (via scripts/ue_remote.py): re-import UnrealProject/MJCF/scene_stand.xml in place.

Use after scripts/build_mjcf.py changes the stand scene (for example the stand's back hook). Unlike
scripts/urlab_setup_scene.py (whose URLab bridge imports into /Game/MuJoCoImports and creates /Game/Levels), this keeps
the project layout: the asset is replaced at /Game/Project/Agents/MuJoCoImports/scene_stand and the articulation is
re-placed in /Game/Project/Environments/Levels/R1_Stand. Run scripts/ue_finalize_level.py afterwards (tag, game
mode, lights).

  .venv/Scripts/python.exe scripts/ue_remote.py scripts/ue_reimport_stand.py --timeout 300
  .venv/Scripts/python.exe scripts/ue_remote.py scripts/ue_finalize_level.py

URLab asks once which Python preprocesses meshes; the answer is stored in the plugin's
Config/LocalUnrealRoboticsLab.ini (PythonPath=...). Without it the import is cancelled in unattended runs.
"""

import pathlib

import unreal

LEVEL = "/Game/Project/Environments/Levels/R1_Stand"
DEST = "/Game/Project/Agents/MuJoCoImports"
NAME = "scene_stand"

project_dir = pathlib.Path(unreal.Paths.convert_relative_path_to_full(unreal.Paths.project_dir()))
xml = project_dir / "MJCF" / "scene_stand.xml"
if not xml.exists():
    raise RuntimeError(f"{xml} not found - run scripts/build_mjcf.py first")

les = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
eas = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
if not les.load_level(LEVEL):
    raise RuntimeError(f"could not load {LEVEL}")

# the old articulation goes first, so nothing references the asset while it is replaced
loc, rot, label = unreal.Vector(0, 0, 0), unreal.Rotator(0, 0, 0), "scene_stand0"
for a in eas.get_all_level_actors():
    if NAME in a.get_class().get_name():
        loc, rot, label = a.get_actor_location(), a.get_actor_rotation(), a.get_actor_label()
        print("removed", label)
        eas.destroy_actor(a)

# URLab's factory refuses to overwrite a Blueprint (its own force re-import destroys it first); do the same, and
# clear the previous import's meshes and materials. With scripts/patches/urlab_import_folder.patch URLab writes them
# next to the Blueprint (<DEST>/<NAME>_ue_Assets) instead of /Game/MuJoCoImports.
lib = unreal.EditorAssetLibrary
bp_path = f"{DEST}/{NAME}"
if lib.does_asset_exist(bp_path):
    print("deleted old", bp_path, lib.delete_asset(bp_path))
if lib.does_directory_exist(f"{DEST}/{NAME}_ue_Assets"):
    print("deleted old assets", lib.delete_directory(f"{DEST}/{NAME}_ue_Assets"))

task = unreal.AssetImportTask()
task.filename = str(xml).replace("\\", "/")
task.destination_path = DEST
task.destination_name = NAME
task.replace_existing = True
task.automated = True
task.save = True
unreal.AssetToolsHelpers.get_asset_tools().import_asset_tasks([task])
paths = list(task.imported_object_paths)
print("imported:", paths)
if lib.does_directory_exist(f"/Game/MuJoCoImports/{NAME}_ue_Assets"):
    raise RuntimeError("URLab wrote the meshes to /Game/MuJoCoImports: apply scripts/patches/urlab_import_folder.patch")
if not paths:
    raise RuntimeError("import failed (see the URLab log lines in Saved/Logs/R1Twin.log)")

bp_class = unreal.EditorAssetLibrary.load_blueprint_class(f"{DEST}/{NAME}")
actor = eas.spawn_actor_from_class(bp_class, loc, rot)
actor.set_actor_label(label)
print("placed", actor.get_actor_label(), "class", actor.get_class().get_name())
les.save_current_level()
unreal.EditorLoadingAndSavingUtils.save_dirty_packages(True, True)
print("saved")
