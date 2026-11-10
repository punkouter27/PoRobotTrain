"""Runs INSIDE the Unreal editor (via scripts/ue_remote.py). Finalizes /Game/Levels/R1_Stand:

* World Settings GameMode -> R1TwinGameMode (follow camera pawn + in-engine policy host, Chaos disabled)
* Lighting/rendering: sun, real-time SkyLight, SkyAtmosphere, unbound PostProcessVolume with manual exposure
* Visual floor (physics floor stays the MJCF plane), PlayerStart framing the stand
* Tags the URLab articulation actor 'R1Articulation' so R1 components find it deterministically
* Saves every dirty package (level + imported MuJoCo blueprint/meshes)

Everything is idempotent: actors are found by label and updated, never duplicated.
"""
import unreal

LEVEL = "/Game/Levels/R1_Stand"
TAG = "R1Articulation"

les = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
eas = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
ues = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem)

if les.get_current_level() is None or not ues.get_editor_world().get_path_name().startswith(LEVEL):
    les.load_level(LEVEL)
world = ues.get_editor_world()
print("level:", world.get_path_name())


def by_label(label):
    for a in eas.get_all_level_actors():
        if a.get_actor_label() == label:
            return a
    return None


def ensure(cls, label, loc=(0, 0, 0), rot=(0, 0, 0)):
    a = by_label(label)
    if a is None:
        a = eas.spawn_actor_from_class(cls, unreal.Vector(*loc), unreal.Rotator(*rot))
        a.set_actor_label(label)
        print("spawned", label)
    else:
        a.set_actor_location(unreal.Vector(*loc), False, False)
        a.set_actor_rotation(unreal.Rotator(*rot), False)
    return a


# --- GameMode
gm = unreal.load_class(None, "/Script/R1Twin.R1TwinGameMode")
world.get_world_settings().set_editor_property("default_game_mode", gm)
print("GameMode override:", gm.get_name())

# --- articulation tag
art = None
for a in eas.get_all_level_actors():
    if "scene_stand" in a.get_class().get_name():
        art = a
        break
if art is None:
    raise RuntimeError("URLab articulation actor (scene_stand) not found - run scripts/urlab_setup_scene.py first")
tags = list(art.tags)
if TAG not in [str(t) for t in tags]:
    tags.append(unreal.Name(TAG))
    art.tags = tags
print("articulation:", art.get_actor_label(), "tags", [str(t) for t in art.tags])

# --- lighting (Rotator args: roll, pitch, yaw)
sun = None
for a in eas.get_all_level_actors():
    if isinstance(a, unreal.DirectionalLight):
        sun = a
        break
if sun is None:
    sun = ensure(unreal.DirectionalLight, "R1_Sun", (0, 0, 500), (0, -50, 120))
sun.set_actor_rotation(unreal.Rotator(0, -45, 135), False)
sun.light_component.set_editor_property("intensity", 8.0)
sun.light_component.set_editor_property("atmosphere_sun_light", True)

sky = ensure(unreal.SkyLight, "R1_SkyLight", (0, 0, 300))
sky.light_component.set_editor_property("real_time_capture", True)
sky.light_component.set_editor_property("intensity", 1.0)
sky.light_component.set_editor_property("mobility", unreal.ComponentMobility.MOVABLE)
ensure(unreal.SkyAtmosphere, "R1_SkyAtmosphere")

ppv = ensure(unreal.PostProcessVolume, "R1_PostProcess")
ppv.set_editor_property("unbound", True)
s = ppv.settings
s.set_editor_property("override_auto_exposure_method", True)
s.set_editor_property("auto_exposure_method", unreal.AutoExposureMethod.AEM_MANUAL)
s.set_editor_property("override_auto_exposure_apply_physical_camera_exposure", True)
s.set_editor_property("auto_exposure_apply_physical_camera_exposure", False)
s.set_editor_property("override_auto_exposure_bias", True)
s.set_editor_property("auto_exposure_bias", -2.5)
ppv.settings = s

# --- visual floor (no collision; MuJoCo owns contact)
floor = ensure(unreal.StaticMeshActor, "R1_Floor", (0, 0, 0))
smc = floor.static_mesh_component
smc.set_static_mesh(unreal.load_asset("/Engine/BasicShapes/Plane"))
floor.set_actor_scale3d(unreal.Vector(400, 400, 1))  # 400 m: no visible floor edge
smc.set_collision_enabled(unreal.CollisionEnabled.NO_COLLISION)
mat = unreal.load_asset("/Engine/MapTemplates/Materials/BasicGrid01") or unreal.load_asset("/Engine/BasicShapes/BasicShapeMaterial")
if mat:
    smc.set_material(0, mat)

# --- PlayerStart: MuJoCo (1.9, -1.6, 1.3) m -> UE (190, 160, 130) cm, looking back at the stand
ensure(unreal.PlayerStart, "R1_PlayerStart", (190, 160, 130), (0, -8, -139))

saved = unreal.EditorLoadingAndSavingUtils.save_dirty_packages(True, True)
print("saved dirty packages:", saved)
