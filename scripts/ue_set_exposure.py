"""Runs inside the editor during PIE: live-tune exposure on the R1_PostProcess volume (editor + PIE worlds)."""
import unreal
BIAS = -2.5
eas = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
worlds = [unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem).get_editor_world()]
try:
    worlds.append(unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem).get_game_world())
except Exception:
    pass
for w in worlds:
    if w is None:
        continue
    for a in unreal.GameplayStatics.get_all_actors_of_class(w, unreal.PostProcessVolume):
        s = a.settings
        s.set_editor_property("override_auto_exposure_bias", True)
        s.set_editor_property("auto_exposure_bias", BIAS)
        a.settings = s
        print(w.get_name(), a.get_actor_label(), "bias", BIAS)
