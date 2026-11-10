"""Runs inside the editor during PIE: stop per-frame scene captures (robot head camera) when no consumer needs them."""
import unreal
w = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem).get_game_world()
n = 0
for a in unreal.GameplayStatics.get_all_actors_of_class(w, unreal.Actor):
    for c in a.get_components_by_class(unreal.SceneCaptureComponent2D):
        c.set_editor_property("capture_every_frame", False)
        c.set_editor_property("capture_on_movement", False)
        n += 1
print("scene captures disabled:", n)
