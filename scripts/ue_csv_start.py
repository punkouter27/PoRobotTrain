import unreal
w = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem).get_game_world()
unreal.SystemLibrary.execute_console_command(w, "csvprofile start")
print("csvprofile start in", w.get_name() if w else None)
