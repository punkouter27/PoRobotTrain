"""Runs inside the editor: stop the content-directory monitor from auto-importing URLab's generated *_ue.xml files."""
import unreal
s = unreal.get_default_object(unreal.EditorLoadingSavingSettings)
for prop in ("monitor_content_directories", "auto_reimport_textures", "auto_reimport_csv"):
    try:
        s.set_editor_property(prop, False)
        print(prop, "->", s.get_editor_property(prop))
    except Exception as e:
        print(prop, "n/a:", e)
try:
    s.save_config()
except Exception as e:
    print("save_config:", e)
