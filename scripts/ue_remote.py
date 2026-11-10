"""Run a Python file inside the open Unreal editor via the PythonScriptPlugin remote-execution protocol.

Requires [/Script/PythonScriptPlugin.PythonScriptPluginSettings] bRemoteExecution=True (set in
UnrealProject/Config/DefaultEngine.ini). Uses the engine's own client module, remote_execution.py.

Usage: .venv/Scripts/python.exe scripts/ue_remote.py scripts/ue_finalize_level.py [--timeout 60]
"""
from __future__ import annotations

import argparse
import importlib.util
import pathlib
import sys
import time

UE_ROOT = pathlib.Path(r"C:\Program Files\Epic Games\UE_5.8")
RE_PATH = UE_ROOT / "Engine/Plugins/Experimental/PythonScriptPlugin/Content/Python/remote_execution.py"


def load_remote_execution():
    spec = importlib.util.spec_from_file_location("remote_execution", RE_PATH)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("script", type=pathlib.Path)
    ap.add_argument("--timeout", type=float, default=60.0)
    args = ap.parse_args()

    re_mod = load_remote_execution()
    cfg = re_mod.RemoteExecutionConfig()
    cfg.multicast_bind_address = "127.0.0.1"
    rex = re_mod.RemoteExecution(cfg)
    rex.start()
    try:
        deadline = time.time() + args.timeout
        while not rex.remote_nodes and time.time() < deadline:
            time.sleep(0.5)
        if not rex.remote_nodes:
            sys.exit("no Unreal editor answered remote-execution discovery (is bRemoteExecution enabled and the editor open?)")
        node = rex.remote_nodes[0]
        print("editor node:", node.get("project_name"), node.get("engine_version"))
        rex.open_command_connection(node["node_id"])
        res = rex.run_command(str(args.script.resolve()).replace("\\", "/"), unattended=True,
                              exec_mode=re_mod.MODE_EXEC_FILE)
        for line in res.get("output", []):
            print(line.get("output", "").rstrip())
        if not res.get("success"):
            print("REMOTE ERROR:", res.get("result"))
            sys.exit(1)
    finally:
        rex.stop()


if __name__ == "__main__":
    main()
