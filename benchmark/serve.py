"""Benchmark page server (stdlib only). Started by benchmark.ps1 at the repo root; opens the page in the browser.

GET /         the page            GET /history  benchmark/history.json
POST /run     start a Benchmark   GET /status   {running, line} (the latest "BENCH:" progress line)

benchmark.ps1 -Run (or --run): no page; runs one Benchmark in this console, records it in benchmark/history.json like
the page does, prints a summary and exits with the Workload's exit code (0 = recorded, 1 = failed).
"""

from __future__ import annotations

import http.server
import json
import subprocess
import sys
import threading
import webbrowser
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
# Repo-specific: the Python that can import the trainer, and the Workload script it runs.
PYTHON = ROOT / ".venv" / "Scripts" / "python.exe"
WORKLOAD = HERE / "workload.py"
TITLE = "PoRobotTrain"

state = {"proc": None, "line": "idle", "log": []}
lock = threading.Lock()


def pump(proc: subprocess.Popen) -> None:
    for raw in proc.stdout:
        line = raw.rstrip()
        with lock:
            state["log"] = (state["log"] + [line])[-40:]
            if line.startswith("BENCH: "):
                state["line"] = line[7:]
    code = proc.wait()
    with lock:
        if code and not state["line"].startswith("failed"):
            state["line"] = f"failed (exit {code}) - see the log below"
        state["proc"] = None


class Handler(http.server.BaseHTTPRequestHandler):
    def send(self, code: int, body: bytes, ctype: str) -> None:
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self) -> None:  # noqa: N802
        if self.path == "/":
            html = (HERE / "page.html").read_text(encoding="utf-8").replace("{{TITLE}}", TITLE)
            self.send(200, html.encode(), "text/html; charset=utf-8")
        elif self.path == "/history":
            p = HERE / "history.json"
            self.send(200, p.read_bytes() if p.exists() else b'{"schema":1,"runs":[]}', "application/json")
        elif self.path == "/status":
            with lock:
                body = {"running": state["proc"] is not None, "line": state["line"], "log": state["log"][-12:]}
            self.send(200, json.dumps(body).encode(), "application/json")
        else:
            self.send(404, b"not found", "text/plain")

    def do_POST(self) -> None:  # noqa: N802
        if self.path != "/run":
            return self.send(404, b"not found", "text/plain")
        with lock:
            if state["proc"] is not None:
                return self.send(409, b'{"error":"a benchmark is already running"}', "application/json")
            if not PYTHON.exists():
                state["line"] = f"failed: missing {PYTHON.relative_to(ROOT)} (set up the training environment)"
                return self.send(500, json.dumps({"error": state["line"]}).encode(), "application/json")
            proc = subprocess.Popen(
                [str(PYTHON), "-u", str(WORKLOAD)],
                cwd=ROOT,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                encoding="utf-8",
                errors="replace",
                creationflags=0x08000000 if sys.platform == "win32" else 0,
            )
            state.update(proc=proc, line="starting", log=[])
        threading.Thread(target=pump, args=(proc,), daemon=True).start()
        self.send(202, b'{"started":true}', "application/json")

    def log_message(self, *a) -> None:
        pass


def history_runs() -> list:
    try:
        return json.loads((HERE / "history.json").read_text(encoding="utf-8")).get("runs", [])
    except (OSError, ValueError):
        return []


def run_once() -> int:
    """-Run: one Benchmark in this console, no page. The Workload writes the history entry itself."""
    if not PYTHON.exists():
        print(f"BENCH: failed: missing {PYTHON} (set up the training environment)", flush=True)
        return 1
    before = len(history_runs())
    code = subprocess.call([str(PYTHON), "-u", str(WORKLOAD)], cwd=ROOT)
    runs = history_runs()
    if len(runs) <= before:
        print(f"BENCH: failed (exit {code}) - nothing was recorded in benchmark/history.json", flush=True)
        return code or 1
    run = runs[-1]
    r = run.get("result")
    if not r:  # the Workload already printed "BENCH: failed: <error>"
        return code or 1
    m = run.get("machine", {})
    print(
        f"\nBenchmark {TITLE}: {r['step_time_ms']} ms/step, {r['env_steps_per_s']} env steps/s"
        f" (measured {r['measured_s']} s, startup {r['startup_s']} s) on {m.get('gpu') or m.get('cpu')}"
        + f"\nVerdict: {', '.join(run.get('verdict') or []) or 'clean'}"
        + "\nRecorded in benchmark/history.json (commit it to compare machines).",
        flush=True,
    )
    return code


def main() -> None:
    if any(a.lower() in ("-run", "--run") for a in sys.argv[1:]):
        sys.exit(run_once())
    srv = http.server.ThreadingHTTPServer(("127.0.0.1", 0), Handler)
    url = f"http://127.0.0.1:{srv.server_address[1]}/"
    print(f"Benchmark page: {url}  (Ctrl+C to stop)", flush=True)
    if "--no-browser" not in sys.argv:
        webbrowser.open(url)
    srv.serve_forever()


if __name__ == "__main__":
    main()
