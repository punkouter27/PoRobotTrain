"""Training benchmark core: machine detection, machine load, the step meter and benchmark/history.json.

Stdlib only, so it runs in any of the repo's Python environments. The repo-specific part (how to start the real
training loop and where to hook its per-iteration timings) lives in workload.py next to this file.
Terms (Step, Step time, Workload, Machine load, ...) are defined in GLOSSARY.md.
"""

from __future__ import annotations

import ctypes
import datetime
import hashlib
import json
import os
import platform
import socket
import subprocess
import sys
import threading
import time
from pathlib import Path

T_PROCESS = time.time()  # imported first by workload.py, so Startup time covers env build and JIT/graph compile
HERE = Path(__file__).resolve().parent
HISTORY = HERE / "history.json"
MEASURE_S = 10.0  # measured window (after warm-up)
BUSY_GPU_PCT = 20  # load_before GPU % above which a result is flagged busy


def say(msg: str) -> None:
    """One progress line for the Benchmark page (serve.py shows the latest one)."""
    print(f"BENCH: {msg}", flush=True)


# ---------------------------------------------------------------- machine


def _run(cmd: list[str], timeout: float = 10) -> str:
    try:
        flags = 0x08000000 if os.name == "nt" else 0  # CREATE_NO_WINDOW
        return subprocess.run(cmd, capture_output=True, text=True, timeout=timeout, creationflags=flags).stdout
    except (OSError, subprocess.SubprocessError):
        return ""


def _cpu_name() -> str:
    if os.name == "nt":
        try:
            import winreg

            k = winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, r"HARDWARE\DESCRIPTION\System\CentralProcessor\0")
            return winreg.QueryValueEx(k, "ProcessorNameString")[0].strip()
        except OSError:
            pass
    return platform.processor() or "unknown"


class _MemStatus(ctypes.Structure):
    _fields_ = [("dwLength", ctypes.c_ulong), ("dwMemoryLoad", ctypes.c_ulong)] + [
        (n, ctypes.c_ulonglong)
        for n in ("total", "avail", "pagetotal", "pageavail", "virttotal", "virtavail", "extavail")
    ]


def _ram() -> tuple[float, float]:
    """(total GB, used GB)."""
    if os.name != "nt":
        return 0.0, 0.0
    m = _MemStatus()
    m.dwLength = ctypes.sizeof(m)
    ctypes.windll.kernel32.GlobalMemoryStatusEx(ctypes.byref(m))
    return round(m.total / 2**30, 1), round((m.total - m.avail) / 2**30, 1)


def _cpu_times() -> tuple[int, int]:
    """(idle, total) 100-ns ticks since boot."""
    if os.name != "nt":
        return 0, 0
    idle, kernel, user = (ctypes.c_ulonglong() for _ in range(3))
    ctypes.windll.kernel32.GetSystemTimes(ctypes.byref(idle), ctypes.byref(kernel), ctypes.byref(user))
    return idle.value, kernel.value + user.value  # kernel time includes idle


def _on_battery() -> bool | None:
    if os.name != "nt":
        return None

    class S(ctypes.Structure):
        _fields_ = [
            ("ac", ctypes.c_byte),
            ("flag", ctypes.c_byte),
            ("pct", ctypes.c_byte),
            ("r", ctypes.c_byte),
            ("life", ctypes.c_ulong),
            ("full", ctypes.c_ulong),
        ]

    s = S()
    if not ctypes.windll.kernel32.GetSystemPowerStatus(ctypes.byref(s)):
        return None
    return {0: True, 1: False}.get(s.ac)


def _gpu_query() -> dict | None:
    out = _run(
        [
            "nvidia-smi",
            "--query-gpu=name,memory.total,memory.used,utilization.gpu,temperature.gpu,driver_version,clocks.sm",
            "--format=csv,noheader,nounits",
        ]
    )
    line = out.strip().splitlines()[0] if out.strip() else ""
    if not line:
        return None
    f = [x.strip() for x in line.split(",")]
    num = lambda s: float(s) if s.replace(".", "", 1).isdigit() else None  # noqa: E731
    return dict(
        name=f[0],
        vram_total_gb=round(num(f[1]) / 1024, 1) if num(f[1]) else None,
        vram_used_gb=round(num(f[2]) / 1024, 1) if num(f[2]) is not None else None,
        gpu_pct=num(f[3]),
        gpu_temp_c=num(f[4]),
        driver=f[5],
        sm_clock_mhz=num(f[6]),
    )


def _gpu_processes() -> list[str]:
    """Other processes holding the GPU for compute (graphics-only apps are not listed by nvidia-smi on Windows)."""
    out = _run(["nvidia-smi", "--query-compute-apps=pid,process_name", "--format=csv,noheader"])
    me = os.getpid()
    procs = []
    for line in out.strip().splitlines():
        pid, _, name = line.partition(",")
        if pid.strip().isdigit() and int(pid) != me:
            procs.append(Path(name.strip()).name)
    return sorted(set(procs))


def machine() -> dict:
    g = _gpu_query() or {}
    ram_total, _ = _ram()
    return dict(
        hostname=socket.gethostname(),
        cpu=f"{_cpu_name()} ({os.cpu_count()} threads)",
        gpu=g.get("name"),
        vram_gb=g.get("vram_total_gb"),
        ram_gb=ram_total,
        gpu_driver=g.get("driver"),
        os=_os_name(),
    )


def _os_name() -> str:
    ver = platform.version()
    if os.name == "nt":  # Python reports Windows 11 as release "10"; build 22000+ is 11
        build = int(ver.split(".")[-1]) if ver.split(".")[-1].isdigit() else 0
        return f"Windows {'11' if build >= 22000 else platform.release()} ({ver})"
    return f"{platform.system()} {platform.release()} ({ver})"


def load_snapshot(cpu_window_s: float = 0.5) -> dict:
    i0, t0 = _cpu_times()
    time.sleep(cpu_window_s)
    i1, t1 = _cpu_times()
    g = _gpu_query() or {}
    _, ram_used = _ram()
    return dict(
        cpu_pct=round(100 * (1 - (i1 - i0) / max(t1 - t0, 1))),
        ram_used_gb=ram_used,
        gpu_pct=g.get("gpu_pct"),
        vram_used_gb=g.get("vram_used_gb"),
        gpu_temp_c=g.get("gpu_temp_c"),
        sm_clock_mhz=g.get("sm_clock_mhz"),
        on_battery=_on_battery(),
        gpu_processes=_gpu_processes(),
    )


class LoadSampler(threading.Thread):
    """Samples machine load once a second while the measured window runs; summary() = means (peak VRAM/temp)."""

    def __init__(self):
        super().__init__(daemon=True)
        self.samples: list[dict] = []
        self._stop = threading.Event()

    def run(self):
        while not self._stop.is_set():
            self.samples.append(load_snapshot(0.5))
            self._stop.wait(0.5)

    def summary(self) -> dict:
        self._stop.set()
        s = self.samples
        if not s:
            return {}
        mean = lambda k: round(sum(x[k] for x in s if x[k] is not None) / max(1, sum(x[k] is not None for x in s)), 1)  # noqa: E731
        peak = lambda k: max((x[k] for x in s if x[k] is not None), default=None)  # noqa: E731
        return dict(
            cpu_pct=mean("cpu_pct"),
            ram_used_gb=peak("ram_used_gb"),
            gpu_pct=mean("gpu_pct"),
            vram_used_gb=peak("vram_used_gb"),
            gpu_temp_c=peak("gpu_temp_c"),
            sm_clock_mhz=mean("sm_clock_mhz"),
            on_battery=s[-1]["on_battery"],
            samples=len(s),
        )


def is_busy(before: dict) -> bool:
    """Was the Machine already busy enough before the run that its Step time should be discounted?
    gpu_processes is only a note: on Windows nvidia-smi also lists desktop apps (explorer, browsers) that barely use it."""
    return (before.get("gpu_pct") or 0) > BUSY_GPU_PCT


# ---------------------------------------------------------------- history


def read_history() -> dict:
    try:
        return json.loads(HISTORY.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return {"schema": 1, "runs": []}


def workload_id(workload: dict) -> str:
    return hashlib.sha1(json.dumps(workload, sort_keys=True, default=str).encode()).hexdigest()[:6]


def workload_changes(prev: dict | None, cur: dict) -> list[str]:
    """Human-readable notes on how this Workload differs from the previous run's (empty = same parameters)."""
    if not prev:
        return []
    keys = sorted(set(prev) | set(cur))
    return [f"{k}: {prev.get(k)!r} -> {cur.get(k)!r}" for k in keys if prev.get(k) != cur.get(k)]


def git_commit() -> str | None:
    return _run(["git", "-C", str(HERE.parent), "rev-parse", "--short", "HEAD"]).strip() or None


def append_run(entry: dict) -> None:
    h = read_history()
    prev = h["runs"][-1]["workload"] if h["runs"] else None
    entry["workload_changes"] = workload_changes(prev, entry["workload"])
    h["runs"].append(entry)
    tmp = HISTORY.with_suffix(".tmp")
    tmp.write_text(json.dumps(h, indent=1), encoding="utf-8")
    os.replace(tmp, HISTORY)


# ---------------------------------------------------------------- meter


class Meter:
    """Fed one block of Steps at a time (an rsl_rl iteration, an ML-Agents env step...). Warm-up ends after
    `warmup_blocks` blocks, or (warmup_blocks=None) when the adapter calls end_warmup(); then it measures MEASURE_S
    seconds, writes the history entry, runs the on_exit callbacks (close env players...) and ends the process."""

    def __init__(self, workload: dict, versions: dict, warmup_blocks: int | None = 2):
        self.workload, self.versions, self.warmup_blocks = workload, versions, warmup_blocks
        self.t_process = T_PROCESS
        self.when = datetime.datetime.now().astimezone().isoformat(timespec="seconds")
        self.machine = machine()
        say("reading machine load before the run")
        self.load_before = load_snapshot(1.0)
        self.blocks = 0
        self.sim = self.learn = 0.0
        self.steps = 0
        self.startup_s = None
        self.sampler: LoadSampler | None = None
        self.on_exit: list = []
        # Adapters whose learning happens in bursts (ML-Agents updates once per buffer) end the window on a cycle
        # boundary; capped at 6x the window so a stalled update cannot hang the Benchmark.
        self.can_finish = lambda: True
        self._said = 0.0
        say("warm-up (starting the training loop)")

    @property
    def measuring(self) -> bool:
        return self.sampler is not None

    def end_warmup(self) -> None:
        if self.measuring:
            return
        self.startup_s = round(time.time() - self.t_process, 1)
        self.sampler = LoadSampler()
        self.sampler.start()
        say(f"measuring for {MEASURE_S:.0f} s")

    def block(self, sim_s: float, learn_s: float, steps: float) -> None:
        self.blocks += 1
        if not self.measuring:
            if self.warmup_blocks is not None and self.blocks >= self.warmup_blocks:
                self.end_warmup()
            return
        self.sim += sim_s
        self.learn += learn_s
        self.steps += steps
        total = self.sim + self.learn
        if total >= MEASURE_S and (self.can_finish() or total >= 6 * MEASURE_S):
            self.finish(ok=True)
        elif time.time() - self._said > 1:
            self._said = time.time()
            say(f"measuring {total:4.1f} / {MEASURE_S:.0f} s")

    def finish(self, ok: bool, error: str | None = None) -> None:
        total = self.sim + self.learn
        entry = dict(
            when=self.when,
            ok=ok,
            error=error,
            machine=self.machine,
            load_before=self.load_before,
            load_during=self.sampler.summary() if self.sampler else {},
            busy=is_busy(self.load_before),
            workload_id=workload_id(self.workload),
            workload=self.workload,
            result=None,
            git_commit=git_commit(),
            versions=self.versions,
        )
        if ok and self.steps:
            entry["result"] = dict(
                step_time_ms=round(1000 * total / self.steps, 3),
                sim_ms=round(1000 * self.sim / self.steps, 3),
                learn_ms=round(1000 * self.learn / self.steps, 3),
                env_steps_per_s=round(self.steps * self.workload.get("num_envs", 1) / total),
                steps_measured=round(self.steps, 1),
                measured_s=round(total, 2),
                startup_s=self.startup_s,
            )
        append_run(entry)
        r = entry["result"]
        say(f"done: {r['step_time_ms']} ms/step" if r else f"failed: {error}")
        for cb in self.on_exit:
            try:
                cb()
            except Exception:  # noqa: BLE001 - cleanup must never lose the recorded result
                pass
        sys.stdout.flush()
        os._exit(0 if ok else 1)  # stop the training loop right here; nothing after the window matters


def fail_text(exc: BaseException) -> str:
    text = f"{type(exc).__name__}: {exc}"
    if "out of memory" in text.lower() or "OutOfMemory" in text:
        text = "out of VRAM: " + text
    return text[:400]
