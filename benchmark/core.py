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
import re
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


class _CoreTimes(ctypes.Structure):  # SYSTEM_PROCESSOR_PERFORMANCE_INFORMATION
    _fields_ = [(n, ctypes.c_longlong) for n in ("idle", "kernel", "user", "dpc", "interrupt")] + [
        ("interrupts", ctypes.c_ulong)
    ]


def _core_times() -> list[tuple[int, int]]:
    """(idle, total) 100-ns ticks per logical core (the first processor group, up to 64 cores)."""
    if os.name != "nt":
        return []
    n = min(os.cpu_count() or 1, 64)
    buf = (_CoreTimes * n)()
    got = ctypes.c_ulong()
    if ctypes.windll.ntdll.NtQuerySystemInformation(8, ctypes.byref(buf), ctypes.sizeof(buf), ctypes.byref(got)):
        return []
    return [(c.idle, c.kernel + c.user) for c in buf[: got.value // ctypes.sizeof(_CoreTimes)]]


# The trainer's own CPU use, read from the thread that runs the training loop (Meter's constructor registers it).
# A workload whose trainer runs in another process (a container) sets TRAINER_IN_PROCESS = False before Meter.
TRAINER_IN_PROCESS = True
_trainer_thread = None


def _k32():
    k = ctypes.windll.kernel32
    k.GetCurrentProcess.restype = k.GetCurrentThread.restype = ctypes.c_void_p
    k.DuplicateHandle.argtypes = [
        ctypes.c_void_p,
        ctypes.c_void_p,
        ctypes.c_void_p,
        ctypes.POINTER(ctypes.c_void_p),
        ctypes.c_ulong,
        ctypes.c_int,
        ctypes.c_ulong,
    ]
    k.GetThreadTimes.argtypes = k.GetProcessTimes.argtypes = [ctypes.c_void_p] + [ctypes.c_void_p] * 4
    return k


def register_trainer_thread() -> None:
    """Remember the calling thread (the training loop) so the load sampler can time it from its own thread."""
    global _trainer_thread
    if os.name != "nt" or not TRAINER_IN_PROCESS:
        return
    k, h = _k32(), ctypes.c_void_p()
    me = k.GetCurrentProcess()
    if k.DuplicateHandle(me, k.GetCurrentThread(), me, ctypes.byref(h), 0, 0, 2):  # DUPLICATE_SAME_ACCESS
        _trainer_thread = h


def _own_times() -> tuple[int, int] | None:
    """(this process, the trainer thread) CPU ticks, kernel + user."""
    if os.name != "nt" or not TRAINER_IN_PROCESS:
        return None
    k = _k32()
    c, e, kt, ut = (ctypes.c_ulonglong() for _ in range(4))
    k.GetProcessTimes(k.GetCurrentProcess(), ctypes.byref(c), ctypes.byref(e), ctypes.byref(kt), ctypes.byref(ut))
    proc = kt.value + ut.value
    thread = 0
    if _trainer_thread:
        k.GetThreadTimes(_trainer_thread, ctypes.byref(c), ctypes.byref(e), ctypes.byref(kt), ctypes.byref(ut))
        thread = kt.value + ut.value
    return proc, thread


def context() -> dict:
    """Machine settings that change speed without changing the hardware: power plan, power mode, GPU scheduling."""
    plan = re.search(r"\((.+)\)", _run(["powercfg", "/getactivescheme"]))
    mode = hags = None
    if os.name == "nt":
        import winreg

        modes = {
            "ded574b5-45a0-4f42-8737-46345c09c238": "best performance",
            "961cc777-2547-4f9d-8174-7d86181b8a7a": "best power efficiency",
            "00000000-0000-0000-0000-000000000000": "balanced",
        }
        try:
            k = winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, r"SYSTEM\CurrentControlSet\Control\Power\User\PowerSchemes")
            key = "ActiveOverlayDcPowerScheme" if _on_battery() else "ActiveOverlayAcPowerScheme"
            guid = str(winreg.QueryValueEx(k, key)[0]).lower()
            mode = modes.get(guid, guid)
        except OSError:
            pass
        try:
            k = winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, r"SYSTEM\CurrentControlSet\Control\GraphicsDrivers")
            hags = {2: True, 1: False}.get(winreg.QueryValueEx(k, "HwSchMode")[0])
        except OSError:
            pass
    return dict(power_plan=plan.group(1) if plan else None, power_mode=mode, gpu_scheduling=hags)


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


_GPU_BASE = "name,memory.total,memory.used,utilization.gpu,temperature.gpu,driver_version,clocks.sm"
# Why the GPU is not at full speed, and how hard it may run. An older driver that rejects any of these fields fails
# the whole query, so _gpu_query() then falls back to _GPU_BASE alone.
_GPU_EXTRA = (
    "clocks.max.sm,power.draw,enforced.power.limit,pstate,utilization.memory,display_active,"
    "pcie.link.gen.current,pcie.link.width.current,clocks_event_reasons.active"
)
# nvidia-smi clocks_event_reasons bits (NVML nvmlClocksEventReason*); "idle" is left out: it is not a slowdown.
_CLOCK_REASONS = {
    0x2: "app clocks",
    0x4: "power cap",
    0x8: "hw slowdown",
    0x10: "sync boost",
    0x20: "thermal (sw)",
    0x40: "thermal (hw)",
    0x80: "power brake",
    0x100: "display clocks",
}


def _gpu_query() -> dict | None:
    def query(fields: str) -> list[str]:
        out = _run(["nvidia-smi", f"--query-gpu={fields}", "--format=csv,noheader,nounits"])
        line = out.strip().splitlines()[0] if out.strip() else ""
        f = [x.strip() for x in line.split(",")] if line else []
        return f if len(f) == fields.count(",") + 1 else []

    f = query(f"{_GPU_BASE},{_GPU_EXTRA}") or query(_GPU_BASE)
    if not f:
        return None
    num = lambda s: float(s) if s.replace(".", "", 1).isdigit() else None  # noqa: E731
    g = dict(
        name=f[0],
        vram_total_gb=round(num(f[1]) / 1024, 1) if num(f[1]) else None,
        vram_used_gb=round(num(f[2]) / 1024, 1) if num(f[2]) is not None else None,
        gpu_pct=num(f[3]),
        gpu_temp_c=num(f[4]),
        driver=f[5],
        sm_clock_mhz=num(f[6]),
    )
    if len(f) > 7:
        try:
            bits = int(f[15], 16)
        except ValueError:
            bits = None
        g.update(
            sm_clock_max_mhz=num(f[7]),
            power_w=num(f[8]),
            power_limit_w=num(f[9]),
            pstate=f[10] if f[10].startswith("P") else None,
            gpu_mem_pct=num(f[11]),
            display_active=f[12] == "Enabled" if f[12] in ("Enabled", "Disabled") else None,
            pcie=f"gen {f[13]} x{f[14]}" if f[13].isdigit() and f[14].isdigit() else None,
            clock_reasons=None if bits is None else [name for bit, name in _CLOCK_REASONS.items() if bits & bit],
        )
    return g


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
    cores0, own0, w0 = _core_times(), _own_times(), time.perf_counter()
    time.sleep(cpu_window_s)
    i1, t1 = _cpu_times()
    cores1, own1, window = _core_times(), _own_times(), time.perf_counter() - w0
    g = _gpu_query() or {}
    _, ram_used = _ram()
    busiest = max(
        (100 * (1 - (b[0] - a[0]) / max(b[1] - a[1], 1)) for a, b in zip(cores0, cores1, strict=False)), default=None
    )
    ticks = window * 1e7  # 100-ns ticks in the window
    return dict(
        cpu_pct=round(100 * (1 - (i1 - i0) / max(t1 - t0, 1))),
        cpu_max_core_pct=None if busiest is None else round(busiest),
        trainer_cpu_cores=round((own1[0] - own0[0]) / ticks, 2) if own0 and own1 else None,
        trainer_thread_pct=round(100 * (own1[1] - own0[1]) / ticks) if own0 and own1 and _trainer_thread else None,
        ram_used_gb=ram_used,
        gpu_pct=g.get("gpu_pct"),
        gpu_mem_pct=g.get("gpu_mem_pct"),
        vram_used_gb=g.get("vram_used_gb"),
        gpu_temp_c=g.get("gpu_temp_c"),
        sm_clock_mhz=g.get("sm_clock_mhz"),
        sm_clock_max_mhz=g.get("sm_clock_max_mhz"),
        power_w=g.get("power_w"),
        power_limit_w=g.get("power_limit_w"),
        pstate=g.get("pstate"),
        clock_reasons=g.get("clock_reasons"),
        display_active=g.get("display_active"),
        pcie=g.get("pcie"),
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

        def mean(k):
            v = [x[k] for x in s if x.get(k) is not None]
            return round(sum(v) / len(v), 1) if v else None

        def peak(k):
            return max((x[k] for x in s if x.get(k) is not None), default=None)

        def last(k):
            return next((x[k] for x in reversed(s) if x.get(k) is not None), None)

        clock, clock_max = mean("sm_clock_mhz"), last("sm_clock_max_mhz")
        with_reasons = [x["clock_reasons"] for x in s if x.get("clock_reasons") is not None]
        reasons = {r: round(sum(r in x for x in with_reasons) / len(with_reasons), 2) for x in with_reasons for r in x}
        pstates = [x["pstate"] for x in s if x.get("pstate")]
        return dict(
            cpu_pct=mean("cpu_pct"),
            cpu_max_core_pct=mean("cpu_max_core_pct"),
            trainer_cpu_cores=mean("trainer_cpu_cores"),
            trainer_thread_pct=mean("trainer_thread_pct"),
            ram_used_gb=peak("ram_used_gb"),
            gpu_pct=mean("gpu_pct"),
            gpu_mem_pct=mean("gpu_mem_pct"),
            vram_used_gb=peak("vram_used_gb"),
            gpu_temp_c=peak("gpu_temp_c"),
            sm_clock_mhz=clock,
            sm_clock_max_mhz=clock_max,
            clock_pct_of_max=round(100 * clock / clock_max) if clock and clock_max else None,
            power_w=mean("power_w"),
            power_limit_w=last("power_limit_w"),
            pstate=max(set(pstates), key=pstates.count) if pstates else None,
            clock_reasons=reasons if with_reasons else None,  # share of samples each slowdown reason was active
            display_active=last("display_active"),
            pcie=last("pcie"),
            on_battery=s[-1]["on_battery"],
            samples=len(s),
        )


def is_busy(before: dict) -> bool:
    """Was the Machine already busy enough before the run that its Step time should be discounted?
    gpu_processes is only a note: on Windows nvidia-smi also lists desktop apps (explorer, browsers) that barely use it."""
    return (before.get("gpu_pct") or 0) > BUSY_GPU_PCT


HOT_C = 85  # GPU temperature at which laptop GPUs start pulling their clocks down
SLOWING_PCT = 15  # Sim time per step, last quarter of the window vs the first


def verdict(entry: dict) -> list[str]:
    """Why this result may not show what the machine can do. Empty = clean. "at power limit" is a note, not a fault:
    a GPU under full load often sits at its power limit."""
    d, r = entry.get("load_during") or {}, entry.get("result") or {}
    reasons = d.get("clock_reasons") or {}
    share = lambda *names: max((reasons.get(n, 0) for n in names), default=0)  # noqa: E731
    out = []
    if entry.get("busy"):
        out.append("busy before")
    if d.get("on_battery"):
        out.append("on battery")
    thermal = share("thermal (sw)", "thermal (hw)") >= 0.25 or (d.get("gpu_temp_c") or 0) >= HOT_C
    if thermal:
        out.append("throttled (thermal)")
    if share("hw slowdown", "power brake") >= 0.25:
        out.append("throttled (hw slowdown)")
    if not thermal and (d.get("clock_pct_of_max") or 100) < 70:
        out.append("low clock")
    gpu_idle = (d.get("gpu_pct") or 100) < 80
    if gpu_idle and ((d.get("trainer_thread_pct") or 0) >= 85 or (d.get("cpu_max_core_pct") or 0) >= 90):
        out.append("CPU-bound")
    if (r.get("sim_drift_pct") or 0) >= SLOWING_PCT:
        out.append("slowing down")
    if share("power cap") >= 0.25:
        out.append("at power limit")
    return out


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
        self.context = context()
        register_trainer_thread()
        say("reading machine load before the run")
        self.load_before = load_snapshot(1.0)
        self.blocks = 0
        self.timeline: list[tuple[float, float, float]] = []  # measured blocks: (sim s, learn s, steps)
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
        self.timeline.append((sim_s, learn_s, steps))
        total = self.sim + self.learn
        if total >= MEASURE_S and (self.can_finish() or total >= 6 * MEASURE_S):
            self.finish(ok=True)
        elif time.time() - self._said > 1:
            self._said = time.time()
            say(f"measuring {total:4.1f} / {MEASURE_S:.0f} s")

    def _spread(self) -> dict:
        return _spread_of(self.timeline)

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
            context=self.context,
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
                **self._spread(),
            )
        entry["verdict"] = verdict(entry) if entry["result"] else []
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


def _quantile(values: list[float], q: float) -> float:
    v = sorted(values)
    return v[min(len(v) - 1, round(q * (len(v) - 1)))]


def _spread_of(timeline: list[tuple[float, float, float]]) -> dict:
    """Step time per measured block (p50 / p95), and how Sim time per step moved from the first quarter of the window
    to the last (heat soak or throttling shows as a rise). Sim only: learning comes in bursts (one PPO update per
    buffer) that would make a quarter's mean depend on where the update landed."""
    blocks = [(s, lr, n) for s, lr, n in timeline if n > 0]
    if len(blocks) < 4:
        return {}
    per_step = [1000 * (s + lr) / n for s, lr, n in blocks]
    elapsed, total, first, last = 0.0, sum(s + lr for s, lr, _ in blocks), [0.0, 0.0], [0.0, 0.0]
    for s, lr, n in blocks:
        mid = (elapsed + (s + lr) / 2) / total
        elapsed += s + lr
        if mid <= 0.25:
            first[0], first[1] = first[0] + s, first[1] + n
        elif mid >= 0.75:
            last[0], last[1] = last[0] + s, last[1] + n
    drift = None
    if first[1] and last[1] and first[0] > 0:
        drift = round(100 * ((last[0] / last[1]) / (first[0] / first[1]) - 1), 1)
    return dict(
        step_p50_ms=round(_quantile(per_step, 0.5), 3),
        step_p95_ms=round(_quantile(per_step, 0.95), 3),
        sim_drift_pct=drift,
        blocks_measured=len(blocks),
    )


def fail_text(exc: BaseException) -> str:
    text = f"{type(exc).__name__}: {exc}"
    if "out of memory" in text.lower() or "OutOfMemory" in text:
        text = "out of VRAM: " + text
    return text[:400]
