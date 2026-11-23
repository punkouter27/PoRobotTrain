"""Training runs grid: logs/runs.html, one row per real training run in this repo, newest first.

    python benchmark/runs.py                                   rebuild logs/runs.html from what the runs left on disk
    python benchmark/runs.py --record <run-id> --ended "<how>" [--launcher <script>] [--started <epoch s>]
                             [--out <run output dir>] [--argv '<json list>']
                                                               note this machine, the run and how it ended, then rebuild

Where the rows come from:
- logs/runs/<run-id>/run_meta.json, which --record writes when a Level 0 training session ends (train_level0.py calls
  it in a finally, crash or not): machine, commit, how it ended, the session's start time and arguments, and a copy of
  the steps / speed read from its train_log.csv at that moment (the Level 0 output folder is overwritten by the next
  run, so the copy is what keeps older sessions listed).
- Level 0 output folders (RobotControl/models/*/ and logs/**/ with a train_log.csv) that no run_meta.json claims: runs
  from before the hook existed, or whose hook never fired. Their machine/commit cells read "-".
- Level 2 rsl_rl run folders (logs/level2_training/rsl_rl/<experiment>/<run>/, written by the Docker trainer that
  scripts/level2/train_level2.ps1 starts detached): steps and speed from the model_<it>.pt checkpoint names and times.
logs/benchmark/ (benchmark/workload.py's throwaway Level 0 run) is never listed.

Speed is the run's own: env steps per second (num_envs x horizon per PPO iteration) between the first and the last
logged iteration (Level 0, train_log.csv) or checkpoint (Level 2) of this session, so startup and the steps of an
earlier session are not counted. Benchmark is the newest benchmark/history.json result on the same machine from
before the run started (ms per env step), for comparison. Stdlib only; core.py next to this file supplies the machine.
"""

from __future__ import annotations

import argparse
import csv
import datetime
import html
import json
import os
import re
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
LOGS = ROOT / "logs"
RUNS = LOGS / "runs"  # one folder per recorded session: run_meta.json
OUT = LOGS / "runs.html"
BENCH = HERE / "history.json"
META = "run_meta.json"
TITLE = "PoRobotTrain"
SKIP = LOGS / "benchmark"  # the Benchmark's own throwaway run dir
MODELS = ROOT / "RobotControl" / "models"
LEVEL2 = LOGS / "level2_training" / "rsl_rl"
LEVEL0_DEFAULTS = {"iters": "1500", "num-envs": "512", "horizon": "24", "seed": "0"}  # train_level0.py's argparse


# ---------------------------------------------------------------- collect


def _json(path: Path) -> dict:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return {}


def _text(path: Path) -> str:
    try:
        return path.read_text(encoding="utf-8", errors="replace")
    except OSError:
        return ""


def _argv(argv: list[str], name: str) -> str | None:
    for i, a in enumerate(argv):
        if a == f"--{name}" and i + 1 < len(argv):
            return argv[i + 1]
        if a.startswith(f"--{name}="):
            return a.split("=", 1)[1]
    return None


def _inside(path: Path, folder: Path) -> bool:
    return path == folder or folder in path.parents


def _rel(path: Path) -> str:
    try:
        return path.relative_to(ROOT).as_posix()
    except ValueError:
        return path.as_posix()


def _train_log(path: Path) -> dict:
    """Level 0 train_log.csv: iterations reached, loop seconds and env steps/s between the first and last row."""
    try:
        with path.open(newline="", encoding="utf-8") as f:
            rows = [(int(r["iter"]), float(r["time_s"])) for r in csv.DictReader(f) if r.get("iter")]
    except (OSError, ValueError, KeyError):
        return {}
    if not rows:
        return {}
    (i0, t0), (i1, t1) = rows[0], rows[-1]
    return dict(iters=i1, loop_s=t1, iters_per_s=(i1 - i0) / (t1 - t0) if t1 > t0 else None, mtime=path.stat().st_mtime)


def _bench_for(host: str | None, before: float) -> dict | None:
    runs = [r for r in _json(BENCH).get("runs", []) if r.get("ok") and r.get("result")]
    if host:
        runs = [r for r in runs if r.get("machine", {}).get("hostname") == host]
    if not runs:
        return None

    def stamp(r: dict) -> float:
        return datetime.datetime.fromisoformat(r["when"]).timestamp()

    earlier = [r for r in runs if stamp(r) <= before]
    return (earlier or runs)[-1]


def _row(**kw) -> dict:
    row = dict(
        run="-",
        start=0.0,
        wall=0.0,
        ended="unknown",
        resumed=False,
        unit=None,
        steps=None,
        rate=None,
        bench=None,
        config=None,
        envs="-",
        warm=[],
        commit=None,
        host=None,
        gpu=None,
        device=None,
        launcher=None,
    )
    row.update(kw)
    row["bench"] = _bench_for(row["host"], row["start"])
    return row


def _level0_row(name: str, meta: dict) -> dict:
    """A session recorded by --record (train_level0.py's hook)."""
    argv = meta.get("argv") or []
    a = {k: _argv(argv, k) or v for k, v in LEVEL0_DEFAULTS.items()}
    envs, horizon = int(a["num-envs"]), int(a["horizon"])
    snap = meta.get("train_log") or {}
    steps = snap["iters"] * envs * horizon if snap.get("iters") else None
    rate = snap["iters_per_s"] * envs * horizon if snap.get("iters_per_s") else None
    started = float(meta.get("started") or 0)
    recorded = datetime.datetime.fromisoformat(meta["recorded"]).timestamp() if meta.get("recorded") else started
    resume = _argv(argv, "resume")
    out = meta.get("out")
    return _row(
        run=name,
        start=started,
        wall=max(0.0, recorded - started),
        ended=meta.get("ended") or "unknown",
        resumed=bool(resume),
        unit=f"{snap['iters']:,} iters" if snap.get("iters") else None,
        steps=steps,
        rate=rate,
        config=f"Level 0 · {a['iters']} iters · horizon {horizon} · seed {a['seed']}"
        + (f" · out {out}" if out and out != _rel(MODELS / "r1_level0_upperbody") else ""),
        envs=envs,
        warm=[f"resume {Path(resume).name}"] if resume else [],
        commit=meta.get("commit"),
        host=meta.get("host"),
        gpu=meta.get("gpu"),
        device=meta.get("device"),
        launcher=meta.get("launcher"),
    )


def _level0_folder_row(folder: Path, log: dict) -> dict:
    """A Level 0 output folder no recorded session claims (a run from before the hook, or whose hook never fired)."""
    deploy = _text(folder / "deploy.yaml")

    def md(key: str) -> str | None:
        m = re.search(rf"^\s+{key}:\s*'?([^'\n]+)'?\s*$", deploy, re.M)
        return m.group(1).strip() if m else None

    exported = (folder / "deploy.yaml").exists() and (folder / "deploy.yaml").stat().st_mtime >= log["mtime"] - 1
    same = exported and md("iters") == str(log["iters"])
    envs = int(md("num_envs") or 0) if same and (md("num_envs") or "").isdigit() else None
    horizon = int(md("horizon") or 0) if same and (md("horizon") or "").isdigit() else None
    end = log["mtime"]
    if same and md("trained_at"):
        try:
            end = datetime.datetime.fromisoformat(md("trained_at")).timestamp()
        except ValueError:
            pass
    per_iter = envs * horizon if envs and horizon else None
    commit = md("git") if same and md("git") not in (None, "nogit") else None
    return _row(
        run=_rel(folder),
        start=end - log["loop_s"],
        wall=log["loop_s"],
        ended="finished (exported)" if same else "stopped or still running (no record)",
        unit=f"{log['iters']:,} iters",
        steps=log["iters"] * per_iter if per_iter else None,
        rate=log["iters_per_s"] * per_iter if per_iter and log["iters_per_s"] else None,
        config=f"Level 0 · {log['iters']} iters · horizon {horizon} · seed {md('seed')}" if same else "Level 0",
        envs=envs or "-",
        warm=[],
        commit=commit,
        device=md("device") if same else None,
    )


def _yaml_value(text: str, key: str) -> str | None:
    m = re.search(rf"^\s*{key}:\s*'?([^'\n#]+?)'?\s*$", text, re.M)
    return m.group(1) if m else None


def _level2_row(folder: Path) -> dict | None:
    """A Level 2 rsl_rl run folder (Docker trainer): model_<it>.pt checkpoints, params/{env,agent}.yaml."""
    cps = sorted(
        (int(m.group(1)), p.stat().st_mtime)
        for p in folder.glob("model_*.pt")
        for m in [re.fullmatch(r"model_(\d+)\.pt", p.name)]
        if m
    )
    env_yaml, agent_yaml = _text(folder / "params" / "env.yaml"), _text(folder / "params" / "agent.yaml")
    if not cps and not env_yaml:
        return None
    envs = _yaml_value(env_yaml, "num_envs")
    horizon = _yaml_value(agent_yaml, "num_steps_per_env")
    per_iter = int(envs) * int(horizon) if (envs or "").isdigit() and (horizon or "").isdigit() else None
    try:
        start = datetime.datetime.strptime(folder.name[:19], "%Y-%m-%d_%H-%M-%S").timestamp()
    except ValueError:
        start = min([t for _, t in cps] or [folder.stat().st_mtime])
    files = [p.stat().st_mtime for p in folder.rglob("*") if p.is_file()]
    end = max(files or [start])
    rate = None
    if per_iter and len(cps) >= 2 and cps[-1][1] > cps[0][1]:
        rate = (cps[-1][0] - cps[0][0]) * per_iter / (cps[-1][1] - cps[0][1])
    resumed = (_yaml_value(agent_yaml, "resume") or "").lower() == "true"
    load_run = _yaml_value(agent_yaml, "load_run")
    max_it = _yaml_value(agent_yaml, "max_iterations")
    return _row(
        run=f"level2/{folder.parent.name}/{folder.name}",
        start=start,
        wall=max(0.0, end - start),
        ended="stopped or still running (detached container)",
        resumed=resumed,
        unit=f"{cps[-1][0]:,} iters" if cps else None,
        steps=(cps[-1][0] + 1) * per_iter if per_iter and cps else None,
        rate=rate,
        config="Level 2 · Unitree-R1-Flat" + (f" · {max_it} iters max" if max_it else "") + " · Docker",
        envs=envs or "-",
        warm=[f"resume {load_run}"] if resumed and load_run else (["resume"] if resumed else []),
        launcher="scripts/level2/train_level2.ps1",
    )


def collect() -> list[dict]:
    rows = []
    claimed: list[tuple[Path, float, float]] = []  # (out dir, session start, session end) of recorded sessions
    if RUNS.is_dir():
        for d in RUNS.iterdir():
            meta = _json(d / META)
            if not meta or (meta.get("out") and _inside((ROOT / meta["out"]).resolve(), SKIP)):
                continue
            rows.append(_level0_row(d.name, meta))
            if meta.get("out"):
                claimed.append((ROOT / meta["out"], rows[-1]["start"], rows[-1]["start"] + rows[-1]["wall"]))

    logs = list(MODELS.glob("*/train_log.csv"))
    logs += [p for p in LOGS.rglob("train_log.csv") if not _inside(p, SKIP)]  # logs/ may not exist: rglob is empty
    for p in logs:
        log = _train_log(p)
        folder = p.parent.resolve()
        if not log or any(folder == o.resolve() and s - 5 <= log["mtime"] <= e + 5 for o, s, e in claimed):
            continue
        rows.append(_level0_folder_row(folder, log))

    if LEVEL2.is_dir():
        for exp in LEVEL2.iterdir():
            for run in exp.iterdir() if exp.is_dir() else []:
                row = _level2_row(run) if run.is_dir() else None
                if row:
                    rows.append(row)
    rows.sort(key=lambda r: r["start"], reverse=True)
    return rows


# ---------------------------------------------------------------- render


def _dur(s: float) -> str:
    h, m = divmod(int(s) // 60, 60)
    return f"{h} h {m:02d} m" if h else f"{m} m"


def _num(n: float | None, digits: int = 0) -> str:
    return "-" if n is None else f"{n:,.{digits}f}"


def render(rows: list[dict]) -> str:
    e = html.escape
    body = []
    for r in rows:
        b = r["bench"]
        if b:
            notes = [f"{b['result']['env_steps_per_s']:,} env steps/s", e(b["when"][:10])]
            if datetime.datetime.fromisoformat(b["when"]).timestamp() > r["start"]:
                notes.append("taken after the run")
            if not r["host"]:
                notes.append("machine unknown")
            bench = f"{b['result']['step_time_ms']} ms<small>{' · '.join(notes)}</small>"
        else:
            bench = "-<small>no benchmark on this machine</small>"
        ended = e(r["ended"])
        cls = "bad" if re.search(r"crash|died|killed|HALT|error|exception", r["ended"], re.I) else ""
        body.append(
            "<tr>"
            f"<td><b>{e(r['run'])}</b>{'<small>resumed</small>' if r['resumed'] else ''}</td>"
            f"<td>{datetime.datetime.fromtimestamp(r['start']).strftime('%Y-%m-%d %H:%M')}</td>"
            f'<td class="n">{_dur(r["wall"])}</td>'
            f'<td class="{cls}">{ended}</td>'
            f'<td class="n">{_num(r["steps"])}<small>{e(r["unit"] or "")}</small></td>'
            f'<td class="n"><b>{_num(r["rate"])}</b><small>env steps/s</small></td>'
            f'<td class="n">{bench}</td>'
            f"<td>{e(r['config'] or '-')}</td>"
            f'<td class="n">{e(str(r["envs"]))}</td>'
            f"<td>{e(', '.join(r['warm']) or 'cold')}</td>"
            f'<td class="mono">{e(r["commit"] or "-")}</td>'
            f"<td>{e(r['host'] or '-')}<small>{e(r['gpu'] or '')}{' · ' + e(r['device']) if r['device'] else ''}"
            f"</small></td>"
            "</tr>"
        )
    latest = rows[0] if rows else None
    prev = next((r for r in rows[1:] if r["rate"]), None) if rows else None
    delta = ""
    if latest and latest["rate"] and prev:
        pct = 100 * (latest["rate"] / prev["rate"] - 1)
        delta = f'<span class="{"up" if pct >= 0 else "down"}">{pct:+.0f}% vs {e(prev["run"])}</span>'
    summary = (
        f'<div class="sum"><div><span>Latest run</span><b>{e(latest["run"])}</b></div>'
        f"<div><span>Speed</span><b>{_num(latest['rate'])} env steps/s</b>{delta}</div>"
        f"<div><span>Wall-clock</span><b>{_dur(latest['wall'])}</b></div>"
        f"<div><span>Runs listed</span><b>{len(rows)}</b></div></div>"
        if latest
        else "<p>No training runs found yet.</p>"
    )
    now = datetime.datetime.now().strftime("%Y-%m-%d %H:%M")
    return f"""<!doctype html><html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1"><title>{e(TITLE)} training runs</title>
<style>
:root{{--bg:#f5f7f6;--card:#fff;--ink:#18211f;--muted:#5d6b67;--line:#d8e0dd;--accent:#0f6b5c;--bad:#b42318;--up:#1f7a43}}
@media (prefers-color-scheme:dark){{:root{{--bg:#0f1514;--card:#161e1c;--ink:#e2eae7;--muted:#91a19c;--line:#27332f;
--accent:#5cc9b0;--bad:#ff8a7a;--up:#5fcf8a;color-scheme:dark}}}}
body{{margin:0;background:var(--bg);color:var(--ink);font:14px/1.5 "Segoe UI",system-ui,sans-serif;padding:24px 16px}}
h1{{font-size:20px;margin:0 0 4px}} p{{margin:0;color:var(--muted)}} .wrap{{max-width:1500px;margin:auto;display:grid;gap:16px}}
.sum{{display:flex;flex-wrap:wrap;gap:12px 40px;background:var(--card);border:1px solid var(--line);border-radius:8px;padding:14px 18px}}
.sum span{{display:block;color:var(--muted);font-size:12px}} .sum b{{font-size:20px}} .sum .up,.sum .down{{display:block;font-size:12px}}
.up{{color:var(--up)}} .down{{color:var(--bad)}}
.t{{overflow-x:auto;background:var(--card);border:1px solid var(--line);border-radius:8px}}
table{{border-collapse:collapse;width:100%;font-variant-numeric:tabular-nums}}
th,td{{padding:8px 12px;border-bottom:1px solid var(--line);text-align:left;vertical-align:top;white-space:nowrap}}
th{{font-size:11px;text-transform:uppercase;letter-spacing:.06em;color:var(--muted);position:sticky;top:0;background:var(--card)}}
tr:last-child td{{border-bottom:0}} td.n{{text-align:right}} td small{{display:block;color:var(--muted);font-size:11px}}
td.bad{{color:var(--bad)}} .mono{{font-family:Consolas,monospace}}
</style></head><body><div class="wrap">
<div><h1>{e(TITLE)} training runs</h1><p>Every real training run in this repo (Level 0 sessions, Level 0 model
folders, Level 2 rsl_rl runs), newest first. Rebuilt {now} by benchmark/runs.py. Speed = env steps/s (num_envs x
horizon per PPO iteration) this session; Benchmark = the newest benchmark.ps1 result on the same machine before the
run (ms per env step).</p></div>
{summary}
<div class="t"><table><thead><tr><th>Run</th><th>Started</th><th>Wall-clock</th><th>Ended</th><th>Steps</th>
<th>Speed</th><th>Benchmark</th><th>Config</th><th>Envs</th><th>Warm start</th><th>Commit</th><th>Machine</th></tr></thead>
<tbody>{"".join(body)}</tbody></table></div></div></body></html>
"""


def build() -> Path:
    LOGS.mkdir(exist_ok=True)
    tmp = OUT.with_suffix(".tmp")
    tmp.write_text(render(collect()), encoding="utf-8")
    os.replace(tmp, OUT)
    return OUT


def record(
    run_id: str, ended: str, launcher: str | None, started: float | None, out: str | None, argv: list[str]
) -> None:
    """Write logs/runs/<run>/run_meta.json: this machine, the commit, how the run ended and what it reached."""
    sys.path.insert(0, str(HERE))
    import core

    m = core.machine()
    d = RUNS / run_id
    d.mkdir(parents=True, exist_ok=True)
    out_dir = Path(out).resolve() if out else None
    log = _train_log(out_dir / "train_log.csv") if out_dir else {}
    if started and log.get("mtime", 0) < started - 1:
        log = {}  # an older run's log: this session wrote nothing
    deploy = _text(out_dir / "deploy.yaml") if out_dir and log else ""
    fresh = bool(deploy) and (out_dir / "deploy.yaml").stat().st_mtime >= (started or 0)
    device = _yaml_value(deploy, "device") if fresh else None
    meta = dict(
        host=m["hostname"],
        gpu=m["gpu"],
        cpu=m["cpu"],
        device=device or ("CPU" if os.environ.get("CUDA_VISIBLE_DEVICES") == "-1" else None),
        commit=core.git_commit(),
        ended=ended,
        launcher=launcher,
        started=started,
        out=_rel(out_dir) if out_dir else None,
        argv=argv,
        train_log={k: v for k, v in log.items() if k != "mtime"} or None,
        recorded=datetime.datetime.now().astimezone().isoformat(timespec="seconds"),
    )
    (d / META).write_text(json.dumps(meta, indent=1), encoding="utf-8")


def main() -> None:
    p = argparse.ArgumentParser(description="Rebuild logs/runs.html (the training runs grid).")
    p.add_argument("--record", metavar="RUN_ID", help="note this machine and how the run ended first")
    p.add_argument("--ended", default="exited", help="how the run ended, in words")
    p.add_argument("--launcher", help="the script that ran it")
    p.add_argument("--started", type=float, help="session start, epoch seconds (default: now)")
    p.add_argument("--out", help="the run's output folder (where train_log.csv is)")
    p.add_argument("--argv", default="[]", help="the trainer's command-line arguments, as a JSON list")
    a = p.parse_args()
    if a.record:
        record(a.record, a.ended, a.launcher, a.started or time.time(), a.out, json.loads(a.argv))
    print(f"Training runs grid: {build()}")


if __name__ == "__main__":
    main()
