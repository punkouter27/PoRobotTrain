"""Build the static showcase site into web/dist (what the workflow deploys to the Static Web App).

    python web/build.py            # writes web/dist
    python web/build.py --check    # build into a temp dir and validate only (nothing written in the repo)

Version: ONE stamp, computed here once and injected into index.html, diag.html and version.json: "v" + the UTC
yyMMdd of the HEAD commit's committer date (falls back to today, UTC, outside a git checkout). Unreal's Build.cs
uses the same rule, so the game's build stamp and the site's agree for the same commit.
"""

from __future__ import annotations

import argparse
import datetime as dt
import html
import json
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

WEB = Path(__file__).resolve().parent
ROOT = WEB.parent
STATIC = ["style.css", "diag.js", "staticwebapp.config.json"]
GROUPS = ("api", "data", "service")
KINDS = ("files", "version", "none")


def version_stamp() -> tuple[str, str]:
    """('vYYMMDD', commit sha or '') from the HEAD commit time in UTC."""
    try:
        out = subprocess.run(
            ["git", "log", "-1", "--format=%ct %H"], cwd=ROOT, capture_output=True, text=True, check=True
        ).stdout.split()
        when = dt.datetime.fromtimestamp(int(out[0]), tz=dt.UTC)
        return "v" + when.strftime("%y%m%d"), out[1]
    except (OSError, subprocess.CalledProcessError, IndexError, ValueError):
        return "v" + dt.datetime.now(dt.UTC).strftime("%y%m%d"), ""


def render_content(site: dict, shots: list[dict]) -> str:
    e = html.escape
    parts = [f"<h1>{e(site['title'])}</h1>", f'<p class="lede">{e(site["tagline"])}</p>', '<div class="prose">']
    parts += [f"<p>{e(p)}</p>" for p in site["intro"]]
    parts.append("</div>")
    parts.append('<ul class="facts">')
    parts += [f"<li><b>{e(k)}</b>{e(v)}</li>" for k, v in site["facts"]]
    parts.append("</ul>")
    parts.append('<h2 id="screenshots">Screenshots</h2><div class="shots">')
    for s in shots:
        parts.append(
            f'<figure><img src="{e(s["url"])}" width="{s["width"]}" height="{s["height"]}" loading="lazy" '
            f'alt="{e(s["caption"])}"><figcaption>{e(s["caption"])}</figcaption></figure>'
        )
    parts.append("</div>")
    parts.append(
        f'<h2>Source</h2><p><a class="cta" href="{e(site["repo"])}" rel="noopener">View the repository</a></p>'
    )
    return "\n".join(parts)


def png_size(path: Path) -> tuple[int, int]:
    head = path.read_bytes()[:24]
    if head[:8] != b"\x89PNG\r\n\x1a\n":
        raise SystemExit(f"{path}: not a PNG")
    return int.from_bytes(head[16:20], "big"), int.from_bytes(head[20:24], "big")


def expand_checks(spec: dict, dist: Path, shot_urls: list[str]) -> dict:
    out = []
    names = set()
    for c in spec["checks"]:
        for key in ("name", "group", "kind", "label", "purpose"):
            if not c.get(key):
                raise SystemExit(f"diag_checks.json: check {c} has no {key}")
        if c["group"] not in GROUPS or c["kind"] not in KINDS:
            raise SystemExit(f"diag_checks.json: {c['name']}: group must be {GROUPS}, kind {KINDS}")
        if c["name"] in names:
            raise SystemExit(f"diag_checks.json: duplicate check {c['name']}")
        names.add(c["name"])
        item = {k: c[k] for k in ("name", "group", "kind", "label", "purpose")}
        if c["kind"] == "files":
            paths = shot_urls if c["files"] == "@screenshots" else c["files"]
            files = []
            for p in paths:
                f = dist / p.lstrip("/")
                if not f.is_file():
                    raise SystemExit(f"diag_checks.json: {c['name']}: {p} is not in the build")
                kind = "image" if f.suffix.lower() in (".png", ".jpg", ".jpeg", ".webp") else "file"
                files.append({"path": "/" + p.lstrip("/"), "bytes": f.stat().st_size, "type": kind})
            item["files"] = files
        elif c["kind"] == "version":
            if not (dist / c["path"].lstrip("/")).is_file():
                raise SystemExit(f"diag_checks.json: {c['path']} is not in the build")
            item["path"] = c["path"]
        else:
            item["detail"] = c["detail"]
        out.append(item)
    groups = {c["group"] for c in out}
    if groups != set(GROUPS):
        raise SystemExit(f"diag_checks.json: every section needs a check, missing {set(GROUPS) - groups}")
    return {"checks": out}


def build(dist: Path) -> str:
    site = json.loads((WEB / "site.json").read_text(encoding="utf-8"))
    version, commit = version_stamp()
    if dist.exists():
        shutil.rmtree(dist)
    (dist / "screenshots").mkdir(parents=True)
    shots = []
    for s in site["screenshots"]:
        src = ROOT / s["src"]
        if not src.is_file():
            raise SystemExit(f"site.json: screenshot {s['src']} does not exist")
        dst = dist / "screenshots" / src.name
        shutil.copyfile(src, dst)
        w, h = png_size(src)
        shots.append({"url": f"screenshots/{src.name}", "caption": s["caption"], "width": w, "height": h})
    for name in STATIC:
        shutil.copyfile(WEB / name, dist / name)
    # X-Commit on every response, so the workflow (and anyone) can confirm which commit is live
    swa = json.loads((dist / "staticwebapp.config.json").read_text(encoding="utf-8"))
    swa.setdefault("globalHeaders", {})["x-commit"] = commit or "unknown"
    (dist / "staticwebapp.config.json").write_text(json.dumps(swa, indent=2) + "\n", encoding="utf-8", newline="\n")
    for name in site.get("extra_files", []):
        shutil.copyfile(ROOT / name, dist / Path(name).name)
    subs = {
        "%%TITLE%%": html.escape(site["title"]),
        "%%TAGLINE%%": html.escape(site["tagline"]),
        "%%REPO%%": html.escape(site["repo"]),
        "%%VERSION%%": version,
        "%%CONTENT%%": render_content(site, shots),
    }
    for page in ("index.html", "diag.html"):
        text = (WEB / page).read_text(encoding="utf-8")
        for k, v in subs.items():
            text = text.replace(k, v)
        if "%%" in text:
            raise SystemExit(f"{page}: unreplaced placeholder")
        (dist / page).write_text(text, encoding="utf-8", newline="\n")
    (dist / "version.json").write_text(json.dumps({"version": version, "commit": commit}) + "\n", encoding="utf-8")
    spec = json.loads((WEB / "diag_checks.json").read_text(encoding="utf-8"))
    checks = expand_checks(spec, dist, [s["url"] for s in shots])
    (dist / "diag_checks.json").write_text(json.dumps(checks, indent=1) + "\n", encoding="utf-8")
    return version


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true", help="build into a temp dir and validate; write nothing")
    a = ap.parse_args()
    if a.check:
        with tempfile.TemporaryDirectory() as tmp:
            v = build(Path(tmp) / "dist")
        print(f"web build OK ({v})")
    else:
        v = build(WEB / "dist")
        print(f"built web/dist ({v})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
