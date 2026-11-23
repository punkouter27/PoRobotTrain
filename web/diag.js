// /diag: every check is probed live from the browser (fetch + latency). Nothing is assumed healthy.
// The check list is diag_checks.json, written by web/build.py from web/diag_checks.json.
(function () {
  "use strict";
  const pageVersion = document.querySelector('meta[name="po-version"]').content;
  const groups = { api: "g-api", data: "g-data", service: "g-service" };

  function row(c) {
    const li = document.createElement("li");
    li.className = "check";
    const parts = [
      ["span", "check__label", c.label],
      ["span", "check__ms", ""],
      ["span", "pill", "Checking"],
      ["span", "check__purpose", c.purpose],
      ["span", "check__detail", ""],
    ];
    for (const [tag, cls, text] of parts) {
      const el = document.createElement(tag);
      el.className = cls;
      el.textContent = text;
      li.appendChild(el);
    }
    li.querySelector(".pill").dataset.s = "Checking";
    document.getElementById(groups[c.group]).appendChild(li);
    return li;
  }

  function show(li, status, ms, detail) {
    const pill = li.querySelector(".pill");
    pill.dataset.s = status;
    pill.textContent = status;
    li.querySelector(".check__ms").textContent = ms == null ? "" : Math.round(ms) + " ms";
    li.querySelector(".check__detail").textContent = detail || "";
  }

  async function timedFetch(url) {
    const t0 = performance.now();
    const res = await fetch(url, { cache: "no-store" });
    const body = await res.arrayBuffer();
    return { res, body, ms: performance.now() - t0 };
  }

  // Files of the deployed bundle: served (200), the size the build wrote, and images decode.
  async function probeFiles(c) {
    let status = "Healthy";
    let total = 0;
    const problems = [];
    for (const f of c.files) {
      try {
        const { res, body, ms } = await timedFetch(f.path);
        total += ms;
        if (!res.ok) {
          status = "Unhealthy";
          problems.push(f.path + " -> HTTP " + res.status);
          continue;
        }
        if (f.bytes != null && body.byteLength !== f.bytes) {
          if (status === "Healthy") status = "Degraded";
          problems.push(f.path + ": " + body.byteLength + " bytes, the build wrote " + f.bytes);
        }
        if (f.type === "image") {
          try {
            const bmp = await createImageBitmap(new Blob([body], { type: res.headers.get("content-type") || "image/png" }));
            if (bmp.close) bmp.close();
          } catch (e) {
            status = "Unhealthy";
            problems.push(f.path + ": does not decode as an image");
          }
        }
      } catch (e) {
        status = "Unhealthy";
        problems.push(f.path + ": " + e.message);
      }
    }
    const detail = problems.length ? problems.join("; ") : c.files.length + " file(s) served";
    return { status, ms: total, detail };
  }

  // version.json carries the build's one stamp; this page must carry the same (else a stale cache or mixed deploy).
  async function probeVersion(c) {
    const { res, body, ms } = await timedFetch(c.path);
    if (!res.ok) return { status: "Unhealthy", ms, detail: c.path + " -> HTTP " + res.status };
    const v = JSON.parse(new TextDecoder().decode(body)).version;
    if (v !== pageVersion) return { status: "Degraded", ms, detail: "server says " + v + ", this page is " + pageVersion };
    return { status: "Healthy", ms, detail: v };
  }

  function finish(results, t0) {
    const bad = results.filter((r) => r.status === "Unhealthy");
    const deg = results.filter((r) => r.status === "Degraded");
    const b = document.getElementById("banner");
    if (bad.length) {
      b.dataset.state = "bad";
      b.textContent = "Not working: " + bad.map((r) => r.label).join(", ");
    } else if (deg.length) {
      b.dataset.state = "degraded";
      b.textContent = "Working, with " + deg.length + " thing" + (deg.length > 1 ? "s" : "") + " degraded";
    } else {
      b.dataset.state = "ok";
      b.textContent = "Everything is working";
    }
    document.getElementById("checked").textContent =
      "Checked " + new Date().toISOString().replace("T", " ").slice(0, 19) + " UTC from your browser in " +
      Math.round(performance.now() - t0) + " ms";
    document.body.dataset.diagDone = "1";
  }

  async function run() {
    const t0 = performance.now();
    const listRow = row({ group: "data", label: "Check list", purpose: "diag_checks.json: what this page probes" });
    let list;
    try {
      const { res, body, ms } = await timedFetch("/diag_checks.json");
      if (!res.ok) throw new Error("HTTP " + res.status);
      list = JSON.parse(new TextDecoder().decode(body));
      show(listRow, "Healthy", ms, list.checks.length + " checks");
    } catch (e) {
      show(listRow, "Unhealthy", null, "diag_checks.json: " + e.message);
      finish([{ label: "Check list", status: "Unhealthy" }], t0);
      return;
    }
    const results = [{ label: "Check list", status: "Healthy" }];
    await Promise.all(
      list.checks.map(async (c) => {
        const li = row(c);
        if (c.kind === "none") {
          show(li, "Off", null, c.detail); // switched off on purpose: neutral, not counted in the banner
          return;
        }
        let r;
        try {
          r = c.kind === "version" ? await probeVersion(c) : await probeFiles(c);
        } catch (e) {
          r = { status: "Unhealthy", ms: null, detail: e.message };
        }
        show(li, r.status, r.ms, r.detail);
        results.push({ label: c.label, status: r.status });
      })
    );
    finish(results, t0);
  }

  run();
})();
