// Operator console helpers (docs/service_api.md sections 4-7): E-STOP hotkey, markdown for the help drawer,
// small line charts with a hover readout, file download, and the timeline editor's drag handling.

// ---------------------------------------------------------------------------------------------- E-STOP hotkey
let estopRef = null;
function onKey(e) {
  if (e.key === "Escape" && estopRef && !e.repeat) {
    estopRef.invokeMethodAsync("EstopFromKey");
  }
}
export function bindEstop(dotnetRef) {
  estopRef = dotnetRef;
  window.removeEventListener("keydown", onKey);
  window.addEventListener("keydown", onKey);
}
export function unbindEstop() {
  window.removeEventListener("keydown", onKey);
  estopRef = null;
}

// ---------------------------------------------------------------------------------------------- markdown
const esc = (s) => s.replace(/&/g, "&amp;").replace(/</g, "&lt;").replace(/>/g, "&gt;").replace(/"/g, "&quot;");

function inline(s) {
  // s is already escaped
  return s
    .replace(/`([^`]+)`/g, "<code>$1</code>")
    .replace(/\*\*([^*]+)\*\*/g, "<strong>$1</strong>")
    .replace(/(^|[^*])\*([^*\s][^*]*)\*/g, "$1<em>$2</em>")
    .replace(/\[([^\]]+)\]\(([^)\s]+)\)/g, (m, text, href) => {
      const safe = /^(https?:|#|\.{0,2}\/|[a-z0-9_\-]+\.md)/i.test(href) ? href : "#";
      return `<a href="${safe}" target="_blank" rel="noopener">${text}</a>`;
    });
}

/** Small, safe markdown subset: headings, paragraphs, lists, fenced code, tables, inline code/bold/links. */
export function markdownToHtml(md) {
  const lines = (md || "").replace(/\r\n/g, "\n").split("\n");
  const out = [];
  let i = 0;
  while (i < lines.length) {
    const ln = lines[i];
    if (/^```/.test(ln)) {
      const buf = [];
      i++;
      while (i < lines.length && !/^```/.test(lines[i])) buf.push(lines[i++]);
      i++;
      out.push(`<pre><code>${esc(buf.join("\n"))}</code></pre>`);
      continue;
    }
    const h = /^(#{1,4})\s+(.*)$/.exec(ln);
    if (h) {
      const n = Math.min(4, h[1].length + 1);
      out.push(`<h${n}>${inline(esc(h[2]))}</h${n}>`);
      i++;
      continue;
    }
    if (/^\|/.test(ln) && i + 1 < lines.length && /^\|[\s:|-]+\|?\s*$/.test(lines[i + 1])) {
      const cells = (r) => r.replace(/^\||\|\s*$/g, "").split("|").map((c) => inline(esc(c.trim())));
      const head = cells(ln);
      i += 2;
      const rows = [];
      while (i < lines.length && /^\|/.test(lines[i])) rows.push(cells(lines[i++]));
      out.push(`<div class="md-table"><table><thead><tr>${head.map((c) => `<th>${c}</th>`).join("")}</tr></thead><tbody>` +
        rows.map((r) => `<tr>${r.map((c) => `<td>${c}</td>`).join("")}</tr>`).join("") + "</tbody></table></div>");
      continue;
    }
    if (/^\s*([-*]|\d+\.)\s+/.test(ln)) {
      const ordered = /^\s*\d+\./.test(ln);
      const items = [];
      while (i < lines.length && /^\s*([-*]|\d+\.)\s+/.test(lines[i])) {
        let item = lines[i].replace(/^\s*([-*]|\d+\.)\s+/, "");
        i++;
        while (i < lines.length && /^\s{2,}\S/.test(lines[i]) && !/^\s*([-*]|\d+\.)\s+/.test(lines[i])) item += " " + lines[i++].trim();
        items.push(`<li>${inline(esc(item))}</li>`);
      }
      out.push(ordered ? `<ol>${items.join("")}</ol>` : `<ul>${items.join("")}</ul>`);
      continue;
    }
    if (/^\s*$/.test(ln) || /^---\s*$/.test(ln)) {
      if (/^---/.test(ln)) out.push("<hr>");
      i++;
      continue;
    }
    const buf = [ln];
    i++;
    while (i < lines.length && lines[i].trim() && !/^(#|```|\||\s*([-*]|\d+\.)\s)/.test(lines[i])) buf.push(lines[i++]);
    out.push(`<p>${inline(esc(buf.join(" ")))}</p>`);
  }
  return out.join("\n");
}

export function renderMarkdown(el, md) {
  if (el) el.innerHTML = markdownToHtml(md);
}

// ---------------------------------------------------------------------------------------------- line chart
/**
 * One series per chart (small multiples): 2px line, recessive grid, optional dashed reference lines, hover
 * crosshair with a readout. spec = {xs, ys, unit, xUnit, refs:[{y,label}], yMin, yMax, label}
 */
export function lineChart(el, specJson) {
  if (!el) return;
  const s = JSON.parse(specJson);
  const W = el.clientWidth || 300, H = el.clientHeight || 90, P = { l: 34, r: 8, t: 8, b: 16 };
  const xs = s.xs || [], ys = s.ys || [];
  if (!xs.length) {
    el.innerHTML = `<div class="chart-empty">${esc(s.empty || "no samples yet")}</div>`;
    return;
  }
  const yVals = ys.concat((s.refs || []).map((r) => r.y));
  let y0 = s.yMin ?? Math.min(...yVals), y1 = s.yMax ?? Math.max(...yVals);
  if (y1 - y0 < 1e-6) { y0 -= 1; y1 += 1; }
  const pad = (y1 - y0) * 0.08;
  y0 -= pad; y1 += pad;
  const x0 = Math.min(...xs), x1 = Math.max(...xs) === x0 ? x0 + 1 : Math.max(...xs);
  const X = (x) => P.l + ((x - x0) / (x1 - x0)) * (W - P.l - P.r);
  const Y = (y) => P.t + (1 - (y - y0) / (y1 - y0)) * (H - P.t - P.b);
  const pts = xs.map((x, k) => `${X(x).toFixed(1)},${Y(ys[k]).toFixed(1)}`).join(" ");
  const fmt = (v) => (Math.abs(v) >= 100 ? v.toFixed(0) : v.toFixed(1));
  const refs = (s.refs || []).map((r) =>
    `<line x1="${P.l}" x2="${W - P.r}" y1="${Y(r.y)}" y2="${Y(r.y)}" class="chart-ref"/>` +
    `<text x="${W - P.r - 2}" y="${Y(r.y) - 3}" class="chart-ref-label" text-anchor="end">${esc(r.label)}</text>`).join("");
  el.innerHTML =
    `<svg width="${W}" height="${H}" role="img" aria-label="${esc(s.label || "chart")}">` +
    `<line x1="${P.l}" x2="${W - P.r}" y1="${H - P.b}" y2="${H - P.b}" class="chart-axis"/>` +
    `<text x="${P.l - 4}" y="${Y(y1 - pad) + 4}" class="chart-tick" text-anchor="end">${fmt(y1 - pad)}</text>` +
    `<text x="${P.l - 4}" y="${Y(y0 + pad) + 4}" class="chart-tick" text-anchor="end">${fmt(y0 + pad)}</text>` +
    `<text x="${P.l}" y="${H - 3}" class="chart-tick">${fmt(x0)} ${esc(s.xUnit || "")}</text>` +
    `<text x="${W - P.r}" y="${H - 3}" class="chart-tick" text-anchor="end">${fmt(x1)} ${esc(s.xUnit || "")}</text>` +
    refs +
    `<polyline points="${pts}" class="chart-line"/>` +
    `<line class="chart-cross" y1="${P.t}" y2="${H - P.b}" x1="-10" x2="-10"/>` +
    `<circle class="chart-dot" r="4" cx="-10" cy="-10"/>` +
    `<rect x="${P.l}" y="0" width="${W - P.l - P.r}" height="${H}" fill="transparent" class="chart-hit"/></svg>` +
    `<div class="chart-tip" hidden></div>`;
  const svg = el.querySelector("svg"), tip = el.querySelector(".chart-tip");
  const cross = el.querySelector(".chart-cross"), dot = el.querySelector(".chart-dot");
  el.querySelector(".chart-hit").addEventListener("pointermove", (e) => {
    const r = svg.getBoundingClientRect();
    const px = e.clientX - r.left;
    let k = 0, best = Infinity;
    xs.forEach((x, j) => { const d = Math.abs(X(x) - px); if (d < best) { best = d; k = j; } });
    cross.setAttribute("x1", X(xs[k])); cross.setAttribute("x2", X(xs[k]));
    dot.setAttribute("cx", X(xs[k])); dot.setAttribute("cy", Y(ys[k]));
    tip.hidden = false;
    tip.textContent = `${fmt(ys[k])} ${s.unit || ""} at ${fmt(xs[k])} ${s.xUnit || ""}`;
    tip.style.left = `${Math.min(X(xs[k]) + 8, W - 140)}px`;
  });
  el.querySelector(".chart-hit").addEventListener("pointerleave", () => {
    tip.hidden = true; cross.setAttribute("x1", -10); cross.setAttribute("x2", -10); dot.setAttribute("cx", -10);
  });
}

// ---------------------------------------------------------------------------------------------- files
export function downloadText(filename, text, mime) {
  const blob = new Blob([text], { type: mime || "application/json" });
  const a = document.createElement("a");
  a.href = URL.createObjectURL(blob);
  a.download = filename;
  document.body.appendChild(a);
  a.click();
  setTimeout(() => { URL.revokeObjectURL(a.href); a.remove(); }, 1000);
}

// ---------------------------------------------------------------------------------------------- timeline drag
/** Drag keyframe diamonds horizontally; reports (index, newTime) to .NET on release. */
export function bindTimeline(el, dotnetRef, duration) {
  if (!el) return;
  if (el._tl) el.removeEventListener("pointerdown", el._tl);
  el._dur = duration;
  el._tl = (e) => {
    const k = e.target?.dataset?.kf;
    if (k === undefined) return;
    e.preventDefault();
    const idx = Number(k);
    const r = el.getBoundingClientRect();
    const move = (ev) => {
      const t = Math.max(0, ((ev.clientX - r.left) / r.width) * el._dur);
      e.target.setAttribute("cx", `${(t / el._dur) * 100}%`);
      e.target.dataset.t = t.toFixed(2);
    };
    const up = (ev) => {
      window.removeEventListener("pointermove", move);
      window.removeEventListener("pointerup", up);
      const t = Math.max(0, ((ev.clientX - r.left) / r.width) * el._dur);
      dotnetRef.invokeMethodAsync("OnKeyframeDragged", idx, Math.round(t * 100) / 100);
    };
    window.addEventListener("pointermove", move);
    window.addEventListener("pointerup", up);
  };
  el.addEventListener("pointerdown", el._tl);
}
