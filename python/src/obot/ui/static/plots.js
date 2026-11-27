// Live plots of signals from the robot's state: one panel per unit (never two scales on
// one axis), scrolling with time, with a legend that carries each signal's value.
//
// Colours are the eight slots of a palette checked for colour-blind separation on both
// surfaces. A signal keeps its slot for as long as it is selected: taking another one out
// does not repaint it.
const LIGHT = ["#257de0", "#f47230", "#00b894", "#f2a600", "#ee7fac", "#008a17", "#6a4ce2", "#ec4647"];
const DARK = ["#338ced", "#e76624", "#00a888", "#d28e05", "#e24d8b", "#0aac23", "#9887ef", "#eb6263"];
const MAX_SIGNALS = LIGHT.length;
const KEEP_S = 60;                 // samples older than this are dropped
const MARGIN = { left: 56, right: 12, top: 8, bottom: 22 };
const PANEL_HEIGHT = 170;
const STORE = "obot-plots";

function el(tag, props = {}, ...children) {
  const node = Object.assign(document.createElement(tag), props);
  node.append(...children);
  return node;
}

function format(value) {
  if (value === null || value === undefined || Number.isNaN(value)) return "-";
  const a = Math.abs(value);
  return value.toFixed(a >= 100 ? 1 : a >= 10 ? 2 : a >= 0.01 || a === 0 ? 3 : 5);
}

// Round numbers for an axis: steps of 1, 2 or 5 times a power of ten.
function ticks(low, high, about) {
  const raw = (high - low) / about;
  const power = Math.pow(10, Math.floor(Math.log10(raw)));
  const step = [1, 2, 5, 10].map((m) => m * power).find((s) => s >= raw);
  const out = [];
  for (let v = Math.ceil(low / step) * step; v <= high + step * 1e-9; v += step) out.push(Math.abs(v) < step * 1e-9 ? 0 : v);
  return { step, values: out };
}

// Index of the last sample at or before time t.
function indexAt(times, t) {
  let low = 0, high = times.length - 1;
  if (high < 0 || times[0] > t) return -1;
  while (low < high) {
    const mid = (low + high + 1) >> 1;
    if (times[mid] <= t) low = mid; else high = mid - 1;
  }
  return low;
}

export function createPlots(root, send) {
  let catalogue = new Map();        // id -> { id, label, unit, group }
  const selected = new Map();       // id -> { slot, t: [], v: [] }
  let windowS = 10, rate = 100, paused = false, frozenEnd = 0, hover = null;
  const panels = new Map();         // unit -> { box, legend, canvas }

  try {
    const saved = JSON.parse(localStorage.getItem(STORE) || "{}");
    if ([5, 10, 30, 60].includes(saved.windowS)) windowS = saved.windowS;
    if (Number.isInteger(saved.rate)) rate = saved.rate;
    root.dataset.saved = JSON.stringify(Array.isArray(saved.signals) ? saved.signals : []);
  } catch (e) { /* no storage: start empty */ }

  // --- controls: one row above the panels -------------------------------------------------
  const picker = el("details", { className: "plot-picker" });
  const pickerSummary = el("summary");
  const pickerList = el("div", { className: "plot-picker-list" });
  picker.append(pickerSummary, pickerList);
  const windowSelect = el("select", { title: "How much time the plots show" });
  for (const s of [5, 10, 30, 60]) windowSelect.append(el("option", { value: s, textContent: s + " s", selected: s === windowS }));
  const rateSelect = el("select", { title: "Samples per second" });
  const pauseButton = el("button", { textContent: "Pause" });
  const controls = el("div", { className: "plot-controls" }, picker,
    el("label", {}, "window ", windowSelect), el("label", {}, "rate ", rateSelect), pauseButton);
  const holder = el("div", { className: "plot-panels" });
  const empty = el("p", { className: "muted", textContent: "Choose signals to plot." });
  const tip = el("div", { className: "plot-tip" });
  root.replaceChildren(controls, empty, holder, tip);

  function save() {
    try {
      localStorage.setItem(STORE, JSON.stringify({ signals: [...selected.keys()], windowS, rate }));
    } catch (e) { /* not kept, that is all */ }
  }
  function subscribe() {
    send({ type: "plot", signals: [...selected.keys()], rate });
  }
  function colour(slot) {
    return (matchMedia("(prefers-color-scheme: dark)").matches ? DARK : LIGHT)[slot];
  }

  function rebuildPanels() {
    const units = [];
    for (const id of selected.keys()) {
      const unit = catalogue.get(id).unit;
      if (!units.includes(unit)) units.push(unit);
    }
    for (const [unit, panel] of panels) {
      if (!units.includes(unit)) { panel.box.remove(); panels.delete(unit); }
    }
    for (const unit of units) {
      if (panels.has(unit)) continue;
      const legend = el("div", { className: "plot-legend" });
      const canvas = el("canvas", { className: "plot-canvas" });
      const box = el("div", { className: "plot-panel" }, legend, canvas);
      canvas.addEventListener("mousemove", (e) => {
        const r = canvas.getBoundingClientRect();
        hover = { fraction: (e.clientX - r.left - MARGIN.left) / (r.width - MARGIN.left - MARGIN.right), x: e.clientX, y: e.clientY };
        if (hover.fraction < 0 || hover.fraction > 1) hover = null;
      });
      canvas.addEventListener("mouseleave", () => { hover = null; });
      holder.append(box);
      panels.set(unit, { box, legend, canvas });
    }
    for (const [unit, panel] of panels) {
      panel.legend.replaceChildren(...[...selected].filter(([id]) => catalogue.get(id).unit === unit).map(([id, series]) => {
        series.value = el("b");
        return el("span", { className: "plot-key" }, el("i", { style: "background:" + colour(series.slot) }),
          catalogue.get(id).label + " ", series.value, " " + unit);
      }));
    }
    empty.style.display = selected.size ? "none" : "";
    pickerSummary.textContent = "Signals (" + selected.size + " of " + MAX_SIGNALS + ")";
    for (const box of pickerList.querySelectorAll("input")) {
      box.checked = selected.has(box.value);
      box.disabled = !box.checked && selected.size >= MAX_SIGNALS;
    }
  }

  function toggle(id, on) {
    if (on && !selected.has(id) && selected.size < MAX_SIGNALS && catalogue.has(id)) {
      const used = new Set([...selected.values()].map((s) => s.slot));
      let slot = 0;
      while (used.has(slot)) slot++;   // the lowest free slot: the others keep theirs
      selected.set(id, { slot, t: [], v: [] });
    } else if (!on) {
      selected.delete(id);
    }
    rebuildPanels();
    subscribe();
    save();
  }

  windowSelect.addEventListener("change", () => { windowS = Number(windowSelect.value); save(); });
  rateSelect.addEventListener("change", () => { rate = Number(rateSelect.value); subscribe(); save(); });
  pauseButton.addEventListener("click", () => {
    paused = !paused;
    pauseButton.textContent = paused ? "Resume" : "Pause";
    if (!paused) for (const series of selected.values()) { series.t.length = 0; series.v.length = 0; }
  });

  // --- drawing ----------------------------------------------------------------------------
  function latest() {
    let end = 0;
    for (const series of selected.values()) if (series.t.length) end = Math.max(end, series.t[series.t.length - 1]);
    return end;
  }

  function draw() {
    requestAnimationFrame(draw);
    if (!selected.size || document.hidden || root.offsetParent === null) return;
    const style = getComputedStyle(root);
    const muted = style.getPropertyValue("--muted").trim();
    const line = style.getPropertyValue("--line").trim(), surface = style.getPropertyValue("--panel").trim();
    const end = paused ? frozenEnd : (frozenEnd = latest());
    const start = end - windowS;
    const hoverT = hover ? start + hover.fraction * windowS : null;
    const readout = [];

    for (const [unit, panel] of panels) {
      const canvas = panel.canvas;
      const width = canvas.clientWidth, height = PANEL_HEIGHT, dpr = window.devicePixelRatio || 1;
      if (canvas.width !== Math.round(width * dpr) || canvas.height !== Math.round(height * dpr)) {
        canvas.width = Math.round(width * dpr);
        canvas.height = Math.round(height * dpr);
      }
      const g = canvas.getContext("2d");
      g.setTransform(dpr, 0, 0, dpr, 0, 0);
      g.clearRect(0, 0, width, height);
      const plotW = width - MARGIN.left - MARGIN.right, plotH = height - MARGIN.top - MARGIN.bottom;
      const mine = [...selected].filter(([id]) => catalogue.get(id).unit === unit);

      // What is in view decides the scale.
      let low = Infinity, high = -Infinity;
      for (const [, series] of mine) {
        for (let i = Math.max(0, indexAt(series.t, start)); i < series.t.length && series.t[i] <= end; i++) {
          const v = series.v[i];
          if (v === null) continue;
          if (v < low) low = v;
          if (v > high) high = v;
        }
      }
      if (low > high) { low = -1; high = 1; }
      const pad = (high - low) * 0.08 || Math.max(Math.abs(high) * 0.05, 0.5);
      low -= pad; high += pad;
      const X = (t) => MARGIN.left + (t - start) / windowS * plotW;
      const Y = (v) => MARGIN.top + (high - v) / (high - low) * plotH;

      // Grid and axis labels: there to be read off, not looked at.
      g.font = "11px system-ui, sans-serif";
      g.fillStyle = muted;
      g.strokeStyle = line;
      g.lineWidth = 1;
      g.textAlign = "right";
      g.textBaseline = "middle";
      for (const v of ticks(low, high, 4).values) {
        const y = Math.round(Y(v)) + 0.5;
        g.beginPath(); g.moveTo(MARGIN.left, y); g.lineTo(width - MARGIN.right, y); g.stroke();
        g.fillText(String(Number(v.toPrecision(6))), MARGIN.left - 6, y);
      }
      g.textAlign = "center";
      g.textBaseline = "top";
      const xt = ticks(-windowS, 0, 5);
      for (const s of xt.values) {
        g.fillText(s === 0 ? (paused ? "paused" : "now") : s + " s", X(end + s), height - MARGIN.bottom + 6);
      }

      // The lines. More samples than pixels: each pixel column is drawn from its lowest
      // to its highest value, so that a spike between two columns is not lost.
      g.save();
      g.beginPath(); g.rect(MARGIN.left, MARGIN.top, plotW, plotH); g.clip();
      g.lineWidth = 2;
      g.lineJoin = "round";
      for (const [, series] of mine) {
        g.strokeStyle = colour(series.slot);
        g.beginPath();
        const first = Math.max(0, indexAt(series.t, start));
        const perPixel = (indexAt(series.t, end) - first) / plotW;
        let pen = false;
        if (perPixel <= 2) {
          for (let i = first; i < series.t.length && series.t[i] <= end; i++) {
            if (series.v[i] === null) { pen = false; continue; }
            if (pen) g.lineTo(X(series.t[i]), Y(series.v[i])); else g.moveTo(X(series.t[i]), Y(series.v[i]));
            pen = true;
          }
        } else {
          let column = -1, lo = 0, hi = 0, lastV = 0;
          const flush = () => {
            if (column < 0) return;
            const x = MARGIN.left + column + 0.5;
            if (pen) g.lineTo(x, Y(lo)); else g.moveTo(x, Y(lo));
            g.lineTo(x, Y(hi));
            g.lineTo(x, Y(lastV));
            pen = true;
          };
          for (let i = first; i < series.t.length && series.t[i] <= end; i++) {
            const v = series.v[i];
            if (v === null) { flush(); column = -1; pen = false; continue; }
            const c = Math.floor(X(series.t[i]) - MARGIN.left);
            if (c !== column) { flush(); column = c; lo = hi = v; }
            if (v < lo) lo = v;
            if (v > hi) hi = v;
            lastV = v;
          }
          flush();
        }
        g.stroke();
      }
      g.restore();

      // Crosshair: the same instant in every panel, with a dot on each line.
      if (hoverT !== null) {
        const x = Math.round(X(hoverT)) + 0.5;
        g.strokeStyle = muted;
        g.lineWidth = 1;
        g.beginPath(); g.moveTo(x, MARGIN.top); g.lineTo(x, height - MARGIN.bottom); g.stroke();
      }
      for (const [id, series] of mine) {
        const i = hoverT !== null ? indexAt(series.t, hoverT) : series.t.length - 1;
        const v = i >= 0 ? series.v[i] : null;
        series.value.textContent = format(v);
        if (hoverT !== null && v !== null && i >= 0 && series.t[i] >= start) {
          g.beginPath(); g.arc(X(series.t[i]), Y(v), 4, 0, 2 * Math.PI);
          g.fillStyle = colour(series.slot); g.fill();
          g.lineWidth = 2; g.strokeStyle = surface; g.stroke();
          readout.push([colour(series.slot), catalogue.get(id).label, format(v) + " " + unit]);
        }
      }
    }

    if (hover && readout.length) {
      tip.replaceChildren(el("div", { className: "muted", textContent: (hoverT - end).toFixed(2) + " s" }),
        ...readout.map(([c, label, value]) => el("div", {}, el("i", { style: "background:" + c }), label + " ", el("b", { textContent: value }))));
      tip.style.display = "block";
      const box = root.getBoundingClientRect();
      const left = Math.min(hover.x - box.left + 14, box.width - tip.offsetWidth - 4);
      tip.style.left = Math.max(0, left) + "px";
      tip.style.top = (hover.y - box.top + 14) + "px";
    } else {
      tip.style.display = "none";
    }
  }
  requestAnimationFrame(draw);

  return {
    get count() { return selected.size; },
    get panels() { return panels.size; },
    samples(id) { return selected.has(id) ? selected.get(id).t.length : 0; },
    last(id) { const s = selected.get(id); return s && s.v.length ? s.v[s.v.length - 1] : null; },
    select(id, on = true) { toggle(id, on); },
    // What can be plotted, from the server; arrives again when the robot's daemon restarts.
    setCatalogue(signals, rates) {
      catalogue = new Map(signals.map((s) => [s.id, s]));
      for (const id of [...selected.keys()]) if (!catalogue.has(id)) selected.delete(id);
      rateSelect.replaceChildren(...rates.map((r) => el("option", { value: r, textContent: r + " Hz" })));
      if (!rates.includes(rate)) rate = rates.includes(100) ? 100 : rates[0];
      rateSelect.value = rate;
      const groups = new Map();
      for (const s of signals) {
        if (!groups.has(s.group)) groups.set(s.group, []);
        groups.get(s.group).push(s);
      }
      pickerList.replaceChildren(...[...groups].map(([group, list]) => el("fieldset", {}, el("legend", { textContent: group }),
        ...list.map((s) => {
          const box = el("input", { type: "checkbox", value: s.id });
          box.addEventListener("change", () => toggle(s.id, box.checked));
          return el("label", {}, box, " " + s.label.replace(group + " ", "") + " (" + s.unit + ")");
        }))));
      if (root.dataset.saved) {     // what was plotted the last time this page was open
        for (const id of JSON.parse(root.dataset.saved)) if (catalogue.has(id) && selected.size < MAX_SIGNALS) {
          const used = new Set([...selected.values()].map((s) => s.slot));
          let slot = 0;
          while (used.has(slot)) slot++;
          if (!selected.has(id)) selected.set(id, { slot, t: [], v: [] });
        }
        delete root.dataset.saved;
      }
      rebuildPanels();
      subscribe();
    },
    // A new connection knows nothing of what this page plots.
    resubscribe() { if (catalogue.size) subscribe(); },
    addBatch(message) {
      if (paused) return;
      for (const [id, values] of Object.entries(message.y)) {
        const series = selected.get(id);
        if (!series) continue;
        // The daemon restarted: its clock of samples starts again.
        if (series.t.length && message.t[0] < series.t[series.t.length - 1]) { series.t.length = 0; series.v.length = 0; }
        for (let i = 0; i < values.length; i++) { series.t.push(message.t[i]); series.v.push(values[i]); }
        const cut = indexAt(series.t, series.t[series.t.length - 1] - KEEP_S);
        if (cut > 2000) { series.t.splice(0, cut); series.v.splice(0, cut); }
      }
    },
  };
}
