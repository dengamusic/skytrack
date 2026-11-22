'use strict';
// skytrack replay viewer. Loads data/<scene>.json (written by tools/viewer_data.py)
// and draws it on a canvas: true paths, sensor detections as the tracker sees
// them, tracks with their 95% ellipses, and where the zoom camera is looking.
// No dependencies.

const POINT = 0;                      // detection kinds, see tools/viewer_data.py
const CONFIRMED = 1;                  // track status index
const ELLIPSE_95 = Math.sqrt(5.991);  // chi-square, 2 degrees of freedom, 95%
const DETECTION_WINDOW_S = 1.0;       // how long a detection stays on the map
const TRAIL_S = 30;
const COAST_AFTER_S = 2.0;            // no update for this long: drawn as coasting
const LEADER_S = 5;                   // velocity leader length
const SPEEDS = [1, 2, 4, 8, 16];
const LAYERS = [
  ['truth', 'True paths', true],
  ['detections', 'Detections', true],
  ['sourceColours', 'Colour detections by source', true],
  ['tentative', 'Tentative tracks', true],
  ['ellipses', 'Uncertainty', true],
  ['trails', 'Track trails', true],
  ['coverage', 'Sensor coverage', true],
  ['labels', 'Labels', true],
];
// Row fields of a track sample.
const T = 0, E = 1, N = 2, U = 3, VE = 4, VN = 5, PEE = 6, PEN = 7, PNN = 8, STATUS = 9, CLASS = 10,
      IDENT = 11, SENSORS = 12, LAST_UPDATE = 13;

const params = new URLSearchParams(location.search);
const capture = params.has('capture');

const state = {
  index: [],
  cache: new Map(),
  data: null,
  setName: null,
  t: 0,
  playing: !capture && !params.has('t'),
  speed: Number(params.get('speed')) || 4,
  view: { cx: 0, cy: 0, scale: 0.15 },
  layers: Object.fromEntries(LAYERS.map(([key, , on]) => [key, on])),
  hot: null,          // track id under the pointer (map or table)
  colours: {},
};

const $ = (id) => document.getElementById(id);
const canvas = $('map');
const ctx = canvas.getContext('2d');
const spark = $('spark');
const sctx = spark.getContext('2d');

// ---- small helpers ------------------------------------------------------------

/** First index with times[i] > t. */
function upper(times, t) {
  let lo = 0, hi = times.length;
  while (lo < hi) {
    const mid = (lo + hi) >> 1;
    if (times[mid] <= t) lo = mid + 1; else hi = mid;
  }
  return lo;
}
const lerp = (a, b, f) => a + (b - a) * f;
const fmt = (x, digits = 0) => (x == null || Number.isNaN(x) ? '—' : x.toFixed(digits));

function readColours() {
  const css = getComputedStyle(document.documentElement);
  const get = (name) => css.getPropertyValue(name).trim();
  state.colours = {
    surface: get('--surface'), ink: get('--ink'), ink2: get('--ink-2'), muted: get('--muted'),
    grid: get('--grid'), axis: get('--axis'), detection: get('--detection'), false: get('--false'),
    series: Array.from({ length: 8 }, (_, i) => get(`--series-${i + 1}`)),
  };
}
/** Target ids map to fixed palette slots, so a target keeps its colour in every scene. */
const targetColour = (id) => (id == null ? state.colours.false : state.colours.series[(id - 1) % 8]);

function withAlpha(colour, alpha) {
  const hex = colour.replace('#', '');
  const v = parseInt(hex.length === 3 ? hex.split('').map((c) => c + c).join('') : hex, 16);
  return `rgba(${(v >> 16) & 255}, ${(v >> 8) & 255}, ${v & 255}, ${alpha})`;
}

function shortSensor(id) {
  const s = state.data.sensorById.get(id);
  if (!s) return id.startsWith('site-') ? `site ${id.slice(5).toUpperCase()}` : id;
  if (s.type === 'radar') return 'radar';
  if (s.type === 'camera') return s.gimballed ? 'zoom cam' : 'cam';
  if (s.type === 'rf_bearing') return 'RF';
  if (s.protocol === 'remote_id') return 'Remote ID';
  return s.pos ? 'ADS-B' : 'feed';
}
function sensorList(set, index) {
  const ids = set.sensor_sets[index] ? set.sensor_sets[index].split('+') : [];
  return [...new Set(ids.map(shortSensor))].join(' · ');
}

// ---- data -----------------------------------------------------------------------

function prepare(data) {
  data.sensorById = new Map(data.sensors.map((s) => [s.id, s]));
  data.targetById = new Map(data.targets.map((t) => [t.id, t]));
  for (const target of data.targets) target.times = Float64Array.from(target.rows, (r) => r[0]);
  data.detectionTimes = Float64Array.from(data.detections, (d) => d[0]);
  for (const rows of Object.values(data.pointing)) rows.times = Float64Array.from(rows, (r) => r[0]);
  for (const set of Object.values(data.track_sets)) {
    for (const track of set.tracks) {
      track.times = Float64Array.from(track.rows, (r) => r[T]);
      track.everConfirmed = track.rows.some((r) => r[STATUS] === CONFIRMED);
    }
  }
  // One marker per site: sensors within 20 m of each other share it.
  data.sites = [];
  for (const s of data.sensors) {
    if (!s.pos) continue;
    const near = data.sites.find((p) => Math.hypot(p.e - s.pos[0], p.n - s.pos[1]) < 20);
    if (!near) data.sites.push({ e: s.pos[0], n: s.pos[1], name: s.site });
  }
  return data;
}

async function loadScene(name) {
  if (!state.cache.has(name)) {
    const entry = state.index.find((s) => s.name === name) || state.index[0];
    const response = await fetch(entry.file);
    if (!response.ok) throw new Error(`${entry.file}: ${response.status}`);
    state.cache.set(entry.name, prepare(await response.json()));
  }
  return state.cache.get(name);
}

function trackAt(track, t) {
  const times = track.times;
  if (t < times[0] || t > times[times.length - 1]) return null;
  const i = Math.max(0, upper(times, t) - 1);
  const a = track.rows[i];
  const b = track.rows[i + 1];
  const f = b ? (t - a[T]) / (b[T] - a[T]) : 0;
  const at = (k) => (b ? lerp(a[k], b[k], f) : a[k]);
  return {
    track, e: at(E), n: at(N), u: at(U), ve: at(VE), vn: at(VN),
    pee: at(PEE), pen: at(PEN), pnn: at(PNN),
    status: a[STATUS], cls: a[CLASS], ident: a[IDENT], sensors: a[SENSORS],
    sinceUpdate: Math.max(0, t - a[LAST_UPDATE]),
  };
}

function truthAt(target, t) {
  const times = target.times;
  if (t < times[0] || t > times[times.length - 1]) return null;
  const i = Math.max(0, upper(times, t) - 1);
  const a = target.rows[i];
  const b = target.rows[i + 1] || a;
  const f = b[0] > a[0] ? (t - a[0]) / (b[0] - a[0]) : 0;
  return { e: lerp(a[1], b[1], f), n: lerp(a[2], b[2], f), u: lerp(a[3], b[3], f) };
}

/** Compass bearing the camera looks along at t (degrees). */
function boresightAt(sensor, t) {
  const rows = state.data.pointing[sensor.id];
  if (!rows || !rows.length) return sensor.boresight_deg;
  const i = Math.max(0, upper(rows.times, t) - 1);
  const a = rows[i];
  const b = rows[i + 1];
  if (!b) return a[1];
  const d = ((b[1] - a[1] + 540) % 360) - 180;  // shortest way round
  return a[1] + d * Math.min(1, Math.max(0, (t - a[0]) / (b[0] - a[0])));
}

/** 95% ellipse of a horizontal covariance: radii (m) and the long axis angle from east. */
function ellipse(pee, pen, pnn) {
  const mid = 0.5 * (pee + pnn);
  const half = Math.sqrt(0.25 * (pee - pnn) ** 2 + pen * pen);
  return {
    major: ELLIPSE_95 * Math.sqrt(Math.max(mid + half, 0)),
    minor: ELLIPSE_95 * Math.sqrt(Math.max(mid - half, 0)),
    angle: 0.5 * Math.atan2(2 * pen, pee - pnn),
  };
}

// ---- view -----------------------------------------------------------------------

const size = () => ({ w: canvas.clientWidth, h: canvas.clientHeight });
function toScreen(e, n) {
  const { w, h } = size();
  return [w / 2 + (e - state.view.cx) * state.view.scale, h / 2 - (n - state.view.cy) * state.view.scale];
}
function toWorld(x, y) {
  const { w, h } = size();
  return [state.view.cx + (x - w / 2) / state.view.scale, state.view.cy - (y - h / 2) / state.view.scale];
}
function fit(box) {
  const { w, h } = size();
  const dx = Math.max(box.maxE - box.minE, 500);
  const dy = Math.max(box.maxN - box.minN, 500);
  state.view = { cx: (box.minE + box.maxE) / 2, cy: (box.minN + box.maxN) / 2, scale: 0.84 * Math.min(w / dx, h / dy) };
}
function sceneBox(includeAircraft) {
  const box = { minE: Infinity, maxE: -Infinity, minN: Infinity, maxN: -Infinity };
  const add = (e, n) => {
    box.minE = Math.min(box.minE, e); box.maxE = Math.max(box.maxE, e);
    box.minN = Math.min(box.minN, n); box.maxN = Math.max(box.maxN, n);
  };
  for (const s of state.data.sites) add(s.e, s.n);
  for (const target of state.data.targets) {
    if (!includeAircraft && target.class === 'aircraft') continue;
    for (const r of target.rows) add(r[1], r[2]);
  }
  return box;
}

// ---- drawing --------------------------------------------------------------------

function resize() {
  const dpr = window.devicePixelRatio || 1;
  for (const c of [canvas, spark]) {
    c.width = Math.round(c.clientWidth * dpr);
    c.height = Math.round(c.clientHeight * dpr);
    c.getContext('2d').setTransform(dpr, 0, 0, dpr, 0, 0);
  }
}

function label(text, x, y, colour, align = 'left') {
  ctx.font = '12px system-ui, -apple-system, "Segoe UI", sans-serif';
  ctx.textAlign = align;
  ctx.textBaseline = 'middle';
  ctx.lineWidth = 3;
  ctx.strokeStyle = state.colours.surface;
  ctx.strokeText(text, x, y);
  ctx.fillStyle = colour;
  ctx.fillText(text, x, y);
}

function drawGrid() {
  const c = state.colours;
  const [ox, oy] = toScreen(0, 0);
  const steps = [250, 500, 1000, 2000, 5000];
  const step = steps.find((s) => s * state.view.scale > 70) || 10000;
  const { w, h } = size();
  const far = Math.max(...[[0, 0], [w, 0], [0, h], [w, h]].map(([x, y]) => Math.hypot(x - ox, y - oy))) / state.view.scale;
  ctx.lineWidth = 1;
  for (let r = step; r <= far; r += step) {
    ctx.strokeStyle = c.grid;
    ctx.beginPath();
    ctx.arc(ox, oy, r * state.view.scale, 0, 2 * Math.PI);
    ctx.stroke();
    label(r >= 1000 ? `${r / 1000} km` : `${r} m`, ox, oy - r * state.view.scale, c.muted, 'center');
  }
  // North arrow and scale bar.
  ctx.fillStyle = c.muted;
  ctx.strokeStyle = c.muted;
  ctx.beginPath();
  ctx.moveTo(24, 22); ctx.lineTo(19, 34); ctx.lineTo(29, 34); ctx.closePath();
  ctx.fill();
  label('N', 24, 44, c.muted, 'center');
  const bar = steps.find((s) => s * state.view.scale > 60) || 10000;
  const y = h - 18;
  ctx.beginPath();
  ctx.moveTo(16, y - 4); ctx.lineTo(16, y); ctx.lineTo(16 + bar * state.view.scale, y); ctx.lineTo(16 + bar * state.view.scale, y - 4);
  ctx.stroke();
  label(bar >= 1000 ? `${bar / 1000} km` : `${bar} m`, 22 + bar * state.view.scale, y - 2, c.muted);
}

function wedge(sensor, bearingDeg, rangeM, fillAlpha, edgeAlpha) {
  const [x, y] = toScreen(sensor.pos[0], sensor.pos[1]);
  const half = (sensor.hfov_deg / 2) * Math.PI / 180;
  const b = bearingDeg * Math.PI / 180;
  // Canvas angles run clockwise from +x (east); a compass bearing runs clockwise from north.
  const a0 = b - half - Math.PI / 2;
  const a1 = b + half - Math.PI / 2;
  const r = rangeM * state.view.scale;
  ctx.beginPath();
  ctx.moveTo(x, y);
  ctx.arc(x, y, r, a0, a1);
  ctx.closePath();
  ctx.fillStyle = withAlpha(state.colours.ink2, fillAlpha);
  ctx.fill();
  ctx.strokeStyle = withAlpha(state.colours.ink2, edgeAlpha);
  ctx.lineWidth = 1;
  ctx.stroke();
}

function drawSensors(t) {
  const c = state.colours;
  if (state.layers.coverage) {
    for (const s of state.data.sensors) {
      if (!s.pos) continue;
      if (s.type === 'radar' && s.range_m) {
        const [x, y] = toScreen(s.pos[0], s.pos[1]);
        ctx.setLineDash([6, 6]);
        ctx.strokeStyle = c.axis;
        ctx.lineWidth = 1;
        ctx.beginPath();
        ctx.arc(x, y, s.range_m * state.view.scale, 0, 2 * Math.PI);
        ctx.stroke();
        ctx.setLineDash([]);
      }
      if (s.type === 'camera') {
        if (s.gimballed) wedge(s, boresightAt(s, t), s.range_m || 6000, 0.14, 0.45);
        else wedge(s, s.boresight_deg, Math.min(s.range_m || 3000, 1500), 0.045, 0.12);
      }
    }
  }
  for (const site of state.data.sites) {
    const [x, y] = toScreen(site.e, site.n);
    ctx.fillStyle = c.ink;
    ctx.strokeStyle = c.surface;
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(x, y - 7); ctx.lineTo(x - 6, y + 5); ctx.lineTo(x + 6, y + 5); ctx.closePath();
    ctx.stroke();
    ctx.fill();
    if (state.layers.labels && site.name && state.data.sites.length > 1) {
      label(`site ${site.name.replace('site-', '').toUpperCase()}`, x + 10, y + 2, c.ink2);
    }
  }
}

function drawTruth(t) {
  if (!state.layers.truth) return;
  for (const target of state.data.targets) {
    ctx.strokeStyle = withAlpha(targetColour(target.id), 0.22);
    ctx.lineWidth = 5;
    ctx.lineCap = 'round';
    ctx.lineJoin = 'round';
    ctx.beginPath();
    target.rows.forEach((r, i) => {
      const [x, y] = toScreen(r[1], r[2]);
      if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
    });
    ctx.stroke();
  }
}

function drawTruthMarkers(t) {
  if (!state.layers.truth) return;
  for (const target of state.data.targets) {
    const p = truthAt(target, t);
    if (!p) continue;
    const [x, y] = toScreen(p.e, p.n);
    ctx.strokeStyle = targetColour(target.id);
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(x - 6, y); ctx.lineTo(x + 6, y);
    ctx.moveTo(x, y - 6); ctx.lineTo(x, y + 6);
    ctx.stroke();
    if (state.layers.labels) label(target.name, x + 9, y + 12, state.colours.muted);
  }
}

function detectionColour(truth, alpha) {
  if (!state.layers.sourceColours) return withAlpha(state.colours.detection, alpha);
  return truth >= 0 ? withAlpha(targetColour(truth), alpha) : withAlpha(state.colours.false, alpha);
}

function drawDetections(t) {
  if (!state.layers.detections) return;
  const d = state.data;
  const i0 = upper(d.detectionTimes, t - DETECTION_WINDOW_S);
  const i1 = upper(d.detectionTimes, t);
  for (let i = i0; i < i1; i++) {
    const [td, sensorIndex, kind, a, b, truth] = d.detections[i];
    const alpha = 1 - 0.8 * (t - td) / DETECTION_WINDOW_S;
    const s = d.sensors[sensorIndex];
    if (kind === POINT) {
      const [x, y] = toScreen(a, b);
      ctx.strokeStyle = detectionColour(truth, alpha);
      ctx.lineWidth = 1.3;
      ctx.beginPath();
      if (s.type === 'radar') ctx.arc(x, y, 3, 0, 2 * Math.PI);
      else ctx.rect(x - 3, y - 3, 6, 6);
      ctx.stroke();
    } else if (s.pos) {
      const [x0, y0] = toScreen(s.pos[0], s.pos[1]);
      const reach = (s.range_m || 6000) * state.view.scale;
      const rad = a * Math.PI / 180;
      ctx.strokeStyle = detectionColour(truth, 0.55 * alpha);
      ctx.lineWidth = 1;
      if (s.type === 'rf_bearing') ctx.setLineDash([5, 4]);
      ctx.beginPath();
      ctx.moveTo(x0, y0);
      ctx.lineTo(x0 + reach * Math.sin(rad), y0 - reach * Math.cos(rad));
      ctx.stroke();
      ctx.setLineDash([]);
    }
  }
}

function visibleTracks(t) {
  const set = state.data.track_sets[state.setName];
  const out = [];
  for (const track of set.tracks) {
    const s = trackAt(track, t);
    if (!s) continue;
    if (s.status !== CONFIRMED && !state.layers.tentative) continue;
    out.push(s);
  }
  return out;
}

function drawTracks(t, tracks) {
  const c = state.colours;
  const set = state.data.track_sets[state.setName];
  // Tentative first, so confirmed tracks sit on top.
  const ordered = [...tracks].sort((a, b) => a.status - b.status);
  for (const s of ordered) {
    const confirmed = s.status === CONFIRMED;
    const colour = targetColour(s.track.target);
    const hot = state.hot === s.track.id;
    const [x, y] = toScreen(s.e, s.n);
    if (state.layers.trails && confirmed) {
      const rows = s.track.rows;
      const i0 = upper(s.track.times, t - TRAIL_S);
      const i1 = upper(s.track.times, t);
      ctx.strokeStyle = colour;
      ctx.lineWidth = hot ? 3 : 2;
      ctx.lineJoin = 'round';
      ctx.beginPath();
      for (let i = i0; i < i1; i++) {
        const [px, py] = toScreen(rows[i][E], rows[i][N]);
        if (i === i0) ctx.moveTo(px, py); else ctx.lineTo(px, py);
      }
      ctx.lineTo(x, y);
      ctx.stroke();
    }
    if (state.layers.ellipses && confirmed) {
      const el = ellipse(s.pee, s.pen, s.pnn);
      ctx.beginPath();
      ctx.ellipse(x, y, Math.max(el.major * state.view.scale, 1), Math.max(el.minor * state.view.scale, 1),
                  -el.angle, 0, 2 * Math.PI);
      ctx.fillStyle = withAlpha(colour, 0.12);
      ctx.fill();
      ctx.strokeStyle = withAlpha(colour, 0.7);
      ctx.lineWidth = 1;
      if (s.sinceUpdate > COAST_AFTER_S) ctx.setLineDash([4, 3]);
      ctx.stroke();
      ctx.setLineDash([]);
    }
    if (confirmed) {
      ctx.strokeStyle = colour;
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.moveTo(x, y);
      ctx.lineTo(x + s.ve * LEADER_S * state.view.scale, y - s.vn * LEADER_S * state.view.scale);
      ctx.stroke();
      ctx.beginPath();
      ctx.arc(x, y, hot ? 6 : 4.5, 0, 2 * Math.PI);
      ctx.fillStyle = colour;
      ctx.fill();
      ctx.strokeStyle = c.surface;
      ctx.lineWidth = 2;
      ctx.stroke();
      if (state.layers.labels || hot) {
        const cls = set.classes[s.cls];
        label(`T${s.track.id}${cls && cls !== 'unknown' ? ` ${cls}` : ''}`, x + 9, y - 10, c.ink);
      }
    } else {
      ctx.beginPath();
      ctx.arc(x, y, hot ? 5 : 3, 0, 2 * Math.PI);
      ctx.strokeStyle = withAlpha(s.track.target ? colour : c.muted, 0.8);
      ctx.lineWidth = 1.2;
      ctx.stroke();
    }
  }
}

function drawCaption(t) {
  if (!capture) return;
  const c = state.colours;
  const title = '600 15px system-ui, -apple-system, "Segoe UI", sans-serif';
  const body = '13px system-ui, -apple-system, "Segoe UI", sans-serif';
  const note = currentNote(t);
  ctx.font = body;
  const lines = [`t = ${t.toFixed(1)} s`, ...(note ? wrapLines(note.text, Math.min(size().w - 120, 520)) : [])];
  const width = Math.max(...lines.map((l) => ctx.measureText(l).width),
                         (ctx.font = title, ctx.measureText(`skytrack · ${state.data.title}`).width));
  ctx.fillStyle = withAlpha(c.surface, 0.85);
  ctx.fillRect(40, 8, width + 20, 34 + 18 * lines.length);
  ctx.textAlign = 'left';
  ctx.textBaseline = 'top';
  ctx.fillStyle = c.ink;
  ctx.font = title;
  ctx.fillText(`skytrack · ${state.data.title}`, 50, 16);
  ctx.font = body;
  ctx.fillStyle = c.ink2;
  lines.forEach((line, i) => ctx.fillText(line, 50, 38 + 18 * i));
}
/** Splits text into lines no wider than `width` in the current font. */
function wrapLines(text, width) {
  const lines = [];
  let line = '';
  for (const word of text.split(' ')) {
    const next = line ? `${line} ${word}` : word;
    if (ctx.measureText(next).width > width && line) {
      lines.push(line);
      line = word;
    } else {
      line = next;
    }
  }
  if (line) lines.push(line);
  return lines;
}

function draw() {
  if (!state.data) return;
  const { w, h } = size();
  ctx.fillStyle = state.colours.surface;
  ctx.fillRect(0, 0, w, h);
  const t = state.t;
  const tracks = visibleTracks(t);
  drawGrid();
  drawSensors(t);
  drawTruth(t);
  drawDetections(t);
  drawTracks(t, tracks);
  drawTruthMarkers(t);
  drawCaption(t);
  state.lastTracks = tracks;
}

function drawSpark() {
  if (!state.data) return;
  const c = state.colours;
  const w = spark.clientWidth;
  const h = spark.clientHeight;
  const set = state.data.track_sets[state.setName];
  const duration = state.data.duration_s;
  const xOf = (t) => (t / duration) * w;
  sctx.clearRect(0, 0, w, h);
  for (const note of state.data.notes) {
    sctx.fillStyle = withAlpha(c.ink2, 0.08);
    sctx.fillRect(xOf(note.t0), 0, xOf(note.t1) - xOf(note.t0), h);
  }
  const top = Math.max(...set.gospa.map((g) => g[1]), 1);
  const yOf = (v) => h - 2 - (v / top) * (h - 14);
  sctx.beginPath();
  sctx.moveTo(xOf(set.gospa[0][0]), h);
  for (const [t, g] of set.gospa) sctx.lineTo(xOf(t), yOf(g));
  sctx.lineTo(xOf(set.gospa[set.gospa.length - 1][0]), h);
  sctx.closePath();
  sctx.fillStyle = withAlpha(c.series[0], 0.18);
  sctx.fill();
  sctx.beginPath();
  set.gospa.forEach(([t, g], i) => (i ? sctx.lineTo(xOf(t), yOf(g)) : sctx.moveTo(xOf(t), yOf(g))));
  sctx.strokeStyle = c.series[0];
  sctx.lineWidth = 1.5;
  sctx.stroke();
  sctx.font = '11px system-ui, -apple-system, "Segoe UI", sans-serif';
  sctx.fillStyle = c.muted;
  sctx.textBaseline = 'top';
  sctx.fillText('GOSPA over time (m)', 4, 1);
  const x = xOf(state.t);
  sctx.strokeStyle = c.ink;
  sctx.lineWidth = 1.5;
  sctx.beginPath();
  sctx.moveTo(x, 0);
  sctx.lineTo(x, h);
  sctx.stroke();
}

// ---- side panel -----------------------------------------------------------------

function currentNote(t) {
  return state.data.notes.find((n) => t >= n.t0 && t < n.t1) || null;
}

function renderPanel() {
  const d = state.data;
  const set = d.track_sets[state.setName];
  const t = state.t;
  $('clock').textContent = `${t.toFixed(1)} s / ${d.duration_s.toFixed(0)} s`;
  $('time').value = t;
  const note = currentNote(t);
  $('note').hidden = !note;
  if (note) $('note').textContent = `${note.t0}–${note.t1} s: ${note.text}`;

  const tracks = (state.lastTracks || visibleTracks(t))
    .filter((s) => s.status === CONFIRMED || s.track.everConfirmed)
    .sort((a, b) => b.status - a.status || a.track.id - b.track.id);
  const tentative = (state.lastTracks || []).filter((s) => s.status !== CONFIRMED).length;
  $('track-count').textContent = `· ${tracks.filter((s) => s.status === CONFIRMED).length} confirmed` +
    (state.layers.tentative ? `, ${tentative} tentative` : '');
  const rows = tracks.map((s) => {
    const el = ellipse(s.pee, s.pen, s.pnn);
    const cls = set.classes[s.cls];
    return `<tr data-id="${s.track.id}" class="${s.status === CONFIRMED ? '' : 'tentative'}${state.hot === s.track.id ? ' hot' : ''}">
      <td><span class="swatch" style="background:${targetColour(s.track.target)}"></span>T${s.track.id}</td>
      <td>${cls}</td>
      <td class="num">${fmt(Math.hypot(s.ve, s.vn), 0)} m/s</td>
      <td class="num">${fmt(el.major, 0)} m</td>
      <td>${sensorList(set, s.sensors)}</td></tr>`;
  });
  $('track-rows').innerHTML = rows.join('') || '<tr class="empty"><td colspan="5">no confirmed tracks</td></tr>';
}

function renderScores() {
  const set = state.data.track_sets[state.setName];
  $('gospa-mean').textContent = `${set.metrics.gospa_mean_m.toFixed(1)} m`;
  $('score-rows').innerHTML = set.metrics.targets.map((m) => `<tr>
      <td><span class="swatch" style="background:${targetColour(m.id)}"></span>${m.name} <span class="hint">${m.class}</span></td>
      <td class="num">${fmt(100 * m.tracked_fraction, 0)}%</td>
      <td class="num">${fmt(m.rmse_horizontal_m, 0)} m</td>
      <td class="num">${m.breaks}</td>
      <td class="num">${m.time_to_classify_s == null ? '<span class="hint" title="never given the right class">never</span>' : `${fmt(m.time_to_classify_s, 0)} s`}</td>
    </tr>`).join('');
}

function renderSetPicker() {
  const names = Object.keys(state.data.track_sets);
  const picker = $('set-picker');
  picker.hidden = names.length < 2;
  const caption = (n) => (n === 'fused' ? 'Fusion centre' : n.startsWith('site-') ? `Site ${n.slice(5).toUpperCase()} alone` : n);
  picker.innerHTML = names.map((n) => `<button data-set="${n}" aria-pressed="${n === state.setName}">${caption(n)}</button>`).join('');
}

function renderTooltip(x, y) {
  const tip = $('tooltip');
  const s = (state.lastTracks || []).find((k) => k.track.id === state.hot);
  if (!s) { tip.hidden = true; return; }
  const set = state.data.track_sets[state.setName];
  const el = ellipse(s.pee, s.pen, s.pnn);
  const target = state.data.targetById.get(s.track.target);
  const truth = target && truthAt(target, state.t);
  const heading = (Math.atan2(s.ve, s.vn) * 180 / Math.PI + 360) % 360;
  const ident = set.identities[s.ident];
  const rows = [
    ['Class', set.classes[s.cls]],
    ident ? ['Identity', ident.replace(':', ' ')] : null,
    ['Sensors', sensorList(set, s.sensors) || '—'],
    ['Speed', `${fmt(Math.hypot(s.ve, s.vn), 1)} m/s, heading ${fmt(heading, 0)}°`],
    ['Altitude', `${fmt(s.u, 0)} m`],
    ['95% ellipse', `${fmt(el.major, 0)} × ${fmt(el.minor, 0)} m`],
    ['Last update', `${fmt(s.sinceUpdate, 1)} s ago`],
    ['Following', target ? target.name : 'nothing (false track)'],
    truth ? ['Error vs truth', `${fmt(Math.hypot(s.e - truth.e, s.n - truth.n), 0)} m`] : null,
  ].filter(Boolean);
  tip.innerHTML = `<div class="tt-head"><span class="swatch" style="background:${targetColour(s.track.target)}"></span>
      Track T${s.track.id} <span class="hint">${s.status === CONFIRMED ? 'confirmed' : 'tentative'}</span></div>
      <dl>${rows.map(([k, v]) => `<dt>${k}</dt><dd>${v}</dd>`).join('')}</dl>`;
  tip.hidden = false;
  const box = canvas.parentElement.getBoundingClientRect();
  const left = Math.min(x + 16, box.width - tip.offsetWidth - 8);
  const top = Math.min(y + 16, box.height - tip.offsetHeight - 8);
  tip.style.left = `${Math.max(8, left)}px`;
  tip.style.top = `${Math.max(8, top)}px`;
}

// ---- scenes and controls --------------------------------------------------------

async function showScene(name, setName) {
  $('loading').hidden = false;
  const data = await loadScene(name);
  state.data = data;
  const sets = Object.keys(data.track_sets);
  state.setName = sets.includes(setName) ? setName : sets[0];
  state.t = Math.min(state.t, data.duration_s);
  $('time').max = data.duration_s;
  $('scene-title').textContent = data.title;
  $('scene-blurb').textContent = data.blurb;
  for (const b of $('scenes').querySelectorAll('button')) b.setAttribute('aria-pressed', String(b.dataset.scene === data.name));
  renderSetPicker();
  renderScores();
  fit(sceneBox(false));
  $('loading').hidden = true;
  updateUrl();
  draw();
  drawSpark();
  renderPanel();
}

function updateUrl() {
  const p = new URLSearchParams(location.search);
  p.set('scene', state.data.name);
  if (Object.keys(state.data.track_sets).length > 1) p.set('set', state.setName); else p.delete('set');
  history.replaceState(null, '', `?${p}`);
}

function setPlaying(on) {
  state.playing = on && state.t < state.data.duration_s;
  $('play').textContent = state.playing ? '❚❚' : '▶';
  $('play').setAttribute('aria-label', state.playing ? 'Pause' : 'Play');
}

function seek(t) {
  state.t = Math.max(0, Math.min(state.data.duration_s, t));
  draw();
  drawSpark();
  renderPanel();
}

function bindControls() {
  $('scenes').addEventListener('click', (ev) => {
    const b = ev.target.closest('button[data-scene]');
    if (b) showScene(b.dataset.scene, state.setName);
  });
  $('set-picker').addEventListener('click', (ev) => {
    const b = ev.target.closest('button[data-set]');
    if (!b) return;
    state.setName = b.dataset.set;
    renderSetPicker();
    renderScores();
    updateUrl();
    seek(state.t);
  });
  $('play').addEventListener('click', () => {
    if (state.t >= state.data.duration_s) state.t = 0;
    setPlaying(!state.playing);
  });
  const speed = $('speed');
  speed.innerHTML = SPEEDS.map((s) => `<option value="${s}">${s}×</option>`).join('');
  speed.value = String(SPEEDS.includes(state.speed) ? state.speed : 4);
  speed.addEventListener('change', () => { state.speed = Number(speed.value); });
  $('time').addEventListener('input', (ev) => seek(Number(ev.target.value)));
  spark.addEventListener('click', (ev) => {
    const r = spark.getBoundingClientRect();
    seek(((ev.clientX - r.left) / r.width) * state.data.duration_s);
  });
  $('fit-site').addEventListener('click', () => { fit(sceneBox(false)); draw(); });
  $('fit-all').addEventListener('click', () => { fit(sceneBox(true)); draw(); });
  $('theme').addEventListener('click', () => {
    const next = document.documentElement.dataset.theme === 'dark' ? 'light' : 'dark';
    document.documentElement.dataset.theme = next;
    localStorage.setItem('skytrack-theme', next);
    readColours();
    draw();
    drawSpark();
    renderPanel();
    renderScores();
  });
  $('layers').innerHTML = LAYERS.map(([key, text]) =>
    `<label><input type="checkbox" data-layer="${key}" ${state.layers[key] ? 'checked' : ''}> ${text}</label>`).join('');
  $('layers').addEventListener('change', (ev) => {
    state.layers[ev.target.dataset.layer] = ev.target.checked;
    draw();
    renderPanel();
  });
  $('track-rows').addEventListener('pointerover', (ev) => {
    const row = ev.target.closest('tr[data-id]');
    state.hot = row ? Number(row.dataset.id) : null;
    draw();
  });
  $('track-rows').addEventListener('pointerleave', () => { state.hot = null; draw(); });

  // Map: drag to pan, wheel to zoom about the pointer, hover for a track's details.
  let drag = null;
  canvas.addEventListener('pointerdown', (ev) => {
    drag = { x: ev.clientX, y: ev.clientY, cx: state.view.cx, cy: state.view.cy };
    canvas.setPointerCapture(ev.pointerId);
    canvas.classList.add('dragging');
  });
  canvas.addEventListener('pointerup', (ev) => {
    drag = null;
    canvas.releasePointerCapture(ev.pointerId);
    canvas.classList.remove('dragging');
  });
  canvas.addEventListener('pointermove', (ev) => {
    const r = canvas.getBoundingClientRect();
    const x = ev.clientX - r.left;
    const y = ev.clientY - r.top;
    if (drag) {
      state.view.cx = drag.cx - (ev.clientX - drag.x) / state.view.scale;
      state.view.cy = drag.cy + (ev.clientY - drag.y) / state.view.scale;
      $('tooltip').hidden = true;
      draw();
      return;
    }
    let best = null;
    let bestDist = 14;
    for (const s of state.lastTracks || []) {
      const [sx, sy] = toScreen(s.e, s.n);
      const dist = Math.hypot(sx - x, sy - y) - (s.status === CONFIRMED ? 4 : 0);
      if (dist < bestDist) { best = s; bestDist = dist; }
    }
    const id = best ? best.track.id : null;
    if (id !== state.hot) { state.hot = id; draw(); renderPanel(); }
    state.pointer = [x, y];
    renderTooltip(x, y);
  });
  canvas.addEventListener('pointerleave', () => {
    state.hot = null;
    state.pointer = null;
    $('tooltip').hidden = true;
    draw();
  });
  canvas.addEventListener('wheel', (ev) => {
    ev.preventDefault();
    const r = canvas.getBoundingClientRect();
    const x = ev.clientX - r.left;
    const y = ev.clientY - r.top;
    const [e, n] = toWorld(x, y);
    state.view.scale = Math.min(20, Math.max(0.005, state.view.scale * Math.exp(-ev.deltaY * 0.0015)));
    const [e2, n2] = toWorld(x, y);
    state.view.cx += e - e2;
    state.view.cy += n - n2;
    draw();
  }, { passive: false });

  window.addEventListener('keydown', (ev) => {
    if (ev.target.matches('input, select, textarea')) return;
    if (ev.key === ' ') { ev.preventDefault(); $('play').click(); }
    if (ev.key === 'ArrowRight') seek(state.t + (ev.shiftKey ? 10 : 1));
    if (ev.key === 'ArrowLeft') seek(state.t - (ev.shiftKey ? 10 : 1));
  });
  new ResizeObserver(() => { resize(); draw(); drawSpark(); }).observe(canvas.parentElement);
}

let lastFrame = null;
let lastPanel = 0;
function frame(now) {
  if (state.playing && lastFrame != null && state.data) {
    state.t += ((now - lastFrame) / 1000) * state.speed;
    if (state.t >= state.data.duration_s) {
      state.t = state.data.duration_s;
      setPlaying(false);
    }
    draw();
    if (state.pointer) renderTooltip(...state.pointer);
    drawSpark();
    if (now - lastPanel > 120) { renderPanel(); lastPanel = now; }
  }
  lastFrame = now;
  requestAnimationFrame(frame);
}

async function main() {
  const theme = params.get('theme') || localStorage.getItem('skytrack-theme');
  if (theme === 'light' || theme === 'dark') document.documentElement.dataset.theme = theme;
  if (capture) document.body.classList.add('capture');
  readColours();
  resize();
  try {
    state.index = await (await fetch('data/scenes.json')).json();
  } catch (err) {
    $('loading').textContent = 'No scene data found. Build it with: python3 tools/viewer_data.py';
    return;
  }
  $('scenes').innerHTML = state.index.map((s) => `<button data-scene="${s.name}">${s.title}</button>`).join('');
  bindControls();
  state.t = Number(params.get('t')) || 0;
  await showScene(params.get('scene') || state.index[0].name, params.get('set'));
  setPlaying(state.playing);
  if (capture) document.body.dataset.ready = '1';
  requestAnimationFrame(frame);
}

main().catch((err) => {
  $('loading').hidden = false;
  $('loading').textContent = `Could not load the replay: ${err.message}`;
});
