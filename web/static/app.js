import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { RoomEnvironment } from "three/addons/environments/RoomEnvironment.js";

const DEBOUNCE_MS = 400;

// ---------- scene ----------
const view = document.getElementById("view");
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
renderer.toneMapping = THREE.ACESFilmicToneMapping;
renderer.localClippingEnabled = true;
view.appendChild(renderer.domElement);

const scene = new THREE.Scene();
scene.background = new THREE.Color("#15171b");
scene.environment = new THREE.PMREMGenerator(renderer).fromScene(new RoomEnvironment(), 0.04).texture;

const camera = new THREE.PerspectiveCamera(35, 1, 1, 5000);
const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;

scene.add(new THREE.HemisphereLight(0xffffff, 0x303540, 0.6));
const sun = new THREE.DirectionalLight(0xffffff, 1.6);
sun.position.set(100, 160, -60);   // over the home view's shoulder
sun.castShadow = true;
sun.shadow.mapSize.set(2048, 2048);
Object.assign(sun.shadow.camera, { left: -120, right: 120, top: 120, bottom: -120, near: 1, far: 600 });
scene.add(sun);

const ground = new THREE.Mesh(new THREE.PlaneGeometry(2000, 2000), new THREE.ShadowMaterial({ opacity: 0.3 }));
ground.rotation.x = -Math.PI / 2;
ground.receiveShadow = true;
scene.add(ground);

// cad (Z-up -> Y-up, centered on the gearbox) > motion groups
const cad = new THREE.Group();
cad.rotation.x = -Math.PI / 2;
scene.add(cad);
const groups = {};
for (const g of ["housing", "input", "cycloid", "output"]) {
  groups[g] = new THREE.Group();
  groups[g].matrixAutoUpdate = false;
  cad.add(groups[g]);
}

function resize() {
  const { clientWidth: w, clientHeight: h } = view;
  if (!w || !h) return;   // hidden or not laid out yet
  renderer.setSize(w, h);
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
}
new ResizeObserver(resize).observe(view);   // also catches sidebar toggles and pane reveals
resize();

// ---------- model ----------
let motion = null;
let framed = false;
// section plane: contains the gearbox axis, faces the camera. When off it sits far away so
// nothing is clipped (changing the plane count would force shader recompiles).
const sectionPlane = new THREE.Plane(new THREE.Vector3(0, 1, 0), 1e6);
const clip = [sectionPlane];
const edgeMaterial = new THREE.LineBasicMaterial({ color: 0x111316, transparent: true, opacity: 0.55, clippingPlanes: clip });

function decode(b64, Type) {
  const bin = atob(b64);
  const bytes = new Uint8Array(bin.length);
  for (let i = 0; i < bin.length; i++) bytes[i] = bin.charCodeAt(i);
  return new Type(bytes.buffer);
}

function clearModel() {
  const geoms = new Set();
  for (const g of Object.values(groups)) {
    for (const child of [...g.children]) {
      g.remove(child);
      geoms.add(child.geometry);    // mesh and cap share a geometry
      if (child.material !== edgeMaterial) child.material?.dispose();
    }
  }
  for (const geom of geoms) geom?.dispose();
}

function loadModel(payload) {
  clearModel();
  motion = payload.motion;
  for (const [name, part] of Object.entries(payload.parts)) {
    const geom = new THREE.BufferGeometry();
    geom.setAttribute("position", new THREE.BufferAttribute(decode(part.verts, Float32Array), 3));
    geom.setIndex(new THREE.BufferAttribute(decode(part.tris, Uint32Array), 1));
    geom.computeVertexNormals();
    const custom = part.kind === "custom";
    const mat = new THREE.MeshStandardMaterial({
      color: part.color,
      metalness: custom ? 0.05 : 0.7,
      roughness: custom ? 0.65 : 0.35,
      clippingPlanes: clip,
    });
    // back faces only show where the section plane opens a solid; flat and darker, they read
    // as the cut face
    const capMat = new THREE.MeshBasicMaterial({
      color: new THREE.Color(part.color).multiplyScalar(0.45),
      side: THREE.BackSide,
      clippingPlanes: clip,
    });
    const mats = payload.instances[name].map((m) => new THREE.Matrix4().fromArray(m));
    for (const [m, shadows] of [[mat, true], [capMat, false]]) {
      const mesh = new THREE.InstancedMesh(geom, m, mats.length);
      mats.forEach((x, i) => mesh.setMatrixAt(i, x));
      mesh.castShadow = mesh.receiveShadow = shadows;
      mesh.userData = { name, custom };
      groups[part.group].add(mesh);
    }

    if (custom && mats.length === 1) {   // crisp CAD-style edges on the custom parts
      const edges = new THREE.LineSegments(new THREE.EdgesGeometry(geom, 25), edgeMaterial);
      edges.matrixAutoUpdate = false;
      edges.matrix.copy(mats[0]);
      edges.userData = { name, custom };
      groups[part.group].add(edges);
    }
  }
  renderPartsBar(payload);
  applyVisibility();
  cad.position.set(0, -motion.height / 2, 0);       // center at mid-height (cad is rotated, so world Y)
  ground.position.y = -motion.height / 2 - 6;
  if (!framed) {
    setView("home", false);
    framed = true;
  }
}

// ---------- camera views ----------
// directions in CAD coordinates (Z = gearbox axis); world = (x, z, -y)
const deg = THREE.MathUtils.degToRad;
const VIEWS = {
  x: [1, 0, 0],
  y: [0, 1, 0],
  z: [0, -1e-4, 1],     // nudged so CAD +Y stays up on screen
  home: [Math.cos(deg(45)) * Math.cos(deg(30)), Math.sin(deg(45)) * Math.cos(deg(30)), Math.sin(deg(30))],
};
let flight = null;

function viewDistance() {
  return (motion ? motion.radius * 3.6 : 200) / Math.min(1, camera.aspect || 1) ** 0.85;   // fits portrait too
}

function setView(name, animate = true) {
  const [x, y, z] = VIEWS[name];
  const to = new THREE.Vector3(x, z, -y).normalize().multiplyScalar(viewDistance());
  if (!animate) {
    camera.position.copy(to);
    controls.target.set(0, 0, 0);
    controls.update();
    return;
  }
  flight = { from: camera.position.clone(), fromTarget: controls.target.clone(), to, t: 0 };
}

function updateFlight(dt) {
  if (!flight) return;
  flight.t = Math.min(1, flight.t + dt / 0.45);
  const k = 1 - (1 - flight.t) ** 3;    // ease out
  // swing around the target rather than cutting through the model
  const r = THREE.MathUtils.lerp(flight.from.length(), flight.to.length(), k);
  camera.position.copy(flight.from).normalize().lerp(flight.to.clone().normalize(), k).normalize().multiplyScalar(r);
  controls.target.copy(flight.fromTarget).lerp(new THREE.Vector3(), k);
  if (flight.t >= 1) flight = null;
}

for (const b of document.querySelectorAll("[data-view]")) b.addEventListener("click", () => setView(b.dataset.view));

// custom parts the user has hidden from the parts bar; kept across rebuilds
const hiddenParts = new Set();
const partsBar = document.getElementById("parts-bar");

function applyVisibility() {
  const hardware = document.getElementById("hardware").checked;
  for (const g of Object.values(groups)) {
    for (const c of g.children) {
      const { name, custom } = c.userData;
      c.visible = custom ? !hiddenParts.has(name) : hardware;
    }
  }
  for (const chip of partsBar.querySelectorAll("[data-part]")) {
    const on = !hiddenParts.has(chip.dataset.part);
    chip.classList.toggle("off", !on);
    chip.setAttribute("aria-pressed", on);
  }
}

function renderPartsBar(payload) {
  const custom = Object.entries(payload.parts).filter(([, p]) => p.kind === "custom");
  const names = new Set(custom.map(([n]) => n));
  for (const n of [...hiddenParts]) if (!names.has(n)) hiddenParts.delete(n);
  partsBar.replaceChildren();
  for (const [name, part] of custom) {
    const chip = document.createElement("button");
    chip.type = "button";
    chip.className = "chip";
    chip.dataset.part = name;
    chip.title = "click to show / hide";
    const sw = document.createElement("span");
    sw.className = "swatch";
    sw.style.background = part.color;
    chip.append(sw, name.replaceAll("_", " "));
    chip.addEventListener("click", () => {
      hiddenParts.has(name) ? hiddenParts.delete(name) : hiddenParts.add(name);
      applyVisibility();
    });
    partsBar.append(chip);
  }
  const all = document.createElement("button");
  all.type = "button";
  all.className = "chip all";
  all.textContent = "show all";
  all.addEventListener("click", () => { hiddenParts.clear(); applyVisibility(); });
  partsBar.append(all);
}

// ---------- motion ----------
let theta = 0;        // input angle, CCW about the motor axis (viewed from above)
let paused = false;
const clock = new THREE.Clock();
const m1 = new THREE.Matrix4(), m2 = new THREE.Matrix4(), m3 = new THREE.Matrix4();
const Y = new THREE.Vector3(0, 1, 0);
const pivotW = new THREE.Vector3(), nW = new THREE.Vector3();

function updateSection() {
  const on = document.getElementById("section").checked;
  ground.visible = !on;    // shadow maps ignore clipping, so the shadow would show the uncut body
  if (!on) {
    sectionPlane.set(Y, 1e6);
    return;
  }
  // plane containing the gearbox axis whose normal points away from the camera, so the
  // camera-side half is cut away; the slider turns it about the axis
  cad.updateMatrixWorld(true);
  cad.getWorldPosition(pivotW);                 // the gearbox axis is world Y through here
  nW.copy(camera.position).sub(pivotW);
  nW.y = 0;
  if (nW.lengthSq() < 1e-9) nW.set(1, 0, 0);    // looking straight down the axis
  const angle = THREE.MathUtils.degToRad(Number(document.getElementById("section-angle").value));
  nW.normalize().applyAxisAngle(Y, angle);
  sectionPlane.setFromNormalAndCoplanarPoint(nW.negate(), pivotW);
}

function updateMotion() {
  if (!motion) return;
  const { e, ratio } = motion;
  const out = -theta / ratio;   // output (and cycloid spin) turns the opposite way, 1/ratio as fast
  groups.input.matrix.makeRotationZ(theta);
  groups.output.matrix.makeRotationZ(out);
  // cycloid: undo the baked-in +e·X offset, spin about its own center, re-center on the orbit
  groups.cycloid.matrix
    .makeTranslation(e * Math.cos(theta), e * Math.sin(theta), 0)
    .multiply(m1.makeRotationZ(out))
    .multiply(m2.makeTranslation(-motion.cycloid_center[0], -motion.cycloid_center[1], 0));
  for (const g of Object.values(groups)) g.matrixWorldNeedsUpdate = true;

  const turns = theta / (2 * Math.PI);
  document.getElementById("readout").textContent =
    `input  ${turns.toFixed(2)} rev\noutput ${(turns / ratio).toFixed(3)} rev  (${ratio}:1)\ne = ${display(e)} ${units}`;
}

function tick() {
  const dt = Math.min(clock.getDelta(), 0.1);
  if (!paused) theta += Number(document.getElementById("speed").value) * dt;
  updateMotion();
  updateFlight(dt);
  controls.update();
  updateSection();
  renderer.render(scene, camera);
  requestAnimationFrame(tick);
}

// ---------- sidebar form ----------
const form = document.getElementById("dims");
const statusEl = document.getElementById("status");
const errorEl = document.getElementById("error");
let schema = null;

function setStatus(text, cls) {
  statusEl.textContent = text;
  statusEl.className = `status ${cls || ""}`;
}

// ---------- units ----------
// The model is in mm. Each length input keeps its exact mm value in data-mm and only its
// display changes with the unit, so toggling back and forth never accumulates rounding.
const MM_PER = { mm: 1, in: 25.4 };
let units = "mm";
try { if (localStorage.getItem("pco-units") === "in") units = "in"; } catch {}

function display(mm) {
  const v = mm / MM_PER[units];
  return String(units === "in" ? +v.toFixed(4) : +v.toFixed(3));
}

function showLength(input) {
  input.value = display(Number(input.dataset.mm));
  input.step = String(Number(input.dataset.stepMm) / (units === "in" ? 20 : 1));
}

function setUnits(u) {
  units = u;
  try { localStorage.setItem("pco-units", u); } catch {}
  document.querySelectorAll("input.len").forEach(showLength);
  document.querySelectorAll(".unit-name").forEach((el) => (el.textContent = u));
  for (const b of document.querySelectorAll("[data-units]")) {
    b.classList.toggle("active", b.dataset.units === u);
    b.setAttribute("aria-pressed", b.dataset.units === u);
  }
}

// typing into a length updates its mm value
document.addEventListener("input", (ev) => {
  const input = ev.target;
  if (!input.classList?.contains("len")) return;
  const v = Number.parseFloat(input.value);
  if (Number.isFinite(v)) input.dataset.mm = String(v * MM_PER[units]);
}, true);

function readLength(input) {
  const raw = input.value.trim();
  const ok = raw !== "" && Number.isFinite(Number.parseFloat(raw));
  return ok ? Number(input.dataset.mm) : NaN;
}

function field(path, value, type) {
  const label = document.createElement("label");
  label.className = "field";
  const span = document.createElement("span");
  span.textContent = path.split(".").pop();
  span.title = path;
  const input = document.createElement("input");
  input.type = "number";
  input.dataset.path = path;
  if (type === "float") {        // every float is a length
    input.classList.add("len");
    input.dataset.mm = String(value);
    input.dataset.stepMm = "0.1";
    showLength(input);
  } else {
    input.step = "1";
    input.value = value;
  }
  input.dataset.type = type;
  label.append(span, input);
  return label;
}

function node(path, value, type) {
  if (typeof type === "object") {
    const fs = document.createElement("fieldset");
    const legend = document.createElement("legend");
    legend.textContent = path.split(".").pop();
    fs.append(legend);
    for (const k of Object.keys(type)) fs.append(node(`${path}.${k}`, value[k], type[k]));
    return fs;
  }
  return field(path, value, type);
}

// remembers the selected tab at each level of the Manual Configuration layout, across re-renders
const activeTab = {};

function tabStrip(key, names, level, onPick) {
  const strip = document.createElement("div");
  strip.className = `subtabs level-${level}`;
  strip.setAttribute("role", "tablist");
  for (const n of names) {
    const b = document.createElement("button");
    b.type = "button";
    b.setAttribute("role", "tab");
    b.textContent = n;
    b.addEventListener("click", () => { activeTab[key] = n; onPick(); });
    strip.append(b);
  }
  return strip;
}

function renderTabs(key, tabs, values, level) {
  const wrap = document.createElement("div");
  const names = Object.keys(tabs);
  if (!names.includes(activeTab[key])) activeTab[key] = names[0];
  const panels = {};
  const sync = () => {
    for (const [n, p] of Object.entries(panels)) p.hidden = n !== activeTab[key];
    strip.querySelectorAll("button").forEach((b) => {
      const on = b.textContent === activeTab[key];
      b.classList.toggle("active", on);
      b.setAttribute("aria-selected", on);
    });
  };
  const strip = tabStrip(key, names, level, sync);
  wrap.append(strip);
  for (const [n, content] of Object.entries(tabs)) {
    const panel = document.createElement("div");
    panel.className = "subpanel";
    if (Array.isArray(content)) {
      for (const k of content) panel.append(node(k, values[k], schema.types[k]));
    } else {
      panel.append(renderTabs(`${key}/${n}`, content, values, level + 1));
    }
    panels[n] = panel;
    wrap.append(panel);
  }
  sync();
  return wrap;
}

function renderForm(values) {
  form.replaceChildren();
  const pinned = document.createElement("section");
  pinned.className = "pinned";
  for (const k of schema.layout.pinned) {
    const el = node(k, values[k], schema.types[k]);
    el.querySelector("span").textContent = schema.labels[k] ?? k;
    pinned.append(el);
  }
  form.append(pinned, renderTabs("design", schema.layout.tabs, values, 1));
  // friendlier labels wherever a labelled argument appears
  for (const [k, label] of Object.entries(schema.labels)) {
    const input = form.querySelector(`input[data-path="${k}"]`);
    if (input) input.previousElementSibling.textContent = label;
  }
}

function readForm() {
  const out = {};
  let ok = true;
  for (const input of form.querySelectorAll("input[data-path]")) {
    const raw = input.value.trim();
    const v = input.dataset.type === "int" ? Number.parseInt(raw, 10) : readLength(input);
    const bad = raw === "" || !Number.isFinite(v);
    input.classList.toggle("invalid", bad);
    ok &&= !bad;
    const keys = input.dataset.path.split(".");
    let o = out;
    for (const k of keys.slice(0, -1)) o = o[k] ??= {};
    o[keys.at(-1)] = v;
  }
  return ok ? out : null;
}

// ---------- builds ----------
let seq = 0;
let applied = 0;
let timer = null;

async function rebuild() {
  const kwargs = readForm();
  if (!kwargs) return setStatus("fix highlighted fields", "bad");
  const mySeq = ++seq;
  setStatus("rebuilding…", "busy");
  const t0 = performance.now();
  try {
    const res = await fetch("/api/build", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ seq: mySeq, kwargs }),
    });
    const body = await res.json();
    if (mySeq < applied || mySeq !== seq) return;   // a newer request is in flight or done
    applied = mySeq;
    if (!res.ok) {
      errorEl.textContent = body.error || `build failed (${res.status})`;
      errorEl.hidden = false;
      return setStatus("invalid, showing last good model", "bad");
    }
    errorEl.hidden = true;
    loadModel(body);
    const n = Object.values(body.instances).reduce((a, l) => a + l.length, 0);
    setStatus(`ready · ${((performance.now() - t0) / 1000).toFixed(1)} s · ${n} parts`, "ok");
  } catch (err) {
    if (mySeq === seq) setStatus(`server unreachable: ${err.message}`, "bad");
  }
}

form.addEventListener("input", () => {
  clearTimeout(timer);
  timer = setTimeout(rebuild, DEBOUNCE_MS);
});
document.getElementById("pause").addEventListener("click", (ev) => {
  paused = !paused;
  ev.target.textContent = paused ? "Play" : "Pause";
});
document.getElementById("reset").addEventListener("click", () => {
  renderForm(schema.values);
  rebuild();
});
document.getElementById("hardware").addEventListener("change", applyVisibility);
document.getElementById("view-menu").addEventListener("click", () => document.body.classList.toggle("view-open"));
document.getElementById("menu").addEventListener("click", () => document.body.classList.toggle("menu-open"));
const sectionToggle = document.getElementById("section");
const syncSectionRow = () => (document.getElementById("section-angle-row").hidden = !sectionToggle.checked);
sectionToggle.addEventListener("change", syncSectionRow);
syncSectionRow();

for (const tab of document.querySelectorAll(".tab")) {
  tab.addEventListener("click", () => {
    for (const t of document.querySelectorAll(".tab")) {
      const on = t === tab;
      t.classList.toggle("active", on);
      t.setAttribute("aria-selected", on);
      document.getElementById(`tab-${t.dataset.tab}`).hidden = !on;
    }
  });
}

// ---------- export tab ----------
const tolEnabled = document.getElementById("tol-enabled");
const tolFields = document.getElementById("tol-fields");
const exportStatus = document.getElementById("export-status");

function renderExport() {
  const { magnitudes } = schema.tolerancing;
  const mags = document.getElementById("tol-magnitudes");
  mags.replaceChildren();
  const labels = { bearing: "bearing", threaded: "bolt threaded (tight)", loose: "bolt loose",
                   cycloid_clearance: "cycloid clearance" };
  for (const [k, v] of Object.entries(magnitudes)) {
    const row = field(k, v, "float");
    row.querySelector("span").textContent = labels[k] ?? k;
    const input = row.querySelector("input");
    input.dataset.stepMm = "0.01";
    showLength(input);
    input.min = "0";
    input.dataset.mag = k;
    delete input.dataset.path;      // keep it out of the dimensions form
    mags.append(row);
  }
  renderFits();
}

function renderFits() {
  const fits = document.getElementById("tol-fits");
  fits.replaceChildren();
  const byPart = new Map();
  for (const f of schema.tolerancing.features) {
    if (!byPart.has(f.part)) byPart.set(f.part, []);
    byPart.get(f.part).push(f);
  }
  for (const [part, feats] of byPart) {
    const group = document.createElement("div");
    group.className = "fit-group";
    const h = document.createElement("h3");
    h.textContent = part;
    group.append(h);
    for (const f of feats) {
      const row = document.createElement("label");
      row.className = "field fit-row";
      const span = document.createElement("span");
      span.textContent = f.label;
      span.title = f.id;
      const tag = document.createElement("span");
      tag.className = "tag";
      tag.textContent = f.kind === "bearing" ? `bearing ${f.shape}` : "bolt";
      span.append(tag);
      const sel = document.createElement("select");
      sel.dataset.fit = f.id;
      for (const o of ["tight", "loose"]) sel.append(new Option(o, o, false, o === f.default));
      row.append(span, sel);
      group.append(row);
    }
    fits.append(group);
  }
}

function readTolerancing() {
  const magnitudes = {};
  let ok = true;
  for (const input of document.querySelectorAll("input[data-mag]")) {
    const v = readLength(input);
    const bad = !Number.isFinite(v) || v < 0;
    input.classList.toggle("invalid", bad);
    ok &&= !bad;
    magnitudes[input.dataset.mag] = v;
  }
  const fits = {};
  for (const sel of document.querySelectorAll("select[data-fit]")) fits[sel.dataset.fit] = sel.value;
  return ok ? { enabled: tolEnabled.checked, magnitudes, fits } : null;
}

tolEnabled.addEventListener("change", () => (tolFields.disabled = !tolEnabled.checked));
document.getElementById("fits-reset").addEventListener("click", renderFits);

document.getElementById("export-btn").addEventListener("click", async (ev) => {
  const kwargs = readForm();
  const tolerancing = readTolerancing();
  if (!kwargs) return (exportStatus.textContent = "Fix the highlighted dimensions on the Manual Configuration tab first.");
  if (!tolerancing) return (exportStatus.textContent = "Tolerances must be numbers >= 0.");
  const btn = ev.currentTarget;
  btn.disabled = true;
  exportStatus.textContent = "building export…";
  try {
    const res = await fetch("/api/export", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        kwargs, tolerancing,
        format: document.getElementById("export-format").value,
        scope: document.getElementById("export-scope").value,
      }),
    });
    if (!res.ok) {
      exportStatus.textContent = `Export failed: ${(await res.json()).error}`;
      return;
    }
    const blob = await res.blob();
    const name = /filename="([^"]+)"/.exec(res.headers.get("Content-Disposition") || "")?.[1] || "gearbox.zip";
    const a = document.createElement("a");
    a.href = URL.createObjectURL(blob);
    a.download = name;
    a.click();
    setTimeout(() => URL.revokeObjectURL(a.href), 10000);
    exportStatus.textContent = `downloaded ${name} (${(blob.size / 1e6).toFixed(2)} MB)`;
  } catch (err) {
    exportStatus.textContent = `server unreachable: ${err.message}`;
  } finally {
    btn.disabled = false;
  }
});

if (new URLSearchParams(location.search).has("debug")) {
  window.gearboxDebug = {
    groups, cad, camera, sectionPlane, hiddenParts,
    pose(t) { theta = t; updateMotion(); },
  };
}

schema = await (await fetch("/api/defaults")).json();
renderForm(schema.values);
renderExport();
for (const b of document.querySelectorAll("[data-units]")) b.addEventListener("click", () => setUnits(b.dataset.units));
setUnits(units);
await rebuild();
tick();
