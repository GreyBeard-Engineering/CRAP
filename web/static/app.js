import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { RoomEnvironment } from "three/addons/environments/RoomEnvironment.js";


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
let cycloidStages = [];

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
// overlapping volumes of colliding parts, in the assembled (input angle 0) pose
const collisionGroup = new THREE.Group();
cad.add(collisionGroup);
const collisionMaterial = new THREE.MeshBasicMaterial({ color: 0xff2a2a, transparent: true, opacity: 0.9,
                                                        depthTest: false });
const TINT = new THREE.Color(1.0, 0.35, 0.35), WHITE = new THREE.Color(1, 1, 1);

// "roller_bearing_12" -> ["roller_bearing", 11]; single-instance labels are the part name
function splitLabel(label, parts) {
  if (parts[label]) return [label, 0];
  const i = label.lastIndexOf("_");
  return [label.slice(0, i), Number(label.slice(i + 1)) - 1];
}

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
    g.traverse((child) => {
      if (child !== g && !child.isGroup) {
        if (child.geometry) geoms.add(child.geometry);
        if (child.material && child.material !== edgeMaterial) child.material?.dispose();
      }
    });
    while (g.children.length > 0) {
      g.remove(g.children[0]);
    }
  }
  for (const geom of geoms) geom?.dispose();
  cycloidStages = [];
}

function loadModel(payload) {
  clearModel();
  motion = payload.motion;
  const numDisks = motion?.num_disks || (payload.instances["cycloid"] ? payload.instances["cycloid"].length : 1);
  cycloidStages = [];
  for (let k = 0; k < numDisks; k++) {
    const stage = new THREE.Group();
    stage.matrixAutoUpdate = false;
    groups.cycloid.add(stage);
    cycloidStages.push(stage);
  }

  // which instances collide (simple builds report collisions)
  const hits = {};
  for (const [a, b] of payload.solver?.collisions ?? []) {
    for (const label of [a, b]) {
      const [name, idx] = splitLabel(label, payload.parts);
      (hits[name] ??= new Set()).add(idx);
    }
  }
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

    if (part.group === "cycloid" && cycloidStages.length > 0) {
      const perStage = Math.max(1, Math.floor(mats.length / cycloidStages.length));
      for (let k = 0; k < cycloidStages.length; k++) {
        const stageMats = mats.slice(k * perStage, (k + 1) * perStage);
        if (stageMats.length === 0) continue;
        for (const [m, shadows] of [[mat, true], [capMat, false]]) {
          const mesh = new THREE.InstancedMesh(geom, m, stageMats.length);
          stageMats.forEach((x, i) => {
            mesh.setMatrixAt(i, x);
            const globalIdx = k * perStage + i;
            if (shadows && hits[name]) {
              mesh.setColorAt(i, hits[name].has(globalIdx) ? TINT : WHITE);
            }
          });
          mesh.castShadow = mesh.receiveShadow = shadows;
          mesh.userData = { name, custom, stock: part.stock };
          cycloidStages[k].add(mesh);
        }

        if (custom && stageMats.length === 1) {
          const edges = new THREE.LineSegments(new THREE.EdgesGeometry(geom, 25), edgeMaterial);
          edges.matrixAutoUpdate = false;
          edges.matrix.copy(stageMats[0]);
          edges.userData = { name, custom };
          cycloidStages[k].add(edges);
        }
      }
    } else {
      for (const [m, shadows] of [[mat, true], [capMat, false]]) {
        const mesh = new THREE.InstancedMesh(geom, m, mats.length);
        mats.forEach((x, i) => mesh.setMatrixAt(i, x));
        if (shadows && hits[name]) {             // tint the colliding instances
          mats.forEach((_, i) => mesh.setColorAt(i, hits[name].has(i) ? TINT : WHITE));
        }
        mesh.castShadow = mesh.receiveShadow = shadows;
        mesh.userData = { name, custom, stock: part.stock };
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
  }
  // the overlapping volumes themselves, drawn on top so they can't hide inside the parts
  for (const child of [...collisionGroup.children]) { collisionGroup.remove(child); child.geometry.dispose(); }
  for (const c of payload.collision_meshes ?? []) {
    const g = new THREE.BufferGeometry();
    g.setAttribute("position", new THREE.BufferAttribute(decode(c.verts, Float32Array), 3));
    g.setIndex(new THREE.BufferAttribute(decode(c.tris, Uint32Array), 1));
    const m = new THREE.Mesh(g, collisionMaterial);
    m.renderOrder = 10;
    collisionGroup.add(m);
  }
  if (collisionGroup.children.length) {     // collisions are found in the assembled pose: show it
    theta = 0;
    paused = true;
    document.getElementById("pause").textContent = "Play";
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
  const show = { screw: document.getElementById("show-screws").checked,
                 bearing: document.getElementById("show-bearings").checked };
  for (const g of Object.values(groups)) {
    g.traverse((c) => {
      if (c.userData && c.userData.name) {
        const { name, custom, stock } = c.userData;
        c.visible = custom ? !hiddenParts.has(name) : show[stock];
      }
    });
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
  const numStages = cycloidStages.length || 1;
  for (let k = 0; k < cycloidStages.length; k++) {
    const phi = (k * 2 * Math.PI) / numStages;
    const cx = e * Math.cos(phi);
    const cy = e * Math.sin(phi);
    cycloidStages[k].matrix
      .makeTranslation(e * Math.cos(theta + phi), e * Math.sin(theta + phi), 0)
      .multiply(m1.makeRotationZ(out))
      .multiply(m2.makeTranslation(-cx, -cy, 0));
    cycloidStages[k].matrixWorldNeedsUpdate = true;
  }
  groups.cycloid.matrix.identity();
  for (const g of Object.values(groups)) g.matrixWorldNeedsUpdate = true;
  collisionGroup.visible = theta === 0;     // the overlaps are only right in the assembled pose

  const turns = theta / (2 * Math.PI);
  document.getElementById("readout").textContent =
    `input  ${turns.toFixed(2)} rev\noutput ${(turns / ratio).toFixed(3)} rev  (${ratio}:1)\ne = ${e} mm`;
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

function field(path, value, type) {
  const label = document.createElement("label");
  label.className = "field";
  const span = document.createElement("span");
  span.textContent = path.split(".").pop();
  span.title = path;
  const input = document.createElement("input");
  if (type === "bool") {
    input.type = "checkbox";
    input.checked = !!value;
  } else {
    input.type = "number";
    input.step = type === "int" ? "1" : "0.1";
    input.value = value;
  }
  input.dataset.path = path;
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
  for (const [title, keys] of Object.entries(schema.layout.pinned)) {
    const box = document.createElement("section");
    box.className = "pinned";
    box.dataset.box = title;
    const h = document.createElement("h2");
    h.textContent = title;
    box.append(h);
    for (const k of keys) box.append(node(k, values[k], schema.types[k]));
    form.append(box);
  }
  form.append(renderTabs("design", schema.layout.tabs, values, 1));
  // friendlier labels wherever a labelled argument appears
  for (const [k, label] of Object.entries(schema.labels)) {
    const input = form.querySelector(`input[data-path="${k}"]`);
    if (input) input.previousElementSibling.textContent = label;
  }
  applyMode();
}

// ---------- simple / advanced ----------
let mode = "simple";
try { if (localStorage.getItem("pco-mode") === "advanced") mode = "advanced"; } catch {}

function applyMode() {
  const simple = mode === "simple";
  for (const b of document.querySelectorAll("[data-mode]")) {
    b.classList.toggle("active", b.dataset.mode === mode);
    b.setAttribute("aria-pressed", b.dataset.mode === mode);
  }
  // Parts tab only in Advanced
  const partsBtn = [...form.querySelectorAll(".subtabs.level-1 button")].find((b) => b.textContent === "Parts");
  if (partsBtn) {
    partsBtn.hidden = simple;
    if (simple && activeTab.design === "Parts") {
      [...form.querySelectorAll(".subtabs.level-1 button")].find((b) => b.textContent === "Bearings").click();
    }
  }
  // plate spacing is derived in Simple, so it isn't shown there
  const spacing = form.querySelector('[data-box="Plate Spacing"]');
  if (spacing) spacing.hidden = simple;
  document.getElementById("solver-report").hidden = !simple;
}

function setMode(m) {
  mode = m;
  try { localStorage.setItem("pco-mode", m); } catch {}
  applyMode();
  updateDirty();
}

// write a full set of values (e.g. the solver's derived dimensions) into the form
function setFormValues(values) {
  for (const input of form.querySelectorAll("input[data-path]")) {
    let v = values;
    for (const k of input.dataset.path.split(".")) v = v?.[k];
    if (v === undefined) continue;
    if (input.type === "checkbox") input.checked = !!v;
    else input.value = v;
  }
}

function simplePart(kwargs) {
  return Object.fromEntries(schema.simple_args.map((k) => [k, kwargs[k]]));
}

function readForm() {
  const out = {};
  let ok = true;
  for (const input of form.querySelectorAll("input[data-path]")) {
    const raw = input.value.trim();
    const isBool = input.dataset.type === "bool";
    const v = isBool ? input.checked
      : input.dataset.type === "int" ? Number.parseInt(raw, 10) : Number.parseFloat(raw);
    const bad = !isBool && (raw === "" || !Number.isFinite(v));
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
let built = null;         // full parameters of the model on screen
let lastSolver = null;    // solver report for the model on screen (simple builds)
let lastValid = null;     // {payload, values, solver} of the last build with no problems
let shownHasProblems = false;
const revertBtn = document.getElementById("revert");

function prettyPart(label, parts) {
  const [name] = splitLabel(label, parts);
  return name.replace(/_(M|D)\d[\d.]*x[\d.]+$/, "").replaceAll("_", " ");
}

// "cycloid × roller bearing (23)" style list of colliding part pairs
function showProblems(body) {
  const pairs = new Map();
  for (const [a, b] of body.solver?.collisions ?? []) {
    const key = [prettyPart(a, body.parts), prettyPart(b, body.parts)].sort().join(" × ");
    pairs.set(key, (pairs.get(key) ?? 0) + 1);
  }
  errorEl.replaceChildren();
  const list = (title, items) => {
    const head = document.createElement("div");
    head.textContent = title;
    const ul = document.createElement("ul");
    ul.className = "collision-list";
    for (const text of items) {
      const li = document.createElement("li");
      li.textContent = text;
      ul.append(li);
    }
    errorEl.append(head, ul);
  };
  if (pairs.size) list("Colliding parts (shown in red):", [...pairs].map(([k, n]) => (n > 1 ? `${k} (${n})` : k)));
  if (body.failed_parts?.length) list("Couldn't build:", body.failed_parts);
  errorEl.hidden = false;
}
const rebuildBtn = document.getElementById("rebuild");

// light up Rebuild whenever the form differs from what is displayed
function updateDirty() {
  const kwargs = readForm();
  const inputsOf = (kw) => JSON.stringify(mode === "simple" ? simplePart(kw) : kw);
  const dirty = !kwargs || !built || inputsOf(kwargs) !== inputsOf(built);
  rebuildBtn.classList.toggle("dirty", dirty);
  rebuildBtn.title = dirty ? "parameters changed: rebuild the model (Enter)" : "model is up to date";
  if (!dirty && !errorEl.hidden && !shownHasProblems) {   // back to the displayed model: a failed build's error is stale
    errorEl.hidden = true;
    setStatus("ready", "ok");
    if (lastSolver) showSolverReport(lastSolver);
  }
}

async function rebuild() {
  const kwargs = readForm();
  if (!kwargs) return setStatus("fix highlighted fields", "bad");
  const mySeq = ++seq;
  const buildMode = mode;
  setStatus(buildMode === "simple" ? "solving…" : "rebuilding…", "busy");
  const t0 = performance.now();
  // simple builds take a while (exact collision check): show what the solver is doing
  const poll = buildMode !== "simple" ? null : setInterval(async () => {
    try {
      const { stage } = await (await fetch("/api/progress")).json();
      if (mySeq === seq && stage !== "idle") setStatus(`${stage}…`, "busy");
    } catch {}
  }, 500);
  try {
    const res = await fetch("/api/build", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ seq: mySeq, mode: buildMode, kwargs }),
    });
    const body = await res.json();
    if (mySeq < applied || mySeq !== seq) return;   // a newer request is in flight or done
    applied = mySeq;
    if (!res.ok) {
      errorEl.textContent = body.error || `build failed (${res.status})`;
      errorEl.hidden = false;
      if (buildMode === "simple") showSolverReport(null, body.solver_log);
      return setStatus("invalid, showing last good model", "bad");
    }
    errorEl.hidden = true;
    loadModel(body);
    if (body.derived) {           // simple: fill every field with what the solver chose
      setFormValues(body.derived);
      showSolverReport(body.solver);
    }
    lastSolver = body.solver ?? null;
    built = readForm();
    const n = Object.values(body.instances).reduce((a, l) => a + l.length, 0);
    const secs = ((performance.now() - t0) / 1000).toFixed(1);
    shownHasProblems = body.valid === false;     // only collisions or unbuildable parts count
    if (shownHasProblems) {
      showProblems(body);
      setStatus(`built with problems · ${secs} s`, "bad");
    } else {
      lastValid = { payload: body, values: built, solver: lastSolver };
      setStatus(`ready · ${secs} s · ${n} parts`, "ok");
    }
    revertBtn.hidden = !(shownHasProblems && lastValid);
  } catch (err) {
    if (mySeq === seq) setStatus(`server unreachable: ${err.message}`, "bad");
  } finally {
    if (poll) clearInterval(poll);
    syncMotorFit();
    updateDirty();
  }
}

function showSolverReport(solver, failLog) {
  const el = document.getElementById("solver-report");
  if (!solver) {
    el.textContent = "";
    return;
  }
  const n = solver.collisions.length;
  el.textContent = `collision check (${solver.check_seconds} s): ` + (n ? `${n} colliding pairs` : "none");
  el.className = n ? "bad" : "";
}

form.addEventListener("input", () => { updateDirty(); syncMotorFit(); });
for (const b of document.querySelectorAll("[data-mode]")) b.addEventListener("click", () => setMode(b.dataset.mode));
form.addEventListener("keydown", (ev) => {   // Enter in any field rebuilds
  if (ev.key !== "Enter") return;
  ev.preventDefault();
  rebuild();
});
form.addEventListener("submit", (ev) => ev.preventDefault());
rebuildBtn.addEventListener("click", rebuild);
revertBtn.addEventListener("click", () => {
  if (!lastValid) return;
  loadModel(lastValid.payload);
  setFormValues(lastValid.values);
  built = lastValid.values;
  lastSolver = lastValid.solver;
  if (lastSolver) showSolverReport(lastSolver);
  shownHasProblems = false;
  errorEl.hidden = true;
  revertBtn.hidden = true;
  setStatus("reverted to the last valid design", "ok");
  syncMotorFit();
  updateDirty();
});
document.getElementById("pause").addEventListener("click", (ev) => {
  paused = !paused;
  ev.target.textContent = paused ? "Play" : "Pause";
});
document.getElementById("reset").addEventListener("click", () => {
  renderForm(schema.values);
  // simple mode: if the inputs now match the model on screen, show its derived values again
  const kw = readForm();
  if (mode === "simple" && built && kw &&
      JSON.stringify(simplePart(kw)) === JSON.stringify(simplePart(built))) setFormValues(built);
  syncMotorFit();
  updateDirty();
});
document.getElementById("show-screws").addEventListener("change", applyVisibility);
document.getElementById("show-bearings").addEventListener("change", applyVisibility);
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
    input.step = "0.01";
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
      if (f.id === "base.motor_holes") {     // follows the motor's face-mount setting
        sel.disabled = true;
        sel.title = "set by face mount on the Motor tab";
        tag.textContent = "set by face mount";
      }
      row.append(span, sel);
      group.append(row);
    }
    fits.append(group);
  }
}

function syncMotorFit() {
  const fm = form.querySelector('input[data-path="drive_motor.face_mount"]');
  const sel = document.querySelector('select[data-fit="base.motor_holes"]');
  if (fm && sel) sel.value = fm.checked ? "loose" : "tight";
}

function readTolerancing() {
  const magnitudes = {};
  let ok = true;
  for (const input of document.querySelectorAll("input[data-mag]")) {
    const v = Number.parseFloat(input.value);
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

// ---------- save / load configuration ----------
const cfgStatus = document.getElementById("cfg-status");

document.getElementById("cfg-save").addEventListener("click", () => {
  const values = readForm();
  const tol = readTolerancing();
  if (!values) return (cfgStatus.textContent = "Fix the highlighted dimensions first.");
  const cfg = {
    app: "CRAP", version: 1, mode, values,
    tolerancing: tol,
    export: { format: document.getElementById("export-format").value,
              scope: document.getElementById("export-scope").value },
  };
  const a = document.createElement("a");
  a.href = URL.createObjectURL(new Blob([JSON.stringify(cfg, null, 2)], { type: "application/json" }));
  const stamp = new Date().toISOString().slice(0, 16).replace(/[:T]/g, "-");
  a.download = `crap_config_${stamp}.json`;
  a.click();
  setTimeout(() => URL.revokeObjectURL(a.href), 10000);
  cfgStatus.textContent = `saved ${a.download}`;
});

const cfgFile = document.getElementById("cfg-file");
document.getElementById("cfg-load").addEventListener("click", () => cfgFile.click());
cfgFile.addEventListener("change", async () => {
  const file = cfgFile.files[0];
  cfgFile.value = "";
  if (!file) return;
  let cfg;
  try {
    cfg = JSON.parse(await file.text());
  } catch (err) {
    return (cfgStatus.textContent = `Couldn't read ${file.name}: ${err.message}`);
  }
  if (!cfg || typeof cfg.values !== "object") {
    return (cfgStatus.textContent = `${file.name} isn't a CRAP configuration (no "values").`);
  }
  // start from the defaults so anything the file leaves out is sensible, then apply it
  renderForm(schema.values);
  setFormValues(cfg.values);
  if (cfg.mode === "simple" || cfg.mode === "advanced") setMode(cfg.mode);
  const t = cfg.tolerancing;
  if (t) {
    if (typeof t.enabled === "boolean") {
      tolEnabled.checked = t.enabled;
      tolFields.disabled = !t.enabled;
    }
    for (const input of document.querySelectorAll("input[data-mag]")) {
      if (t.magnitudes?.[input.dataset.mag] !== undefined) input.value = t.magnitudes[input.dataset.mag];
    }
    for (const sel of document.querySelectorAll("select[data-fit]")) {
      if (t.fits?.[sel.dataset.fit]) sel.value = t.fits[sel.dataset.fit];
    }
  }
  if (cfg.export?.format) document.getElementById("export-format").value = cfg.export.format;
  if (cfg.export?.scope) document.getElementById("export-scope").value = cfg.export.scope;
  syncMotorFit();
  updateDirty();
  cfgStatus.textContent = `loaded ${file.name}; press Rebuild to see it`;
});
document.getElementById("fits-reset").addEventListener("click", () => { renderFits(); syncMotorFit(); });

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
        mode, kwargs, tolerancing,
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
syncMotorFit();
await rebuild();
tick();
