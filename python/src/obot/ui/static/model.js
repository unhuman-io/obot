// The robot drawn from its URDF and posed with the measured joint angles.
import * as THREE from "three";
import { STLLoader } from "/static/vendor/STLLoader.js";
import { OrbitControls } from "/static/vendor/OrbitControls.js";

function frame(origin) {
  const group = new THREE.Group();
  group.position.set(...origin.xyz);
  // URDF rpy is roll about x, then pitch about y, then yaw about z, in the fixed frame.
  group.rotation.set(origin.rpy[0], origin.rpy[1], origin.rpy[2], "ZYX");
  return group;
}

// Something to look at for the simulated cameras: a floor and a few household-sized props.
// They are on layer 1, which only the simulated cameras see, so the overview stays clear.
function addProps(scene) {
  const prop = (geometry, color, x, y, z) => {
    const mesh = new THREE.Mesh(geometry, new THREE.MeshStandardMaterial({ color, roughness: 0.8 }));
    mesh.position.set(x, y, z);
    mesh.layers.set(1);
    scene.add(mesh);
    return mesh;
  };
  prop(new THREE.PlaneGeometry(12, 12), 0xb9b2a6, 0, 0, 0);                          // floor
  prop(new THREE.BoxGeometry(0.4, 0.6, 0.45), 0x8a6f52, 1.6, 0.9, 0.225);            // a box
  prop(new THREE.CylinderGeometry(0.15, 0.18, 0.4, 24), 0x3f7fbf, 1.3, -0.8, 0.2).rotation.x = Math.PI / 2;  // a basket
  prop(new THREE.SphereGeometry(0.12, 24, 12), 0xc94f4f, 0.9, 0.25, 0.04).scale.set(1.4, 1, 0.35);   // a shirt on the floor
  prop(new THREE.SphereGeometry(0.1, 24, 12), 0x4faf6a, 1.1, -0.3, 0.03).scale.set(1, 1.5, 0.3);     // a sock
  prop(new THREE.BoxGeometry(0.1, 6, 1.5), 0xd8d4cc, 4.5, 0, 0.75);                  // a wall
}

// Distance along the viewing direction as colour: near is warm, far is cool, like the real
// depth stream is drawn; beyond the range is black.
const depthMaterial = new THREE.ShaderMaterial({
  uniforms: { range: { value: 4.0 } },
  vertexShader: "varying float d; void main() { vec4 p = modelViewMatrix * vec4(position, 1.0); d = -p.z; gl_Position = projectionMatrix * p; }",
  fragmentShader: `uniform float range; varying float d;
    void main() {
      float t = clamp(d / range, 0.0, 1.0);
      vec3 c = clamp(vec3(1.5 - abs(4.0 * t - 1.0), 1.5 - abs(4.0 * t - 2.0), 1.5 - abs(4.0 * t - 3.0)), 0.0, 1.0);
      gl_FragColor = vec4(d > range ? vec3(0.0) : c, 1.0);
    }`,
});

// A view from a URDF link whose x axis is the viewing direction and z is up, as a colour
// or a depth picture (range 0.1 .. 4 m).
function addSimCamera(scene, link, cam) {
  const depth = cam.mode === "depth";
  // RealSense D435 vertical fields of view.
  const camera = new THREE.PerspectiveCamera(depth ? 58 : 42, 4 / 3, depth ? 0.1 : 0.02, depth ? 4 : 30);
  camera.quaternion.setFromRotationMatrix(new THREE.Matrix4().makeBasis(
    new THREE.Vector3(0, -1, 0), new THREE.Vector3(0, 0, 1), new THREE.Vector3(-1, 0, 0)));
  camera.layers.enable(1);
  link.add(camera);
  const renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setSize(480, 360, false);
  cam.element.replaceChildren(renderer.domElement);
  const background = new THREE.Color(depth ? 0x000000 : 0xcfd8e3);
  return () => {
    const savedBackground = scene.background;
    scene.background = background;
    scene.overrideMaterial = depth ? depthMaterial : null;
    renderer.render(scene, camera);
    scene.overrideMaterial = null;
    scene.background = savedBackground;
  };
}

// A stand-in for robots without a URDF: a disc with a nose that shows which way is forward.
function placeholderRobot() {
  const group = new THREE.Group();
  const body = new THREE.Mesh(new THREE.CylinderGeometry(0.17, 0.17, 0.1, 32),
                              new THREE.MeshStandardMaterial({ color: 0x2456c9, roughness: 0.6 }));
  body.rotation.x = Math.PI / 2;
  body.position.z = 0.1;
  const nose = new THREE.Mesh(new THREE.ConeGeometry(0.06, 0.16, 16),
                              new THREE.MeshStandardMaterial({ color: 0xe0a040, roughness: 0.6 }));
  nose.rotation.z = -Math.PI / 2;               // a cone points along y; forward is x
  nose.position.set(0.2, 0, 0.1);
  group.add(body, nose);
  return group;
}

// Layer 1 is what only the simulated cameras see (their props); layer 2 is what only the
// overview sees (the map, which is not something a camera on the robot would look at).
const MAP_LAYER = 2;

const SRGB_TO_LINEAR = Float32Array.from({ length: 256 }, (_, i) => {
  const c = i / 255;
  return c <= 0.04045 ? c / 12.92 : Math.pow((c + 0.055) / 1.055, 2.4);
});

// The map of the surroundings that the robot builds from its depth cameras: one point per
// map cell, kept up to date by asking for what changed a few times per second. Also where
// the robot has been, and where each depth camera is looking.
function addWorld(scene, query, frustums, fovDegrees) {
  let capacity = 1 << 16, count = 0;
  let positions = new Float32Array(capacity * 3);
  let colors = new Float32Array(capacity * 3);
  let rgb = new Uint8Array(capacity * 3);       // as sent: 0, 0, 0 = no colour known
  let keys = new Float64Array(capacity);        // slot -> cell
  const slots = new Map();                      // cell -> slot
  const geometry = new THREE.BufferGeometry();
  const material = new THREE.PointsMaterial({ size: 0.1, vertexColors: true });
  const points = new THREE.Points(geometry, material);
  points.frustumCulled = false;
  points.layers.set(MAP_LAYER);
  scene.add(points);
  function attach() {
    geometry.setAttribute("position", new THREE.BufferAttribute(positions, 3));
    geometry.setAttribute("color", new THREE.BufferAttribute(colors, 3));
  }
  attach();
  geometry.setDrawRange(0, 0);
  function grow() {
    capacity *= 2;
    const bigger = (old, make) => { const next = make(); next.set(old); return next; };
    positions = bigger(positions, () => new Float32Array(capacity * 3));
    colors = bigger(colors, () => new Float32Array(capacity * 3));
    rgb = bigger(rgb, () => new Uint8Array(capacity * 3));
    keys = bigger(keys, () => new Float64Array(capacity));
    attach();
  }
  let byHeight = false;
  function paint(slot) {
    const at = slot * 3;
    const t = Math.max(0, Math.min(1, positions[at + 2] / 1.5));
    if (byHeight) {
      // Height above the floor: blue at the floor, red at 1.5 m.
      const ramp = (centre) => Math.pow(Math.max(0, Math.min(1, 1.5 - Math.abs(4 * t - centre))), 2.2);
      colors[at] = ramp(3); colors[at + 1] = ramp(2); colors[at + 2] = ramp(1);
    } else if ((rgb[at] | rgb[at + 1] | rgb[at + 2]) === 0) {
      // Seen by the depth camera only: grey, lighter the higher it is.
      const grey = 0.12 + 0.5 * t;
      colors[at] = grey * 0.9; colors[at + 1] = grey; colors[at + 2] = grey * 1.15;
    } else {
      colors[at] = SRGB_TO_LINEAR[rgb[at]]; colors[at + 1] = SRGB_TO_LINEAR[rgb[at + 1]]; colors[at + 2] = SRGB_TO_LINEAR[rgb[at + 2]];
    }
  }
  const cell = (ix, iy, iz) => (ix + 32768) * 4294967296 + (iy + 32768) * 65536 + (iz + 32768);

  const TRAIL_HEIGHT = 0.06;                     // m
  const trail = [];
  const trailLine = new THREE.Line(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({ color: 0xe0a040 }));
  trailLine.frustumCulled = false;
  trailLine.layers.set(MAP_LAYER);
  scene.add(trailLine);

  const cameras = {};
  for (const f of frustums) {
    const d = 0.35, x = f.tan_x * d, y = f.tan_y * d;
    const corners = [[x, y, d], [x, -y, d], [-x, -y, d], [-x, y, d]];
    const lines = [];
    corners.forEach((corner, i) => lines.push(0, 0, 0, ...corner, ...corner, ...corners[(i + 1) % 4]));
    const shape = new THREE.BufferGeometry();
    shape.setAttribute("position", new THREE.Float32BufferAttribute(lines, 3));
    const object = new THREE.LineSegments(shape, new THREE.LineBasicMaterial({ color: 0x2ea8a0 }));
    object.matrixAutoUpdate = false;
    object.frustumCulled = false;
    object.layers.set(MAP_LAYER);
    scene.add(object);
    cameras[f.name] = object;
  }

  let voxel = 0.03, epoch = 0, rev = 0, trailCount = 0, busy = false, failed = 0;
  async function poll() {
    if (busy || document.hidden) return;
    busy = true;
    try {
      const response = await fetch(`/api/world${query ? query + "&" : "?"}since=${rev}&epoch=${epoch}&trail=${trailCount}`);
      if (!response.ok) throw new Error(response.status);
      const buffer = await response.arrayBuffer();
      const head = new DataView(buffer);
      if (head.getUint32(0, true) !== 0x3157424f) throw new Error("not a map update");
      const flags = head.getUint32(12, true), changed = head.getUint32(16, true), removed = head.getUint32(20, true);
      const trailStart = head.getUint32(24, true), trailNew = head.getUint32(28, true);
      epoch = head.getUint32(4, true);
      rev = head.getUint32(8, true);
      voxel = head.getFloat32(32, true);
      let at = 36;
      const path = new Float32Array(buffer, at, trailNew * 2); at += trailNew * 8;
      const index = new Int16Array(buffer, at, changed * 3); at += changed * 6;
      const gone = new Int16Array(buffer, at, removed * 3); at += removed * 6;
      const colour = new Uint8Array(buffer, at, changed * 3);
      if (flags & 1) { slots.clear(); count = 0; trail.length = 0; }
      for (let i = 0; i < removed; i++) {
        const key = cell(gone[i * 3], gone[i * 3 + 1], gone[i * 3 + 2]);
        const slot = slots.get(key);
        if (slot === undefined) continue;
        slots.delete(key);
        const last = --count;                    // the last point moves into the gap
        if (slot !== last) {
          for (let c = 0; c < 3; c++) {
            positions[slot * 3 + c] = positions[last * 3 + c];
            colors[slot * 3 + c] = colors[last * 3 + c];
            rgb[slot * 3 + c] = rgb[last * 3 + c];
          }
          keys[slot] = keys[last];
          slots.set(keys[slot], slot);
        }
      }
      for (let i = 0; i < changed; i++) {
        const key = cell(index[i * 3], index[i * 3 + 1], index[i * 3 + 2]);
        let slot = slots.get(key);
        if (slot === undefined) {
          if (count === capacity) grow();
          slot = count++;
          slots.set(key, slot);
          keys[slot] = key;
          for (let c = 0; c < 3; c++) positions[slot * 3 + c] = index[i * 3 + c] * voxel;
        }
        for (let c = 0; c < 3; c++) rgb[slot * 3 + c] = colour[i * 3 + c];
        paint(slot);
      }
      if (changed || removed || (flags & 1)) {
        geometry.attributes.position.needsUpdate = true;
        geometry.attributes.color.needsUpdate = true;
        geometry.setDrawRange(0, count);
        // Point size is in pixels at one unit of distance times half the picture height;
        // this makes a point cover a little more than its cell.
        material.size = voxel * 1.3 / Math.tan(fovDegrees * Math.PI / 360);
      }
      if (trailNew || (flags & 1)) {
        // drawn a little above the floor: the map's floor cells would cover it otherwise
        for (let i = 0; i < trailNew; i++) trail.push(path[i * 2], path[i * 2 + 1], TRAIL_HEIGHT);
        if (trail.length > 60000) trail.splice(0, trail.length - 60000);
        trailLine.geometry.setAttribute("position", new THREE.Float32BufferAttribute(trail, 3));
      }
      trailCount = trailStart + trailNew;
      failed = 0;
    } catch (e) {
      failed++;
    } finally {
      busy = false;
    }
  }
  setInterval(poll, 300);
  poll();

  return {
    get count() { return count; },
    get failed() { return failed; },
    get trail() { return trail.length / 3; },
    get byHeight() { return byHeight; },
    colourByHeight(on) {
      byHeight = on;
      for (let slot = 0; slot < count; slot++) paint(slot);
      geometry.attributes.color.needsUpdate = true;
    },
    // The first point within `tolerance` of a place, as [x, y, z, r, g, b] (for tests).
    near(x, y, z, tolerance) {
      for (let slot = 0; slot < count; slot++) {
        const at = slot * 3;
        if (Math.abs(positions[at] - x) < tolerance && Math.abs(positions[at + 1] - y) < tolerance &&
            Math.abs(positions[at + 2] - z) < tolerance) {
          return [positions[at], positions[at + 1], positions[at + 2], rgb[at], rgb[at + 1], rgb[at + 2]];
        }
      }
      return null;
    },
    setCameras(poses) {
      for (const [name, m] of Object.entries(poses || {})) {
        if (cameras[name]) cameras[name].matrix.set(m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8], m[9], m[10], m[11], 0, 0, 0, 1);
      }
    },
  };
}

export async function show(container, query, simCameras = [], options = { model: true }) {
  const model = options.model ? await (await fetch("/api/model" + query)).json() : { links: {}, joints: [], root: null };
  const scene = new THREE.Scene();
  const camera = new THREE.PerspectiveCamera(40, 1, 0.01, 50);
  camera.up.set(0, 0, 1);                       // URDF: z up, x forward
  camera.layers.enable(MAP_LAYER);
  const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
  container.replaceChildren(renderer.domElement);
  const controls = new OrbitControls(camera, renderer.domElement);
  if (options.world) {                          // from behind and above: what is ahead is in view
    camera.position.set(-1.9, -1.3, 1.7);
    controls.target.set(0.6, 0, 0.2);
  } else {
    camera.position.set(1.3, -1.3, 0.9);
    controls.target.set(0, 0, 0.25);
  }
  scene.add(new THREE.HemisphereLight(0xffffff, 0x445566, 2.2));
  const sun = new THREE.DirectionalLight(0xffffff, 1.5);
  sun.position.set(2, -1, 3);
  scene.add(sun);
  const grid = options.world ? new THREE.GridHelper(20, 40, 0x8899aa, 0x8899aa)
                             : new THREE.GridHelper(4, 16, 0x8899aa, 0x8899aa);
  grid.rotation.x = Math.PI / 2;
  grid.material.opacity = 0.35;
  grid.material.transparent = true;
  scene.add(grid);

  const links = {};
  const loader = new STLLoader();
  for (const [name, visuals] of Object.entries(model.links)) {
    links[name] = new THREE.Group();
    for (const visual of visuals) {
      const holder = frame(visual);
      holder.scale.set(...visual.scale);
      links[name].add(holder);
      loader.load("/model/" + visual.mesh + query, (geometry) => {
        geometry.computeVertexNormals();
        const [r, g, b] = visual.color;
        const material = new THREE.MeshStandardMaterial({ color: new THREE.Color(r, g, b), metalness: 0.2, roughness: 0.6 });
        holder.add(new THREE.Mesh(geometry, material));
      });
    }
  }
  const moving = {};                             // joint name -> { group, axis }
  for (const joint of model.joints) {
    const origin = frame(joint);
    const turn = new THREE.Group();
    origin.add(turn);
    turn.add(links[joint.child]);
    links[joint.parent].add(origin);
    if (joint.type !== "fixed") moving[joint.name] = { group: turn, axis: new THREE.Vector3(...joint.axis).normalize() };
  }
  const base = new THREE.Group();                // moves with the odometry
  if (model.root) base.add(links[model.root]);
  else base.add(placeholderRobot());
  scene.add(base);
  const world = options.world ? addWorld(scene, query, options.world, camera.fov) : null;

  function resize() {
    const width = container.clientWidth, height = container.clientHeight;
    renderer.setSize(width, height, false);
    renderer.domElement.style.width = width + "px";
    renderer.domElement.style.height = height + "px";
    camera.aspect = width / height;
    camera.updateProjectionMatrix();
  }
  new ResizeObserver(resize).observe(container);
  resize();
  const views = [];
  if (simCameras.length) addProps(scene);
  for (const cam of simCameras) {
    if (links[cam.link] && cam.element) views.push(addSimCamera(scene, links[cam.link], cam));
  }
  // A click on the floor (not a drag, which turns the view), for whoever asked for it.
  let floorClick = null, pressed = null;
  const ray = new THREE.Raycaster(), floor = new THREE.Plane(new THREE.Vector3(0, 0, 1), 0);
  renderer.domElement.addEventListener("pointerdown", (e) => { pressed = [e.clientX, e.clientY]; });
  renderer.domElement.addEventListener("pointerup", (e) => {
    const moved = pressed ? Math.hypot(e.clientX - pressed[0], e.clientY - pressed[1]) : 99;
    pressed = null;
    if (!floorClick || moved > 4) return;
    const box = renderer.domElement.getBoundingClientRect();
    ray.setFromCamera(new THREE.Vector2((e.clientX - box.left) / box.width * 2 - 1,
                                        -((e.clientY - box.top) / box.height) * 2 + 1), camera);
    const hit = ray.ray.intersectPlane(floor, new THREE.Vector3());
    if (hit) floorClick(hit.x, hit.y);
  });
  // Where the robot is driving to by itself, and the way it means to take.
  const goalMark = new THREE.Mesh(new THREE.RingGeometry(0.08, 0.12, 32),
                                  new THREE.MeshBasicMaterial({ color: 0x2f9e44, side: THREE.DoubleSide }));
  goalMark.position.z = 0.07;
  goalMark.visible = false;
  const wayLine = new THREE.Line(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({ color: 0x2f9e44 }));
  wayLine.frustumCulled = false;
  scene.add(goalMark, wayLine);

  let tick = 0;
  renderer.setAnimationLoop(() => {
    controls.update();
    renderer.render(scene, camera);
    if (tick++ % 2 === 0) views.forEach((draw) => draw());   // camera views at half the display rate
  });

  return {
    links: Object.keys(links).length,
    views: views.length,
    joints: Object.keys(moving),
    world,
    onFloorClick(callback) {
      floorClick = callback;
      renderer.domElement.style.cursor = callback ? "crosshair" : "";
    },
    setNav(nav) {
      const going = nav && nav.goal && ["planning", "driving", "looking"].includes(nav.state);
      goalMark.visible = !!going;
      wayLine.visible = !!going;
      if (!going) return;
      goalMark.position.set(nav.goal[0], nav.goal[1], 0.07);
      const way = [base.position.x, base.position.y, 0.07];
      for (const [x, y] of nav.path) way.push(x, y, 0.07);
      wayLine.geometry.setAttribute("position", new THREE.Float32BufferAttribute(way, 3));
    },
    setJoints(names, q, odom) {
      names.forEach((name, i) => {
        const joint = moving[name];
        if (joint && q[i] !== null) joint.group.setRotationFromAxisAngle(joint.axis, q[i]);
      });
      if (odom) {
        // The view travels with the robot.
        const dx = odom[0] - base.position.x, dy = odom[1] - base.position.y;
        camera.position.x += dx;
        camera.position.y += dy;
        controls.target.x += dx;
        controls.target.y += dy;
        base.position.set(odom[0], odom[1], 0);
        base.rotation.z = odom[2];
        grid.position.set(Math.round(odom[0] * 2) / 2, Math.round(odom[1] * 2) / 2, 0);
      }
    },
  };
}
