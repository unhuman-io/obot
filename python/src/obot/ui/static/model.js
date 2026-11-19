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

export async function show(container, query, simCameras = []) {
  const model = await (await fetch("/api/model" + query)).json();
  const scene = new THREE.Scene();
  const camera = new THREE.PerspectiveCamera(40, 1, 0.01, 50);
  camera.up.set(0, 0, 1);                       // URDF: z up, x forward
  camera.position.set(1.3, -1.3, 0.9);
  const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
  container.replaceChildren(renderer.domElement);
  const controls = new OrbitControls(camera, renderer.domElement);
  controls.target.set(0, 0, 0.25);
  scene.add(new THREE.HemisphereLight(0xffffff, 0x445566, 2.2));
  const sun = new THREE.DirectionalLight(0xffffff, 1.5);
  sun.position.set(2, -1, 3);
  scene.add(sun);
  const grid = new THREE.GridHelper(4, 16, 0x8899aa, 0x8899aa);
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
  scene.add(base);

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
    setJoints(names, q, odom) {
      names.forEach((name, i) => {
        const joint = moving[name];
        if (joint && q[i] !== null) joint.group.setRotationFromAxisAngle(joint.axis, q[i]);
      });
      if (odom) {
        // The view travels with the robot.
        camera.position.x += odom[0] - base.position.x;
        camera.position.y += odom[1] - base.position.y;
        base.position.set(odom[0], odom[1], 0);
        base.rotation.z = odom[2];
        controls.target.set(odom[0], odom[1], 0.25);
      }
    },
  };
}
