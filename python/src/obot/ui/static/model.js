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

export async function show(container, query) {
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
  renderer.setAnimationLoop(() => { controls.update(); renderer.render(scene, camera); });

  return {
    links: Object.keys(links).length,
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
