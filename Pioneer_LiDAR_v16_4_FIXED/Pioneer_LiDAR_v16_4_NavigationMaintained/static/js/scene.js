import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { state } from './state.js';

export function initScene() {
  state.scene = new THREE.Scene();
  state.scene.background = new THREE.Color(0x0a0a0f);

  state.camera = new THREE.PerspectiveCamera(
    60,
    window.innerWidth / window.innerHeight,
    0.1,
    1000
  );
  state.camera.position.set(5, 8, 8);
  state.camera.lookAt(0, 0, 0);

  state.renderer = new THREE.WebGLRenderer({
    antialias: true,
    powerPreference: 'high-performance'
  });
  state.renderer.setSize(window.innerWidth, window.innerHeight);
  state.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  document.body.appendChild(state.renderer.domElement);

  state.controls = new OrbitControls(
    state.camera,
    state.renderer.domElement
  );
  state.controls.enableDamping = true;
  state.controls.dampingFactor = 0.08;

  state.scene.add(new THREE.AmbientLight(0x404040, 1.5));

  const dirLight = new THREE.DirectionalLight(0xffffff, 0.8);
  dirLight.position.set(10, 20, 10);
  state.scene.add(dirLight);

  /*
   * Everything that represents the current travelling area lives under one
   * transform. The camera never moves when the area is dragged.
   */
  state.areaGroup = new THREE.Group();
  state.areaGroup.position.set(0, 0, 0);
  state.scene.add(state.areaGroup);

  console.log('[AREA DEBUG] scene: areaGroup created', state.areaGroup);


  // The travelling-area reference is a world-space object. The point cloud
  // itself stays in world coordinates so moving the area does not cancel out
  // the spatial movement.
  state.grid = new THREE.GridHelper(80, 80, 0x22222e, 0x15151c);
  state.grid.name = 'TravellingAreaGrid';
  state.areaGroup.add(state.grid);

  state.groundPlane = new THREE.Mesh(
    new THREE.PlaneGeometry(80, 80),
    new THREE.MeshBasicMaterial({
      color: 0x0c0c12,
      side: THREE.DoubleSide,
      transparent: true,
      opacity: 0.2
    })
  );
  state.groundPlane.rotation.x = -Math.PI / 2;
  state.areaGroup.add(state.groundPlane);
}

export function updateTravellingAreaTransform() {
  if (!state.areaGroup) {
    console.warn('[AREA DEBUG] updateTransform: areaGroup is missing');
    return;
  }

  // The stored map coordinates are WORLD coordinates. The travelling area is
  // a moving window under a fixed camera, so the selected world region must
  // be translated by -areaCenter into the camera's local view.
  //
  // IMPORTANT: pointCloud, robotGroup and trajectory are children of this
  // group. This single transform therefore moves the COMPLETE visible scene
  // consistently instead of only moving a visual grid.
  state.areaGroup.position.set(
    -state.areaCenter.x,
    0,
    -state.areaCenter.z
  );

  state.areaGroup.updateMatrixWorld(true);

  const worldPos = new THREE.Vector3();
  state.areaGroup.getWorldPosition(worldPos);

  console.log('[AREA DEBUG] transform', {
    center: {...state.areaCenter},
    groupPosition: {
      x: state.areaGroup.position.x,
      y: state.areaGroup.position.y,
      z: state.areaGroup.position.z
    },
    worldPosition: { x: worldPos.x, y: worldPos.y, z: worldPos.z },
    camera: {
      x: state.camera?.position.x,
      y: state.camera?.position.y,
      z: state.camera?.position.z
    },
    pointCloudParent: state.pointCloud?.parent?.name || state.pointCloud?.parent?.type,
    robotParent: state.robotGroup?.parent?.name || state.robotGroup?.parent?.type
  });
}

export function startRenderLoop(onFps) {
  state.frameLastTime = performance.now();
  state.frameCount = 0;

  function animate() {
    requestAnimationFrame(animate);
    state.controls.update();
    state.renderer.render(state.scene, state.camera);

    const now = performance.now();
    state.frameCount++;
    if (now - state.frameLastTime >= 1000) {
      onFps(state.frameCount);
      state.frameCount = 0;
      state.frameLastTime = now;
    }
  }

  animate();
}

export function handleResize() {
  state.camera.aspect = window.innerWidth / window.innerHeight;
  state.camera.updateProjectionMatrix();
  state.renderer.setSize(window.innerWidth, window.innerHeight);
}
