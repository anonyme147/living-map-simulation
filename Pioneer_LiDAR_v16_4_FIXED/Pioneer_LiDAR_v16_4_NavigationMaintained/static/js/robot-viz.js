import * as THREE from 'three';
import { state } from './state.js';
import { MAX_TRAJ, runtimeConfig } from './config.js';
import { enuToThree } from './utils.js';

export function initRobotViz() {
  const maxTraj = Number(runtimeConfig.trajectory?.max_render_points || MAX_TRAJ);
  state.trajRenderMax = maxTraj;

  state.robotGroup = new THREE.Group();
  state.robotGroup.name = 'RobotGroup';
  state.areaGroup.add(state.robotGroup);
  console.log('[AREA DEBUG] robot group attached to areaGroup');

  const chassis = new THREE.Mesh(
    new THREE.BoxGeometry(0.508, 0.277, 0.497),
    new THREE.MeshBasicMaterial({
      color: 0x00f0c8,
      wireframe: true,
      transparent: true,
      opacity: 0.4
    })
  );
  state.robotGroup.add(chassis);

  const deck = new THREE.Mesh(
    new THREE.BoxGeometry(0.45, 0.02, 0.42),
    new THREE.MeshBasicMaterial({
      color: 0x00f0c8,
      transparent: true,
      opacity: 0.25
    })
  );
  deck.position.set(0, 0.14, 0);
  state.robotGroup.add(deck);

  const corner = new THREE.Mesh(
    new THREE.SphereGeometry(0.04, 16, 16),
    new THREE.MeshBasicMaterial({ color: 0xffd700 })
  );
  corner.position.set(0.20, 0.16, 0.18);
  state.robotGroup.add(corner);

  const arrow = new THREE.Mesh(
    new THREE.ConeGeometry(0.04, 0.12, 8),
    new THREE.MeshBasicMaterial({ color: 0xff4757 })
  );
  arrow.rotation.z = -Math.PI / 2;
  arrow.position.set(0.32, 0, 0);
  state.robotGroup.add(arrow);

  state.robotGroup.add(new THREE.AxesHelper(0.5));

  state.trajectoryGeometry = new THREE.BufferGeometry();
  state.trajectoryPositions = new Float32Array(state.trajRenderMax * 3);
  state.trajectoryGeometry.setAttribute(
    'position',
    new THREE.BufferAttribute(state.trajectoryPositions, 3)
  );
  state.trajectoryGeometry.setDrawRange(0, 0);

  const trajectory = new THREE.Line(
    state.trajectoryGeometry,
    new THREE.LineBasicMaterial({
      color: 0x00f0c8,
      transparent: true,
      opacity: 0.6
    })
  );
  state.trajectory = trajectory;
  state.trajectory.name = 'Trajectory';
  state.areaGroup.add(trajectory);
  console.log('[AREA DEBUG] trajectory attached to areaGroup');
}

export function resetTrajectory() {
  state.trajIdx = 0;
  state.trajPrimitive = [];
  state.trajectoryGeometry.setDrawRange(0, 0);
  state.trajectoryGeometry.attributes.position.needsUpdate = true;
}

export function addTrajPoint(x, y, z) {
  const v = enuToThree(x, y, z);
  const i = state.trajIdx % state.trajRenderMax;

  state.trajectoryPositions[i*3] = v.x;
  state.trajectoryPositions[i*3+1] = v.y;
  state.trajectoryPositions[i*3+2] = v.z;

  state.trajIdx++;
  state.trajectoryGeometry.setDrawRange(
    0,
    Math.min(state.trajIdx, state.trajRenderMax)
  );
  state.trajectoryGeometry.attributes.position.needsUpdate = true;
  state.trajPrimitive.push([x, y, z]);
}

export function resetRobotViz() {
  resetTrajectory();
}

export function updateRobotPose(data, hud) {
  if (!data.robot) return;

  const r = data.robot;
  const gt = data.ground_truth || {x: r.x, y: r.y, z: 0};

  hud.ox.textContent = r.x.toFixed(2);
  hud.oy.textContent = r.y.toFixed(2);

  state.robotGroup.position.copy(enuToThree(gt.x, gt.y, gt.z));

  if (r.orientation && r.orientation.length === 9) {
    const R = r.orientation;

    const rt00 = R[0],  rt01 = R[2],  rt02 = -R[1];
    const rt10 = R[6],  rt11 = R[8],  rt12 = -R[7];
    const rt20 = -R[3], rt21 = -R[5], rt22 = R[4];

    const rotMat = new THREE.Matrix4();
    rotMat.set(
      rt00, rt01, rt02, 0,
      rt10, rt11, rt12, 0,
      rt20, rt21, rt22, 0,
      0, 0, 0, 1
    );

    state.robotGroup.quaternion.setFromRotationMatrix(rotMat);
    hud.rotMode.textContent = 'MATRIX';
    hud.rotMode.className = 'hud-value accent';
  } else {
    const rawYaw =
      (data.imu && data.imu.yaw !== undefined) ? data.imu.yaw : r.theta;

    state.robotGroup.rotation.set(0, -rawYaw, 0);
    hud.rotMode.textContent = 'YAW ONLY';
    hud.rotMode.className = 'hud-value warning';
  }

  addTrajPoint(gt.x, gt.y, gt.z);

  const rawYaw =
    (gt.yaw !== undefined && Number.isFinite(Number(gt.yaw)))
      ? Number(gt.yaw)
      : r.theta;

  hud.oth.textContent = (rawYaw * 180 / Math.PI).toFixed(1) + '°';
}
