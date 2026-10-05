import * as THREE from 'three';
import { state } from './state.js';
import {
  MAX_RENDER_POINTS,
  MAX_POINTS,
  SURROUNDING_MAX_POINTS,
  TRAVELLING_VISIBLE_HALF_SIZE,
  TRAVELLING_SURROUNDING_HALF_SIZE,
  runtimeConfig
} from './config.js';
import { heightColor } from './utils.js';
import { mapStore } from './map-store.js';
import { notePointsAdded } from './session-backup.js';
import { getAdaptiveRenderHint } from './navigation.js';

function setHudCounts() {
  if (state.hud.pts) {
    state.hud.pts.textContent = state.mapStore.length.toLocaleString();
  }
  if (state.hud.areaVisible) {
    state.hud.areaVisible.textContent =
      state.areaVisibleCount.toLocaleString();
  }
  if (state.hud.areaSurrounding) {
    state.hud.areaSurrounding.textContent =
      state.areaSurroundingCount.toLocaleString();
  }
  if (state.hud.areaCenter) {
    state.hud.areaCenter.textContent =
      `${state.areaCenter.x.toFixed(2)}, ${state.areaCenter.z.toFixed(2)}`;
  }
}

function ensurePointRenderCapacity(requiredPoints) {
  const required = Math.max(0, Math.ceil(requiredPoints));
  const current = state.pointPositions ? Math.floor(state.pointPositions.length / 3) : 0;
  if (required <= current) return;

  let next = Math.max(262144, current || 262144);
  while (next < required) next = Math.ceil(next * 1.5);

  const positions = new Float32Array(next * 3);
  const colors = new Float32Array(next * 3);
  if (state.pointPositions) positions.set(state.pointPositions);
  if (state.pointColors) colors.set(state.pointColors);
  state.pointPositions = positions;
  state.pointColors = colors;

  if (state.pointGeometry) {
    state.pointGeometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    state.pointGeometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));
  }
}

function ensureVoxelCapacity(required) {
  const need = Math.max(1, Math.ceil(required));
  const current = state.voxelMesh ? state.voxelMesh.instanceMatrix.count : 0;
  if (current >= need) return;

  let next = Math.max(4096, current || 4096);
  while (next < need) next = Math.ceil(next * 1.5);

  const old = state.voxelMesh;
  const mesh = new THREE.InstancedMesh(state.voxelGeometry, state.voxelMaterial, next);
  mesh.count = 0;
  mesh.instanceMatrix.setUsage(THREE.DynamicDrawUsage);
  mesh.frustumCulled = false;
  mesh.name = 'AdaptiveVoxelMap';
  if (old && old.instanceMatrix && old.count > 0) {
    const matrix = new THREE.Matrix4();
    const color = new THREE.Color();
    for (let i = 0; i < old.count; i++) {
      old.getMatrixAt(i, matrix);
      mesh.setMatrixAt(i, matrix);
      if (old.instanceColor) {
        old.getColorAt(i, color);
        mesh.setColorAt(i, color);
      }
    }
    mesh.count = old.count;
  }
  if (old) state.areaGroup.remove(old);
  state.voxelMesh = mesh;
  state.areaGroup.add(mesh);
}

export function initPointCloud() {
  state.mapStore = mapStore;

  const viewer = runtimeConfig.viewer || {};
  const visibleMax = Number(viewer.visible_max_points || MAX_POINTS);
  const surroundingMax = Number(viewer.surrounding_max_points || SURROUNDING_MAX_POINTS);
  const renderMax = visibleMax + surroundingMax;

  state.pointGeometry = new THREE.BufferGeometry();
  // Start with a modest GPU buffer and grow it to exactly what the visible
  // area needs. The visible area is never spatially sampled or capped.
  state.pointPositions = new Float32Array(Math.max(262144, renderMax) * 3);
  state.pointColors = new Float32Array(Math.max(262144, renderMax) * 3);

  state.pointGeometry.setAttribute(
    'position',
    new THREE.BufferAttribute(state.pointPositions, 3)
  );
  state.pointGeometry.setAttribute(
    'color',
    new THREE.BufferAttribute(state.pointColors, 3)
  );
  state.pointGeometry.setDrawRange(0, 0);

  const material = new THREE.PointsMaterial({
    size: 0.05,
    vertexColors: true,
    sizeAttenuation: true,
    transparent: true,
    opacity: 0.85
  });

  state.pointCloud = new THREE.Points(state.pointGeometry, material);
  // The travelling area already spatially bounds the cloud. Avoid repeatedly
  // computing large point-cloud bounding spheres on the CPU.
  state.pointCloud.frustumCulled = false;

  // Bounded voxel representation. It is deliberately capped and kept
  // separate from the persistent point cloud so navigation never depends on
  // the number of rendered primitives.
  state.voxelGeometry = new THREE.BoxGeometry(1, 1, 1);
  state.voxelMaterial = new THREE.MeshBasicMaterial({
    // InstancedMesh supplies per-voxel colors through instanceColor.
    // Do not enable geometry vertexColors here: BoxGeometry has no color
    // attribute, and enabling it makes the material multiply against a
    // missing vertex-color attribute, producing black voxels.
    color: 0xffffff
  });
  state.voxelMesh = new THREE.InstancedMesh(state.voxelGeometry, state.voxelMaterial, Math.max(4096, Number(viewer.adaptive_initial_voxels || 4096)));
  state.voxelMesh.count = 0;
  state.voxelMesh.instanceMatrix.setUsage(THREE.DynamicDrawUsage);
  state.voxelMesh.frustumCulled = false;
  state.voxelMesh.name = 'AdaptiveVoxelMap';
  // The point cloud is part of the travelling scene. Its vertex positions
  // remain COMPLETE-MAP WORLD coordinates; areaGroup applies the -center
  // transform so the selected region is moved under the fixed camera.
  state.pointCloud.name = 'PointCloud';
  state.areaGroup.add(state.pointCloud);
  state.areaGroup.add(state.voxelMesh);
  state.threeDMapRenderingEnabled = true;
  console.log('[AREA DEBUG] point cloud attached to areaGroup');
  setHudCounts();
}

export function set3DMapRenderingEnabled(enabled) {
  const on = !!enabled;
  state.threeDMapRenderingEnabled = on;
  if (state.pointCloud) state.pointCloud.visible = on;
  if (state.voxelMesh) state.voxelMesh.visible = on && state.voxelMesh.count > 0;
  if (!on) {
    state.areaRefreshPending = false;
    return;
  }
  requestTravellingAreaRefresh();
}

export function resetPointCloud() {
  mapStore.clear();
  state.writeIdx = 0;
  state.areaVisibleCount = 0;
  state.areaSurroundingCount = 0;

  if (state.pointGeometry) {
    state.pointGeometry.setDrawRange(0, 0);
    state.pointGeometry.attributes.position.needsUpdate = true;
    state.pointGeometry.attributes.color.needsUpdate = true;
  }

  setHudCounts();
}

function writePoint(renderIndex, pointId, surrounding) {
  const source = pointId * 3;

  let ex = mapStore.positions[source];
  let ey = mapStore.positions[source + 1];
  let ez = mapStore.positions[source + 2];

  if (state.invX) ex = -ex;
  if (state.invY) ey = -ey;
  if (state.invZ) ez = -ez;

  const dst = renderIndex * 3;

  // ENU -> Three.js. The areaGroup supplies the travelling-area offset,
  // leaving the stored world coordinates untouched.
  state.pointPositions[dst] = ex;
  state.pointPositions[dst + 1] = ez;
  state.pointPositions[dst + 2] = -ey;

  if (surrounding) {
    state.pointColors[dst] = 0.42;
    state.pointColors[dst + 1] = 0.42;
    state.pointColors[dst + 2] = 0.45;
  } else {
    const [r, g, b] = heightColor(ez);
    state.pointColors[dst] = r;
    state.pointColors[dst + 1] = g;
    state.pointColors[dst + 2] = b;
  }
}

export function refreshTravellingArea() {
  if (!state.mapStore || !state.pointGeometry) return;
  if (state.threeDMapRenderingEnabled === false) return;

  const started = performance.now();
  const viewer = runtimeConfig.viewer || {};
  // IMPORTANT: the visible window is a spatial window, not a point budget.
  // Every source point inside it must reach the renderer (or the voxelizer).
  // Only the optional surrounding ring may be sampled.
  const surroundingMax = Number.isFinite(Number(viewer.surrounding_max_points))
    ? Number(viewer.surrounding_max_points) : Infinity;

  const { visible, surrounding } = mapStore.queryRegions(
    state.areaCenter.x,
    state.areaCenter.z,
    Number(viewer.visible_half_size_m || TRAVELLING_VISIBLE_HALF_SIZE),
    Number(viewer.surrounding_half_size_m || TRAVELLING_SURROUNDING_HALF_SIZE),
    Infinity,
    surroundingMax
  );

  const adaptive = viewer.adaptive_representation_enabled !== false;
  const mode = String(viewer.adaptive_representation_mode || 'auto').toLowerCase();
  const hint = getAdaptiveRenderHint(state.navPose);
  const overloadByCount = mapStore.length >= Number(viewer.adaptive_overload_point_threshold || 1000000);
  const overloadByVisible = visible.length >= Number(viewer.adaptive_overload_point_threshold || 1000000) * Number(viewer.adaptive_overload_visible_ratio || 0.9);
  const overloadByTime = Number(state.renderLoad?.elapsedMs || 0) >= Number(viewer.adaptive_overload_refresh_ms || 35);
  const forceVoxels = adaptive && (mode === 'voxels' || (mode === 'auto' && (hint.mature || overloadByCount || overloadByVisible || overloadByTime)));
  // Once voxel rendering is selected, voxelize the ENTIRE visible window.
  // Do not mix point sampling with voxelization inside the same visible area.
  const voxelAll = forceVoxels;
  const matureKeys = new Set(hint.keys || []);
  const voxelSize = Math.max(0.001, Number(viewer.adaptive_voxel_size_m || 0.10));
  // No spatial voxels are discarded because of a renderer count limit.
  // adaptive_max_voxels is retained only as a backwards-compatible optional
  // emergency limit when explicitly set to a positive value smaller than the
  // generated voxel count. By default it is unlimited.
  const configuredVoxelLimit = Number(viewer.adaptive_max_voxels);
  const maxVoxels = configuredVoxelLimit > 0 ? configuredVoxelLimit : Infinity;

  const useVoxelForVisible = forceVoxels;
  const directPointCount = useVoxelForVisible ? surrounding.length : visible.length + surrounding.length;
  ensurePointRenderCapacity(directPointCount);

  let write = 0;
  // Each voxel is a real spatial cube. All points falling into that cube are
  // accumulated and the rendered voxel is placed at their AVERAGE position,
  // rather than at the first point encountered. This keeps the Minecraft-like
  // 3D structure while preserving the cloud's local surface position.
  const voxels = new Map();
  const pushVoxel = (id) => {
    const off=id*3;
    const x=mapStore.positions[off], sy=mapStore.positions[off+1], z=mapStore.positions[off+2];
    const key=`${Math.floor(x/voxelSize)},${Math.floor(sy/voxelSize)},${Math.floor(z/voxelSize)}`;
    let v = voxels.get(key);
    if (!v) {
      if (voxels.size >= maxVoxels) return;
      v = {sx:0, sy:0, sz:0, count:0};
      voxels.set(key, v);
    }
    v.sx += x; v.sy += sy; v.sz += z; v.count++;
  };

  for (const id of visible) {
    const off=id*3;
    const x=mapStore.positions[off], sy=mapStore.positions[off+1];
    const tileKey = `${Math.floor(x / Math.max(0.05, Number(runtimeConfig.navigation?.exploration_tile_size_m || 0.5)))},${Math.floor(sy / Math.max(0.05, Number(runtimeConfig.navigation?.exploration_tile_size_m || 0.5)))}`;
    const useVoxel = voxelAll;
    if (useVoxel) pushVoxel(id);
    else if (write < state.pointPositions.length / 3) writePoint(write++, id, false);
  }

  for (const id of surrounding) {
    if (write < state.pointPositions.length / 3) writePoint(write++, id, true);
  }

  state.areaVisibleCount = visible.length;
  state.areaSurroundingCount = surrounding.length;
  state.writeIdx = write;

  state.pointGeometry.setDrawRange(0, write);
  state.pointGeometry.attributes.position.needsUpdate = true;
  state.pointGeometry.attributes.color.needsUpdate = true;
  // Frustum culling is disabled for this bounded travelling-area object, so
  // no large CPU bounding-sphere scan is required after every refresh.

  if (state.voxelMesh) {
    ensureVoxelCapacity(voxels.size);
    let vi=0;
    const color = new THREE.Color();
    const matrix = new THREE.Matrix4();
    for (const v of voxels.values()) {
      const avgX = v.sx / v.count;
      const avgY = v.sy / v.count;
      const avgZ = v.sz / v.count;
      let x=avgX, y=avgZ, z=-avgY;
      if (state.invX) x=-x;
      if (state.invY) z=-z;
      if (state.invZ) y=-y;
      matrix.compose(new THREE.Vector3(x,y,z), new THREE.Quaternion(), new THREE.Vector3(voxelSize,voxelSize,voxelSize));
      state.voxelMesh.setMatrixAt(vi, matrix);
      const rgb = heightColor(avgZ);
      color.setRGB(rgb[0], rgb[1], rgb[2]);
      state.voxelMesh.setColorAt(vi, color);
      vi++;
    }
    state.voxelMesh.count=vi;
    state.voxelMesh.instanceMatrix.needsUpdate=true;
    if (state.voxelMesh.instanceColor) state.voxelMesh.instanceColor.needsUpdate=true;
    state.voxelMesh.visible = forceVoxels && vi > 0;
  }

  const elapsed = performance.now() - started;
  state.renderLoad = { elapsedMs: elapsed, visible: visible.length, surrounding: surrounding.length, voxels: voxels.size, mapTotal: mapStore.length, localCoverage: hint.localCoverage || 0, mature: !!hint.mature, mode: forceVoxels ? 'VOXELS' : 'POINTS' };
  setHudCounts();
}

export function requestTravellingAreaRefresh() {
  if (state.threeDMapRenderingEnabled === false) return;
  if (state.areaRefreshPending) return;
  const configuredMinMs = Math.max(0, Number(runtimeConfig.viewer?.adaptive_refresh_min_interval_ms || 160));
  const loadMs = Number(state.renderLoad?.elapsedMs || 0);
  // If a refresh is expensive, deliberately reduce how often the renderer is
  // rebuilt. Navigation and point storage continue independently.
  const backoffMs = loadMs > 80 ? 500 : loadMs > 40 ? 300 : configuredMinMs;
  const minMs = Math.max(configuredMinMs, backoffMs);
  const now = performance.now();
  if (state.lastAreaRefreshAt && now - state.lastAreaRefreshAt < minMs) return;
  state.areaRefreshPending = true;
  requestAnimationFrame(() => {
    state.areaRefreshPending = false;
    state.lastAreaRefreshAt = performance.now();
    refreshTravellingArea();
  });
}

export function addPointsChunk(pts) {
  const count = pts.length / 3;
  if (count === 0) return;

  mapStore.append(pts);
  notePointsAdded(count);
  requestTravellingAreaRefresh();
}

export async function addPoints(pts) {
  addPointsChunk(pts);
}

export async function ensureCapacityFor() {
  // Kept as a compatibility shim for older callers. The complete map is no
  // longer limited by the renderer capacity, so there is no capacity reset.
}
