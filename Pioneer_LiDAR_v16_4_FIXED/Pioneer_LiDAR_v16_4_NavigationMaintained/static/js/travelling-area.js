import * as THREE from 'three';
import { state } from './state.js';
import {
  TRAVELLING_DOUBLE_CLICK_MS,
  TRAVELLING_DOUBLE_CLICK_MOVE_PX,
  runtimeConfig
} from './config.js';
import { updateTravellingAreaTransform } from './scene.js';
import { requestTravellingAreaRefresh, refreshTravellingArea } from './point-cloud.js';
import { toast, log } from './utils.js';

const raycaster = new THREE.Raycaster();
const groundPlane = new THREE.Plane(new THREE.Vector3(0, 1, 0), 0);
const pointer = new THREE.Vector2();

function pointerToGround(event) {
  const rect = state.renderer.domElement.getBoundingClientRect();

  pointer.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
  pointer.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;

  raycaster.setFromCamera(pointer, state.camera);

  const hit = new THREE.Vector3();
  const result = raycaster.ray.intersectPlane(groundPlane, hit);
  if (!result) {
    console.warn('[AREA DEBUG] pointerToGround: ray did not hit y=0 plane', {
      pointer: {x: pointer.x, y: pointer.y},
      rayOrigin: raycaster.ray.origin.clone(),
      rayDirection: raycaster.ray.direction.clone()
    });
    return null;
  }
  return hit;
}

function setModeUI() {
  const badge = document.getElementById('area-nav-badge');
  if (!badge) return;

  badge.classList.toggle('active', state.areaNavMode);
  badge.textContent = state.areaNavMode
    ? 'AREA NAVIGATION — DOUBLE-CLICK + DRAG'
    : 'AREA NAVIGATION OFF';

  if (state.hud.areaMode) {
    state.hud.areaMode.textContent =
      state.areaNavMode ? 'ACTIVE' : 'OFF';
    state.hud.areaMode.className =
      'hud-value ' + (state.areaNavMode ? 'accent' : 'warning');
  }
}

export function setAreaCenter(x, z, refresh = true) {
  const old = {...state.areaCenter};
  state.areaCenter.x = Number.isFinite(x) ? x : 0;
  state.areaCenter.z = Number.isFinite(z) ? z : 0;
  console.log('[AREA DEBUG] setAreaCenter', {old, next: {...state.areaCenter}, refresh});

  updateTravellingAreaTransform();

  if (refresh) {
    requestTravellingAreaRefresh();
  }
}

export function resetTravellingArea() {
  setAreaCenter(0, 0, true);
}

export function toggleAreaNavigation(force) {
  state.areaNavMode =
    force === undefined ? !state.areaNavMode : !!force;

  state.areaDrag = null;
  state.controls.enabled = !state.areaNavMode;
  console.log('[AREA DEBUG] navigation mode', {
    enabled: state.areaNavMode,
    controlsEnabled: state.controls.enabled,
    canvas: state.renderer?.domElement
  });
  state.renderer.domElement.classList.toggle(
    'area-nav-cursor',
    state.areaNavMode
  );

  setModeUI();

  if (state.areaNavMode) {
    toast(
      'Area Navigation: double-click, then drag the map',
      'info',
      3500
    );
    log('Area Navigation mode ENABLED', 'ok');
  } else {
    toast('Area Navigation mode disabled', 'info', 2200);
    log('Area Navigation mode DISABLED', 'info');
  }
}

function beginDrag(event) {
  const hit = pointerToGround(event);
  if (!hit) {
    console.warn('[AREA DEBUG] beginDrag: no ground hit');
    return;
  }

  state.areaDrag = {
    startHit: hit.clone(),
    startCenter: {
      x: state.areaCenter.x,
      z: state.areaCenter.z
    },
    pointerId: event.pointerId
  };

  state.renderer.domElement.setPointerCapture?.(event.pointerId);
  console.log('[AREA DEBUG] DRAG START', {
    pointerId: event.pointerId,
    startHit: {x: hit.x, z: hit.z},
    startCenter: {...state.areaCenter}
  });
}

function moveDrag(event) {
  if (!state.areaDrag) return;

  const hit = pointerToGround(event);
  if (!hit) return;

  const dx = hit.x - state.areaDrag.startHit.x;
  const dz = hit.z - state.areaDrag.startHit.z;

  // The rendered travelling-area group is positioned at -areaCenter so the
  // selected world region stays under the fixed camera. Therefore the stored
  // areaCenter must move opposite to the mouse delta for the *visible map*
  // itself to follow the user's drag.
  //
  // Example: drag right -> group moves right -> areaCenter decreases in X.
  const nextX = state.areaDrag.startCenter.x - dx;
  const nextZ = state.areaDrag.startCenter.z - dz;

  console.log('[AREA DEBUG] DRAG MOVE', {
    hit: {x: hit.x, z: hit.z},
    delta: {x: dx, z: dz},
    nextCenter: {x: nextX, z: nextZ}
  });

  setAreaCenter(nextX, nextZ, true);
}

function endDrag(event) {
  if (!state.areaDrag) return;

  try {
    state.renderer.domElement.releasePointerCapture?.(
      state.areaDrag.pointerId
    );
  } catch (_) {
    // Pointer capture may already have been released by the browser.
  }

  console.log('[AREA DEBUG] DRAG END', {...state.areaCenter});
  state.areaDrag = null;
}

function bindPointer() {
  const canvas = state.renderer.domElement;

  // Navigation gesture:
  //   - Preferred: double-click + hold the second click, then drag.
  //   - Fallback: in Area Navigation mode, a normal press-and-drag also
  //     starts navigation after a very small movement threshold.
  //
  // The fallback is intentional: browsers/trackpads can produce pointer
  // sequences where a user visually performs a double-click but the second
  // pointerdown arrives too late to satisfy a strict double-click timer.
  let pendingClick = null;
  let pressCandidate = null;

  const clearPending = () => {
    pendingClick = null;
    if (bindPointer._pendingTimer) {
      window.clearTimeout(bindPointer._pendingTimer);
      bindPointer._pendingTimer = null;
    }
  };

  const startFromPress = event => {
    if (state.areaDrag) return;
    console.log('[AREA DEBUG] PRESS-DRAG fallback -> beginDrag');
    beginDrag(event);
  };

  canvas.addEventListener('pointerdown', event => {
    if (!state.areaNavMode || event.button !== 0) return;

    const now = performance.now();
    console.log('[AREA DEBUG] pointerdown', {
      x: event.clientX,
      y: event.clientY,
      pending: !!pendingClick,
      pendingAgeMs: pendingClick ? now - pendingClick.time : null,
      time: now
    });

    event.preventDefault();
    event.stopPropagation();

    // A second press inside the normal double-click window starts the
    // requested double-click+hold gesture immediately.
    if (pendingClick &&
        now - pendingClick.time <= Number(runtimeConfig.viewer?.double_click_ms || TRAVELLING_DOUBLE_CLICK_MS)) {
      console.log('[AREA DEBUG] SECOND CLICK -> beginDrag', {
        first: {x: pendingClick.x, y: pendingClick.y},
        second: {x: event.clientX, y: event.clientY},
        elapsedMs: now - pendingClick.time
      });

      clearPending();
      pressCandidate = null;
      beginDrag(event);
      return;
    }

    // Start a normal press candidate. This enables click-drag fallback.
    clearPending();
    pendingClick = {
      time: now,
      x: event.clientX,
      y: event.clientY
    };
    pressCandidate = {
      pointerId: event.pointerId,
      x: event.clientX,
      y: event.clientY,
      startedAt: now
    };

    console.log('[AREA DEBUG] FIRST CLICK STORED / press candidate', pendingClick);

    bindPointer._pendingTimer = window.setTimeout(() => {
      if (pendingClick &&
          performance.now() - pendingClick.time >= Number(runtimeConfig.viewer?.double_click_ms || TRAVELLING_DOUBLE_CLICK_MS)) {
        console.log('[AREA DEBUG] first-click timeout -> single-drag fallback remains available');
        pendingClick = null;
      }
    }, Number(runtimeConfig.viewer?.double_click_ms || TRAVELLING_DOUBLE_CLICK_MS) + 20);
  }, true);

  canvas.addEventListener('dblclick', event => {
    if (!state.areaNavMode) return;
    console.log('[AREA DEBUG] native dblclick observed', {
      x: event.clientX,
      y: event.clientY
    });
  }, true);

  canvas.addEventListener('pointermove', event => {
    if (!state.areaNavMode) return;

    // Preferred double-click gesture already started.
    if (state.areaDrag) {
      event.preventDefault();
      event.stopPropagation();
      moveDrag(event);
      return;
    }

    // Fallback: if the user holds the mouse and moves, begin navigation.
    // This also makes the feature usable with trackpads where double-click
    // timing is inconsistent.
    if (pressCandidate &&
        pressCandidate.pointerId === event.pointerId) {
      const dx = event.clientX - pressCandidate.x;
      const dy = event.clientY - pressCandidate.y;
      const distance = Math.hypot(dx, dy);

      if (distance >= 4) {
        console.log('[AREA DEBUG] PRESS MOVE -> starting fallback drag', {
          distancePx: distance,
          dx,
          dy
        });
        startFromPress(event);
        if (state.areaDrag) {
          event.preventDefault();
          event.stopPropagation();
          moveDrag(event);
        }
      }
    }
  }, true);

  canvas.addEventListener('pointerup', event => {
    if (!state.areaNavMode) return;

    event.preventDefault();
    event.stopPropagation();

    if (state.areaDrag) {
      endDrag(event);
    }

    if (pressCandidate?.pointerId === event.pointerId) {
      pressCandidate = null;
    }
  }, true);

  canvas.addEventListener('pointercancel', event => {
    if (state.areaDrag) endDrag(event);
    pressCandidate = null;
    clearPending();
  }, true);

  canvas.addEventListener('contextmenu', event => {
    if (state.areaNavMode) event.preventDefault();
  }, true);
}
export function initTravellingArea() {
  bindPointer();
  updateTravellingAreaTransform();
  setModeUI();
}
