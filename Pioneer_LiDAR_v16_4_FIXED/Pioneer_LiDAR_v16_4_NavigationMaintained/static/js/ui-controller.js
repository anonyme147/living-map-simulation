import { state } from './state.js';
import { toast, log, clampAngle } from './utils.js';
import { sendConfig } from './ws-client.js';
import { resetMap } from './map-manager.js';
import { API_URL, runtimeConfig } from './config.js';
import { toggleAreaNavigation } from './travelling-area.js';
import { clearSessionArchive } from './session-backup.js';
import { toggleNavMap, toggleExplorationMap } from './nav-map.js';
import { toggleAutoPilot } from './navigation.js';
import { sendVelocity, sendStop, sendScanControl } from './control.js';
import { requestTravellingAreaRefresh, set3DMapRenderingEnabled } from './point-cloud.js';

function cacheDom() {
  const ids = [
    'status-dot','status-text','scan-badge','scan-badge-text',
    'pts','fps','ox','oy','oth','imu-roll','imu-pitch','imu-yaw',
    'gx','gy','gz','rot-mode','gps-mode','hud-offset','hud-angle-min',
    'hud-angle-max','hud-view','hud-ground','msgs','lastpts','area-mode','area-center','area-visible','area-surrounding',
    'sidebar','sidebar-toggle','area-nav-badge','offset-slider','offset-value',
    'apply-btn','angle-min','angle-max','angle-apply-btn',
    'angle-reset-btn','ground-toggle','flip-btn','inv-x','inv-y',
    'inv-z','inv-reset-btn','adaptive-representation-toggle','show-3d-autopilot-toggle','help-overlay','autopilot-badge','nav-mode','nav-coverage','nav-goal'
  ];

  for (const id of ids) {
    state.dom[id] = document.getElementById(id);
  }

  state.hud = {
    statusDot: state.dom['status-dot'],
    statusTxt: state.dom['status-text'],
    scanBadge: state.dom['scan-badge'],
    scanBadgeText: state.dom['scan-badge-text'],
    pts: state.dom.pts,
    fps: state.dom.fps,
    ox: state.dom.ox,
    oy: state.dom.oy,
    oth: state.dom.oth,
    imuRoll: state.dom['imu-roll'],
    imuPitch: state.dom['imu-pitch'],
    imuYaw: state.dom['imu-yaw'],
    gx: state.dom.gx,
    gy: state.dom.gy,
    gz: state.dom.gz,
    rotMode: state.dom['rot-mode'],
    gpsMode: state.dom['gps-mode'],
    hudOffset: state.dom['hud-offset'],
    hudAngleMin: state.dom['hud-angle-min'],
    hudAngleMax: state.dom['hud-angle-max'],
    hudView: state.dom['hud-view'],
    hudGround: state.dom['hud-ground'],
    msgs: state.dom.msgs,
    lastpts: state.dom.lastpts,
    areaMode: state.dom['area-mode'],
    areaCenter: state.dom['area-center'],
    areaVisible: state.dom['area-visible'],
    areaSurrounding: state.dom['area-surrounding'],
  };
}

function toggleSidebar() {
  state.sidebarOpen = !state.sidebarOpen;
  state.dom.sidebar.classList.toggle('open', state.sidebarOpen);
  state.dom['sidebar-toggle'].innerHTML = state.sidebarOpen ? '✕' : '☰';
}

async function applyOffset() {
  const deg = parseFloat(state.dom['offset-slider'].value);
  const rad = deg * Math.PI / 180;

  // Do not archive the map that is intentionally being discarded. First
  // make sure the LiDAR configuration can actually be sent; only then clear
  // both the live map and its session archive.
  if (!sendConfig({type: 'config', offset_angle: rad})) {
    toast('WebSocket not connected — map was not reset', 'error');
    return;
  }

  try {
    await resetMap();
    toast(`Offset set to ${deg}° — map reset and session backup cleared`, 'info');
    state.hud.hudOffset.textContent = deg.toFixed(1) + '°';
  } catch (e) {
    toast('Offset sent, but map/session reset failed: ' + e.message, 'error');
  }
}

function applyAngleFilter() {
  const minDeg = clampAngle(state.dom['angle-min'].value);
  const maxDeg = clampAngle(state.dom['angle-max'].value);
  const minRad = minDeg * Math.PI / 180;
  const maxRad = maxDeg * Math.PI / 180;

  if (sendConfig({
    type: 'config',
    angle_min: minRad,
    angle_max: maxRad
  })) {
    toast(`Angle filter: ${minDeg}° → ${maxDeg}°`, 'info');
    state.hud.hudAngleMin.textContent = minDeg.toFixed(0) + '°';
    state.hud.hudAngleMax.textContent = maxDeg.toFixed(0) + '°';
    log(`Angle filter set: ${minDeg}° to ${maxDeg}°`, 'info');
  } else {
    toast('WebSocket not connected', 'error');
  }
}

function resetAngleFilter() {
  state.dom['angle-min'].value = 0;
  state.dom['angle-max'].value = 360;

  if (sendConfig({
    type: 'config',
    angle_min: 0,
    angle_max: Math.PI * 2
  })) {
    toast('Angle filter reset to 0–360°', 'info');
    state.hud.hudAngleMin.textContent = '0°';
    state.hud.hudAngleMax.textContent = '360°';
  } else {
    toast('WebSocket not connected', 'error');
  }
}

function updateGroundState() {
  state.grid.visible = state.groundVisible;
  state.groundPlane.visible = state.groundVisible;

  state.hud.hudGround.textContent =
    state.groundVisible ? 'ON' : 'OFF';
  state.hud.hudGround.className =
    'hud-value ' + (state.groundVisible ? 'success' : 'danger');
}

function toggleGroundView() {
  state.viewFromBelow = !state.viewFromBelow;

  if (state.viewFromBelow) {
    state.camera.position.y = -Math.abs(state.camera.position.y);
    state.camera.up.set(0, -1, 0);
    state.dom['flip-btn'].innerHTML =
      '<span>⇅</span> Flip to Top View';
    state.hud.hudView.textContent = 'BOTTOM';
    toast('Switched to bottom view', 'info');
  } else {
    state.camera.position.y = Math.abs(state.camera.position.y);
    state.camera.up.set(0, 1, 0);
    state.dom['flip-btn'].innerHTML =
      '<span>⇅</span> Flip to Bottom View';
    state.hud.hudView.textContent = 'TOP';
    toast('Switched to top view', 'info');
  }

  state.controls.update();
}

function updateInvState() {
  state.invX = state.dom['inv-x'].checked;
  state.invY = state.dom['inv-y'].checked;
  state.invZ = state.dom['inv-z'].checked;
}

function resetInversions() {
  state.dom['inv-x'].checked = false;
  state.dom['inv-y'].checked = false;
  state.dom['inv-z'].checked = false;
  updateInvState();
  toast('Scan inversions reset', 'info');
}

function toggleHelp() {
  state.dom['help-overlay'].classList.toggle('show');
}

function isTypingTarget(e) {
  const t = e.target;
  return t && (t.tagName === 'INPUT' || t.tagName === 'TEXTAREA' || t.tagName === 'SELECT' || t.isContentEditable);
}

function toggleOverlay(id, stateKey) {
  const el = document.getElementById(id);
  if (!el) return;
  state[stateKey] = !state[stateKey];
  el.classList.toggle('show', state[stateKey]);
}

function bindOverlayControls() {
  document.querySelectorAll('[data-close]').forEach(btn => {
    btn.addEventListener('click', () => {
      const id = btn.getAttribute('data-close');
      const el = document.getElementById(id);
      if (el) el.classList.remove('show');
      if (id === 'debug-window') state.debugVisible = false;
      if (id === 'status-window') state.statusVisible = false;
    });
  });
}

function bindKeyboard() {
  window.addEventListener('keydown', e => {
    if (isTypingTarget(e)) return;
    const key = e.key;
    if (key === 'Tab') {
      e.preventDefault(); toggleSidebar();
    } else if (key === 'v' || key === 'V') {
      toggleGroundView();
    } else if (key === 'g' || key === 'G') {
      toggleAreaNavigation();
    } else if (key === 'a' || key === 'A') {
      e.preventDefault(); toggleAutoPilot();
    } else if (key === 'm' || key === 'M') {
      e.preventDefault(); toggleNavMap();
    } else if (key === 't' || key === 'T') {
      e.preventDefault(); toggleExplorationMap();
    } else if (key === 'p' || key === 'P') {
      e.preventDefault(); toggleOverlay('palette-legend', 'paletteVisible');
    } else if (key === 'd' || key === 'D') {
      e.preventDefault(); toggleOverlay('debug-window', 'debugVisible');
    } else if (key === 's' || key === 'S') {
      e.preventDefault();
      state.statusVisible = !state.statusVisible;
      document.getElementById('hud')?.classList.toggle('status-hidden', !state.statusVisible);
    } else if (key === ' ') {
      e.preventDefault(); sendScanControl(!state.lastScanState);
    } else if (key === 'ArrowUp') {
      e.preventDefault(); sendVelocity(0.35, 0, 'manual', 900);
    } else if (key === 'ArrowDown') {
      e.preventDefault(); sendVelocity(-0.35, 0, 'manual', 900);
    } else if (key === 'ArrowLeft') {
      e.preventDefault(); sendVelocity(0, 1.0, 'manual', 900);
    } else if (key === 'ArrowRight') {
      e.preventDefault(); sendVelocity(0, -1.0, 'manual', 900);
    } else if (key === '?' || key === 'Slash') {
      toggleHelp();
    } else if (key === 'Escape') {
      state.dom['help-overlay'].classList.remove('show');
      ['palette-legend','debug-window','status-window'].forEach(id => document.getElementById(id)?.classList.remove('show'));
      state.paletteVisible = state.debugVisible = false;
      state.statusVisible = false;
      document.getElementById('hud')?.classList.add('status-hidden');
    }
  });
  window.addEventListener('keyup', e => {
    if (['ArrowUp','ArrowDown','ArrowLeft','ArrowRight'].includes(e.key)) {
      e.preventDefault();
      sendStop('manual');
    }
  });
}
export function initUI() {
  cacheDom();

  state.dom['sidebar-toggle'].addEventListener('click', toggleSidebar);

  state.dom['offset-slider'].addEventListener('input', () => {
    state.dom['offset-value'].textContent =
      state.dom['offset-slider'].value + '°';
  });

  state.dom['apply-btn'].addEventListener('click', applyOffset);
  state.dom['angle-apply-btn'].addEventListener('click', applyAngleFilter);
  state.dom['angle-reset-btn'].addEventListener('click', resetAngleFilter);

  state.dom['ground-toggle'].addEventListener('change', () => {
    state.groundVisible = state.dom['ground-toggle'].checked;
    updateGroundState();
    toast(
      state.groundVisible ? 'Ground plane visible' : 'Ground plane hidden',
      'info'
    );
  });

  state.dom['flip-btn'].addEventListener('click', toggleGroundView);

  state.dom['adaptive-representation-toggle']?.addEventListener('change', () => {
    runtimeConfig.viewer.adaptive_representation_enabled = state.dom['adaptive-representation-toggle'].checked;
    requestTravellingAreaRefresh();
    toast(state.dom['adaptive-representation-toggle'].checked ? 'Adaptive rendering enabled' : 'Adaptive rendering disabled — points only', 'info');
  });

  state.dom['show-3d-autopilot-toggle']?.addEventListener('change', () => {
    runtimeConfig.viewer.show_3d_map_during_autopilot = state.dom['show-3d-autopilot-toggle'].checked;
    const visible = !state.autoPilot || runtimeConfig.viewer.show_3d_map_during_autopilot;
    set3DMapRenderingEnabled(visible);
    toast(visible ? '3D map rendering enabled' : '3D map hidden during Auto Pilot — local 2D navigation remains active', 'info');
  });

  window.addEventListener('autopilot-render-state', (event) => {
    const active = !!event.detail?.active;
    const allow3D = runtimeConfig.viewer?.show_3d_map_during_autopilot !== false;
    set3DMapRenderingEnabled(!active || allow3D);
  });

  [state.dom['inv-x'], state.dom['inv-y'], state.dom['inv-z']]
    .forEach(box => box.addEventListener('change', updateInvState));

  state.dom['inv-reset-btn'].addEventListener(
    'click',
    resetInversions
  );

  const viewer = runtimeConfig.viewer || {};
  state.dom['inv-x'].checked = !!viewer.invert_x;
  state.dom['inv-y'].checked = !!viewer.invert_y;
  state.dom['inv-z'].checked = !!viewer.invert_z;
  state.invX = !!viewer.invert_x;
  state.invY = !!viewer.invert_y;
  state.invZ = !!viewer.invert_z;
  state.groundVisible = viewer.ground_visible !== false;
  state.dom['ground-toggle'].checked = state.groundVisible;
  if (state.dom['adaptive-representation-toggle']) {
    state.dom['adaptive-representation-toggle'].checked = viewer.adaptive_representation_enabled !== false;
  }
  if (state.dom['show-3d-autopilot-toggle']) {
    state.dom['show-3d-autopilot-toggle'].checked = viewer.show_3d_map_during_autopilot !== false;
  }

  updateInvState();
  updateGroundState();
  bindKeyboard();
  bindOverlayControls();

  return state.hud;
}
