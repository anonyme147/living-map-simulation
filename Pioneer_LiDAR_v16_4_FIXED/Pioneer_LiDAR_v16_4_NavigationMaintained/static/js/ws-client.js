import * as THREE from 'three';
import { state } from './state.js';
import { getWsUrl, LSCN_MAGIC } from './config.js';
import { enuToThree, toast, log } from './utils.js';
import { addPoints } from './point-cloud.js';
import { initializeSession } from './session-backup.js';
import { addTrajPoint, updateRobotPose } from './robot-viz.js';
import { setNavMap, setNavState } from './nav-map.js';
import { onScanFrame, onRobotPose } from './navigation.js';
import { sendConfig, sendVelocity, sendStop, sendScanControl } from './control.js';
export { sendConfig, sendVelocity, sendStop, sendScanControl } from './control.js';
import { updateDetections } from './detection-viz.js';

export function connect() {
  setStatus('connecting', 'Connecting…');
  const wsUrl = getWsUrl();
  state.wsUrl = wsUrl;
  log('Connecting to ' + wsUrl, 'info');

  state.ws = new WebSocket(wsUrl);
  if (state.simulationStaleTimer) clearInterval(state.simulationStaleTimer);
  state.simulationStaleTimer = setInterval(() => {
    if (!state.ws || state.ws.readyState !== WebSocket.OPEN) return;
    const age = performance.now() - Number(state.simulationLastTelemetryAt || 0);
    if (age > 1200) {
      state.simulationState = 'STALE';
      setStatus('connected', 'Connected · simulation paused/stale');
    }
  }, 500);
  state.ws.binaryType = 'arraybuffer';

  state.ws.onopen = () => {
    initializeSession();
    applyStartupLidarConfig();
    setStatus('connected', 'Connected');
    toast('WebSocket connected', 'success');
    log('WebSocket CONNECTED', 'ok');
  };

  state.ws.onmessage = handleMessage;

  state.ws.onclose = () => {
    setStatus('disconnected', 'Disconnected');
    toast('Connection lost — reconnecting…', 'error');
    log('WebSocket CLOSED — reconnecting in 2s', 'err');

    if (state.reconnectTimer) clearTimeout(state.reconnectTimer);
    state.reconnectTimer = setTimeout(connect, 2000);
  };

  state.ws.onerror = () => {
    setStatus('disconnected', 'Error');
    log('WebSocket ERROR', 'err');
    state.ws.close();
  };
}

function applyStartupLidarConfig() {
  const cfg = state.appConfig?.lidar;
  if (!cfg) return;

  const offsetRad = Number(cfg.offset_angle_deg || 0) * Math.PI / 180;
  const minRad = Number(cfg.angle_filter?.min_deg ?? 0) * Math.PI / 180;
  const maxRad = Number(cfg.angle_filter?.max_deg ?? 360) * Math.PI / 180;

  sendConfig({
    type: 'config',
    offset_angle: offsetRad,
    angle_min: minRad,
    angle_max: maxRad,
    command_timeout_sec: Number(state.appConfig?.navigation?.command_watchdog_timeout_sec ?? 0.75)
  });

  const slider = document.getElementById('offset-slider');
  const offsetValue = document.getElementById('offset-value');
  const angleMin = document.getElementById('angle-min');
  const angleMax = document.getElementById('angle-max');

  if (slider) slider.value = Number(cfg.offset_angle_deg || 0);
  if (offsetValue) offsetValue.textContent = `${Number(cfg.offset_angle_deg || 0)}°`;
  if (angleMin) angleMin.value = Number(cfg.angle_filter?.min_deg ?? 0);
  if (angleMax) angleMax.value = Number(cfg.angle_filter?.max_deg ?? 360);

  if (state.hud?.hudOffset) {
    state.hud.hudOffset.textContent = `${Number(cfg.offset_angle_deg || 0).toFixed(1)}°`;
  }
  if (state.hud?.hudAngleMin) {
    state.hud.hudAngleMin.textContent = `${Number(cfg.angle_filter?.min_deg ?? 0).toFixed(0)}°`;
  }
  if (state.hud?.hudAngleMax) {
    state.hud.hudAngleMax.textContent = `${Number(cfg.angle_filter?.max_deg ?? 360).toFixed(0)}°`;
  }

  console.log('[CONFIG] LiDAR startup config sent', cfg);
}

function setGroundTruthNavPose(data, reason = 'scan') {
  const gt = data?.ground_truth;
  if (!gt) return false;

  const x = Number(gt.x);
  const y = Number(gt.y);
  if (!Number.isFinite(x) || !Number.isFinite(y)) return false;

  const robot = data?.robot || {};
  const rawYaw = Number(gt.yaw);
  const controllerYaw = Number.isFinite(rawYaw)
    ? rawYaw
    : (Number.isFinite(Number(robot.theta)) ? Number(robot.theta) : 0);
  const headingOffset = Number(state.appConfig?.navigation?.heading_offset_rad ?? Math.PI / 2);
  const theta = Math.atan2(
    Math.sin(controllerYaw + headingOffset),
    Math.cos(controllerYaw + headingOffset)
  );

  state.navPose = {
    x,
    y,
    z: Number.isFinite(Number(gt.z)) ? Number(gt.z) : 0,
    theta,
    rawYaw: controllerYaw,
    roll: Number.isFinite(Number(gt.roll)) ? Number(gt.roll) : undefined,
    pitch: Number.isFinite(Number(gt.pitch)) ? Number(gt.pitch) : undefined,
    yaw: theta,
    raw_yaw: controllerYaw,
    orientation: Array.isArray(gt.orientation)
      ? gt.orientation
      : (Array.isArray(robot.orientation) ? robot.orientation : undefined),
    source: 'ground_truth'
  };

  state.groundTruth = {
    ...gt,
    x, y, z: state.navPose.z,
    yaw: theta,
    orientation: state.navPose.orientation
  };

  return true;
}

function updateGpsPoseStatus(data) {
  const robot = data?.robot;
  if (!robot || typeof robot.gps_denied !== 'boolean') return;

  const denied = robot.gps_denied;
  const source = robot.pose_source || (denied ? 'ekf' : 'ground_truth');
  const changed = state.gpsDenied !== denied;
  state.gpsDenied = denied;
  state.poseSource = source;

  if (state.hud?.gpsMode) {
    state.hud.gpsMode.textContent = denied ? 'DENIED · EKF' : 'AVAILABLE · GPS';
    state.hud.gpsMode.className = 'hud-value ' + (denied ? 'warn' : 'success');
  }

  if (changed) {
    const message = denied
      ? 'GPS DENIED ZONE entered — position estimator switched to EKF'
      : 'GPS available — ground-truth position restored';
    toast(message, denied ? 'warn' : 'success', 5000);
    log(`[LOCALIZATION] ${message}`, denied ? 'warn' : 'ok');
  }
}


async function handleMessage(evt) {
  if (typeof evt.data === 'string') {
    try {
      const data = JSON.parse(evt.data);
      if (data.type === 'simulation_state') {
        state.simulationTime = Number(data.simulation_time);
        state.simulationLastTelemetryAt = performance.now();
        state.simulationState = data.scanning ? 'RUNNING_SCANNING' : 'RUNNING_IDLE';
        // GT is authoritative whenever it is available.  A heartbeat without
        // GT must not overwrite the last valid navigation pose with odometry.
        setGroundTruthNavPose(data, 'simulation_state');
        setStatus('connected', data.scanning ? 'Connected · simulation running' : 'Connected · scan OFF');
        return;
      } else if (data.type === 'nav_state') {
        // Backend navigation telemetry is diagnostic/UI state in GT mode. It
        // must never replace the controller-provided GT pose.
        if (data.ground_truth) setGroundTruthNavPose(data, 'nav_state');
        setNavState(data);
      } else if (data.type === 'nav_map') {
        setNavMap(data);
      }
    } catch (e) {
      log('Received invalid JSON message: ' + e.message, 'warn');
    }
    return;
  }

  if (!(evt.data instanceof ArrayBuffer)) {
    log('Received unsupported message', 'warn');
    return;
  }

  const buf = evt.data;
  const view = new DataView(buf);

  if (view.byteLength < 4 || view.getUint32(0, true) !== LSCN_MAGIC) {
    log('Bad frame magic', 'warn');
    return;
  }

  if (view.byteLength < 8) {
    log('Frame too short', 'warn');
    return;
  }

  const headerLen = view.getUint32(4, true);
  const headerOff = 8;
  const pointsOff = headerOff + headerLen;

  if (view.byteLength < pointsOff) {
    log('Header incomplete', 'warn');
    return;
  }

  const headerBytes = new Uint8Array(buf, headerOff, headerLen);
  let data;

  try {
    data = JSON.parse(new TextDecoder().decode(headerBytes));
  } catch (e) {
    log('Header parse failed: ' + e.message, 'err');
    return;
  }

  state.msgCount++;

  if (data.type === 'detection') {
    // Store detections in state
    state.detections = data.detections || [];
    log(`[PERCEPTION] detector=${data.detector_ready ? 'ready' : 'unavailable'} ` +
        `depth=${data.depth_ready ? 'ready' : 'unavailable'} ` +
        `detections=${data.detection_count ?? state.detections.length}`, 'scan-on');
    state.detections.forEach(det => {
      if (det.localization_valid && Array.isArray(det.position_world_frame)) {
        const p = det.position_world_frame.map(v => Number(v).toFixed(2)).join(', ');
        log(`[PERCEPTION] ${det.class} world=(${p}) depth=${Number(det.depth_m).toFixed(2)}m`, 'scan-on');
      } else {
        log(`[PERCEPTION] ${det.class} detected; localization invalid`, 'warn');
      }
    });
    // Update visualization if function exists
    if (typeof updateDetections === 'function') {
      updateDetections(state.detections);
    }
    return;
  }
  if (data.type !== 'scan') {
    log('Unknown type: ' + data.type, 'err');
    return;
  }

  const newPointCount = data.new_point_count || 0;
  const total = data.total_points || 0;

  if (
    data.scanning !== undefined &&
    data.scanning !== state.lastScanState
  ) {
    state.lastScanState = data.scanning;

    if (data.scanning) {
      log('>>> SCAN STARTED <<<', 'scan-on');
      toast('3D Scan started', 'success');
    } else {
      log('>>> SCAN STOPPED <<<', 'scan-off');
      toast('3D Scan stopped', 'warn');
    }
  }

  setScanState(!!data.scanning);
  updateGpsPoseStatus(data);

  if (data.offset_angle !== undefined) {
    state.hud.hudOffset.textContent =
      (data.offset_angle * 180 / Math.PI).toFixed(1) + '°';
  }

  if (data.angle_min_deg !== undefined) {
    state.hud.hudAngleMin.textContent =
      data.angle_min_deg.toFixed(0) + '°';
  }

  if (data.angle_max_deg !== undefined) {
    state.hud.hudAngleMax.textContent =
      data.angle_max_deg.toFixed(0) + '°';
  }

  state.hud.msgs.textContent = state.msgCount;
  state.hud.lastpts.textContent = newPointCount;
  state.hud.pts.textContent = (state.mapStore ? state.mapStore.length : total).toLocaleString();

  if (data.robot || data.ground_truth) {
    // Binary scan telemetry is the authoritative pose source in this GT-mode
    // build. The exact same state.navPose is then consumed by navigation.js.
    if (setGroundTruthNavPose(data, 'scan')) {
      updateRobotPose(data, state.hud);
      onRobotPose(data);
    } else {
      log('[GT] Scan packet has no valid ground_truth pose — keeping last GT navigation pose', 'warn');
    }
  }

  if (data.imu) {
    state.imu = data.imu;
    const imu = data.imu;
    state.hud.imuRoll.textContent =
      (imu.roll * 180 / Math.PI).toFixed(1) + '°';
    state.hud.imuPitch.textContent =
      (imu.pitch * 180 / Math.PI).toFixed(1) + '°';
    state.hud.imuYaw.textContent =
      (imu.yaw * 180 / Math.PI).toFixed(1) + '°';
  }

  if (state.groundTruth) {
    state.hud.gx.textContent = Number(state.groundTruth.x).toFixed(2);
    state.hud.gy.textContent = Number(state.groundTruth.y).toFixed(2);
    state.hud.gz.textContent = Number(state.groundTruth.z).toFixed(2);
  }

  if (
    newPointCount > 0 &&
    view.byteLength >= pointsOff + newPointCount * 12
  ) {
    const rawBytes = new Uint8Array(buf, pointsOff, newPointCount * 12);
    const alignedBuf = new ArrayBuffer(newPointCount * 12);
    new Uint8Array(alignedBuf).set(rawBytes);
    const framePoints = new Float32Array(alignedBuf);

    try {
      await addPoints(framePoints);
      onScanFrame(framePoints, data);
      state.hud.pts.textContent = state.mapStore.length.toLocaleString();
    } catch (e) {
      log('Point frame paused: ' + e.message, 'err');
      toast('Backup required before continuing scan', 'warn', 2500);
      return;
    }

    if (!state.firstPointsReceived) {
      state.firstPointsReceived = true;
      toast(`First ${newPointCount} points received!`, 'success');
      log(`First ${newPointCount} binary points!`, 'ok');
    }
  }
}

function setStatus(status, text) {
  state.hud.statusDot.className = 'status-dot';
  state.hud.statusTxt.className = 'status-text ' + status;

  if (status === 'connected') {
    state.hud.statusDot.classList.add('connected');
  } else if (status === 'connecting') {
    state.hud.statusDot.classList.add('connecting');
  }

  state.hud.statusTxt.textContent = text;
  const link = document.getElementById('operator-link');
  if (link) link.textContent = status === 'connected' ? 'ONLINE' : status.toUpperCase();
}

function setScanState(scanning) {
  if (scanning) {
    state.hud.scanBadge.className = 'scan-badge on';
    state.hud.scanBadgeText.textContent = 'SCANNING';
  } else {
    state.hud.scanBadge.className = 'scan-badge off';
    state.hud.scanBadgeText.textContent = 'OFF';
  }
}
