import { state } from './state.js';

// Command arbitration lives here so every velocity command from the viewer
// passes through the same ownership/priority policy before reaching the robot.
const COMMAND_PRIORITY = Object.freeze({
  safety: 100,
  manual: 90,
  terminal_target: 80,
  autopilot: 50,
  unknown: 10,
});

let localCommandOwner = null;
let localCommandOwnerUntil = 0;

function normalizedSource(source) {
  const value = String(source || 'unknown').trim().toLowerCase();
  return Object.prototype.hasOwnProperty.call(COMMAND_PRIORITY, value) ? value : 'unknown';
}

function canIssueVelocity(source, now = performance.now()) {
  source = normalizedSource(source);
  if (!localCommandOwner || now >= localCommandOwnerUntil || localCommandOwner === source) {
    return true;
  }
  return COMMAND_PRIORITY[source] >= COMMAND_PRIORITY[localCommandOwner];
}

function claimLocalCommand(source, leaseMs, now = performance.now()) {
  source = normalizedSource(source);
  localCommandOwner = source;
  localCommandOwnerUntil = now + Math.max(150, Number(leaseMs) || 650);
}

function releaseLocalCommand(source) {
  source = normalizedSource(source);
  if (!localCommandOwner || localCommandOwner === source) {
    localCommandOwner = null;
    localCommandOwnerUntil = 0;
    return true;
  }
  return false;
}

export function getCommandPriority(source) {
  return COMMAND_PRIORITY[normalizedSource(source)];
}

export function getCommandArbitrationState() {
  const now = performance.now();
  if (localCommandOwner && now >= localCommandOwnerUntil) {
    localCommandOwner = null;
    localCommandOwnerUntil = 0;
  }
  return {
    owner: localCommandOwner,
    until: localCommandOwnerUntil,
  };
}

export function sendConfig(payload) {
  if (state.ws && state.ws.readyState === WebSocket.OPEN) {
    state.ws.send(JSON.stringify(payload));
    return true;
  }
  return false;
}


export function claimCommandOwnership(source = 'unknown', leaseMs = 1200) {
  source = normalizedSource(source);
  const now = performance.now();
  if (!canIssueVelocity(source, now)) return false;
  claimLocalCommand(source, leaseMs, now);
  return true;
}

export function sendVelocity(v, omega, source = 'unknown', leaseMs = 650) {
  source = normalizedSource(source);
  const now = performance.now();
  if (!canIssueVelocity(source, now)) return false;
  claimLocalCommand(source, leaseMs, now);
  return sendConfig({
    type: 'velocity', v, omega, source, lease_ms: Math.max(150, Number(leaseMs) || 650)
  });
}

export function sendStop(source = 'unknown') {
  source = normalizedSource(source);
  const now = performance.now();
  if (!canIssueVelocity(source, now)) return false;
  const sent = sendConfig({ type: 'stop', source, release: true });
  releaseLocalCommand(source);
  return sent;
}

export function sendScanControl(enabled) {
  return sendConfig({ type: 'scan_control', enabled: !!enabled });
}
