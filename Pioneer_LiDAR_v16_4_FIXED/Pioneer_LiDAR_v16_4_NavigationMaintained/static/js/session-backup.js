import { state } from './state.js';
import {
  API_URL,
  VERSION,
  AUTO_BACKUP_INTERVAL_MS,
  runtimeConfig
} from './config.js';
import {
  encodeLmap,
  gzipCompress,
  toast,
  log
} from './utils.js';
import { mapStore } from './map-store.js';
import { getExplorationPersistence } from './navigation.js';

function pad(n) {
  return String(n).padStart(2, '0');
}

export function makeSessionId(date = new Date()) {
  return `session_${date.getFullYear()}_${pad(date.getMonth() + 1)}_${pad(date.getDate())}_${pad(date.getHours())}_${pad(date.getMinutes())}`;
}

function currentSettings() {
  const slider = document.getElementById('offset-slider');
  const angleMinInput = document.getElementById('angle-min');
  const angleMaxInput = document.getElementById('angle-max');

  return {
    saved_offset_deg: slider ? parseFloat(slider.value) || 0 : 0,
    saved_inv_x: state.invX,
    saved_inv_y: state.invY,
    saved_inv_z: state.invZ,
    saved_angle_min: angleMinInput ? parseFloat(angleMinInput.value) || 0 : 0,
    saved_angle_max: angleMaxInput ? parseFloat(angleMaxInput.value) || 360 : 360
  };
}

function snapshotMetadata(reason, start, end, generation) {
  state.sessionLastSnapshotId += 1;

  return {
    name: state.sessionId,
    timestamp: new Date().toISOString(),
    version: VERSION,
    session_id: state.sessionId,
    generation,
    snapshot_id: state.sessionLastSnapshotId,
    backup_reason: reason,

    // Each generation is now an immutable point segment. This prevents
    // periodic backups from duplicating millions of already archived points.
    start_sequence: start,
    end_sequence: end,
    total_points: end - start,

    // The trajectory is global and is intentionally repeated in snapshots.
    total_traj_points: state.trajPrimitive.length,
    trajectory_type: 'primitive_lines',
    trajectory_format: 'xyz_vertices_connected_in_order',
    // Persist exploration tiles independently from the raw trajectory so
    // resumed progress reflects actual exploration state.
    explored_tile_keys: getExplorationPersistence(),

    ...currentSettings()
  };
}

async function performSnapshot(reason, showToast = false) {
  if (!state.sessionFilename || !mapStore) {
    return false;
  }

  const start = state.sessionPersistedPoints;
  const end = mapStore.length;

  if (end <= start) {
    return true;
  }

  const generation = state.sessionGeneration;
  const metadata = snapshotMetadata(reason, start, end, generation);
  const positions = mapStore.getRange(start, end);

  const lmap = encodeLmap(
    metadata,
    positions,
    state.trajPrimitive
  );
  const compressed = await gzipCompress(lmap);

  const resp = await fetch(
    `${API_URL}/session/${encodeURIComponent(state.sessionFilename)}`,
    {
      method: 'POST',
      headers: {
        'Content-Type': 'application/octet-stream'
      },
      body: compressed
    }
  );

  if (!resp.ok) {
    throw new Error(`Session backup HTTP ${resp.status}`);
  }

  const result = await resp.json();
  if (!result.success) {
    throw new Error(result.error || 'Session backup failed');
  }

  state.sessionPersistedPoints = end;
  state.sessionGeneration += 1;
  state.sessionLastBackupAt = Date.now();

  if (showToast) {
    toast('Session auto-saved', 'success');
  }

  log(
    `Session backup: ${reason}, ${positions.length / 3}` +
    ` new pts, generation ${generation}`,
    'ok'
  );

  return true;
}

/*
 * Serialize all backup operations. A capacity backup can therefore never
 * race with the 60-second backup.
 */
export function queueSessionBackup(reason, showToast = false) {
  const run = state.sessionBackupPromise
    .catch(() => false)
    .then(() => performSnapshot(reason, showToast));

  state.sessionBackupPromise = run.catch(err => {
    log(`Session backup failed: ${err.message}`, 'err');
    throw err;
  });

  return run;
}

export async function initializeSession() {
  if (state.sessionFilename) return;

  state.sessionId = makeSessionId();
  state.sessionFilename = `${state.sessionId}.lmap.gz`;

  // If the user loaded an existing map before the WebSocket connected, that
  // map is already persistent and should not be duplicated into the session.
  state.sessionPersistedPoints = mapStore ? mapStore.length : 0;
  state.sessionTotalPoints = mapStore ? mapStore.length : 0;
  state.sessionGeneration = 0;
  state.sessionGenerationStart = state.sessionPersistedPoints;
  state.sessionLastSnapshotId = 0;

  log(`Session started: ${state.sessionFilename}`, 'ok');

  const backupInterval = Number(runtimeConfig.backup?.interval_ms || AUTO_BACKUP_INTERVAL_MS);

  if (state.sessionBackupTimer) {
    clearInterval(state.sessionBackupTimer);
  }

  state.sessionBackupTimer = setInterval(() => {
    queueSessionBackup('time', true).catch(() => {});
  }, backupInterval);
}

export function notePointsAdded(count) {
  state.sessionTotalPoints += count;
}

export function beginNextGeneration() {
  // Compatibility shim. The new storage model creates generations on
  // successful backups rather than clearing the render buffer.
  state.sessionGenerationStart = state.sessionPersistedPoints;
}

export async function autoBackupNow(reason = 'manual', showToast = false) {
  if (!state.sessionFilename) return false;
  return queueSessionBackup(reason, showToast);
}


export async function clearSessionArchive(reason = 'reset') {
  // Wait for any backup already in flight. A reset must never delete the
  // archive while an older POST is still writing to it.
  try {
    await state.sessionBackupPromise;
  } catch (_) {
    // The failed backup is irrelevant once the archive is being cleared.
  }

  if (state.sessionFilename) {
    const filename = state.sessionFilename;

    const resp = await fetch(
      `${API_URL}/session/${encodeURIComponent(filename)}`,
      { method: 'DELETE' }
    );

    if (!resp.ok && resp.status !== 404) {
      let detail = '';
      try {
        const data = await resp.json();
        detail = data.error ? `: ${data.error}` : '';
      } catch (_) {}
      throw new Error(`Session archive clear HTTP ${resp.status}${detail}`);
    }

    log(`Session archive cleared: ${filename} (${reason})`, 'ok');
  }

  // Keep the current session identity so the next points are written into a
  // fresh archive with the same filename, but start its snapshot history
  // from zero.
  state.sessionPersistedPoints = 0;
  state.sessionTotalPoints = 0;
  state.sessionGeneration = 0;
  state.sessionGenerationStart = 0;
  state.sessionLastSnapshotId = 0;
  state.sessionLastBackupAt = 0;

  return true;
}

export async function backupBeforeCapacityReset() {
  // Compatibility API for the previous ring-buffer implementation. The
  // complete map is never cleared when the renderer reaches its limit.
  return autoBackupNow('capacity', false);
}
