import { state } from './state.js';
import { API_URL, VERSION } from './config.js';
import {
  encodeLmap,
  decodeLmap,
  gzipCompress,
  gzipDecompress,
  toast
} from './utils.js';
import { resetPointCloud, refreshTravellingArea } from './point-cloud.js';
import { addTrajPoint, resetTrajectory } from './robot-viz.js';
import { mapStore } from './map-store.js';
import { resetTravellingArea } from './travelling-area.js';
import {
  clearSessionArchive,
  initializeSession,
  autoBackupNow
} from './session-backup.js';
import { resetAutopilotSessionAnchor, invalidateKnowledgeSeed } from './navigation.js';

function collectDom() {
  if (!state.dom.mapNameInput) {
    state.dom.mapNameInput = document.getElementById('map-name');
    state.dom.saveBtn = document.getElementById('save-btn');
    state.dom.loadBtn = document.getElementById('load-btn');
    state.dom.mapListEl = document.getElementById('map-list');
  }
  return state.dom;
}

export async function resetMap() {
  /*
   * A map reset means a NEW mapping session from the application's point of
   * view. The old auto-backup must therefore be deleted too. Otherwise a
   * later load/recovery can resurrect points collected before the reset.
   */
  if (state.sessionFilename) {
    await clearSessionArchive('map-reset');
  }

  resetPointCloud();
  resetTrajectory();
  resetAutopilotSessionAnchor();
  invalidateKnowledgeSeed();
  resetTravellingArea();
}

export function initMapManager() {
  const dom = collectDom();

  dom.saveBtn.addEventListener('click', saveCurrentMap);
  dom.loadBtn.addEventListener('click', refreshMapList);
}

async function saveCurrentMap() {
  const dom = collectDom();

  const name =
    dom.mapNameInput.value.trim() ||
    `map_${new Date().toISOString().slice(0,19).replace(/[:T]/g,'-')}`;

  if (!mapStore || mapStore.length === 0) {
    toast('No points to save! Start scanning first.', 'warn');
    return;
  }

  /*
   * IMPORTANT:
   * The session archive is the authoritative copy of the complete map.
   * The Three.js point buffer is only the current travelling-area render
   * window and may contain far fewer points than the real map.
   *
   * Before saving, make sure all points currently in mapStore are committed
   * to the session archive. Then ask the backend to create the user-named
   * map directly from that archive. This avoids copying millions of points
   * through the browser a second time and guarantees that points which have
   * already disappeared from the renderer are still included.
   */
  try {
    if (!state.sessionFilename) {
      await initializeSession();
    }

    await autoBackupNow('save-prep', false);

    const resp = await fetch(
      `${API_URL}/session/${encodeURIComponent(state.sessionFilename)}/save-as`,
      {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json'
        },
        body: JSON.stringify({ name })
      }
    );

    const result = await resp.json();

    if (!resp.ok || !result.success) {
      throw new Error(result.error || `HTTP ${resp.status}`);
    }

    toast(
      `Saved: ${result.filename} (${(result.point_count || mapStore.length).toLocaleString()} pts, ${(result.trajectory_points || state.trajPrimitive.length / 3).toLocaleString()} path pts)`,
      'success',
      5000
    );
    refreshMapList();
  } catch (e) {
    toast('Save error: ' + e.message, 'error');
  }
}

export async function refreshMapList() {
  const dom = collectDom();

  try {
    const resp = await fetch(API_URL);
    const maps = await resp.json();

    dom.mapListEl.innerHTML = '';

    if (maps.length === 0) {
      dom.mapListEl.innerHTML =
        '<div class="empty-state">No saved maps yet</div>';
      return;
    }

    maps.forEach(m => {
      const div = document.createElement('div');
      div.className = 'map-card';
      div.innerHTML = `
        <div class="map-card-header"><span class="map-name">${m.name}</span></div>
        <div class="map-meta">${m.point_count.toLocaleString()} pts · ${m.size_kb} KB · ${m.timestamp ? m.timestamp.slice(0,10) : 'N/A'}</div>
        <div class="map-actions">
          <button class="btn btn-secondary btn-small btn-load">Load</button>
          <button class="btn btn-danger btn-small btn-del">Delete</button>
        </div>
      `;

      div.querySelector('.btn-load').addEventListener('click', e => {
        e.stopPropagation();
        loadMap(m.filename);
      });

      div.querySelector('.btn-del').addEventListener('click', e => {
        e.stopPropagation();
        deleteMap(m.filename);
      });

      dom.mapListEl.appendChild(div);
    });
  } catch (e) {
    toast('Failed to refresh map list', 'error');
  }
}

export async function loadMap(filename) {
  const slider = document.getElementById('offset-slider');
  const offsetVal = document.getElementById('offset-value');
  const angleMinInput = document.getElementById('angle-min');
  const angleMaxInput = document.getElementById('angle-max');

  try {
    toast(`Loading ${filename}...`, 'info');

    const resp = await fetch(`${API_URL}/${filename}`);
    if (!resp.ok) {
      throw new Error(`HTTP ${resp.status}`);
    }

    const buf = await resp.arrayBuffer();

    const bytes = new Uint8Array(buf);
    const isGz =
      bytes.length >= 2 &&
      bytes[0] === 0x1f &&
      bytes[1] === 0x8b;

    const lmapBuf = isGz ? await gzipDecompress(buf) : buf;
    const { metadata, positions, trajectory } = decodeLmap(lmapBuf);

    await resetMap();

    if (metadata.saved_offset_deg !== undefined) {
      slider.value = metadata.saved_offset_deg;
      offsetVal.textContent = metadata.saved_offset_deg + '°';
      state.hud.hudOffset.textContent =
        Number(metadata.saved_offset_deg).toFixed(1) + '°';
    }

    if (metadata.saved_inv_x !== undefined) {
      document.getElementById('inv-x').checked = !!metadata.saved_inv_x;
      document.getElementById('inv-y').checked = !!metadata.saved_inv_y;
      document.getElementById('inv-z').checked = !!metadata.saved_inv_z;
      state.invX = !!metadata.saved_inv_x;
      state.invY = !!metadata.saved_inv_y;
      state.invZ = !!metadata.saved_inv_z;
    }

    if (metadata.saved_angle_min !== undefined) {
      angleMinInput.value = metadata.saved_angle_min;
      angleMaxInput.value = metadata.saved_angle_max;
      state.hud.hudAngleMin.textContent =
        Number(metadata.saved_angle_min).toFixed(0) + '°';
      state.hud.hudAngleMax.textContent =
        Number(metadata.saved_angle_max).toFixed(0) + '°';
    }

    toast(
      `Indexing ${metadata.total_points.toLocaleString()} points from the world origin...`,
      'info',
      3500
    );

    await mapStore.replace(positions);

    for (const p of trajectory) {
      addTrajPoint(p[0], p[1], p[2]);
    }
    // Preserve the loaded navigation history as the baseline for exploration
    // progress and outside-area recovery. Navigation will seed its persistent
    // exploration tiles from this trajectory on the next robot-pose update.
    state.explorationResumeData = Array.isArray(metadata.explored_tile_keys)
      ? metadata.explored_tile_keys
      : null;
    state.explorationResumePending =
      (Array.isArray(state.explorationResumeData) && state.explorationResumeData.length > 0) ||
      (Array.isArray(trajectory) && trajectory.length > 0);

    refreshTravellingArea();

    // A loaded map always starts at the stable world origin. It is not
    // automatically centered on the map bounding box.
    resetTravellingArea();

    // The loaded saved map is the new baseline for the live session too.
    // resetMap() cleared the old live archive, so commit the loaded points once
    // before new scans are appended. Otherwise Save-As would omit the loaded map.
    if (state.sessionFilename) {
      state.sessionPersistedPoints = 0;
      state.sessionTotalPoints = state.mapStore.length;
      state.sessionGeneration = 0;
      await autoBackupNow('load-baseline', false);
    }

    toast(
      `Loaded ${metadata.name}: ${metadata.total_points.toLocaleString()} pts. Area starts at world origin.`,
      'success',
      5000
    );
  } catch (e) {
    toast('Load error: ' + e.message, 'error');
  }
}

export async function deleteMap(filename) {
  if (!confirm(`Delete "${filename}"?`)) return;

  try {
    const resp = await fetch(`${API_URL}/${filename}`, { method: 'DELETE' });
    const result = await resp.json();

    if (result.success) {
      toast(`Deleted ${filename}`, 'info');
      refreshMapList();
    } else {
      toast('Delete failed', 'error');
    }
  } catch (e) {
    toast('Delete error: ' + e.message, 'error');
  }
}
