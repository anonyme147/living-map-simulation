import { state } from './state.js';

function decodeBase64(text) {
  const raw = atob(text);
  const out = new Uint8Array(raw.length);
  for (let i = 0; i < raw.length; i++) out[i] = raw.charCodeAt(i);
  return out;
}

function ensureCanvas() {
  if (state.navMapCanvas) return;
  state.navMapCanvas = document.getElementById('nav-map-canvas');
  state.navMapCtx = state.navMapCanvas?.getContext('2d');
}

export function refreshNavMap() {
  drawNavMap();
}

export function setNavMap(data) {
  ensureCanvas();
  if (!state.navMapCtx) return;

  const cells = decodeBase64(data.data);
  state.navMap = {
    width: data.width,
    height: data.height,
    resolution: data.resolution,
    originX: data.origin_x,
    originY: data.origin_y,
    cells,
  };
  drawNavMap();
}

export function setNavState(data) {
  // The full TILES map has its own persistent UI state. Receiving an Auto
  // Pilot OFF navigation update must not hide or reset that panel.
  const tileMapWasVisible = !!state.explorationMapVisible;
  state.autoPilot = !!data.autopilot;
  state.navMode = data.mode || 'IDLE';
  state.navCoverage = Number(data.coverage || 0);
  state.navGoal = data.goal || null;
  state.navPathLength = Number(data.path_length || 0);
  state.navPath = Array.isArray(data.path) ? data.path : [];
  state.navArea = data.area || null;

  state.explorationMapVisible = tileMapWasVisible;
  updateNavUI();
  if (state.navMapVisible) drawNavMap();
  if (state.explorationMapVisible) drawExplorationMap();
}

function updateNavUI() {
  const badge = document.getElementById('autopilot-badge');
  if (badge) {
    badge.classList.toggle('active', state.autoPilot);
    badge.textContent = state.autoPilot
      ? `AUTO PILOT — ${state.navMode}`
      : 'AUTO PILOT OFF';
  }

  const mode = document.getElementById('nav-mode');
  const cov = document.getElementById('nav-coverage');
  const goal = document.getElementById('nav-goal');
  if (mode) mode.textContent = state.navMode;
  if (cov) cov.textContent = `${(state.navCoverage * 100).toFixed(1)}%`;
  if (goal) {
    goal.textContent = state.navGoal
      ? `${state.navGoal[0].toFixed(2)}, ${state.navGoal[1].toFixed(2)}`
      : '—';
  }
}

export function toggleNavMap(force) {
  ensureCanvas();
  state.navMapVisible = force === undefined ? !state.navMapVisible : !!force;

  const panel = document.getElementById('nav-map-panel');
  if (panel) panel.classList.toggle('show', state.navMapVisible);

  if (state.navMapVisible) {
    drawNavMap();
    refreshNavMap();
  }
}

function worldToCanvas(x, y, map, width, height) {
  return {
    x: (x - map.originX) / map.resolution * (width / map.width),
    y: height - (y - map.originY) / map.resolution * (height / map.height),
  };
}

function drawArea(ctx, width, height) {
  const area = state.navArea;
  const map = state.navMap;
  if (!area || !map || map.local) return;

  const px = x => (x - map.originX) / map.resolution * (width / map.width);
  const py = y => height - (y - map.originY) / map.resolution * (height / map.height);

  ctx.save();
  ctx.lineWidth = 3;
  ctx.setLineDash([8, 5]);
  ctx.strokeStyle = '#00e5ff';

  if (area.type === 'circle' && area.circle) {
    const cx = px(area.circle.center_x ?? area.circle.cx ?? 0);
    const cy = py(area.circle.center_y ?? area.circle.cy ?? 0);
    const r = (area.circle.radius_m ?? area.circle.radius) / map.resolution * (width / map.width);
    ctx.beginPath();
    ctx.arc(cx, cy, r, 0, Math.PI * 2);
    ctx.stroke();

    const safe = Math.max(0, (area.circle.radius_m ?? area.circle.radius) - (area.safety_margin ?? 0));
    const sr = safe / map.resolution * (width / map.width);
    ctx.setLineDash([3, 4]);
    ctx.strokeStyle = '#ffd166';
    ctx.beginPath();
    ctx.arc(cx, cy, sr, 0, Math.PI * 2);
    ctx.stroke();
  } else {
    ctx.strokeRect(
      px(area.min_x),
      py(area.max_y),
      (area.max_x - area.min_x) / map.resolution * (width / map.width),
      (area.max_y - area.min_y) / map.resolution * (height / map.height)
    );

    const m = area.safety_margin || 0;
    ctx.setLineDash([3, 4]);
    ctx.strokeStyle = '#ffd166';
    ctx.strokeRect(
      px(area.min_x + m),
      py(area.max_y - m),
      (area.max_x - area.min_x - 2 * m) / map.resolution * (width / map.width),
      (area.max_y - area.min_y - 2 * m) / map.resolution * (height / map.height)
    );
  }
  ctx.restore();
}

function drawNavMap() {
  ensureCanvas();
  const canvas = state.navMapCanvas;
  const ctx = state.navMapCtx;
  const map = state.navMap;
  if (!canvas || !ctx) return;
  if (!map) {
    const rect = canvas.getBoundingClientRect();
    const width = Math.max(300, Math.floor(rect.width || 720));
    const height = Math.max(300, Math.floor(rect.height || 520));
    const dpr = window.devicePixelRatio || 1;
    canvas.width = width * dpr;
    canvas.height = height * dpr;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.fillStyle = '#071018';
    ctx.fillRect(0, 0, width, height);
    ctx.fillStyle = '#83909c';
    ctx.font = '14px sans-serif';
    ctx.textAlign = 'center';
    ctx.fillText('Waiting for LiDAR cloud…', width / 2, height / 2);
    return;
  }

  const rect = canvas.getBoundingClientRect();
  const width = Math.max(300, Math.floor(rect.width || 720));
  const height = Math.max(300, Math.floor(rect.height || 520));
  const dpr = window.devicePixelRatio || 1;
  canvas.width = width * dpr;
  canvas.height = height * dpr;
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);

  ctx.fillStyle = '#071018';
  ctx.fillRect(0, 0, width, height);

  const sx = width / map.width;
  const sy = height / map.height;
  const image = ctx.createImageData(width, height);

  // Draw cells scaled to the viewport. Nearest-neighbour is intentional.
  for (let py = 0; py < height; py++) {
    const cy = Math.min(map.height - 1, Math.floor((height - 1 - py) / sy));
    for (let px = 0; px < width; px++) {
      const cx = Math.min(map.width - 1, Math.floor(px / sx));
      const v = map.cells[cy * map.width + cx] || 0;
      const off = (py * width + px) * 4;

      if (v === 1) {
        image.data[off] = 33;
        image.data[off + 1] = 150;
        image.data[off + 2] = 150;
        image.data[off + 3] = 170;
      } else if (v === 2) {
        image.data[off] = 235;
        image.data[off + 1] = 70;
        image.data[off + 2] = 75;
        image.data[off + 3] = 230;
      } else if (v === 3) {
        image.data[off] = 245;
        image.data[off + 1] = 180;
        image.data[off + 2] = 55;
        image.data[off + 3] = 235;
      } else {
        image.data[off] = 7;
        image.data[off + 1] = 16;
        image.data[off + 2] = 24;
        image.data[off + 3] = 255;
      }
    }
  }
  ctx.putImageData(image, 0, 0);

  // The local map is robot-centred: the center of the canvas is the robot.
  if (map.local) {
    ctx.save();
    const cx = width / 2, cy = height / 2;
    ctx.strokeStyle = '#00f0c8';
    ctx.lineWidth = 2;
    ctx.beginPath(); ctx.moveTo(cx - 10, cy); ctx.lineTo(cx + 10, cy); ctx.moveTo(cx, cy - 10); ctx.lineTo(cx, cy + 10); ctx.stroke();
    ctx.fillStyle = '#00f0c8'; ctx.beginPath(); ctx.arc(cx, cy, 4, 0, Math.PI*2); ctx.fill();
    ctx.fillStyle = '#9fb0bd'; ctx.font = '11px sans-serif'; ctx.textAlign='left'; ctx.fillText(`LOCAL ${map.radius.toFixed(1)} m radius · ${map.resolution.toFixed(2)} m/cell · GT pose · ${Number(map.source_points || 0).toLocaleString()} cloud pts`, 12, 18);
    ctx.restore();
  }

  if (!map.local) drawArea(ctx, width, height);

  // A* path.
  if (!map.local && Array.isArray(state.navPath) && state.navPath.length > 1) {
    ctx.save();
    ctx.strokeStyle = '#ffe66d';
    ctx.lineWidth = 2;
    ctx.beginPath();
    state.navPath.forEach((p, i) => {
      const q = worldToCanvas(p[0], p[1], map, width, height);
      if (i === 0) ctx.moveTo(q.x, q.y);
      else ctx.lineTo(q.x, q.y);
    });
    ctx.stroke();
    ctx.restore();
  }

  // Tile/exploration map is a WORLD-coordinate view. The robot is drawn at
  // its GT world position; it is never recentered on the canvas.
  if (!map.local && state.navPose) {
    const q = worldToCanvas(
      state.navPose.x, state.navPose.y, map, width, height
    );
    ctx.save();
    ctx.translate(q.x, q.y);
    ctx.rotate(-state.navPose.theta);
    ctx.fillStyle = '#00f0c8';
    ctx.beginPath();
    ctx.moveTo(13, 0);
    ctx.lineTo(-9, -8);
    ctx.lineTo(-9, 8);
    ctx.closePath();
    ctx.fill();
    ctx.restore();
  }

  if (!map.local && state.navGoal) {
    const q = worldToCanvas(
      state.navGoal[0], state.navGoal[1], map, width, height
    );
    ctx.save();
    ctx.strokeStyle = '#ff7b72';
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.arc(q.x, q.y, 7, 0, Math.PI * 2);
    ctx.stroke();
    ctx.restore();
  }
}


function ensureExplorationCanvas() {
  if (state.explorationMapCanvas) return true;
  state.explorationMapCanvas = document.getElementById('exploration-map-canvas');
  state.explorationMapCtx = state.explorationMapCanvas?.getContext('2d');
  return !!state.explorationMapCtx;
}

function tileStateColor(tile) {
  if (tile?.state === 'discovered') return 'rgba(55, 190, 95, 0.58)';
  if (tile?.state === 'blocked') return 'rgba(220, 55, 55, 0.62)';
  return 'rgba(145, 145, 145, 0.48)';
}

function drawExplorationMap() {
  if (!ensureExplorationCanvas()) return;
  const canvas = state.explorationMapCanvas;
  const ctx = state.explorationMapCtx;
  const map = state.explorationMap;
  if (!map) return;
  const rect = canvas.getBoundingClientRect();
  const width = Math.max(420, Math.floor(rect.width || 900));
  const height = Math.max(420, Math.floor(rect.height || 650));
  const dpr = window.devicePixelRatio || 1;
  canvas.width = width * dpr; canvas.height = height * dpr;
  ctx.setTransform(dpr,0,0,dpr,0,0);
  ctx.fillStyle = '#071018'; ctx.fillRect(0,0,width,height);

  const a = map.area;
  const minX = a.type === 'circle' ? a.center_x - a.radius_m : a.min_x;
  const maxX = a.type === 'circle' ? a.center_x + a.radius_m : a.max_x;
  const minY = a.type === 'circle' ? a.center_y - a.radius_m : a.min_y;
  const maxY = a.type === 'circle' ? a.center_y + a.radius_m : a.max_y;
  const scale = Math.min(width / Math.max(1e-6,maxX-minX), height / Math.max(1e-6,maxY-minY));
  const ox = (width - (maxX-minX)*scale)/2 - minX*scale;
  const oy = (height + (maxY-minY)*scale)/2 + minY*scale;
  const px = x => ox + x*scale;
  const py = y => oy - y*scale;

  // Draw the global occupancy/knowledge map without copying its arrays.
  const sx = width / Math.max(1,map.width), sy = height / Math.max(1,map.height);
  ctx.save();
  for (let y=0;y<map.height;y++) {
    for (let x=0;x<map.width;x++) {
      const i=y*map.width+x;
      if (!map.seen[i]) continue;
      const wx=map.originX+(x+0.5)*map.resolution;
      const wy=map.originY+(y+0.5)*map.resolution;
      if (a.type==='circle' && (wx-a.center_x)**2+(wy-a.center_y)**2>a.radius_m*a.radius_m) continue;
      const v=map.cells[i];
      ctx.fillStyle = v===2 ? 'rgba(235,70,75,0.78)' : v===1 ? 'rgba(65,165,165,0.62)' : 'rgba(180,170,80,0.5)';
      const cw=Math.max(1,map.resolution*scale+0.25);
      ctx.fillRect(px(wx-map.resolution/2), py(wy+map.resolution/2), cw, cw);
    }
  }
  ctx.restore();

  // Tile state overlay: gray = incomplete/available, red = unreachable,
  // green = discovered. The occupancy layer remains visible underneath.
  const s=map.tileSize;
  for (const [key,tile] of (map.tiles instanceof Map ? map.tiles : [])) {
    const [ix,iy]=key.split(',').map(Number);
    const x0=ix*s, y0=iy*s;
    const cx=x0+s/2, cy=y0+s/2;
    if (a.type==='circle' && (cx-a.center_x)**2+(cy-a.center_y)**2>a.radius_m*a.radius_m) continue;
    ctx.fillStyle=tileStateColor(tile);
    ctx.fillRect(px(x0), py(y0+s), s*scale, s*scale);
    ctx.strokeStyle='rgba(220,230,235,0.28)'; ctx.lineWidth=1;
    ctx.strokeRect(px(x0), py(y0+s), s*scale, s*scale);
    if (s*scale>42) {
      ctx.fillStyle='#ffffff'; ctx.font='10px sans-serif'; ctx.textAlign='center';
      ctx.fillText(`${(Number(tile.coverage||0)*100).toFixed(0)}%`, px(cx), py(cy)+3);
    }
  }

  // Selected navigation boundary.
  ctx.save(); ctx.strokeStyle='#00e5ff'; ctx.lineWidth=3;
  if (a.type==='circle') { ctx.beginPath(); ctx.arc(px(a.center_x),py(a.center_y),a.radius_m*scale,0,Math.PI*2); ctx.stroke(); }
  else ctx.strokeRect(px(a.min_x),py(a.max_y),(a.max_x-a.min_x)*scale,(a.max_y-a.min_y)*scale);
  ctx.restore();

  if (state.navPose) {
    const qx=px(state.navPose.x), qy=py(state.navPose.y);
    // The tile map is a world-coordinate view, so show heading explicitly.
    // Without this, the robot is only a dot and a target on its right can be
    // mistaken for a target directly ahead.
    ctx.save();
    ctx.translate(qx, qy);
    ctx.rotate(-state.navPose.theta);
    ctx.fillStyle='#00f0c8';
    ctx.beginPath(); ctx.moveTo(11, 0); ctx.lineTo(-7, -6); ctx.lineTo(-7, 6); ctx.closePath(); ctx.fill();
    ctx.restore();
  }
  if (state.navGoal) {
    const gx = px(state.navGoal[0]), gy = py(state.navGoal[1]);
    if (state.navPose) {
      ctx.save();
      ctx.strokeStyle='rgba(255,123,114,0.65)'; ctx.lineWidth=1.5; ctx.setLineDash([5,4]);
      ctx.beginPath(); ctx.moveTo(px(state.navPose.x), py(state.navPose.y)); ctx.lineTo(gx, gy); ctx.stroke();
      ctx.restore();
    }
    ctx.save(); ctx.strokeStyle='#ff7b72'; ctx.lineWidth=3; ctx.beginPath(); ctx.arc(gx,gy,7,0,Math.PI*2); ctx.stroke(); ctx.restore();
  }

  ctx.fillStyle='#d9e3ea'; ctx.font='12px sans-serif'; ctx.textAlign='left';
  ctx.fillText(`TILES · WORLD · ${map.tileSize.toFixed(2)} m · GT start anchor · threshold ${(map.tileCoverageCompleteRate*100).toFixed(0)}%`,12,18);
}

export function toggleExplorationMap(force) {
  state.explorationMapVisible = force === undefined ? !state.explorationMapVisible : !!force;
  const panel=document.getElementById('exploration-map-panel');
  if (panel) panel.classList.toggle('show',state.explorationMapVisible);
  if (state.explorationMapVisible) drawExplorationMap();
}

export function refreshExplorationMap() {
  if (state.explorationMapVisible) drawExplorationMap();
}

window.addEventListener('resize', () => {
  if (state.navMapVisible) drawNavMap();
  if (state.explorationMapVisible) drawExplorationMap();
});
