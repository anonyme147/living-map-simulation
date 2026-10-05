import * as THREE from 'three';
import { state } from './state.js';
import { LMAP_MAGIC } from './config.js';

export function enuToThree(x, y, z) {
  return new THREE.Vector3(x, z, -y);
}

export function heightColor(pz) {
  if (pz < -0.5)      return [0.63, 0.13, 0.94];
  if (pz < 0.0)       return [0.50, 0.00, 0.50];
  if (pz < 0.05)      return [0.31, 0.00, 0.00];
  if (pz < 0.10)      return [0.80, 0.00, 0.00];
  if (pz < 0.15)      return [1.00, 0.20, 0.00];
  if (pz < 0.20)      return [1.00, 0.40, 0.00];
  if (pz < 0.50)      return [1.00, 1.00, 0.00];
  if (pz < 1.00)      return [0.00, 1.00, 0.00];
  return [0.00, 0.53, 1.00];
}

export function clampAngle(v) {
  let n = parseFloat(v);
  if (Number.isNaN(n)) return 0;
  return Math.max(0, Math.min(360, n));
}

export function toast(msg, type = 'info', duration = 3000) {
  const container = document.getElementById('toast-container');
  const div = document.createElement('div');
  div.className = `toast ${type}`;
  const icons = { success: '✓', error: '✕', info: 'ℹ', warn: '⚠' };
  div.innerHTML = `<span class="toast-icon">${icons[type] || 'ℹ'}</span><span>${msg}</span>`;
  container.appendChild(div);
  setTimeout(() => div.remove(), duration + 300);
}

export function log(msg, type = 'info') {
  const div = document.createElement('div');
  div.className = 'log-entry ' + type;
  div.textContent = `[${new Date().toLocaleTimeString('en-GB', {hour12:false})}] ${msg}`;
  const targets = [document.getElementById('debug-log'), document.getElementById('debug-stream')].filter(Boolean);
  for (const logEl of targets) {
    logEl.appendChild(div.cloneNode(true));
    logEl.scrollTop = logEl.scrollHeight;
    while (logEl.children.length > 40) logEl.removeChild(logEl.firstChild);
  }
}

export async function gzipCompress(arrayBuffer) {
  const cs = new CompressionStream('gzip');
  const writer = cs.writable.getWriter();
  writer.write(new Uint8Array(arrayBuffer));
  writer.close();

  const chunks = [];
  const reader = cs.readable.getReader();
  while (true) {
    const { done, value } = await reader.read();
    if (done) break;
    chunks.push(value);
  }

  let total = 0;
  for (const c of chunks) total += c.length;
  const out = new Uint8Array(total);
  let pos = 0;
  for (const c of chunks) {
    out.set(c, pos);
    pos += c.length;
  }
  return out.buffer;
}

export async function gzipDecompress(arrayBuffer) {
  const cs = new DecompressionStream('gzip');
  const writer = cs.writable.getWriter();
  writer.write(new Uint8Array(arrayBuffer));
  writer.close();

  const chunks = [];
  const reader = cs.readable.getReader();
  while (true) {
    const { done, value } = await reader.read();
    if (done) break;
    chunks.push(value);
  }

  let total = 0;
  for (const c of chunks) total += c.length;
  const out = new Uint8Array(total);
  let pos = 0;
  for (const c of chunks) {
    out.set(c, pos);
    pos += c.length;
  }
  return out.buffer;
}

export function encodeLmap(metadata, positions, trajectory) {
  const metaBytes = new TextEncoder().encode(JSON.stringify(metadata));
  const pointCount = positions.length / 3;
  const trajCount = trajectory.length;

  const headerSize = 4 + 1 + 4 + 4 + 4 + 4 + 4;
  const metaSize = metaBytes.length;
  const posSize = positions.length * 4;
  const trajSize = trajCount * 3 * 4;

  const buf = new ArrayBuffer(headerSize + metaSize + posSize + trajSize);
  const view = new DataView(buf);
  let off = 0;

  view.setUint32(off, LMAP_MAGIC, true); off += 4;
  view.setUint8(off, 1); off += 1;
  view.setUint32(off, 0, true); off += 4;
  view.setUint32(off, pointCount, true); off += 4;
  view.setUint32(off, trajCount, true); off += 4;
  view.setUint32(off, metaSize, true); off += 4;
  view.setUint32(off, 0, true); off += 4;

  new Uint8Array(buf, off, metaSize).set(metaBytes);
  off += metaSize;

  // Use byte copies rather than constructing a Float32Array directly on
  // `buf`: metadata length is not guaranteed to be 4-byte aligned.
  new Uint8Array(buf, off, posSize).set(
    new Uint8Array(positions.buffer, positions.byteOffset, posSize)
  );
  off += posSize;

  const trajValues = new Float32Array(trajCount * 3);
  for (let i = 0; i < trajCount; i++) {
    trajValues[i * 3] = trajectory[i][0];
    trajValues[i * 3 + 1] = trajectory[i][1];
    trajValues[i * 3 + 2] = trajectory[i][2];
  }

  new Uint8Array(buf, off, trajSize).set(
    new Uint8Array(
      trajValues.buffer,
      trajValues.byteOffset,
      trajSize
    )
  );

  return buf;
}

function decodeOneLmap(arrayBuffer, startOffset = 0) {
  const view = new DataView(arrayBuffer);
  let off = startOffset;

  if (off + 25 > view.byteLength) {
    throw new Error('Truncated LMAP header');
  }

  const magic = view.getUint32(off, true); off += 4;
  const LEGACY_LMAP_MAGIC = 0x4D50414C; // old v9.3 JS encoder wrote 'LAPM'
  if (magic !== LMAP_MAGIC && magic !== LEGACY_LMAP_MAGIC) {
    throw new Error('Invalid LMAP magic');
  }

  const version = view.getUint8(off); off += 1;
  if (version !== 1) throw new Error('Unsupported LMAP version ' + version);

  view.getUint32(off, true); off += 4; // flags
  const pointCount = view.getUint32(off, true); off += 4;
  const trajCount = view.getUint32(off, true); off += 4;
  const metaLen = view.getUint32(off, true); off += 4;
  off += 4; // reserved

  if (
    metaLen < 0 ||
    off + metaLen > view.byteLength
  ) {
    throw new Error('Invalid LMAP metadata length');
  }

  const metaBytes = new Uint8Array(arrayBuffer, off, metaLen);
  const metadata = JSON.parse(new TextDecoder().decode(metaBytes));
  off += metaLen;

  const posBytes = pointCount * 3 * 4;
  const trajBytes = trajCount * 3 * 4;
  const end = off + posBytes + trajBytes;

  if (end > view.byteLength) {
    throw new Error('Truncated LMAP point/trajectory data');
  }

  // Copy into aligned standalone buffers so concatenated LMAP chunks work
  // regardless of the metadata length or chunk offset.
  const positions = new Float32Array(
    arrayBuffer.slice(off, off + posBytes)
  );
  off += posBytes;

  const trajectory = [];
  const trajView = new Float32Array(
    arrayBuffer.slice(off, off + trajBytes)
  );

  for (let i = 0; i < trajCount; i++) {
    trajectory.push([
      trajView[i * 3],
      trajView[i * 3 + 1],
      trajView[i * 3 + 2]
    ]);
  }

  return {
    metadata,
    positions,
    trajectory,
    nextOffset: end
  };
}

export function decodeLmap(arrayBuffer) {
  const first = decodeOneLmap(arrayBuffer, 0);

  // Normal .lmap files contain exactly one block.
  if (first.nextOffset === arrayBuffer.byteLength) {
    return {
      metadata: first.metadata,
      positions: first.positions,
      trajectory: first.trajectory
    };
  }

  /*
   * Session auto-backups are stored as concatenated gzip members. After
   * decompression this becomes a sequence of LMAP blocks.
   *
   * Multiple snapshots of the same generation overlap. The newest snapshot
   * for each generation is therefore the authoritative one for that
   * generation. The newest trajectory is global and is used once.
   */
  const chunks = [first];
  let off = first.nextOffset;

  while (off < arrayBuffer.byteLength) {
    const chunk = decodeOneLmap(arrayBuffer, off);
    chunks.push(chunk);
    off = chunk.nextOffset;
  }

  const isSession = chunks.some(
    c => c.metadata && c.metadata.session_id
  );

  if (!isSession) {
    throw new Error('Unexpected concatenated LMAP data');
  }

  const latestByGeneration = new Map();

  for (const chunk of chunks) {
    const m = chunk.metadata;
    const generation = Number.isInteger(m.generation)
      ? m.generation
      : 0;

    const old = latestByGeneration.get(generation);
    if (!old || (m.snapshot_id || 0) > (old.metadata.snapshot_id || 0)) {
      latestByGeneration.set(generation, chunk);
    }
  }

  const ordered = [...latestByGeneration.entries()]
    .sort((a, b) => a[0] - b[0])
    .map(([, chunk]) => chunk);

  let totalFloats = 0;
  for (const chunk of ordered) {
    totalFloats += chunk.positions.length;
  }

  const positions = new Float32Array(totalFloats);
  let pos = 0;

  for (const chunk of ordered) {
    positions.set(chunk.positions, pos);
    pos += chunk.positions.length;
  }

  const newest = chunks.reduce(
    (a, b) =>
      (b.metadata.snapshot_id || 0) >
      (a.metadata.snapshot_id || 0) ? b : a
  );

  return {
    metadata: {
      ...newest.metadata,
      name: newest.metadata.session_id || newest.metadata.name,
      total_points: positions.length / 3,
      total_traj_points: newest.trajectory.length,
      session_archive: true,
      generations: ordered.length
    },
    positions,
    trajectory: newest.trajectory
  };
}
