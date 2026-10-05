import {
  TRAVELLING_CELL_SIZE,
  runtimeConfig
} from './config.js';

/*
 * Complete map storage.
 *
 * This is deliberately independent from the Three.js render buffer. A map
 * can therefore contain millions of points while the renderer only receives
 * the current travelling-area subset.
 */
class CompleteMapStore {
  constructor() {
    this.length = 0;
    this.capacity = 0;
    this.positions = new Float32Array(0);
    this.index = new Map();
  }

  clear() {
    this.length = 0;
    this.capacity = 0;
    this.positions = new Float32Array(0);
    this.index.clear();
  }

  _ensureCapacity(pointCount) {
    const required = pointCount * 3;
    if (required <= this.positions.length) return;

    let nextCapacity = Math.max(262144, this.capacity || 262144);
    while (nextCapacity < pointCount) {
      nextCapacity = Math.ceil(nextCapacity * 1.5);
    }

    const next = new Float32Array(nextCapacity * 3);
    next.set(this.positions.subarray(0, this.length * 3));
    this.positions = next;
    this.capacity = nextCapacity;
  }

  _cellKey(x, z) {
    const cellSize = Number(runtimeConfig.viewer?.cell_size_m || TRAVELLING_CELL_SIZE);
    const ix = Math.floor(x / cellSize);
    const iz = Math.floor(z / cellSize);
    return `${ix},${iz}`;
  }

  _addToIndex(id) {
    const off = id * 3;
    // The travelling plane is the Three.js X/Z plane. In ENU source data
    // that corresponds to X and -Y; source Z is vertical.
    const key = this._cellKey(
      this.positions[off],
      -this.positions[off + 1]
    );

    let bucket = this.index.get(key);
    if (!bucket) {
      bucket = [];
      this.index.set(key, bucket);
    }
    bucket.push(id);
  }

  append(positions) {
    if (!positions || positions.length === 0) return 0;
    if (positions.length % 3 !== 0) {
      throw new Error('Point positions must contain x/y/z triples');
    }

    const count = positions.length / 3;
    const start = this.length;

    this._ensureCapacity(this.length + count);
    this.positions.set(positions, this.length * 3);

    for (let i = 0; i < count; i++) {
      this._addToIndex(start + i);
    }

    this.length += count;
    return count;
  }

  async replace(positions, yieldEvery = 200_000) {
    if (!positions || positions.length % 3 !== 0) {
      throw new Error('Invalid map positions');
    }

    const count = positions.length / 3;
    this.length = count;
    this.capacity = count;
    this.positions = new Float32Array(positions);
    this.index.clear();

    for (let i = 0; i < count; i++) {
      this._addToIndex(i);
      if (i > 0 && i % yieldEvery === 0) {
        await new Promise(resolve => setTimeout(resolve, 0));
      }
    }
  }

  getAll() {
    return this.positions.slice(0, this.length * 3);
  }

  queryRadiusWorldXYFiltered(centerX, centerY, radius, voxelSize = 0.05, scoreFn = null) {
    const ids = this.queryRadiusWorldXY(centerX, centerY, radius);
    if (!ids.length) return [];
    const r = Math.max(0, Number(radius) || 0), vs = Math.max(0.01, Number(voxelSize) || 0.05), r2 = r*r;
    const buckets = new Map();
    for (const id of ids) {
      const off=id*3, x=this.positions[off], y=this.positions[off+1];
      const dx=x-centerX, dy=y-centerY; if (dx*dx+dy*dy>r2) continue;
      const key=`${Math.floor(x/vs)},${Math.floor(y/vs)}`;
      const score=scoreFn ? Number(scoreFn(id)) : this.positions[off+2];
      const prev=buckets.get(key); if (prev===undefined || score<prev.score) buckets.set(key,{id,score});
    }
    return Array.from(buckets.values(), e=>e.id);
  }

  queryRadiusWorldXY(centerX, centerY, radius) {
    const r = Math.max(0, Number(radius) || 0);
    const minX = centerX - r, maxX = centerX + r;
    const minY = centerY - r, maxY = centerY + r;
    const cellSize = Number(runtimeConfig.viewer?.cell_size_m || TRAVELLING_CELL_SIZE);
    const minCellX = Math.floor(minX / cellSize);
    const maxCellX = Math.floor(maxX / cellSize);
    // mapStore's second horizontal axis is stored as source Y and indexed as -Y
    // because Three.js uses X/Z for the floor plane.
    const minCellZ = Math.floor((-maxY) / cellSize);
    const maxCellZ = Math.floor((-minY) / cellSize);
    const r2 = r * r;
    const result = [];

    for (let cx = minCellX; cx <= maxCellX; cx++) {
      for (let cz = minCellZ; cz <= maxCellZ; cz++) {
        const bucket = this.index.get(`${cx},${cz}`);
        if (!bucket) continue;
        for (const id of bucket) {
          const off = id * 3;
          const x = this.positions[off];
          const y = this.positions[off + 1];
          const dx = x - centerX;
          const dy = y - centerY;
          if (dx * dx + dy * dy <= r2) result.push(id);
        }
      }
    }
    return result;
  }

  getRange(startPoint, endPoint) {
    const start = Math.max(0, Math.min(this.length, startPoint));
    const end = Math.max(start, Math.min(this.length, endPoint));
    return this.positions.slice(start * 3, end * 3);
  }

  getPoint(id, target) {
    const off = id * 3;
    target[0] = this.positions[off];
    target[1] = this.positions[off + 1];
    target[2] = this.positions[off + 2];
  }

  /*
   * Return point ids inside a square region.
   *
   * The first region is the visible square. The second region is the
   * surrounding ring: outer square minus visible square. If a region is
   * denser than its renderer budget, deterministic stride sampling keeps the
   * spatial window bounded without deleting any source points.
   */
  queryRegions(centerX, centerZ, visibleHalf, surroundingHalf,
               visibleLimit = Infinity, surroundingLimit = Infinity) {
    const visible = this._querySquare(
      centerX, centerZ, visibleHalf, visibleLimit, false, visibleHalf
    );

    const surrounding = this._querySquare(
      centerX, centerZ, surroundingHalf, surroundingLimit, true, visibleHalf
    );

    return { visible, surrounding };
  }

  _querySquare(centerX, centerZ, half, limit, ringOnly, innerHalf) {
    const minX = centerX - half;
    const maxX = centerX + half;
    const minZ = centerZ - half;
    const maxZ = centerZ + half;

    const cellSize = Number(runtimeConfig.viewer?.cell_size_m || TRAVELLING_CELL_SIZE);
    const minCellX = Math.floor(minX / cellSize);
    const maxCellX = Math.floor(maxX / cellSize);
    const minCellZ = Math.floor(minZ / cellSize);
    const maxCellZ = Math.floor(maxZ / cellSize);

    let count = 0;

    // First pass determines the sampling stride without allocating a giant
    // candidate array.
    for (let cx = minCellX; cx <= maxCellX; cx++) {
      for (let cz = minCellZ; cz <= maxCellZ; cz++) {
        const bucket = this.index.get(`${cx},${cz}`);
        if (!bucket) continue;

        for (const id of bucket) {
          const off = id * 3;
          const x = this.positions[off];
          const z = -this.positions[off + 1];

          if (
            x < minX || x > maxX ||
            z < minZ || z > maxZ
          ) {
            continue;
          }

          if (ringOnly) {
            if (
              Math.abs(x - centerX) <= innerHalf &&
              Math.abs(z - centerZ) <= innerHalf
            ) {
              continue;
            }
          }

          count++;
        }
      }
    }

    if (count === 0) return [];

    const finiteLimit = Number.isFinite(limit) ? Math.max(0, Math.floor(limit)) : count;
    const stride = count > finiteLimit && finiteLimit > 0 ? Math.ceil(count / finiteLimit) : 1;
    const result = new Array(Math.min(count, finiteLimit));
    let write = 0;
    let seen = 0;

    for (let cx = minCellX; cx <= maxCellX; cx++) {
      for (let cz = minCellZ; cz <= maxCellZ; cz++) {
        const bucket = this.index.get(`${cx},${cz}`);
        if (!bucket) continue;

        for (const id of bucket) {
          const off = id * 3;
          const x = this.positions[off];
          const z = -this.positions[off + 1];

          if (
            x < minX || x > maxX ||
            z < minZ || z > maxZ
          ) {
            continue;
          }

          if (ringOnly) {
            if (
              Math.abs(x - centerX) <= innerHalf &&
              Math.abs(z - centerZ) <= innerHalf
            ) {
              continue;
            }
          }

          if (seen % stride === 0 && write < result.length) {
            result[write++] = id;
          }
          seen++;
        }
      }
    }

    result.length = write;
    return result;
  }
}

export const mapStore = new CompleteMapStore();
