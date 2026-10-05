import { state } from './state.js';
import { sendVelocity, sendStop, sendScanControl, getCommandArbitrationState, claimCommandOwnership } from './control.js';
import { runtimeConfig } from './config.js';
import { toast, log } from './utils.js';
import { refreshNavMap, refreshExplorationMap } from './nav-map.js';
import { mapStore } from './map-store.js';

const N = { COMMAND_HOLD_MS: 450, LAST_VALID_CMD_MS: 0,
  grid: null,
  lastPlanAt: 0,
  lastCoverage: 0,
  coverageProgress: 0,
  missionTargetPercent: 85,
  missionComplete: false,
  missionState: 'IDLE',
  explorationTarget: null,
  lastMapSendAt: 0,
  lastCommandAt: 0,
  active: false,
  home: null,
  // GT session anchor for the world-coordinate exploration/tile map.
  areaAnchor: null,
  areaAnchorSessionKey: null,
  goal: null,
  externalTarget: null,
  externalTargetId: null,
  externalTargetStatus: 'idle',
  externalTargetSync: null,
  externalTargetSyncBusy: false,
  externalTargetStatusQueue: Promise.resolve(),
  targetFailureHold: false,
  targetValidation: null,
  path: [],
  mode: 'IDLE',
  coverage: 0,
  knowledgeCoverage: 0,
  knownCells: 0,
  freeCells: 0,
  obstacleCells: 0,
  temporal: null,
  inflated: null,
  config: null,
  localMap: null,
  localRecoveryWorld: new Map(),
  lastLocalPose: null,
  frontierCount: 0,
  frontierCandidates: [],
  planAttempts: 0,
  lastPlanReason: 'INIT',
  lastError: '',
  debugNav: {
    scanUpdateId: 0, planUpdateId: 0,
    lastPlanStart: null, lastPlanResult: null,
    lastPathResult: null, lastSafetyResult: null,
    stopReason: null, commandReason: null,
    frontierEvaluated: 0, frontierReachable: 0,
    frontierRejected: 0, replans: 0,
    lastGoalDistance: null, lastHeadingError: null,
    targetAStar: null
  },
  lastCommand: { v: 0, omega: 0, sent: false, at: 0 },
  localPathInvalidated: false,
  localAvoidanceHeading: 0,
  localClear: false,
  localBlockedReason: 'NOT_CHECKED',
  probeCount: 0,
  commandCount: 0,
  knowledgeSeededCloudLength: -1,
  exploration: {
    tiles: new Map(),
    allTileKeys: [],
    totalTiles: 0,
    exploredTiles: 0,
    coverage: 0,
    lastPose: null,
    seededFromLoadedMap: false,
    seedPointCount: 0
  },
  boundaryRecoveryPath: [],
  boundaryRecoveryIndex: 0,
  debugLastAt: 0,
  runtimeTestPublisher: null,
  runtimeTestLastAt: 0,
  runtimeTestErrors: 0,
  heartbeat: null,
  modeLoop: { lastMode: null, transitions: [], rebooting: false },
  localBackup: {
    enabled: true, threshold: 0, minKnownNeighbors: 0,
    inferred: 0, passes: 0, coverage: 0, eligible: false,
    realKnown: 0, totalCells: 0, persistentApplied: 0,
    raySections: 0, rayEligible: 0,
    scanPoints: 0, cloudPoints: 0, realCells: 0,
    unknownCells: 0, neighborRejected: 0, rayRejected: 0,
    voteClear: 0, voteObstacle: 0, voteMargin: 0,
    overwrittenByReal: 0, persistentCandidates: 0,
    coverageBeforeInference: 0, coverageAfterInference: 0,
    updateId: 0, lastStage: 'INIT', lastReason: 'INIT',
    sectionCandidates: 0,
    sectionEligible: 0,
    sectionKnownCells: 0,
    sectionTotalCells: 0,
    sectionUnknownCells: 0,
    sectionCoverage: 0,
    sectionRejectNoRay: 0,
    sectionRejectNeighbors: 0,
    sectionInferred: 0,
    sectionPasses: 0,
    predictionsStored: 0,
    predictionsReapplied: 0,
    realOverrides: 0
  },
};

function cfg() {
  const c = runtimeConfig.navigation || {};
  const num = (key, fallback) => Number(c[key] ?? fallback);
  const int = (key, fallback) => Math.floor(Number(c[key] ?? fallback));
  return {
    enabled: c.enabled !== false,
    resolution: num('resolution_m', 0.1),
    radius: num('local_sensor_radius_m', 1.0),
    globalScanMinRange: Math.max(0, num('global_scan_min_range_m', 0.2)),
    globalScanMaxRange: Math.max(0, num('global_scan_max_range_m', 12.0)),
    robotHeight: Math.max(0.01, num('robot_height_m', 0.277)),
    groundReferenceZ: num('ground_reference_z_m', -0.1385),
    obstacleAboveGroundMin: Math.max(0, num('obstacle_positive_min_height_m', 0.05)),
    obstacleAboveGroundMax: Math.max(0, num('obstacle_positive_max_height_m', 1.0)),
    obstacleBelowGroundMax: Math.min(0, num('obstacle_negative_max_height_m', -0.15)),
    minRadius: num('local_min_radius_m', 0.05),
    localVoxel: num('local_voxel_size_m', num('local_map_resolution_m', 0.05)),
    localMapResolution: num('local_map_resolution_m', 0.05),
    robotWidth: Math.max(0.01, num('robot_width_m', 0.5)),
    robotLength: Math.max(0.01, num('robot_length_m', 0.51)),
    robotHalfWidth: Math.max(0.005, num('robot_width_m', 0.5) / 2),
    robotHalfLength: Math.max(0.005, num('robot_length_m', 0.51) / 2),
    // Orientation-independent footprint radius used by the global A* grid.
    // It is derived from BOTH robot dimensions, so A* never underestimates
    // the space required by the real footprint.
    robotRadius: Math.hypot(num('robot_width_m', 0.5) / 2, num('robot_length_m', 0.51) / 2),
    safety: num('safety_margin_m', 0.2),
    vmax: num('max_linear_speed_mps', 0.15),
    wmax: num('max_angular_speed_rps', 1.2),
    lookahead: num('lookahead_m', 0.35),
    goalReach: num('goal_reach_m', 0.2),
    continueExplorationAfterTargetReached: c.continue_exploration_after_target_reached === true ||
      String(c.continue_exploration_after_target_reached ?? '').toLowerCase() === 'true',
    replanMs: num('replan_interval_ms', 350),
    coverageTarget: Math.min(1.0, Math.max(0.0, num('coverage_target', 0.85))),
    explorationTileSize: Math.max(0.05, num('exploration_tile_size_m', 0.50)),
    tileCoverageCompleteRate: Math.min(1, Math.max(0, num('tile_cov_complete_rate', 0.85))),
    explorationMinObservations: Math.max(1, int('exploration_min_observations', 3)),
    explorationMinViewAngles: Math.max(1, int('exploration_min_view_angles', 2)),
    explorationMinViewAngleDeg: Math.max(1, num('exploration_min_view_angle_deg', 30.0)),
    explorationMinViewpointDistance: Math.max(0.01, num('exploration_min_viewpoint_distance_m', 0.20)),
    resumeLoadedExploration: c.resume_loaded_exploration !== false,
    outsideRecoveryEnabled: c.outside_area_recovery_enabled !== false,
    outsideRecoveryMode: String(c.outside_area_recovery_mode ?? 'path_return').toLowerCase(),
    outsideWarningDistance: Math.max(0, num('outside_area_warning_distance_m', 0.50)),
    outsideRecoveryWaypointTolerance: Math.max(0.05, num('outside_area_recovery_waypoint_tolerance_m', 0.20)),
    outsideRecoveryMaxPathSearchDistance: Math.max(0.1, num('outside_area_recovery_max_path_search_distance_m', 5.0)),
    boundaryRecoverySpeedScale: Math.max(0.05, num('outside_area_recovery_speed_scale', 0.45)),
    targetKnownPercent: num('target_known_percent', num('coverage_target', 0.85) * 100),
    frontierMinGain: Math.max(0, int('frontier_min_gain_cells', 3)),
    frontierMinDistance: Math.max(0, num('frontier_min_distance_m', 0.35)),
    probeDistance: Math.max(0, num('exploration_probe_distance_m', 0.65)),
    probeHeadingOffsetDeg: num('probe_heading_offset_deg', 35),
    probeHeadingSamples: Math.max(1, int('probe_heading_samples', 5)),
    probeSpeedScale: Math.max(0, num('probe_speed_scale', 0.65)),
    probeAngularGain: num('probe_angular_gain', 2.0),
    probeSlowHeadingRad: Math.max(0, num('probe_slow_heading_rad', 0.8)),
    probeSlowSpeedScale: Math.max(0, num('probe_slow_speed_scale', 0.35)),
    debugIntervalMs: Math.max(0, num('debug_interval_ms', 250)),
    // NAVIGATING/REPLAN loop rebooting is intentionally disabled.
    modeLoopEnabled: false,
    modeLoopThreshold: Math.max(2, int('navigation_mode_loop_threshold', 6)),
    modeLoopWindowMs: Math.max(1000, num('navigation_mode_loop_window_ms', 12000)),
    modeLoopRebootCooldownMs: Math.max(1000, num('navigation_mode_loop_reboot_cooldown_ms', 15000)),
    modeLoopOffMs: Math.max(1000, num('navigation_mode_loop_off_ms', 1000)),

    // All local-map/recovery behavior is YAML-driven. No algorithmic threshold
    // below is silently replaced by a browser-side constant.
    localInferenceEnabled: c.local_inference_enabled !== false,
    localInferenceCoverage: Math.min(1, Math.max(0, num('local_inference_coverage', 0.75))),
    localInferenceMinKnownNeighbors: Math.max(0, int('local_inference_min_known_neighbors', 6)),
    localInferencePasses: Math.max(0, int('local_inference_passes', 3)),
    localInferenceNeighborRadiusCells: Math.max(1, int('local_inference_neighbor_radius_cells', 1)),
    localInferenceTieClass: String(c.local_inference_tie_class ?? 'clear').toLowerCase(),
    localRecoveryRaySectionCells: Math.max(0, int('local_recovery_ray_section_cells', 2)),
    localRecoveryRequireRealNeighbors: c.local_recovery_require_real_neighbors !== false,

    lidarOriginX: num('local_lidar_origin_x_m', 0.254),
    lidarOriginY: num('local_lidar_origin_y_m', 0.0),
    localMapCircleMarginM: Math.max(0, num('local_map_circle_margin_m', 0.0)),
    localRayClipMarginM: Math.max(0, num('local_ray_clip_margin_m', 0.0)),
    localClearanceStartM: Math.max(0, num('local_clearance_start_m', 0.08)),
    localClearanceStepM: Math.max(0.001, num('local_clearance_step_m', 0.025)),
    localLateralStepM: Math.max(0.001, num('local_lateral_step_m', 0.05)),
    localAvoidanceDistance: Math.max(0, num('local_avoidance_distance_m', 0.75)),
    localAvoidanceHeadingStepDeg: Math.max(1, num('local_avoidance_heading_step_deg', 10)),
    localAvoidanceHeadingMinDeg: num('local_avoidance_heading_min_deg', -170),
    localAvoidanceHeadingMaxDeg: num('local_avoidance_heading_max_deg', 170),
    localAvoidanceAngularGain: num('local_avoidance_angular_gain', 1.8),
    localAvoidanceLinearSpeedScale: Math.max(0, num('local_avoidance_linear_speed_scale', 0.45)),
    localAvoidanceHeadingPenalty: Math.max(0, num('local_avoidance_heading_penalty', 1.8)),
    localAvoidanceTurnPenalty: Math.max(0, num('local_avoidance_turn_penalty', 0.12)),
    localAvoidanceClearanceWeight: Math.max(0, num('local_avoidance_clearance_weight', 2.0)),
    localAvoidanceUnknownPenalty: Math.max(0, num('local_avoidance_unknown_penalty', 0.12)),
    localAvoidanceBoundaryBonus: num('local_avoidance_boundary_bonus', 2.0),
    localCorridorHalfWidthScale: Math.max(0, num('local_corridor_half_width_scale', 0.20)),
    localForwardSafetyDistanceM: Math.max(0, num('local_forward_safety_distance_m', 0.45)),
    pathAngularGain: num('path_angular_gain', 2.2),
     pathSlowHeadingRad: Math.max(0, num('path_slow_heading_rad', 0.9)),
     targetHeadingStopRad: Math.max(0, num('target_heading_stop_rad', 1.20)),
    pathSlowSpeedScale: Math.max(0, num('path_slow_speed_scale', 0.35)),
    goalSlowdownDistanceScale: Math.max(0, num('goal_slowdown_distance_scale', 1.5)),
     goalSlowdownSpeedScale: Math.max(0, num('goal_slowdown_speed_scale', 0.5)),
     terminalApproachDistance: Math.max(0, num('terminal_approach_distance_m', 1.0)),
    boundarySlowSpeedScale: Math.max(0, num('boundary_slow_speed_scale', 0.45)),

    boundaryBuffer: Math.max(0, num('boundary_buffer_m', 0.10)),
    boundaryLookahead: Math.max(0, num('boundary_lookahead_m', 0.45)),
    boundarySampleStepM: Math.max(0.001, num('boundary_sample_step_m', 0.05)),
    returnHome: c.return_home !== false,
    area: c.area || { type:'circle', rectangle:{min_x:0,max_x:20,min_y:-1,max_y:23}, circle:{center_mode:'robot_start',center_x:0,center_y:0,radius_m:15} },
  };
}

function initLocalMap(c) {
  const radius = Math.max(c.localMapResolution, c.radius);
  const resolution = c.localMapResolution;
  const width = Math.max(4, Math.ceil((2 * radius) / resolution));
  const height = width;
  N.localMap = {
    width, height, resolution,
    originX: -radius,
    originY: -radius,
    radius,
    cells: new Uint8Array(width * height),
    seen: new Uint8Array(width * height),
    real: new Uint8Array(width * height),
    // Cells physically traversed by the CURRENT LiDAR rays. This is a transient
    // eligibility mask, not persistent map data.
    rayPassed: new Uint8Array(width * height),
  };
}

function localCell(x, y) {
  const g = N.localMap;
  if (!g) return null;
  const ix = Math.floor((x - g.originX) / g.resolution);
  const iy = Math.floor((y - g.originY) / g.resolution);
  if (ix < 0 || iy < 0 || ix >= g.width || iy >= g.height) return null;
  return [ix, iy];
}

function localIdx(ix, iy) { return iy * N.localMap.width + ix; }

function localMark(ix, iy, type) {
  const g = N.localMap;
  if (!g || ix < 0 || iy < 0 || ix >= g.width || iy >= g.height) return;
  const i = localIdx(ix, iy);
  g.seen[i] = 1;
  g.real[i] = 0;
  if (type === 2) g.cells[i] = 2;
  else if (type === 1 && g.cells[i] !== 2) g.cells[i] = 1;
}

function localMarkReal(ix, iy, type) {
  const g=N.localMap;
  if(!g || ix<0 || iy<0 || ix>=g.width || iy>=g.height) return;
  const i=localIdx(ix,iy);
  if (g.seen[i] && g.cells[i] !== type && g.cells[i] !== 0) {
    N.localBackup.overwrittenByReal++; N.localBackup.realOverrides++;
  }
  g.seen[i]=1;
  g.real[i]=1;
  g.cells[i]=type; // real sensor evidence always overrides recovered evidence
}

function localTraceRay(targetX, targetY, endpointType, c = N.config || cfg()) {
  const g = N.localMap;
  if (!g) return;

  // The local map is robot-centred, while the real LiDAR is physically
  // displaced from that centre.  The ray MUST start at the configured LiDAR
  // origin for every scan, independently of the selected LiDAR offset angle.
  const sxWorld = c.lidarOriginX;
  const syWorld = c.lidarOriginY;
  const vx = targetX - sxWorld;
  const vy = targetY - syWorld;
  const v2 = vx * vx + vy * vy;
  if (v2 <= 1e-12) return;

  // A scan hit may be farther than the 1 m local map.  Previously such a hit
  // was discarded completely, which produced rays=0 even though the LiDAR was
  // actively scanning.  Clip every valid ray to the configured local-map
  // circle so the local safety map receives the free-space evidence up to its
  // boundary.  Only a hit that is actually inside the local map may be marked
  // as an obstacle endpoint.
  const localRadius = Math.max(g.resolution, g.radius - c.localMapCircleMarginM - c.localRayClipMarginM);
  const startR2 = sxWorld * sxWorld + syWorld * syWorld;
  let tEnd = 1.0;
  const targetInside = targetX * targetX + targetY * targetY <= localRadius * localRadius;
  if (!targetInside) {
    // Solve |S + tV|^2 = R^2 for the positive intersection.
    const b = sxWorld * vx + syWorld * vy;
    const disc = b * b - v2 * (startR2 - localRadius * localRadius);
    if (disc <= 0) return;
    const root = Math.sqrt(disc);
    const t1 = (-b + root) / v2;
    const t2 = (-b - root) / v2;
    const candidates = [t1, t2].filter(t => t > 0);
    if (!candidates.length) return;
    tEnd = Math.min(...candidates);
    if (!Number.isFinite(tEnd) || tEnd <= 0) return;
  }

  const ex = sxWorld + vx * tEnd;
  const ey = syWorld + vy * tEnd;
  const start = localCell(sxWorld, syWorld);
  const end = localCell(ex, ey);
  if (!start || !end) return;

  let x0 = start[0], y0 = start[1], x1 = end[0], y1 = end[1];
  const dx = Math.abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
  const dy = -Math.abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
  let err = dx + dy;
  const finalType = targetInside ? endpointType : 1;
  while (true) {
    const ri = localIdx(x0, y0);
    if (ri >= 0 && ri < g.rayPassed.length) g.rayPassed[ri] = 1;
    if (x0 === x1 && y0 === y1) { localMarkReal(x0, y0, finalType); break; }
    localMarkReal(x0, y0, 1);
    const e2 = 2 * err;
    if (e2 >= dy) { err += dy; x0 += sx; }
    if (e2 <= dx) { err += dx; y0 += sy; }
    if (x0 < 0 || y0 < 0 || x0 >= g.width || y0 >= g.height) break;
  }
}

function localRecoveryKey(wx, wy, c) {
  const q = Math.max(0.025, c.localVoxel || c.localInferenceResolution || 0.05);
  return `${Math.round(wx / q)},${Math.round(wy / q)}`;
}

function robotToWorld(point, pose, orientation) {
  if (!orientation || orientation.length !== 9) {
    const c = Math.cos(pose.theta), s = Math.sin(pose.theta);
    return [pose.x + c * point[0] - s * point[1], pose.y + s * point[0] + c * point[1], (pose.z || 0) + point[2]];
  }
  return [
    pose.x + orientation[0] * point[0] + orientation[1] * point[1] + orientation[2] * point[2],
    pose.y + orientation[3] * point[0] + orientation[4] * point[1] + orientation[5] * point[2],
    (pose.z || 0) + orientation[6] * point[0] + orientation[7] * point[1] + orientation[8] * point[2]
  ];
}

function localCellEvidenceType(g, i, c) {
  const t = g.cells[i];
  if (t === 2) return 2;
  if (t === 3) return 3;
  if (!g.seen[i]) return 0;

  // For voting, a measured clear cell inside the obstacle safety radius is
  // considered SAFETY MARGIN evidence. This does not overwrite the raw cell;
  // it only gives the requested 4-class vote access to margin evidence.
  const x = i % g.width, y = Math.floor(i / g.width);
  const n = Math.max(0, Math.ceil(c.safety / g.resolution));
  for (let dy=-n; dy<=n; dy++) for (let dx=-n; dx<=n; dx++) {
    if (dx*dx + dy*dy > n*n) continue;
    const nx=x+dx, ny=y+dy;
    if (nx<0 || ny<0 || nx>=g.width || ny>=g.height) continue;
    if (g.cells[localIdx(nx,ny)] === 2) return 3;
  }
  return 1;
}

function applyPersistentLocalRecovery(c, pose, orientation) {
  const g = N.localMap;
  N.localBackup.persistentCandidates = 0;
  N.localBackup.overwrittenByReal = 0;
  if (!g || !N.localRecoveryWorld?.size) return 0;
  let applied=0;
  for (const [key, rec] of N.localRecoveryWorld) {
    N.localBackup.persistentCandidates++;
    N.localBackup.predictionsReapplied++;
    const rb = worldToRobot([rec.x, rec.y, pose.z || 0], pose, orientation);
    const rr2 = rb[0]*rb[0] + rb[1]*rb[1];
    if (rr2 > g.radius*g.radius || rr2 < (c.minRadius ?? 0.05)**2) continue;
    const ce=localCell(rb[0], rb[1]);
    if (!ce) continue;
    const i=localIdx(ce[0],ce[1]);
    // Sensor/cloud evidence always wins over recovered evidence.
    if (g.seen[i]) continue;
    g.seen[i]=1;
    g.real[i]=0;
    g.cells[i]=rec.type;
    applied++;
  }
  return applied;
}

function isRaySectionCell(g, i, sectionCells) {
  if (!g?.rayPassed) return false;
  const x = i % g.width, y = Math.floor(i / g.width);
  for (let dy = -sectionCells; dy <= sectionCells; dy++) {
    for (let dx = -sectionCells; dx <= sectionCells; dx++) {
      if (dx * dx + dy * dy > sectionCells * sectionCells) continue;
      const nx = x + dx, ny = y + dy;
      if (nx < 0 || ny < 0 || nx >= g.width || ny >= g.height) continue;
      if (g.rayPassed[localIdx(nx, ny)]) return true;
    }
  }
  return false;
}

function countRaySectionCells(g) {
  if (!g?.rayPassed) return 0;
  let n = 0;
  for (let i = 0; i < g.rayPassed.length; i++) if (g.rayPassed[i]) n++;
  return n;
}

function inferLocalUnknownCells(c, pose, orientation) {
  const g = N.localMap;
  if (!g || !c.localInferenceEnabled) return;
  const r2=Math.max(0, g.radius-c.localMapCircleMarginM)**2, cx=0, cy=0;
  const threshold=c.localInferenceCoverage, minNeighbors=c.localInferenceMinKnownNeighbors;
  let total=0, known=0;
  for(let y=0;y<g.height;y++) for(let x=0;x<g.width;x++) {
    const lx=g.originX+(x+0.5)*g.resolution, ly=g.originY+(y+0.5)*g.resolution;
    if((lx-cx)**2+(ly-cy)**2>r2) continue;
    total++; if(g.seen[localIdx(x,y)]) known++;
  }
  const coverage=total?known/total:0;
  N.localBackup.coverage=coverage;
  N.localBackup.realKnown=known;
  N.localBackup.totalCells=total;
  N.localBackup.inferred=0;
  N.localBackup.passes=0;
  N.localBackup.eligible=coverage>=threshold;
  N.localBackup.persistentApplied=0;
  N.localBackup.raySections = countRaySectionCells(g);
  N.localBackup.rayEligible = 0;
  N.localBackup.sectionCandidates = 0;
  N.localBackup.sectionEligible = 0;
  N.localBackup.sectionKnownCells = 0;
  N.localBackup.sectionTotalCells = 0;
  N.localBackup.sectionUnknownCells = 0;
  N.localBackup.sectionCoverage = 0;
  N.localBackup.sectionRejectNoRay = 0;
  N.localBackup.sectionRejectNeighbors = 0;
  N.localBackup.sectionInferred = 0;
  N.localBackup.sectionPasses = 0;
  // sectionCells is the configured ray-section radius in grid cells.
  // The actual current ray section is represented by g.rayPassed.
  const sectionRadius = Math.max(0, c.localRecoveryRaySectionCells);
  let sectionKnown = 0, sectionUnknown = 0, sectionTotal = 0;
  if (g.rayPassed) {
    for (let idx = 0; idx < g.rayPassed.length; idx++) {
      if (!g.rayPassed[idx]) continue;
      sectionTotal++;
      if (g.seen[idx]) sectionKnown++; else sectionUnknown++;
    }
  }
  N.localBackup.sectionKnownCells = sectionKnown;
  N.localBackup.sectionUnknownCells = sectionUnknown;
  N.localBackup.sectionTotalCells = sectionTotal;
  N.localBackup.sectionCoverage = sectionTotal ? sectionKnown / sectionTotal : 0;
  N.localBackup.sectionEligible = sectionTotal > 0;
  N.localBackup.unknownCells = total - known;
  N.localBackup.neighborRejected = 0;
  N.localBackup.rayRejected = 0;
  N.localBackup.voteClear = 0;
  N.localBackup.voteObstacle = 0;
  N.localBackup.voteMargin = 0;
  N.localBackup.coverageBeforeInference = coverage;
  N.localBackup.coverageAfterInference = coverage;
  // Global 1 m coverage is now diagnostic only. Recovery is activated
  // per LiDAR-ray traversed section, so a 49% overall local map can
  // still recover a well-observed section.
  N.localBackup.eligible = true;
  N.localBackup.lastStage = 'RAY_SECTION_RECOVERY';
  N.localBackup.lastReason = `GLOBAL COVERAGE ${(coverage*100).toFixed(1)}% (diagnostic; YAML threshold ${(threshold*100).toFixed(1)}%)`;

  const sectionCells = sectionRadius;
  const neighborRadius = Math.max(1, c.localInferenceNeighborRadiusCells);
  const dirs=[];
  for (let dy=-neighborRadius; dy<=neighborRadius; dy++) {
    for (let dx=-neighborRadius; dx<=neighborRadius; dx++) {
      if (dx===0 && dy===0) continue;
      if (dx*dx + dy*dy <= neighborRadius*neighborRadius) dirs.push([dx,dy]);
    }
  }
  for(let pass=0;pass<c.localInferencePasses;pass++) {
    const pending=[];
    for(let y=1;y<g.height-1;y++) for(let x=1;x<g.width-1;x++) {
      const i=localIdx(x,y);
      if(g.seen[i]) continue;
      // Recovery is local to sections actually crossed by the CURRENT LiDAR
      // rays. A distant unknown pixel is never synthesized merely because the
      // rest of the 1 m map is known.
      if(!isRaySectionCell(g, i, sectionCells)) {
        N.localBackup.rayRejected++;
        N.localBackup.sectionRejectNoRay++;
        continue;
      }
      N.localBackup.rayEligible++;
      N.localBackup.sectionCandidates++;
      const lx=g.originX+(x+0.5)*g.resolution, ly=g.originY+(y+0.5)*g.resolution;
      if((lx-cx)**2+(ly-cy)**2>r2) continue;
      let knownCount=0, clearVotes=0, obstacleVotes=0, marginVotes=0;
      for(const [dx,dy] of dirs) {
        const nx=x+dx, ny=y+dy, ni=localIdx(nx,ny);
        if(!g.seen[ni]) continue;
        if(c.localRecoveryRequireRealNeighbors && !g.real[ni]) continue;
        const et=localCellEvidenceType(g,ni,c);
        if(!et) continue;
        knownCount++;
        if(et===2) obstacleVotes++;
        else if(et===3) marginVotes++;
        else clearVotes++;
      }
      if(knownCount<minNeighbors) {
        N.localBackup.neighborRejected++;
        N.localBackup.sectionRejectNeighbors++;
        continue;
      }

      // Deterministic majority. Tie behavior is explicitly configured in YAML.
      let inferred=1;
      if(obstacleVotes>clearVotes && obstacleVotes>=marginVotes) inferred=2;
      else if(marginVotes>clearVotes && marginVotes>obstacleVotes) inferred=3;
      else if (obstacleVotes===clearVotes && obstacleVotes===marginVotes && c.localInferenceTieClass==='obstacle') inferred=2;
      else if (obstacleVotes===clearVotes && obstacleVotes===marginVotes && c.localInferenceTieClass==='margin') inferred=3;
      if (inferred === 1) N.localBackup.voteClear++;
      else if (inferred === 2) N.localBackup.voteObstacle++;
      else if (inferred === 3) N.localBackup.voteMargin++;
      pending.push([i,inferred]);
    }
    if(!pending.length) break;
    for(const [i,t] of pending) {
      g.seen[i]=1; g.real[i]=0; g.cells[i]=t;
      const x=i%g.width, y=Math.floor(i/g.width);
      const rb=[g.originX+(x+0.5)*g.resolution, g.originY+(y+0.5)*g.resolution, 0];
      const wp=robotToWorld(rb,pose,orientation);
      N.localRecoveryWorld.set(localRecoveryKey(wp[0],wp[1],c), {x:wp[0],y:wp[1],type:t,at:Date.now()});
      N.localBackup.inferred++;
      N.localBackup.sectionInferred++;
    }
    N.localBackup.passes++;
  }
  let afterKnown=0;
  for(let i=0;i<g.seen.length;i++) if(g.seen[i]) afterKnown++;
  N.localBackup.coverageAfterInference = total ? afterKnown/total : coverage;
  N.localBackup.lastStage = N.localBackup.inferred ? 'RAY_SECTION_INFERENCE_COMMITTED' : 'RAY_SECTION_NO_COMMIT';
  N.localBackup.lastReason = N.localBackup.inferred
    ? `${N.localBackup.inferred} cells inferred from current real evidence`
    : `No unknown cell passed current-ray + >=${c.localInferenceMinKnownNeighbors}-real-neighbour criteria`;
}

function rebuildLocalSafety(c) {
  const g = N.localMap;
  if (!g) return;
  for (let i = 0; i < g.cells.length; i++) {
  }
  const n = Math.max(0, Math.ceil(c.safety / g.resolution));
  const obstacles = [];
  for (let y = 0; y < g.height; y++) {
    for (let x = 0; x < g.width; x++) {
      if (g.cells[localIdx(x, y)] === 2) obstacles.push([x, y]);
    }
  }
  for (const [ox, oy] of obstacles) {
    for (let dy = -n; dy <= n; dy++) {
      for (let dx = -n; dx <= n; dx++) {
        if (dx * dx + dy * dy > n * n) continue;
        const x = ox + dx, y = oy + dy;
        if (x < 0 || y < 0 || x >= g.width || y >= g.height) continue;
        const i = localIdx(x, y);
        if (g.cells[i] === 1) g.cells[i] = 3;
      }
    }
  }
}

function rebuildLocalMapFromCompleteCloud(meta, c) {
  const dbg = N.localBackup;
  dbg.updateId++;
  dbg.lastStage = 'REBUILD_START';
  dbg.lastReason = 'STARTING LOCAL MAP UPDATE';
  dbg.scanPoints = Array.isArray(meta?.__currentScanPoints) ? Math.floor(meta.__currentScanPoints.length / 3) : 0;
  if (!N.localMap) initLocalMap(c);
  const g = N.localMap;
  const pose = state.navPose;
  if (!pose || pose.source !== 'ground_truth') return;

  // rayPassed belongs only to this sensor update. Inferred world cells persist
  // separately, so robot movement/new scans cannot be lost when the local grid
  // recenters.
  g.rayPassed.fill(0);

  // Working v2 autopilot is preserved. The local map source is corrected to
  // the requested accumulated/filtered cloud: moving 1 m robot-centred window
  // -> existing World->Robot transform -> 5 cm..1 m -> local Z < -5 cm.
  g.cells.fill(0); g.seen.fill(0); g.real.fill(0);
  const orientation = pose.orientation;
  const robotPose = { x: pose.x, y: pose.y, z: pose.z ?? meta?.ground_truth?.z ?? 0, theta: pose.theta };
  const radius = c.radius, radius2 = radius * radius;
  const minRadius2 = Math.pow(c.minRadius ?? 0.05, 2);
  const voxel = Math.max(g.resolution, c.localVoxel ?? g.resolution);

  // Persistent recovery is applied only after current real cloud/ray evidence
  // has been projected. This guarantees that a current observation always wins
  // over an old prediction.
  N.localBackup.persistentApplied = 0;

  const ids = mapStore.queryRadiusWorldXYFiltered(pose.x, pose.y, radius, voxel, (id) => {
    const off = id * 3;
    const rb = worldToRobot(
      [mapStore.positions[off], mapStore.positions[off + 1], mapStore.positions[off + 2]],
      robotPose, orientation
    );
    return rb[2];
  });

  for (const id of ids) {
    const off = id * 3;
    const p = [mapStore.positions[off], mapStore.positions[off + 1], mapStore.positions[off + 2]];
    const dx = p[0] - pose.x, dy = p[1] - pose.y;
    if (dx * dx + dy * dy > radius2) continue;
    const rb = worldToRobot(p, robotPose, orientation);
    const rr2 = rb[0] * rb[0] + rb[1] * rb[1];
    if (rr2 < minRadius2 || rr2 > radius2) continue;
    const ce = localCell(rb[0], rb[1]);
    if (!ce) continue;
    const kind = classifyRelativeGroundHeight(relativeGroundHeightFromRobotFrameZ(rb[2], c), c);
    if (kind) localMarkReal(ce[0], ce[1], kind);
  }

  // Preserve v2's working free-space ray evidence. The accumulated cloud is
  // still the persistent source; current rays only fill the clear corridor
  // that lets the known-working autopilot leave an initially clear area.
  if (meta?.__currentScanPoints) {
    const points = meta.__currentScanPoints;
    for (let i = 0; i < points.length; i += 3) {
      const wx = points[i], wy = points[i + 1], wz = points[i + 2];
      if (!Number.isFinite(wx) || !Number.isFinite(wy) || !Number.isFinite(wz)) continue;
      const rb = worldToRobot([wx, wy, wz], robotPose, orientation);
      const rr2 = rb[0] * rb[0] + rb[1] * rb[1];
      // Use every valid scan endpoint to build the local free-space ray. The
      // ray is clipped to the configured local radius by localTraceRay().
      // This is essential when the nearest physical return is beyond 1 m.
      if (rr2 < minRadius2) continue;
      const kind = classifyRelativeGroundHeight(relativeGroundHeightFromRobotFrameZ(rb[2], c), c);
      // Rays whose endpoint is above the positive obstacle ceiling still prove
      // free horizontal space up to the local-map boundary, but the endpoint
      // itself is not an obstacle in the 2D ground-navigation layer.
      localTraceRay(rb[0], rb[1], kind || 1, c);
    }
  }

  // Reapply old predictions only into cells that remain genuinely unknown
  // after the current real evidence. They are supporting evidence, never a
  // replacement for current LiDAR/cloud measurements.
  N.localBackup.persistentApplied = applyPersistentLocalRecovery(c, pose, orientation);

  // Recovery happens after real evidence and is conservative: recovered cells
  // are not allowed to become neighbours that recursively generate more
  // recovered cells in the same update.
  inferLocalUnknownCells(c, pose, orientation);
  rebuildLocalSafety(c);
  N.localBackup.realCells = 0;
  N.localBackup.unknownCells = 0;
  for (let i=0;i<g.cells.length;i++) {
    if (g.seen[i]) N.localBackup.realCells++;
    else N.localBackup.unknownCells++;
  }
  if (N.localBackup.lastStage === 'REBUILD_START') {
    N.localBackup.lastStage = 'MAP_READY';
    N.localBackup.lastReason = 'MAP UPDATED';
  }
  N.lastLocalPose = { x: pose.x, y: pose.y, z: robotPose.z, theta: pose.theta };
  state.navMap = { width:g.width, height:g.height, resolution:g.resolution,
    originX:g.originX, originY:g.originY, cells:g.cells.slice(), seen:g.seen.slice(), local:true,
    radius:g.radius, min_radius:c.minRadius,
    robot_height_m:c.robotHeight, ground_reference_z_m:c.groundReferenceZ,
    obstacle_positive_min_m:c.obstacleAboveGroundMin, obstacle_positive_max_m:c.obstacleAboveGroundMax,
    obstacle_negative_max_m:c.obstacleBelowGroundMax,
    real:g.real.slice(),
    safety_margin:c.safety, source_points:ids.length, filtered_cloud_points:ids.length,
    center_x:pose.x, center_y:pose.y };
  refreshNavMap();
}

function resolvedArea(c, pose = state.navPose || N.home) {
  const area = c.area || {};
  const areaType = String(area.type || 'circle').toLowerCase();
  if (areaType === 'circle') {
    const a = area.circle || {};
    let cx = Number(a.center_x ?? 0);
    let cy = Number(a.center_y ?? 0);
    const mode = String(a.center_mode || area.center_mode || 'world');
    // A robot-centred configured area is anchored once at the robot's startup
    // pose. This avoids the common failure where the Webots world origin is
    // nowhere near the robot's actual starting position.
    if (mode === 'robot_start' && N.areaAnchor) {
      cx = N.areaAnchor.x;
      cy = N.areaAnchor.y;
    } else if (mode === 'robot_start' && N.home) {
      cx = N.home.x;
      cy = N.home.y;
    } else if (mode === 'robot_current' && pose) {
      cx = pose.x;
      cy = pose.y;
    }
    return { type:'circle', center_x:cx, center_y:cy, radius_m:Number(a.radius_m ?? 15) };
  }
  const a = area.rectangle || {};
  let minX=Number(a.min_x ?? 0), maxX=Number(a.max_x ?? 20), minY=Number(a.min_y ?? -1), maxY=Number(a.max_y ?? 23);
  const mode = String(a.center_mode || area.center_mode || 'world');
  if (mode === 'robot_start' && N.areaAnchor) {
    const cx=(minX+maxX)/2, cy=(minY+maxY)/2;
    const dx=N.areaAnchor.x-cx, dy=N.areaAnchor.y-cy;
    minX+=dx; maxX+=dx; minY+=dy; maxY+=dy;
  } else if (mode === 'robot_current' && pose) {
    const cx=(minX+maxX)/2, cy=(minY+maxY)/2;
    const dx=pose.x-cx, dy=pose.y-cy;
    minX+=dx; maxX+=dx; minY+=dy; maxY+=dy;
  }
  return { type:'rectangle', min_x:minX,max_x:maxX,min_y:minY,max_y:maxY };
}

function makeBounds(c) {
  const a = resolvedArea(c);
  if (a.type === 'circle') return { minX:a.center_x-a.radius_m,maxX:a.center_x+a.radius_m,minY:a.center_y-a.radius_m,maxY:a.center_y+a.radius_m };
  return { minX:a.min_x,maxX:a.max_x,minY:a.min_y,maxY:a.max_y };
}

function insideArea(x, y, c, safe = false, pose = state.navPose || N.home) {
  const m = safe ? (c.robotRadius + c.safety) : 0;
  const a = resolvedArea(c, pose);
  if (a.type === 'circle') {
    const r = Math.max(0, a.radius_m - m);
    return (x - a.center_x) ** 2 + (y - a.center_y) ** 2 <= r * r;
  }
  return x >= a.min_x + m && x <= a.max_x - m && y >= a.min_y + m && y <= a.max_y - m;
}

function areaDiagnostics(c, pose) {
  const a = resolvedArea(c, pose);
  let inside = false, safeInside = false, distance = null, safeDistance = null;
  if (a.type === 'circle') {
    distance = Math.hypot(pose.x-a.center_x, pose.y-a.center_y);
    safeDistance = Math.max(0, a.radius_m-(c.robotRadius+c.safety));
    inside = distance <= a.radius_m;
    safeInside = distance <= safeDistance;
  } else {
    const dx=Math.min(pose.x-a.min_x,a.max_x-pose.x), dy=Math.min(pose.y-a.min_y,a.max_y-pose.y);
    distance=Math.min(dx,dy);
    safeDistance=Math.min(a.max_x-(c.robotRadius+c.safety)-pose.x, pose.x-(a.min_x+c.robotRadius+c.safety), a.max_y-(c.robotRadius+c.safety)-pose.y, pose.y-(a.min_y+c.robotRadius+c.safety));
    inside=pose.x>=a.min_x&&pose.x<=a.max_x&&pose.y>=a.min_y&&pose.y<=a.max_y;
    safeInside=safeDistance>=0;
  }
  return { ...a, inside, safeInside, distance, safeDistance };
}

function initGrid(c) {
  const b = makeBounds(c);
  const width = Math.max(2, Math.ceil((b.maxX - b.minX) / c.resolution));
  const height = Math.max(2, Math.ceil((b.maxY - b.minY) / c.resolution));
  N.grid = { width, height, resolution: c.resolution, originX: b.minX, originY: b.minY, cells: new Uint8Array(width * height), seen: new Uint8Array(width * height) };
  N.temporal = new Int16Array(width * height);
  N.inflated = new Uint8Array(width * height);
  N.knowledgeSeededCloudLength = -1;
}

function ensureGrid() {
  const c = cfg();
  if (!N.grid || N.config?.resolution !== c.resolution || JSON.stringify(N.config?.area) !== JSON.stringify(c.area)) {
    N.config = c;
    initGrid(c);
    initLocalMap(c);
  }
  return c;
}

function cell(x, y) {
  const g = N.grid;
  const ix = Math.floor((x - g.originX) / g.resolution);
  const iy = Math.floor((y - g.originY) / g.resolution);
  if (ix < 0 || iy < 0 || ix >= g.width || iy >= g.height) return null;
  return [ix, iy];
}

function idx(ix, iy) { return iy * N.grid.width + ix; }
function world(ix, iy) { return [N.grid.originX + (ix + 0.5) * N.grid.resolution, N.grid.originY + (iy + 0.5) * N.grid.resolution]; }

function worldToRobot(point, pose, orientation) {
  const dx = point[0] - pose.x;
  const dy = point[1] - pose.y;
  const dz = point[2] - (pose.z || 0);
  if (!orientation || orientation.length !== 9) {
    const c = Math.cos(pose.theta), s = Math.sin(pose.theta);
    return [c * dx + s * dy, -s * dx + c * dy, dz];
  }
  return [
    orientation[0] * dx + orientation[3] * dy + orientation[6] * dz,
    orientation[1] * dx + orientation[4] * dy + orientation[7] * dz,
    orientation[2] * dx + orientation[5] * dy + orientation[8] * dz,
  ];
}

function mark(ix, iy, type) {
  const i = idx(ix, iy);
  if (i < 0 || i >= N.grid.cells.length) return;
  N.grid.seen[i] = 1;
  if (type === 2) {
    N.temporal[i] = Math.min(12, N.temporal[i] + 3);
  } else if (type === 1) {
    N.temporal[i] = Math.max(-8, N.temporal[i] - 1);
  }
  if (N.temporal[i] >= 3) N.grid.cells[i] = 2;
  else if (N.temporal[i] <= -1) N.grid.cells[i] = 1;
}

function traceRay(startWorld, target, endpointType) {
  const start = cell(startWorld[0], startWorld[1]);
  const end = cell(target[0], target[1]);
  if (!start || !end) return;
  let x0 = start[0], y0 = start[1], x1 = end[0], y1 = end[1];
  const dx = Math.abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
  const dy = -Math.abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
  let err = dx + dy;
  while (true) {
    if (x0 === x1 && y0 === y1) { mark(x0, y0, endpointType); break; }
    mark(x0, y0, 1);
    const e2 = 2 * err;
    if (e2 >= dy) { err += dy; x0 += sx; }
    if (e2 <= dx) { err += dx; y0 += sy; }
    if (x0 < 0 || y0 < 0 || x0 >= N.grid.width || y0 >= N.grid.height) break;
  }
}

function classifyRelativeGroundHeight(relativeHeight, c) {
  const h = Number(relativeHeight);
  if (!Number.isFinite(h)) return 0; // UNKNOWN / unusable
  if (h >= c.obstacleAboveGroundMin && h <= c.obstacleAboveGroundMax) return 2; // OBSTACLE
  if (h <= c.obstacleBelowGroundMax) return 2; // OBSTACLE / deep drop
  if (h > c.obstacleBelowGroundMax && h < c.obstacleAboveGroundMin) return 1; // FREE / ground band
  return 0; // Above positive obstacle ceiling: ignore as navigation obstacle evidence
}

function relativeGroundHeightFromRobotFrameZ(localZ, c) {
  // Assumption for this GT test: Webots Supervisor.getPosition() is the robot
  // geometric centre, so the supporting ground plane is half the 277 mm robot
  // height below that centre. The explicit config value is retained for easy
  // calibration if the Webots model origin is later verified to differ.
  return Number(localZ) - c.groundReferenceZ;
}

function seedKnowledgeFromCompleteCloud(c, endExclusive = null) {
  // Do not reconstruct obstacle classes from old WORLD cloud points here.
  // A world point does not retain the robot-ground reference that existed when
  // it was observed. Reclassifying it later against the current robot pose is
  // physically incorrect on uneven terrain. Global occupancy is therefore
  // learned at scan ingest time, where GT pose + local-ground reference are
  // simultaneously available. Loaded/saved occupancy snapshots retain their
  // already-classified grid cells through the normal snapshot mechanism.
  if (Number.isFinite(endExclusive)) N.knowledgeSeededCloudLength = Math.max(N.knowledgeSeededCloudLength, Math.min(mapStore.length, endExclusive));
  else N.knowledgeSeededCloudLength = Math.max(N.knowledgeSeededCloudLength, mapStore.length);
}

function ingest(points, meta) {
  const c = ensureGrid();
  const pose = state.navPose;
  if (!pose || pose.source !== 'ground_truth') return;
  const orientation = pose.orientation;
  const pz = pose.z ?? 0;
  const robotPose = { x: pose.x, y: pose.y, z: pz, theta: pose.theta };
  if (!N.home) N.home = { x: pose.x, y: pose.y, theta: pose.theta };

  // Navigation uses the configured area as a hard world boundary.
  const lidarWorld = robotToWorld([c.lidarOriginX, c.lidarOriginY, 0], robotPose, orientation);
  for (let i = 0; i < points.length; i += 3) {
    const p = [points[i], points[i + 1], points[i + 2]];
    const dx = p[0] - pose.x, dy = p[1] - pose.y;
    const worldRange = Math.hypot(dx, dy);
    // IMPORTANT: the 1 m dynamic map is a LOCAL safety layer only. The global
    // exploration map must retain the full configured LiDAR range; otherwise
    // A* can only see roughly one metre ahead and the robot repeatedly reaches
    // the edge of its tiny known island and stops.
    if (worldRange < c.globalScanMinRange || worldRange > c.globalScanMaxRange) continue;
    if (!insideArea(p[0], p[1], c, false)) continue;

    const rb = worldToRobot(p, robotPose, orientation);
    const relativeHeight = relativeGroundHeightFromRobotFrameZ(rb[2], c);
    const kind = classifyRelativeGroundHeight(relativeHeight, c);
    if (!kind) continue;

    const ce = cell(p[0], p[1]);
    if (!ce) continue;
    // Global free-space tracing starts at the physical LiDAR origin, not the
    // robot centre. The local map uses the same +0.254 m mounting geometry.
    traceRay(lidarWorld, p, kind);
  }
  computeStats(c);
}

function explorationTileKey(x, y, c) {
  const s = c.explorationTileSize;
  return `${Math.floor(x / s)},${Math.floor(y / s)}`;
}

function explorationTileCenter(key, c) {
  const [ix, iy] = key.split(',').map(Number);
  const s = c.explorationTileSize;
  return [(ix + 0.5) * s, (iy + 0.5) * s];
}

function createExplorationTile(c) {
  return { observations: 0, angleBins: new Set(), explored: false, coverage: 0, knownCells: 0, totalCells: 0, completedAt: null, unreachable: false, unreachableAtCoverage: 0 };
}

function tileCoverage(key, c) {
  if (!N.grid) return { coverage: 0, known: 0, total: 0 };
  const [tx, ty] = key.split(',').map(Number);
  const s = c.explorationTileSize;
  const minX = tx * s, maxX = (tx + 1) * s;
  const minY = ty * s, maxY = (ty + 1) * s;
  const ix0 = Math.max(0, Math.floor((minX - N.grid.originX) / N.grid.resolution));
  const iy0 = Math.max(0, Math.floor((minY - N.grid.originY) / N.grid.resolution));
  const ix1 = Math.min(N.grid.width - 1, Math.ceil((maxX - N.grid.originX) / N.grid.resolution) - 1);
  const iy1 = Math.min(N.grid.height - 1, Math.ceil((maxY - N.grid.originY) / N.grid.resolution) - 1);
  let total = 0, known = 0;
  for (let iy = iy0; iy <= iy1; iy++) for (let ix = ix0; ix <= ix1; ix++) {
    const w = world(ix, iy);
    if (!insideArea(w[0], w[1], c, false)) continue;
    total++;
    if (N.grid.seen[idx(ix, iy)]) known++;
  }
  return { coverage: total ? known / total : 0, known, total };
}

function refreshAllTileCoverage(c) {
  if (!N.exploration.totalTiles) resetExplorationProgress(c);
  let completed = 0;
  for (const [key, tile] of N.exploration.tiles) {
    const m = tileCoverage(key, c);
    tile.coverage = m.coverage;
    tile.knownCells = m.known;
    tile.totalCells = m.total;
    const was = !!tile.explored;
    tile.explored = m.total > 0 && m.coverage >= c.tileCoverageCompleteRate;
    // Red means the tile could not be reached or could not be completed from
    // the information available when it was attempted. Keep it red until the
    // tile is genuinely completed; do not clear it because of insignificant
    // coverage changes elsewhere in the map.
    if (tile.explored) tile.unreachable = false;
    if (!was && tile.explored) tile.completedAt = Date.now();
    if (tile.explored) completed++;
  }
  // Tiles are created for the whole selected area, so tiles that have never
  // received an observation still need their real coverage evaluated.
  for (const key of N.exploration.allTileKeys || []) {
    if (N.exploration.tiles.has(key)) continue;
    const m = tileCoverage(key, c);
    N.exploration.tiles.set(key, { ...createExplorationTile(c), coverage:m.coverage, knownCells:m.known, totalCells:m.total, explored:m.total > 0 && m.coverage >= c.tileCoverageCompleteRate });
    if (m.total > 0 && m.coverage >= c.tileCoverageCompleteRate) completed++;
  }
  N.exploration.exploredTiles = completed;
  N.exploration.coverage = N.exploration.totalTiles ? Math.min(1, completed / N.exploration.totalTiles) : 0;
}

function resetExplorationProgress(c) {
  N.exploration.tiles.clear();
  N.exploration.allTileKeys = [];
  N.exploration.totalTiles = 0;
  N.exploration.exploredTiles = 0;
  N.exploration.coverage = 0;
  N.exploration.lastPose = null;
  N.exploration.seededFromLoadedMap = false;
  N.exploration.seedPointCount = 0;
  const a = resolvedArea(c);
  const s = c.explorationTileSize;
  const minX = a.type === 'circle' ? a.center_x - a.radius_m : a.min_x;
  const maxX = a.type === 'circle' ? a.center_x + a.radius_m : a.max_x;
  const minY = a.type === 'circle' ? a.center_y - a.radius_m : a.min_y;
  const maxY = a.type === 'circle' ? a.center_y + a.radius_m : a.max_y;
  const ix0 = Math.floor(minX / s) - 1, ix1 = Math.floor(maxX / s) + 1;
  const iy0 = Math.floor(minY / s) - 1, iy1 = Math.floor(maxY / s) + 1;
  for (let iy = iy0; iy <= iy1; iy++) for (let ix = ix0; ix <= ix1; ix++) {
    const x = (ix + 0.5) * s, y = (iy + 0.5) * s;
    if (insideArea(x, y, c, false)) {
      const key = `${ix},${iy}`;
      N.exploration.allTileKeys.push(key);
      N.exploration.totalTiles++;
      N.exploration.tiles.set(key, createExplorationTile(c));
    }
  }
}

function markExplorationTile(x, y, c, observation = false, theta = 0) {
  if (!insideArea(x, y, c, false)) return;
  const key = explorationTileKey(x, y, c);
  let t = N.exploration.tiles.get(key);
  if (!t) {
    t = createExplorationTile(c);
    N.exploration.tiles.set(key, t);
  }
  if (observation) {
    t.observations++;
    const binSize = c.explorationMinViewAngleDeg;
    const deg = ((theta * 180 / Math.PI) % 360 + 360) % 360;
    t.angleBins.add(Math.floor(deg / binSize));
  }
}

function updateExplorationProgress(pose, c) {
  if (!pose) return;
  if (!N.exploration.totalTiles) resetExplorationProgress(c);
  const last = N.exploration.lastPose;
  let observation = false;
  if (!last) observation = true;
  else {
    const d = Math.hypot(pose.x - last.x, pose.y - last.y);
    const da = Math.abs(normalizeAngle(pose.theta - last.theta)) * 180 / Math.PI;
    observation = d >= c.explorationMinViewpointDistance || da >= c.explorationMinViewAngleDeg;
  }
  if (observation) N.exploration.lastPose = { x: pose.x, y: pose.y, theta: pose.theta };
  markExplorationTile(pose.x, pose.y, c, observation, pose.theta);
  refreshAllTileCoverage(c);
}

function seedExplorationFromLoadedSnapshot(c, keys) {
  if (!c.resumeLoadedExploration || !Array.isArray(keys) || !keys.length || N.exploration.seededFromLoadedMap) return false;
  if (!N.exploration.totalTiles) resetExplorationProgress(c);
  let used = 0;
  for (const entry of keys) {
    const key = typeof entry === 'string' ? entry : entry?.key;
    if (!key || !N.exploration.tiles.has(key)) continue;
    const t = N.exploration.tiles.get(key);
    t.explored = true;
    t.coverage = Math.max(Number(t.coverage || 0), Number(entry?.coverage || c.tileCoverageCompleteRate));
    used++;
  }
  N.exploration.seededFromLoadedMap = true;
  N.exploration.seedPointCount = used;
  refreshAllTileCoverage(c);
  log(`[AP] Resumed exploration progress from saved exploration tiles: ${N.exploration.exploredTiles}/${N.exploration.totalTiles} tiles (${(N.exploration.coverage*100).toFixed(1)}%)`, 'info');
  return true;
}

function seedExplorationFromLoadedTrajectory(c, trajectory) {
  if (!c.resumeLoadedExploration || !Array.isArray(trajectory) || !trajectory.length || N.exploration.seededFromLoadedMap) return;
  if (!N.exploration.totalTiles) resetExplorationProgress(c);
  let used = 0;
  for (const p of trajectory) {
    if (!Array.isArray(p) || p.length < 2) continue;
    const x = Number(p[0]), y = Number(p[1]);
    if (!Number.isFinite(x) || !Number.isFinite(y) || !insideArea(x, y, c, false)) continue;
    const key = explorationTileKey(x, y, c);
    const t = N.exploration.tiles.get(key);
    if (t) { t.observations = Math.max(t.observations, c.explorationMinObservations); used++; }
  }
  N.exploration.seededFromLoadedMap = true;
  N.exploration.seedPointCount = used;
  refreshAllTileCoverage(c);
  log(`[AP] Resumed exploration progress from loaded trajectory: ${N.exploration.exploredTiles}/${N.exploration.totalTiles} tiles (${(N.exploration.coverage*100).toFixed(1)}%)`, 'info');
}

function computeStats(c) {
  let known = 0, free = 0, obs = 0;
  let allowed = 0;
  for (let y = 0; y < N.grid.height; y++) {
    for (let x = 0; x < N.grid.width; x++) {
      const i = idx(x, y);
      const w = world(x, y);
      if (!insideArea(w[0], w[1], c, false)) continue;
      allowed++;
      if (!N.grid.seen[i]) continue;
      known++;
      if (N.grid.cells[i] === 1) free++;
      if (N.grid.cells[i] === 2) obs++;
    }
  }
  N.knownCells = known;
  N.freeCells = free;
  N.obstacleCells = obs;
  N.knowledgeCoverage = allowed ? Math.min(1, known / allowed) : 0;
  refreshAllTileCoverage(c);
  N.coverage = N.exploration.coverage;
  N.coverageProgress = N.coverage - (N.lastCoverage || 0);
  N.lastCoverage = N.coverage;
  N.missionComplete = N.coverage >= c.coverageTarget;
  state.explorationMap = {
    width: N.grid.width, height: N.grid.height, resolution: N.grid.resolution,
    originX: N.grid.originX, originY: N.grid.originY,
    cells: N.grid.cells, seen: N.grid.seen,
    area: resolvedArea(c),
    tileSize: c.explorationTileSize,
    tileCoverageCompleteRate: c.tileCoverageCompleteRate,
    tiles: N.exploration.tiles
  };
  refreshExplorationMap();
}

function isBlocked(ix, iy, c) {
  if (ix < 0 || iy < 0 || ix >= N.grid.width || iy >= N.grid.height) return true;
  const w = world(ix, iy);
  if (!insideArea(w[0], w[1], c, true)) return true;
  return N.grid.cells[idx(ix, iy)] === 2;
}

function rebuildInflated(c) {
  const g = N.grid;
  N.inflated.fill(0);
  const n = Math.ceil((c.robotRadius + c.safety) / g.resolution);
  for (let y = 0; y < g.height; y++) for (let x = 0; x < g.width; x++) {
    if (g.cells[idx(x,y)] !== 2) continue;
    for (let dy = -n; dy <= n; dy++) for (let dx = -n; dx <= n; dx++) {
      if (dx*dx + dy*dy > n*n) continue;
      const nx=x+dx, ny=y+dy;
      if (nx<0 || ny<0 || nx>=g.width || ny>=g.height) continue;
      const w=world(nx,ny);
      if (insideArea(w[0],w[1],c,true)) N.inflated[idx(nx,ny)] = 1;
    }
  }
  for (let y=0;y<g.height;y++) for (let x=0;x<g.width;x++) {
    const w=world(x,y);
    if (!insideArea(w[0],w[1],c,true)) N.inflated[idx(x,y)] = 1;
  }
}

function inflatedBlocked(ix, iy, allowUnknown = false) {
  if (ix < 0 || iy < 0 || ix >= N.grid.width || iy >= N.grid.height) return true;
  const i = idx(ix, iy);
  const seen = !!N.grid.seen[i];
  const cellType = N.grid.cells[i];

  // IMPORTANT: UNKNOWN is traversable for an explicit terminal target.
  // Only measured obstacles (type 2), conservative inflated safety cells, and
  // cells outside the configured navigation area are hard constraints.
  // Exploration keeps its stricter known-free policy by leaving allowUnknown=false.
  if (!allowUnknown && (!seen || cellType !== 1)) return true;
  if (seen && cellType === 2) return true;
  if (seen && cellType === 3) return true;
  return N.inflated[i] === 1;
}

function nearestFree(target, c, allowUnknown = false) {
  const t = cell(target[0], target[1]);
  if (!t) return null;
  if (!inflatedBlocked(t[0], t[1], allowUnknown)) return t;
  for (let r = 1; r < 20; r++) {
    for (let dy = -r; dy <= r; dy++) for (let dx = -r; dx <= r; dx++) {
      if (Math.abs(dx) !== r && Math.abs(dy) !== r) continue;
      const x = t[0] + dx, y = t[1] + dy;
      if (!inflatedBlocked(x, y, allowUnknown)) return [x, y];
    }
  }
  return null;
}

function heapPush(heap, item) {
  heap.push(item);
  let i = heap.length - 1;
  while (i > 0) {
    const p = (i - 1) >> 1;
    if (heap[p][0] <= item[0]) break;
    heap[i] = heap[p];
    i = p;
  }
  heap[i] = item;
}

function heapPop(heap) {
  if (!heap.length) return null;
  const root = heap[0];
  const last = heap.pop();
  if (heap.length && last) {
    let i = 0;
    while (true) {
      const l = i * 2 + 1;
      if (l >= heap.length) break;
      const r = l + 1;
      let child = l;
      if (r < heap.length && heap[r][0] < heap[l][0]) child = r;
      if (heap[child][0] >= last[0]) break;
      heap[i] = heap[child];
      i = child;
    }
    heap[i] = last;
  }
  return root;
}

function aStar(start, goal, c, allowUnknown = false) {
  if (!start || !goal) return null;

  // The planner uses an 8-connected grid. Octile distance is the exact
  // admissible lower bound for that motion model, so A* remains shortest-path
  // optimal while measured obstacles and inflated safety cells stay hard
  // constraints.
  const octileDistance = (x, y) => {
    const dx = Math.abs(goal[0] - x), dy = Math.abs(goal[1] - y);
    return Math.max(dx, dy) + (Math.SQRT2 - 1) * Math.min(dx, dy);
  };

  const stats = {
    allowUnknown: !!allowUnknown,
    expanded: 0,
    rejectedUnknown: 0,
    rejectedObstacle: 0,
    rejectedInflated: 0,
    rejectedOutside: 0,
    result: 'failed'
  };
  N.debugNav.targetAStar = stats;

  const blockedReason = (ix, iy) => {
    if (ix < 0 || iy < 0 || ix >= N.grid.width || iy >= N.grid.height) return 'outside';
    const i = idx(ix, iy);
    const seen = !!N.grid.seen[i];
    const cellType = N.grid.cells[i];
    if (!allowUnknown && !seen) return 'unknown';
    if (seen && cellType === 2) return 'obstacle';
    if (seen && cellType === 3) return 'inflated';
    if (N.inflated[i] === 1) return 'inflated';
    return null;
  };

  const startReason = blockedReason(start[0], start[1]);
  const goalReason = blockedReason(goal[0], goal[1]);
  if (startReason || goalReason) {
    for (const reason of [startReason, goalReason]) {
      if (reason === 'unknown') stats.rejectedUnknown++;
      else if (reason === 'obstacle') stats.rejectedObstacle++;
      else if (reason === 'inflated') stats.rejectedInflated++;
      else if (reason === 'outside') stats.rejectedOutside++;
    }
    stats.result = `failed:${startReason || goalReason}`;
    return null;
  }

  const w = N.grid.width, h = N.grid.height, total = w * h;
  const gScore = new Float64Array(total);
  const fScore = new Float64Array(total);
  const parent = new Int32Array(total);
  const closed = new Uint8Array(total);
  gScore.fill(Infinity);
  fScore.fill(Infinity);
  parent.fill(-1);

  const open = [];
  const sid = idx(start[0], start[1]);
  const gid = idx(goal[0], goal[1]);
  gScore[sid] = 0;
  fScore[sid] = octileDistance(start[0], start[1]);
  heapPush(open, [fScore[sid], sid]);

  const dirs = [
    [1,0,1],[-1,0,1],[0,1,1],[0,-1,1],
    [1,1,Math.SQRT2],[-1,1,Math.SQRT2],[1,-1,Math.SQRT2],[-1,-1,Math.SQRT2]
  ];

  let guard = 0;
  while (open.length && guard++ < total * 3) {
    const node = heapPop(open);
    if (!node) break;
    const current = node[1];
    if (closed[current]) continue;
    stats.expanded++;

    if (current === gid) {
      const path = [];
      let cur = gid;
      while (cur !== -1) {
        const x = cur % w, y = Math.floor(cur / w);
        path.push([x, y]);
        if (cur === sid) break;
        cur = parent[cur];
      }
      path.reverse();
      stats.result = 'found';
      return path;
    }

    closed[current] = 1;
    const cx = current % w, cy = Math.floor(current / w);

    for (const [dx, dy, cost] of dirs) {
      const nx = cx + dx, ny = cy + dy;
      const reason = blockedReason(nx, ny);
      if (reason) {
        if (reason === 'unknown') stats.rejectedUnknown++;
        else if (reason === 'obstacle') stats.rejectedObstacle++;
        else if (reason === 'inflated') stats.rejectedInflated++;
        else if (reason === 'outside') stats.rejectedOutside++;
        continue;
      }

      // Prevent diagonal corner cutting through two blocked cells.
      if (dx !== 0 && dy !== 0) {
        const r1 = blockedReason(cx + dx, cy);
        const r2 = blockedReason(cx, cy + dy);
        if (r1 || r2) {
          for (const r of [r1, r2]) {
            if (r === 'unknown') stats.rejectedUnknown++;
            else if (r === 'obstacle') stats.rejectedObstacle++;
            else if (r === 'inflated') stats.rejectedInflated++;
            else if (r === 'outside') stats.rejectedOutside++;
          }
          continue;
        }
      }

      const ni = idx(nx, ny);
      if (closed[ni]) continue;

      const tentative = gScore[current] + cost;
      if (tentative < gScore[ni]) {
        parent[ni] = current;
        gScore[ni] = tentative;
        fScore[ni] = tentative + octileDistance(nx, ny);
        heapPush(open, [fScore[ni], ni]);
      }
    }
  }
  return null;
}

function collectFrontiers(c, pose) {
  const start = cell(pose.x, pose.y);
  if (!start) return [];
  const candidates = [];
  const visited = new Uint8Array(N.grid.cells.length);
  const dirs = [[1,0],[-1,0],[0,1],[0,-1]];

  const isFrontierCell = (x, y) => {
    if (x < 1 || y < 1 || x >= N.grid.width-1 || y >= N.grid.height-1) return false;
    const i = idx(x,y);
    if (!N.grid.seen[i] || N.grid.cells[i] !== 1 || N.inflated[i]) return false;
    for (const [dx,dy] of dirs) {
      const nx=x+dx, ny=y+dy;
      if (!N.grid.seen[idx(nx,ny)] && insideArea(...world(nx,ny), c, false)) return true;
    }
    return false;
  };

  for (let y=1; y<N.grid.height-1; y++) {
    for (let x=1; x<N.grid.width-1; x++) {
      const seed=idx(x,y);
      if (visited[seed] || !isFrontierCell(x,y)) continue;

      const queue=[[x,y]];
      visited[seed]=1;
      const cluster=[];
      let unknownGain=0;

      while (queue.length && cluster.length < 500) {
        const [cx,cy]=queue.shift();
        if (!isFrontierCell(cx,cy)) continue;
        cluster.push([cx,cy]);

        // Count nearby unknown cells as the actual information gain.
        for (let dy=-2;dy<=2;dy++) for (let dx=-2;dx<=2;dx++) {
          const nx=cx+dx, ny=cy+dy;
          if (nx<0||ny<0||nx>=N.grid.width||ny>=N.grid.height) continue;
          if (!N.grid.seen[idx(nx,ny)] && insideArea(...world(nx,ny), c, false)) unknownGain++;
        }

        for (const [dx,dy] of dirs) {
          const nx=cx+dx, ny=cy+dy;
          if (nx<1||ny<1||nx>=N.grid.width-1||ny>=N.grid.height-1) continue;
          const ni=idx(nx,ny);
          if (!visited[ni] && isFrontierCell(nx,ny)) {
            visited[ni]=1;
            queue.push([nx,ny]);
          }
        }
      }

      if (cluster.length < c.frontierMinGain) continue;

      // Pick the frontier cell with the largest local unknown gain, not simply
      // the middle of the cluster.
      let best=cluster[0], bestGain=-1;
      for (const [cx,cy] of cluster) {
        let gain=0;
        for (let dy=-3;dy<=3;dy++) for (let dx=-3;dx<=3;dx++) {
          const nx=cx+dx, ny=cy+dy;
          if (nx<0||ny<0||nx>=N.grid.width||ny>=N.grid.height) continue;
          if (!N.grid.seen[idx(nx,ny)] && insideArea(...world(nx,ny), c, false)) gain++;
        }
        if (gain>bestGain) { bestGain=gain; best=[cx,cy]; }
      }

      const wp=world(best[0],best[1]);
      const distance=Math.hypot(best[0]-start[0],best[1]-start[1])*N.grid.resolution;
      if (distance < c.frontierMinDistance) continue;

      // Higher information gain is better; distance is only a cost.
      const gain=Math.max(bestGain, unknownGain / Math.max(1,cluster.length));
      const score=(distance+0.25)/(Math.max(1,gain));
      candidates.push({
        cell:best, world:wp, distance, gain:Math.round(gain),
        clusterSize:cluster.length, score
      });
    }
  }

  candidates.sort((a,b)=>b.gain/(a.distance+0.25)-a.gain/(b.distance+0.25));
  return candidates.slice(0, 40);
}

function nearestFrontier(c, pose) {
  const candidates=collectFrontiers(c,pose);
  N.frontierCandidates=candidates;
  N.frontierCount=candidates.length;
  return candidates.length ? candidates[0].cell : null;
}

function makeProbeGoal(pose, c) {
  const d = c.probeDistance;
  if (!N.localMap || d <= 0) return null;
  const offsets = [0];
  const step = Math.max(1, c.probeHeadingOffsetDeg);
  for (let i=1; i<=c.probeHeadingSamples; i++) {
    offsets.push(i*step, -i*step);
  }
  let best = null;
  let bestScore = -Infinity;
  for (const deg of offsets) {
    const rel = deg * Math.PI / 180;
    if (!boundarySafetyForHeading(rel, Math.min(d, c.localAvoidanceDistance), c, pose)) continue;
    const corridor = localCorridorDiagnostics(rel, Math.min(d, c.localAvoidanceDistance), c.robotRadius + c.safety * c.localCorridorHalfWidthScale, c);
    if (corridor.obstacle || corridor.margin) continue;
    const x = pose.x + Math.cos(pose.theta + rel) * d;
    const y = pose.y + Math.sin(pose.theta + rel) * d;
    if (!insideArea(x, y, c, true, pose)) continue;
    const score = (corridor.unknown ? 0.5 : 1.0) - Math.abs(rel) * c.localAvoidanceTurnPenalty;
    if (score > bestScore) { bestScore = score; best = [x,y]; }
  }
  return best;
}

function localCellBlocked(ix, iy, includeUnknown = true) {
  const g = N.localMap;
  if (!g || ix < 0 || iy < 0 || ix >= g.width || iy >= g.height) return true;
  const i = localIdx(ix, iy);
  // 2 = measured/inferred obstacle, 3 = measured/inferred safety margin.
  // These always block movement.
  if (g.cells[i] === 2 || g.cells[i] === 3) return true;
  // Unknown is uncertainty, not an obstacle. Callers decide whether they
  // require a fully known corridor. Exploration deliberately allows unknown
  // cells when there is no actual obstacle evidence.
  if (includeUnknown && !g.seen[i]) return true;
  return false;
}

function localClearanceAhead(theta, distance, halfWidth, includeUnknown = true, c = N.config || cfg()) {
  const g = N.localMap;
  if (!g) return false;

  const step = Math.max(g.resolution, c.localClearanceStepM);
  const halfW = Math.max(g.resolution, halfWidth, c.robotHalfWidth + c.safety);
  const halfL = Math.max(g.resolution, c.robotHalfLength + c.safety);
  const heading = theta;
  const lateralStep = Math.max(g.resolution, c.localLateralStepM);
  for (let centerD = 0; centerD <= distance; centerD += step) {
    for (let forward = -halfL; forward <= halfL; forward += step) {
      const d = centerD + forward;
      if (d < -g.radius || d > g.radius) continue;
      for (let s = -halfW; s <= halfW; s += lateralStep) {
        const x = d * Math.cos(heading) - s * Math.sin(heading);
        const y = d * Math.sin(heading) + s * Math.cos(heading);
        const ce = localCell(x, y);
        if (!ce || localCellBlocked(ce[0], ce[1], includeUnknown)) return false;
      }
    }
  }
  return true;
}

function localCorridorDiagnostics(theta, distance, halfWidth, c = N.config || cfg()) {
  const g = N.localMap;
  if (!g) return { clear:false, obstacle:false, margin:false, unknown:true };
  const step = Math.max(g.resolution, c.localClearanceStepM);
  const lateral = Math.max(g.resolution, halfWidth, c.robotHalfWidth + c.safety);
  const halfLength = Math.max(g.resolution, c.robotHalfLength + c.safety);
  const lateralStep = Math.max(g.resolution, c.localLateralStepM);
  let unknown = false;

  // Test the full rectangular robot footprint along the candidate motion
  // corridor. This prevents accepting a gap that is wide enough at the centre
  // line but too narrow for the robot body.
  for (let centerD = 0; centerD <= distance; centerD += step) {
    for (let forward = -halfLength; forward <= halfLength; forward += step) {
      const d = centerD + forward;
      if (d < -g.radius || d > g.radius) continue;
      for (let s = -lateral; s <= lateral; s += lateralStep) {
        const ce = localCell(d * Math.cos(theta) - s * Math.sin(theta), d * Math.sin(theta) + s * Math.cos(theta));
        if (!ce) return { clear:false, obstacle:false, margin:false, unknown:true };
        const i = localIdx(ce[0], ce[1]);
        if (g.cells[i] === 2) return { clear:false, obstacle:true, margin:false, unknown };
        if (g.cells[i] === 3) return { clear:false, obstacle:false, margin:true, unknown };
        if (!g.seen[i]) unknown = true;
      }
    }
  }
  return { clear:true, obstacle:false, margin:false, unknown };
}

function normalizeAngle(a) {
  return Math.atan2(Math.sin(a), Math.cos(a));
}

function boundarySafetyForHeading(relativeHeading, distance, c, pose) {
  const a = resolvedArea(c, pose);
  const heading = pose.theta + relativeHeading;
  const samples = Math.max(2, Math.ceil(distance / c.boundarySampleStepM));
  for (let k = 1; k <= samples; k++) {
    const d = distance * (k / samples);
    const x = pose.x + Math.cos(heading) * d;
    const y = pose.y + Math.sin(heading) * d;
    if (!insideArea(x, y, c, true, pose)) return false;
    if (a.type === 'circle') {
      const safeR = Math.max(0, a.radius_m - (c.robotRadius + c.safety) - c.boundaryBuffer);
      if ((x-a.center_x)**2 + (y-a.center_y)**2 > safeR*safeR) return false;
    }
  }
  return true;
}

function inwardHeading(pose, c) {
  const a = resolvedArea(c, pose);
  if (a.type === 'circle') {
    const inward = Math.atan2(a.center_y - pose.y, a.center_x - pose.x);
    return normalizeAngle(inward - pose.theta);
  }
  const dxLeft = pose.x - a.min_x, dxRight = a.max_x - pose.x;
  const dyBottom = pose.y - a.min_y, dyTop = a.max_y - pose.y;
  let targetX = pose.x, targetY = pose.y;
  const m = Math.min(dxLeft, dxRight, dyBottom, dyTop);
  if (m === dxLeft) targetX -= 1;
  else if (m === dxRight) targetX += 1;
  else if (m === dyBottom) targetY -= 1;
  else targetY += 1;
  return normalizeAngle(Math.atan2(targetY-pose.y, targetX-pose.x) - pose.theta);
}

function chooseLocalAvoidanceTurn(desiredRelativeHeading = 0, pose = state.navPose, c = N.config || cfg()) {
  const g = N.localMap;
  if (!g || !pose) return 0.0;

  // Evaluate the actual local occupancy map instead of six hard-coded turn
  // angles. Unknown is allowed for exploration, but obstacles and safety
  // margins are never allowed. Prefer directions that still make progress
  // toward the global target and remain inside the mission boundary.
  let best = 0.0;
  let bestScore = -Infinity;
  const maxDistance = Math.min(c.localAvoidanceDistance, g.radius - c.localClearanceStartM);
  const candidates = [];
  for (let deg = c.localAvoidanceHeadingMinDeg; deg <= c.localAvoidanceHeadingMaxDeg; deg += c.localAvoidanceHeadingStepDeg) candidates.push(deg * Math.PI / 180);

  const boundary = areaDiagnostics(c, pose);
  const nearBoundary = boundary.safeDistance !== null && boundary.safeDistance < c.boundaryLookahead;
  if (nearBoundary) candidates.push(inwardHeading(pose, c));

  for (const a of candidates) {
    const corridor = localCorridorDiagnostics(a, maxDistance, c.robotRadius + c.safety * c.localCorridorHalfWidthScale);
    if (corridor.obstacle || corridor.margin) continue;
    if (!boundarySafetyForHeading(a, maxDistance, c, pose)) continue;

    let clearanceScore = 0;
    const step = Math.max(g.resolution, c.localClearanceStepM);
    for (let d = c.localClearanceStartM; d <= maxDistance; d += step) {
      const ce = localCell(d*Math.cos(a), d*Math.sin(a));
      if (!ce) break;
      const i = localIdx(ce[0], ce[1]);
      if (g.cells[i] === 2 || g.cells[i] === 3) break;
      clearanceScore += 1;
    }

    const headingPenalty = Math.abs(normalizeAngle(a - desiredRelativeHeading));
    const turnPenalty = Math.abs(a) * c.localAvoidanceTurnPenalty;
    const unknownPenalty = corridor.unknown ? c.localAvoidanceUnknownPenalty : 0;
    const boundaryBonus = nearBoundary && Math.abs(normalizeAngle(a - inwardHeading(pose, c))) < (c.localAvoidanceHeadingStepDeg * Math.PI / 180) ? c.localAvoidanceBoundaryBonus : 0;
    const score = clearanceScore * c.localAvoidanceClearanceWeight - headingPenalty * c.localAvoidanceHeadingPenalty - turnPenalty - unknownPenalty + boundaryBonus;
    if (score > bestScore) {
      bestScore = score;
      best = a;
    }
  }
  return best;
}

function nearestPathIndex(path, pose) {
  // The global path array is only rebuilt when plan() reruns A* (goal
  // reached/invalidated/etc). While the robot is healthily following an
  // existing path, the SAME array is reused for many ticks, so any
  // lookahead/corridor check must re-anchor to the robot's current progress
  // along it rather than reading a fixed absolute index from the start —
  // otherwise the "carrot" point never advances and the robot ends up
  // orbiting a single stale waypoint forever.
  let bestIdx = 0, bestDist = Infinity;
  for (let i = 0; i < path.length; i++) {
    const wp = world(path[i][0], path[i][1]);
    const d = (wp[0] - pose.x) ** 2 + (wp[1] - pose.y) ** 2;
    if (d < bestDist) { bestDist = d; bestIdx = i; }
  }
  return bestIdx;
}

function localPathCorridorBlocked(path, pose, c, nearestIdx = 0) {
  if (!path || path.length < 2 || !N.localMap) return null;
  const samples = Math.min(path.length - 1 - nearestIdx, Math.max(3, Math.ceil(c.localAvoidanceDistance / c.resolution)));
  for (let i = 1; i <= samples; i++) {
    const p = path[Math.min(path.length - 1, nearestIdx + i)];
    const wp = world(p[0], p[1]);
    const dx = wp[0] - pose.x, dy = wp[1] - pose.y;
    const heading = Math.atan2(dy, dx);
    const rel = normalizeAngle(heading - pose.theta);
    const d = Math.min(c.localAvoidanceDistance, Math.max(0.20, Math.hypot(dx,dy)));
    const corridor = localCorridorDiagnostics(rel, d, c.robotRadius + c.safety * c.localCorridorHalfWidthScale);
    if (corridor.obstacle || corridor.margin || !boundarySafetyForHeading(rel, d, c, pose)) {
      return { relativeHeading: rel, distance: d, corridor };
    }
  }
  return null;
}

function navigationCommandSource() {
  return N.externalTarget && N.externalTargetStatus !== 'reached' ? 'terminal_target' : 'autopilot';
}

function finishExplicitTarget(c) {
  if (!N.externalTarget) return false;
  const target = N.externalTarget.slice();
  sendStop('terminal_target');
  N.lastCommand = { v: 0, omega: 0, sent: true, at: performance.now() };
  N.externalTargetStatus = 'reached';
  N.targetFailureHold = false;
  N.targetValidation = {
    ...(N.targetValidation || {}),
    x: target[0], y: target[1], stage: 'REACHED', reasonCode: 'WITHIN_GOAL_TOLERANCE'
  };
  N.path = [];
  N.goal = null;
  N.localPathInvalidated = false;
  reportExternalTargetStatus('reached');
  if (c.continueExplorationAfterTargetReached) {
    N.mode = 'REPLAN';
    N.lastPlanReason = `TARGET REACHED — RESUMING FULL-MAP EXPLORATION (${target[0].toFixed(2)}, ${target[1].toFixed(2)})`;
  } else {
    N.mode = 'TARGET REACHED';
    N.lastPlanReason = `TARGET REACHED — ${target[0].toFixed(2)}, ${target[1].toFixed(2)}`;
  }
  return true;
}

function requestedTargetDistance(pose) {
  // A completed target remains available for UI/diagnostics, but it must not
  // continue to act as the active arrival constraint during exploration.
  if (!N.externalTarget || !pose || N.externalTargetStatus === 'reached' || N.externalTargetStatus === 'blocked') return null;
  return Math.hypot(N.externalTarget[0] - pose.x, N.externalTarget[1] - pose.y);
}

function projectedMotionBlocked(relativeHeading, speed, c) {
  // A currently clear cell is not enough: check the footprint over the short
  // distance travelled before the next scan/control update as well.
  const horizon = Math.max(
    c.localClearanceStartM,
    c.localForwardSafetyDistanceM,
    Math.abs(Number(speed) || 0) * 0.5
  );
  return localCorridorDiagnostics(
    relativeHeading,
    Math.min(horizon, Math.max(c.localClearanceStartM, c.localAvoidanceDistance)),
    c.robotRadius + c.safety * c.localCorridorHalfWidthScale,
    c
  );
}

function stopAndRotate(relativeHeading, c, reason) {
  // Local obstacle recovery is a controlled escape arc, not a stationary
  // rotation. Keep forward motion deliberately reduced and scale it down as
  // the escape heading becomes more lateral, while the selected corridor
  // remains hard-rejected if it intersects an obstacle or safety margin.
  const headingScale = Math.max(0, Math.cos(Math.min(Math.PI / 2, Math.abs(relativeHeading))));
  const v = Math.max(0, Math.min(
    c.vmax,
    c.vmax * c.localAvoidanceLinearSpeedScale * (0.25 + 0.75 * headingScale)
  ));
  const omega = Math.max(-c.wmax, Math.min(c.wmax, relativeHeading * c.localAvoidanceAngularGain));
  const sent = recordCommand(v, omega);
  N.lastPlanReason = sent ? `${reason} — FORWARD ESCAPE / REPLAN` : `${reason} — AVOIDANCE SEND FAILED`;
  return sent;
}

function recordCommand(v, omega) {
  N.LAST_VALID_CMD_MS = performance.now();
  const source = navigationCommandSource();
  const accepted = sendVelocity(v, omega, source, 700);
  if (!accepted) {
    N.lastError = `Command arbitration rejected source=${source}`;
    log(`[AP] VELOCITY REJECTED BY COMMAND ARBITRATION — source=${source}`, 'warn');
  }
  const sent = accepted;
  N.lastCommand = { v, omega, sent, at: performance.now() };
  N.commandCount++;
  if (!sent && !N.lastError) { N.lastError = 'WebSocket not OPEN or command arbitration rejected'; log('[AP] VELOCITY SEND FAILED OR REJECTED', 'err'); }
  return sent;
}

function commandProbe(pose, c) {
  if (!N.goal) return false;
  const dx = N.goal[0] - pose.x, dy = N.goal[1] - pose.y;
  const desired = Math.atan2(dy, dx);
  const err = normalizeAngle(desired - pose.theta);
  const corridor = localCorridorDiagnostics(
    err,
    Math.min(c.probeDistance, c.radius - c.localClearanceStartM),
    c.robotRadius + c.safety * c.localCorridorHalfWidthScale
  );
  const clear = !corridor.obstacle && !corridor.margin && boundarySafetyForHeading(err, Math.min(c.probeDistance, c.radius - c.localClearanceStartM), c, pose);
  N.localClear = clear;
  N.localBlockedReason = clear
    ? (corridor.unknown ? 'UNKNOWN / PROBING ALLOWED' : 'CLEAR')
    : (corridor.obstacle ? 'OBSTACLE' : corridor.margin ? 'SAFETY MARGIN' : 'AREA BOUNDARY');
  if (!clear) {
    N.mode = 'LOCAL AVOIDANCE';
    const turn = chooseLocalAvoidanceTurn(err, pose, c);
    const sent = stopAndRotate(turn, c, N.localBlockedReason);
    N.lastPlanReason = sent ? 'PROBE LOCAL AVOIDANCE — PATH INVALIDATED' : 'PROBE AVOIDANCE SEND FAILED';
    N.path = [];
    N.localPathInvalidated = true;
    return sent;
  }
  N.mode = 'PROBING';
  N.probeCount++;
  const headingScale = Math.max(0.0, Math.cos(Math.min(Math.PI / 2, Math.abs(err))));
  let v = c.vmax * c.probeSpeedScale * (0.25 + 0.75 * headingScale);
  if (Math.abs(err) > c.probeSlowHeadingRad) v *= c.probeSlowSpeedScale;
  const omega = Math.max(-c.wmax, Math.min(c.wmax, err * c.probeAngularGain));
  const sent = recordCommand(v, omega);
  N.lastPlanReason = sent ? 'FORWARD PROBE' : 'FORWARD PROBE SEND FAILED';
  return sent;
}

function commandBoundaryRecovery(pose, c) {
  if (!N.goal) { sendStop(navigationCommandSource()); return false; }
  const dx=N.goal[0]-pose.x, dy=N.goal[1]-pose.y;
  const dist=Math.hypot(dx,dy);
  if (dist <= c.outsideRecoveryWaypointTolerance) {
    N.boundaryRecoveryPath=[];
    N.boundaryRecoveryIndex=0;
    N.goal=null;
    N.path=[];
    N.mode='REPLAN';
    N.localPathInvalidated=false;
    N.lastPlanReason='BACK INSIDE AREA — RESUME EXPLORATION';
    sendStop(navigationCommandSource());
    return true;
  }
  const desired=Math.atan2(dy,dx);
  const err=normalizeAngle(desired-pose.theta);
  const omega=Math.max(-c.wmax,Math.min(c.wmax,err*c.pathAngularGain));
  const v=c.vmax*c.boundaryRecoverySpeedScale*(0.25+0.75*Math.max(0,Math.cos(Math.min(Math.PI/2,Math.abs(err)))));
  const sent=recordCommand(Math.max(0,Math.min(c.vmax,v)),omega);
  N.lastPlanReason=sent?'RETURNING ALONG PREVIOUS PATH':'BOUNDARY RECOVERY SEND FAILED';
  return sent;
}

function commandFromPath(path, pose, c) {
  if (!path || path.length < 2) {
    N.lastPlanReason = 'NO PATH';
    return false;
  }

  // The pursuit lookahead point must sit meaningfully ahead of the robot's
  // own footprint, or a normal 8-direction A* zig-zag turns into large
  // relative heading swings every replan cycle and the robot just rotates
  // in place chasing a point that's practically under it. Enforce a floor
  // of ~1.5x the robot+safety radius regardless of the configured value.
  const minLookaheadM = (c.robotRadius + c.safety) * 1.5;
  const effectiveLookaheadM = Math.max(c.lookahead, minLookaheadM);
  const lookaheadCells = Math.max(2, Math.floor(effectiveLookaheadM / c.resolution));

  // Re-anchor to the robot's actual progress along the path before looking
  // ahead — the array itself is only rebuilt on a full replan, so on every
  // other tick we must find where the robot currently is on it.
  const nearestIdx = nearestPathIndex(path, pose);
  const requestedDistance = requestedTargetDistance(pose);
  const terminalApproach = requestedDistance !== null && requestedDistance <= c.terminalApproachDistance;
  const target = path[Math.min(path.length - 1, nearestIdx + lookaheadCells)];
  const wp = terminalApproach ? N.externalTarget : world(target[0], target[1]);
  const dx = wp[0] - pose.x, dy = wp[1] - pose.y;
  const desired = Math.atan2(dy, dx);
  const err = normalizeAngle(desired - pose.theta);
  const dist = Math.hypot(dx, dy);

  const area = areaDiagnostics(c, pose);
  if (!area.inside) {
    N.mode = 'OUTSIDE AREA';
    N.path = [];
    N.goal = null;
    sendStop(navigationCommandSource());
    N.lastPlanReason = 'HARD AREA BOUNDARY — ROBOT OUTSIDE';
    return false;
  }

  // Hard boundary guard: reject commands that would move the robot footprint
  // outside the configured mission circle/rectangle during the short horizon.
  const boundaryHorizon = Math.min(c.boundaryLookahead, Math.max(c.localClearanceStartM, dist));
  const boundarySafe = boundarySafetyForHeading(err, boundaryHorizon, c, pose);

  // Check the actual local 1 m map against the upcoming global path. This is
  // deliberately separate from the global A* grid: a new local obstacle can
  // invalidate a previously valid global path before the next global replan.
  const blocked = localPathCorridorBlocked(path, pose, c, nearestIdx);
  if (!boundarySafe || blocked) {
    const desiredRelative = blocked?.relativeHeading ?? err;
    const turn = chooseLocalAvoidanceTurn(desiredRelative, pose, c);
    const omega = Math.max(-c.wmax, Math.min(c.wmax, turn * c.localAvoidanceAngularGain));
    const reason = !boundarySafe ? 'AREA BOUNDARY' : (blocked?.corridor?.obstacle ? 'OBSTACLE' : 'SAFETY MARGIN');
    N.localClear = false;
    N.localBlockedReason = reason;
    N.debugNav.lastSafetyResult = {clear:false, corridor:blocked?.corridor || null, reason};
    N.debugNav.stopReason = reason;

    // Do not keep following the old path after a local blockage. Invalidate it
    // so the next scan immediately rebuilds the global plan around the new
    // evidence instead of steering back into the obstacle.
    N.path = [];
    N.localPathInvalidated = true;
    N.mode = reason === 'AREA BOUNDARY' ? 'BOUNDARY RECOVERY' : 'LOCAL AVOIDANCE';
    const sent = stopAndRotate(turn, c, reason);
    N.lastPlanReason = sent ? `${reason} — PATH INVALIDATED / REPLAN` : `${reason} — AVOIDANCE SEND FAILED`;
    return sent;
  }

  N.localClear = true;
  N.localBlockedReason = 'CLEAR';
  N.debugNav.lastSafetyResult = {clear:true, reason:'CLEAR'};
  N.mode = 'NAVIGATING';
  N.localPathInvalidated = false;

  const headingScale = Math.max(0.0, Math.cos(Math.min(Math.PI / 2, Math.abs(err))));
  let v = c.vmax * (0.20 + 0.80 * headingScale);
  if (Math.abs(err) > c.pathSlowHeadingRad) v *= c.pathSlowSpeedScale;
  if (requestedDistance !== null && requestedDistance < c.goalReach * c.goalSlowdownDistanceScale) v *= c.goalSlowdownSpeedScale;
  if (area.safeDistance !== null && area.safeDistance < c.boundaryLookahead) v *= c.boundarySlowSpeedScale;
  // Do not keep translating while an explicit target is strongly lateral or
  // behind the robot. Rotate in place until the target/path bearing is aligned;
  // otherwise the minimum forward term above can carry the robot away from the
  // target and make subsequent replanning start from a worse position.
  if (N.externalTarget && N.externalTargetStatus !== 'reached' && N.externalTargetStatus !== 'blocked' &&
      Math.abs(err) > c.targetHeadingStopRad) v = 0;
  v = Math.max(0, Math.min(c.vmax, v));

  const omega = Math.max(-c.wmax, Math.min(c.wmax, err * c.pathAngularGain));
  // Arrival is measured against the requested target, never the lookahead
  // waypoint or the safe A* planning goal.
  if (requestedDistance !== null && requestedDistance <= c.goalReach) {
    if (N.externalTarget) {
      N.debugNav.lastPlanResult = 'EXPLICIT_TARGET_REACHED';
      finishExplicitTarget(c);
    } else {
      sendStop(navigationCommandSource());
      N.lastCommand = { v: 0, omega: 0, sent: true, at: performance.now() };
      N.path = [];
      N.goal = null;
      N.localPathInvalidated = false;
      N.mode = 'REPLAN';
      N.debugNav.lastPlanResult = 'DESTINATION_REACHED';
      N.lastPlanReason = 'DESTINATION REACHED — SELECT NEW EXPLORATION TARGET';
    }
    return true;
  }

  const projected = projectedMotionBlocked(err, v, c);
  if (v > 0 && (projected.obstacle || projected.margin || !projected.clear)) {
    N.localClear = false;
    N.localBlockedReason = projected.obstacle ? 'PROJECTED OBSTACLE' : projected.margin ? 'PROJECTED SAFETY MARGIN' : 'PROJECTED UNKNOWN';
    N.debugNav.lastSafetyResult = {clear:false, corridor:projected, reason:N.localBlockedReason};
    N.debugNav.stopReason = N.localBlockedReason;
    N.path = [];
    N.localPathInvalidated = true;
    N.mode = 'LOCAL AVOIDANCE';
    const turn = chooseLocalAvoidanceTurn(err, pose, c);
    return stopAndRotate(turn, c, N.localBlockedReason);
  }

  const sent = recordCommand(v, omega);
  N.lastPlanReason = sent ? 'PATH FOLLOWING' : 'VELOCITY SEND FAILED';
  N.debugNav.commandReason = N.lastPlanReason;
  N.debugNav.lastGoalDistance = dist;
  N.debugNav.lastHeadingError = err;
  return sent;
}

function emitRecoveryConfigDebug() {
  if (N._recoveryConfigLogged) return;
  N._recoveryConfigLogged = true;
  const t = Number(N.config?.localInferenceCoverage ?? 0);
  log(`[RECOVERY-CONFIG] YAML/runtime threshold=${(t*100).toFixed(1)}% | globalCoverageGate=false | raySectionCells=${N.config?.localRecoveryRaySectionCells ?? 0} | minKnownNeighbors=${N.config?.localInferenceMinKnownNeighbors ?? 0} | neighborRadiusCells=${N.config?.localInferenceNeighborRadiusCells ?? 1} | tie=${N.config?.localInferenceTieClass ?? 'clear'}`, 'info');
}

function emitNavigationDebug() {
  emitRecoveryConfigDebug();
  const now = performance.now();
  if (now - (N.debugLastAt || 0) < (N.config?.debugIntervalMs || 250)) return;
  N.debugLastAt = now;
  const lb = N.localBackup;
  log(`[AP-DBG] scan=${N.debugNav.scanUpdateId} mode=${N.mode} cov=${(N.coverage*100).toFixed(1)}% knowledgeCov=${(N.knowledgeCoverage*100).toFixed(1)}% tiles=${N.exploration.exploredTiles}/${N.exploration.totalTiles} localCov=${(lb.coverage*100).toFixed(1)}% eligible=${lb.eligible} rays=${lb.raySections} rayEligible=${lb.rayEligible} neighborReject=${lb.neighborRejected} inferred=${lb.inferred} stage=${lb.lastStage} reason=${lb.lastReason}`, 'info');
}


async function publishRuntimeTestTelemetry() {
  const now = performance.now();
  if (now - (N.runtimeTestLastAt || 0) < 750) return;
  N.runtimeTestLastAt = now;

  const arb = getCommandArbitrationState();
  const telemetry = {
    client_time_ms: Date.now(),
    viewer_ws_open: !!(state.ws && state.ws.readyState === WebSocket.OPEN),
    scan_last_age_ms: state.simulationLastTelemetryAt ? Math.max(0, now - Number(state.simulationLastTelemetryAt)) : null,
    message_count: Number(state.msgCount || 0),
    point_count: Number(mapStore.length || 0),
    nav: {
      active: !!N.active,
      mode: N.mode,
      external_target: N.externalTarget ? {x:N.externalTarget[0], y:N.externalTarget[1]} : null,
      external_target_id: N.externalTargetId,
      external_target_status: N.externalTargetStatus,
      goal: N.goal ? {x:N.goal[0], y:N.goal[1]} : null,
      path_cells: N.path.length,
      last_plan_reason: N.lastPlanReason,
      last_error: N.lastError || null,
      plan_attempts: N.planAttempts,
      probe_count: N.probeCount,
      replan_count: N.debugNav.replans,
      plan_update_id: N.debugNav.planUpdateId,
      scan_update_id: N.debugNav.scanUpdateId,
      last_goal_distance: N.debugNav.lastGoalDistance,
      last_heading_error_deg: Number.isFinite(N.debugNav.lastHeadingError) ? (N.debugNav.lastHeadingError * 180 / Math.PI) : null,
      pose: state.navPose ? {x:state.navPose.x, y:state.navPose.y, theta_deg: state.navPose.theta * 180 / Math.PI} : null,
      last_safety: N.debugNav.lastSafetyResult,
      target_astar: N.debugNav.targetAStar,
      target_validation: N.targetValidation,
      target_flow: {
        mode: N.mode,
        status: N.externalTargetStatus,
        id: N.externalTargetId,
        owner: arb.owner || null,
        plan_result: N.debugNav.lastPlanResult || null
      },
      area_anchor: N.areaAnchor ? {x:N.areaAnchor.x, y:N.areaAnchor.y} : null,
      area: N.config ? (() => {
        const a = resolvedArea(N.config, state.navPose || N.home);
        const margin = N.config.robotRadius + N.config.safety;
        return a.type === 'circle'
          ? {type:'circle', center_x:a.center_x, center_y:a.center_y, radius_m:a.radius_m, safe_radius_m: Math.max(0, a.radius_m - margin)}
          : {type:'rectangle', min_x:a.min_x, max_x:a.max_x, min_y:a.min_y, max_y:a.max_y, margin_m: margin};
      })() : null,
      target_failure_hold: !!N.targetFailureHold,
      local_path_invalidated: !!N.localPathInvalidated,
      state_transition_count: N.modeLoop.transitions.length,
      coverage: N.coverage,
      knowledge_coverage: N.knowledgeCoverage,
      known_cells: N.knownCells,
      free_cells: N.freeCells,
      obstacle_cells: N.obstacleCells,
      explored_tiles: N.exploration.exploredTiles,
      total_tiles: N.exploration.totalTiles,
      local: {
        coverage: N.localBackup.coverage,
        scan_points: N.localBackup.scanPoints,
        cloud_points: N.localBackup.cloudPoints,
        real_cells: N.localBackup.realCells,
        unknown_cells: N.localBackup.unknownCells,
        inferred: N.localBackup.inferred,
        ray_sections: N.localBackup.raySections,
        ray_rejected: N.localBackup.rayRejected,
        neighbor_rejected: N.localBackup.neighborRejected,
      },
      last_command: N.lastCommand,
    },
    arbitration: {
      owner: arb.owner,
      owner_until_perf_ms: arb.until,
      source: N.externalTarget && N.externalTargetStatus !== 'reached' ? 'terminal_target' : (N.active ? 'autopilot' : null),
    },
  };

  try {
    const r = await fetch('/api/runtime-tests/telemetry', {
      method: 'POST',
      headers: {'Content-Type':'application/json'},
      body: JSON.stringify(telemetry),
      keepalive: true,
    });
    if (!r.ok) throw new Error(`HTTP ${r.status}`);
    N.runtimeTestErrors = 0;
  } catch (_) {
    // Runtime tests are intentionally optional and must never interfere with navigation.
    N.runtimeTestErrors++;
  }
}

function updateUI() {
  state.autoPilot = N.active;
  state.navMode = N.mode;
  state.navCoverage = N.coverage;
  state.navGoal = N.goal;
  state.navPath = N.path.map(p => world(p[0], p[1]));
  state.navPathLength = N.path.length;

  const badge = document.getElementById('autopilot-badge');
  if (badge) { badge.classList.toggle('active', N.active); badge.textContent = N.active ? `AUTO PILOT — ${N.mode}` : 'AUTO PILOT OFF'; }
  const mode = document.getElementById('nav-mode'); if (mode) mode.textContent = N.mode;
  const opMode = document.getElementById('operator-mode'); if (opMode) opMode.textContent = N.mode;
  const cov = document.getElementById('nav-coverage'); if (cov) cov.textContent = `${(N.coverage*100).toFixed(1)}%`;
  const goal = document.getElementById('nav-goal'); if (goal) goal.textContent = N.goal ? `${N.goal[0].toFixed(2)}, ${N.goal[1].toFixed(2)}` : '—';
  publishRuntimeTestTelemetry();

  const pose = state.navPose;
  const d = {
    active: N.active,
    mode: N.mode,
    pose: pose ? { x: pose.x, y: pose.y, z: pose.z ?? null, theta: pose.theta } : null,
    mission: { state: N.missionComplete ? 'COMPLETE' : (N.active ? 'EXPLORING' : 'IDLE'), targetKnownPercent: N.config ? N.config.targetKnownPercent : null, knownPercent: N.coverage*100, unknownPercent: (1-N.coverage)*100 },
    map: { known: N.knownCells, free: N.freeCells, obstacles: N.obstacleCells, coverage: N.knowledgeCoverage, explorationCoverage: N.coverage, exploredTiles: N.exploration.exploredTiles, totalExplorationTiles: N.exploration.totalTiles, cloud: mapStore.length },
    local: { clear: N.localClear, reason: N.localBlockedReason, radius: N.config?.radius, resolution: N.config?.localMapResolution, lidarOrigin: [N.config?.lidarOriginX, N.config?.lidarOriginY], minRadius: N.config?.minRadius, robotHeight: N.config?.robotHeight, groundReferenceZ: N.config?.groundReferenceZ, obstaclePositiveMin: N.config?.obstacleAboveGroundMin, obstaclePositiveMax: N.config?.obstacleAboveGroundMax, obstacleNegativeMax: N.config?.obstacleBelowGroundMax,
      backupInference: {
        enabled: N.config?.localInferenceEnabled,
        persistentApplied: N.localBackup.persistentApplied || 0,
        persistentCandidates: N.localBackup.persistentCandidates || 0,
        overwrittenByReal: N.localBackup.overwrittenByReal || 0,
        coverage: N.localBackup.coverage,
        coverageBeforeInference: N.localBackup.coverageBeforeInference,
        coverageAfterInference: N.localBackup.coverageAfterInference,
        threshold: N.config?.localInferenceCoverage,
        eligible: N.localBackup.eligible,
        realKnown: N.localBackup.realKnown,
        totalCells: N.localBackup.totalCells,
        realCells: N.localBackup.realCells,
        unknownCells: N.localBackup.unknownCells,
        minKnownNeighbors: N.config?.localInferenceMinKnownNeighbors,
        inferred: N.localBackup.inferred,
        passes: N.localBackup.passes,
        raySections: N.localBackup.raySections || 0,
        rayEligible: N.localBackup.rayEligible || 0,
        rayRejected: N.localBackup.rayRejected || 0,
        neighborRejected: N.localBackup.neighborRejected || 0,
        voteClear: N.localBackup.voteClear || 0,
        voteObstacle: N.localBackup.voteObstacle || 0,
        voteMargin: N.localBackup.voteMargin || 0,
        raySectionCells: N.config?.localRecoveryRaySectionCells || 2,
        sectionCandidates: N.localBackup.sectionCandidates || 0,
        sectionEligible: N.localBackup.sectionEligible || 0,
        sectionKnownCells: N.localBackup.sectionKnownCells || 0,
        sectionTotalCells: N.localBackup.sectionTotalCells || 0,
        sectionUnknownCells: N.localBackup.sectionUnknownCells || 0,
        sectionCoverage: N.localBackup.sectionCoverage || 0,
        sectionRejectNoRay: N.localBackup.sectionRejectNoRay || 0,
        sectionRejectNeighbors: N.localBackup.sectionRejectNeighbors || 0,
        sectionInferred: N.localBackup.sectionInferred || 0,
        sectionPasses: N.localBackup.sectionPasses || 0,
        predictionsStored: N.localBackup.predictionsStored || 0,
        predictionsReapplied: N.localBackup.predictionsReapplied || 0,
        realOverrides: N.localBackup.realOverrides || 0,
        updateId: N.localBackup.updateId,
        lastStage: N.localBackup.lastStage,
        lastReason: N.localBackup.lastReason
      } },
    frontier: { count: N.frontierCount, candidates: N.frontierCandidates?.slice(0, 5) || [] },
    goal: N.goal,
    path: { cells: N.path.length },
    command: N.lastCommand,
    planAttempts: N.planAttempts,
    navigationDiagnostics: {
      scanUpdateId: N.debugNav.scanUpdateId,
      planUpdateId: N.debugNav.planUpdateId,
      lastPlanStart: N.debugNav.lastPlanStart,
      lastPlanResult: N.debugNav.lastPlanResult,
      lastPathResult: N.debugNav.lastPathResult,
      lastSafetyResult: N.debugNav.lastSafetyResult,
      stopReason: N.debugNav.stopReason,
      commandReason: N.debugNav.commandReason,
      frontierEvaluated: N.debugNav.frontierEvaluated,
      frontierReachable: N.debugNav.frontierReachable,
      frontierRejected: N.debugNav.frontierRejected,
      replans: N.debugNav.replans,
      lastGoalDistance: N.debugNav.lastGoalDistance,
      lastHeadingError: N.debugNav.lastHeadingError
    },
    reason: N.lastPlanReason,
    error: N.lastError,
    probeCount: N.probeCount,
    area: pose ? areaDiagnostics(N.config || c, pose) : null,
  };
  state.navDebug = d;

  const debugPanel = document.getElementById('debug-panel-content');
  if (debugPanel) {
    debugPanel.textContent = JSON.stringify(d, (k,v) => typeof v === 'number' ? Number(v.toFixed ? v.toFixed(3) : v) : v, 2);
  }
  const statusPanel = document.getElementById('status-panel-content');
  if (statusPanel) {
    statusPanel.innerHTML = `
      <div class="status-kpi"><span>MODE</span><b>${N.active ? N.mode : 'IDLE'}</b></div>
      <div class="status-kpi"><span>POSE</span><b>${pose ? `${pose.x.toFixed(2)} / ${pose.y.toFixed(2)} / ${(pose.theta*180/Math.PI).toFixed(0)}°` : 'WAITING'}</b></div>
      <div class="status-kpi"><span>MISSION</span><b>${N.externalTarget ? `TARGET ${N.externalTargetStatus.toUpperCase()}` : (N.missionComplete ? 'COMPLETE' : 'EXPLORE')}</b></div>
      <div class="status-kpi"><span>KNOWN</span><b>${(N.coverage*100).toFixed(1)}% / ${(N.config?.targetKnownPercent ?? 85).toFixed(0)}%</b></div>
      <div class="status-kpi"><span>MAP</span><b>${N.freeCells.toLocaleString()} clear · ${N.obstacleCells.toLocaleString()} obs</b></div>
      <div class="status-kpi"><span>FRONTIERS</span><b>${N.frontierCount}</b></div>
      <div class="status-kpi"><span>PATH</span><b>${N.path.length} cells</b></div>
      <div class="status-kpi"><span>CMD</span><b>v=${N.lastCommand.v.toFixed(3)} · ω=${N.lastCommand.omega.toFixed(3)} · ${N.lastCommand.sent ? 'SENT' : 'NOT SENT'}</b></div>
      <div class="status-kpi"><span>LOCAL</span><b>${N.localClear ? 'CLEAR' : N.localBlockedReason}</b></div>
      <div class="status-kpi"><span>REASON</span><b>${N.lastPlanReason}</b></div>`;
  }
}
function localStartIsSafe(c) {
  if (!N.localMap) return false;
  // The robot must not be trapped merely because some pixels around it are
  // still unknown. The local safety decision for the occupied footprint is
  // based on actual/inferred obstacle and safety-margin cells.
  return localClearanceAhead(
    0,
    Math.min(c.localForwardSafetyDistanceM, c.radius - c.localClearanceStartM),
    Math.max(c.resolution, c.robotRadius * 0.55),
    false
  );
}

function plan(pose, c) {
  N.planAttempts++;
  N.localPathInvalidated = false;
  N.debugNav.planUpdateId++;
  N.debugNav.lastPlanStart = {x: pose.x, y: pose.y, theta: pose.theta, coverage: N.coverage};
  N.debugNav.lastPlanResult = 'START';
  if (!N.home) N.home={x:pose.x,y:pose.y,theta:pose.theta};

  if (String(c.area?.circle?.center_mode || c.area?.center_mode || '')==='robot_start' &&
      N.planAttempts===1) {
    initGrid(c);
  }

  if (!N.exploration.totalTiles) resetExplorationProgress(c);
  updateExplorationProgress(pose, c);
  seedKnowledgeFromCompleteCloud(c);
  computeStats(c);

  const areaInfo=areaDiagnostics(c,pose);
  const startCell=cell(pose.x,pose.y);
  if (!areaInfo.inside || !startCell) {
    N.debugNav.lastPlanResult = 'OUTSIDE_AREA';
    if (c.outsideRecoveryEnabled && c.outsideRecoveryMode === 'path_return' && Array.isArray(state.trajPrimitive) && state.trajPrimitive.length) {
      let best = null;
      const maxD = c.outsideRecoveryMaxPathSearchDistance;
      for (let i = state.trajPrimitive.length - 1; i >= 0; i--) {
        const p = state.trajPrimitive[i];
        if (!p || p.length < 2 || !insideArea(p[0], p[1], c, true)) continue;
        const d = Math.hypot(pose.x - p[0], pose.y - p[1]);
        if (d <= maxD) { best = { x:p[0], y:p[1], index:i, distance:d }; break; }
      }
      if (!best) {
        for (let i = state.trajPrimitive.length - 1; i >= 0; i--) {
          const p = state.trajPrimitive[i];
          if (p && p.length >= 2 && insideArea(p[0], p[1], c, true)) { best={x:p[0],y:p[1],index:i,distance:Math.hypot(pose.x-p[0],pose.y-p[1])}; break; }
        }
      }
      if (best) {
        N.mode='BOUNDARY RECOVERY';
        N.goal=[best.x,best.y];
        N.path=[];
        N.boundaryRecoveryPath = state.trajPrimitive.slice(Math.max(0,best.index-60), best.index+1).map(p=>[p[0],p[1]]);
        N.boundaryRecoveryIndex = Math.max(0, N.boundaryRecoveryPath.length-1);
        N.lastPlanReason=`OUTSIDE AREA — RETURN TO PREVIOUS PATH (${best.distance.toFixed(2)}m)`;
        return true;
      }
    }
    N.mode='OUTSIDE AREA'; N.path=[]; N.goal=null;
    N.lastPlanReason=`ROBOT OUTSIDE AREA — NO RECOVERY PATH`;
    sendStop(navigationCommandSource()); return false;
  }

  rebuildInflated(c);



  // Explicit terminal target takes precedence over exploration. UNKNOWN is
  // allowed for this A* search, while measured obstacles and inflated safety
  // cells remain hard constraints. The local 2D map is the immediate safety
  // gate while following the resulting global path.
  if (N.externalTarget && N.externalTargetStatus !== 'reached') {
    const [tx, ty] = N.externalTarget;
    if (Math.hypot(tx - pose.x, ty - pose.y) <= c.goalReach) {
      N.targetValidation = {x:tx, y:ty, stage:'REACHED', reasonCode:'WITHIN_GOAL_TOLERANCE'};
      finishExplicitTarget(c);
      return true;
    }
    const targetInfo = {
      x: tx, y: ty,
      safeInside: insideArea(tx, ty, c, true, pose),
      gridCell: null,
      gridInside: false,
      nearestFree: null,
      nearestFreeCell: null,
      stage: 'RECEIVED',
      reasonCode: null,
      aStarEntered: false,
      aStarAllowUnknown: true,
      aStarResult: null,
      reason: null
    };

    if (!targetInfo.safeInside) {
      targetInfo.stage = 'AREA_CHECK';
      targetInfo.reasonCode = 'OUTSIDE_SAFE_AREA';
      targetInfo.reason = `TARGET OUTSIDE SAFE NAVIGATION AREA — ${tx.toFixed(2)}, ${ty.toFixed(2)}`;
      N.targetValidation = targetInfo;
      N.mode = 'TARGET BLOCKED';
      N.path = []; N.goal = null;
      N.lastPlanReason = targetInfo.reason;
      N.targetFailureHold = true;
      sendStop('terminal_target');
      reportExternalTargetStatus('blocked');
      return false;
    }

    targetInfo.stage = 'GRID_CHECK';
    const targetCell = cell(tx, ty);
    targetInfo.gridCell = targetCell;
    targetInfo.gridInside = !!targetCell;
    if (!targetCell) {
      targetInfo.reasonCode = 'OUTSIDE_GLOBAL_GRID';
      targetInfo.reason = 'TARGET IS OUTSIDE GLOBAL GRID';
      N.targetValidation = targetInfo;
      N.mode = 'TARGET BLOCKED';
      N.path = []; N.goal = null;
      N.lastPlanReason = targetInfo.reason;
      N.targetFailureHold = true;
      sendStop('terminal_target');
      reportExternalTargetStatus('blocked');
      return false;
    }

    targetInfo.stage = 'NEAREST_SAFE_CELL';
    const targetGoal = nearestFree([tx, ty], c, true);
    targetInfo.nearestFreeCell = targetGoal;
    targetInfo.nearestFree = targetGoal ? world(targetGoal[0], targetGoal[1]) : null;
    if (!targetGoal) {
      targetInfo.reasonCode = 'NO_SAFE_APPROACH_CELL';
      targetInfo.reason = 'TARGET APPROACH CURRENTLY UNSAFE — WAITING FOR NEW MAP EVIDENCE';
      N.targetValidation = targetInfo;
      N.mode = 'TARGET WAITING';
      N.path = []; N.goal = null;
      N.lastPlanReason = targetInfo.reason;
      sendStop('terminal_target');
      return false;
    }

    targetInfo.stage = 'ASTAR';
    targetInfo.aStarEntered = true;
    const targetPath = aStar(startCell, targetGoal, c, true);
    targetInfo.aStarResult = N.debugNav.targetAStar?.result || (targetPath ? 'success' : 'failed');
    if (!targetPath || targetPath.length < 2) {
      targetInfo.reasonCode = 'ASTAR_NO_PATH';
      targetInfo.reason = 'NO CURRENT A* PATH TO EXPLICIT TARGET — RETRYING';
      N.targetValidation = targetInfo;
      N.mode = 'TARGET WAITING';
      N.path = []; N.goal = null;
      N.lastPlanReason = `${targetInfo.reason} | ${N.debugNav.targetAStar?.result || 'unknown'}`;
      sendStop('terminal_target');
      return false;
    }

    targetInfo.stage = 'PATH_READY';
    targetInfo.reasonCode = 'PATH_FOUND';
    N.targetValidation = targetInfo;
    N.targetFailureHold = false;
    N.mode = 'TARGET NAVIGATING';
    N.path = targetPath;
    N.goal = [tx, ty];
    N.explorationTarget = null;
    N.debugNav.lastPlanResult = 'EXPLICIT_TARGET_PATH_FOUND';
    N.debugNav.replans++;
    N.lastPlanReason = `A* TARGET PATH — ${tx.toFixed(2)}, ${ty.toFixed(2)} (${targetPath.length} cells) UNKNOWN_ALLOWED`;
    return true;
  }


  if (!N.externalTarget && N.targetFailureHold) {
    N.debugNav.lastPlanResult = 'TARGET_FAILURE_HOLD';
    N.mode = N.mode === 'TARGET BLOCKED' || N.mode === 'TARGET WAITING' ? N.mode : 'TARGET BLOCKED';
    N.goal = null;
    N.path = [];
    N.lastPlanReason = N.targetValidation?.reason || 'TERMINAL TARGET FAILED — WAITING FOR NEW TARGET';
    sendStop('terminal_target');
    return false;
  }

  // Mission completion is based on completed exploration tiles. Each tile is
  // complete only when its own knowledge coverage reaches tile_cov_complete_rate.
  if (!N.externalTarget && N.coverage >= c.coverageTarget) {
    N.debugNav.lastPlanResult = 'MISSION_COMPLETE';
    N.missionComplete=true;
    N.mode='COMPLETE';
    N.goal=null; N.path=[];
    N.lastPlanReason=`MISSION COMPLETE — ${ (N.coverage*100).toFixed(1) }% KNOWN >= ${(c.coverageTarget*100).toFixed(1)}%`;
    sendStop(navigationCommandSource());
    return true;
  }



  const startIndex=idx(startCell[0],startCell[1]);
  const startOccupied=N.grid.cells[startIndex]===2;
  const startInflated=!!N.inflated[startIndex];
  let startFree=!startInflated;
  if (N.externalTarget && !startOccupied && localStartIsSafe(c)) {
    // For an explicit target, an inflated global start cell may be a stale or
    // conservative artifact. The local map is the immediate safety gate.
    startFree=true;
  } else if (!startFree && !startOccupied && localStartIsSafe(c)) {
    startFree=true;
    log('[AP] Global start inflation is conservative; local 1 m map is clear — continuing exploration','info');
  }
  if (!startFree) {
    N.debugNav.lastPlanResult = `START_BLOCKED occupied=${startOccupied} inflated=${startInflated}`;
    N.mode='LOCAL RECOVERY';
    N.path=[]; N.goal=null;
    N.lastPlanReason=`START UNSAFE occupied=${startOccupied} inflated=${startInflated}`;
    return false;
  }


  // REPLAN is deliberately a target-selection state only. It chooses the
  // nearest tile that is not yet complete, then hands that tile's reachable
  // destination to NAVIGATING. A* always runs on the GLOBAL accumulated grid;
  // the 1 m local map is only the immediate safety/avoidance layer.
  N.mode='REPLAN';
  refreshAllTileCoverage(c);
  const tileCandidates = [];
  for (const key of N.exploration.allTileKeys || []) {
    const t = N.exploration.tiles.get(key);
    if (!t || t.explored || t.coverage >= c.tileCoverageCompleteRate || t.unreachable) continue;
    const center = explorationTileCenter(key, c);
    const dist = Math.hypot(center[0] - pose.x, center[1] - pose.y);
    tileCandidates.push({ key, tile:t, center, distance:dist });
  }
  tileCandidates.sort((a,b) => a.distance - b.distance);
  N.frontierCandidates = tileCandidates;
  N.frontierCount = tileCandidates.length;
  N.debugNav.frontierEvaluated = 0;
  N.debugNav.frontierReachable = 0;
  N.debugNav.frontierRejected = 0;

  let best = null;
  for (const candidate of tileCandidates) {
    N.debugNav.frontierEvaluated++;
    const goal = nearestFree(candidate.center, c);
    if (!goal) {
      candidate.tile.unreachable = true;
      candidate.tile.unreachableAtCoverage = candidate.tile.coverage;
      N.debugNav.frontierRejected++;
      continue;
    }
    const path = aStar(startCell, goal, c);
    if (!path || path.length < 2) {
      candidate.tile.unreachable = true;
      candidate.tile.unreachableAtCoverage = candidate.tile.coverage;
      N.debugNav.frontierRejected++;
      continue;
    }
    N.debugNav.frontierReachable++;
    const pathCost = (path.length - 1) * c.resolution;
    best = { candidate, goal, path, pathCost };
    break;
  }

  if (best) {
    N.mode='NAVIGATING';
    N.path=best.path;
    const reachableGoal = world(best.goal[0], best.goal[1]);
    N.goal=[reachableGoal[0], reachableGoal[1]];
    N.explorationTarget={
      tileKey: best.candidate.key,
      x: best.candidate.center[0], y: best.candidate.center[1],
      coverage: best.candidate.tile.coverage,
      completeRate: c.tileCoverageCompleteRate,
      distance: best.candidate.distance,
      pathCost: best.pathCost,
      reachableGoal:[reachableGoal[0], reachableGoal[1]]
    };
    N.debugNav.lastPlanResult = 'NEAREST_INCOMPLETE_TILE_SELECTED';
    N.debugNav.replans++;
    N.lastPlanReason=`NEAREST INCOMPLETE TILE ${best.candidate.key} cov=${(best.candidate.tile.coverage*100).toFixed(1)}% < ${(c.tileCoverageCompleteRate*100).toFixed(1)}% distance=${best.candidate.distance.toFixed(2)}m`;
    return true;
  }

  // If no known-free frontier is reachable, probe into the direction with the
  // largest immediately safe unknown sector. This is the mechanism that lets
  // the robot expand the known map instead of repeatedly stopping.
  const probe=makeProbeGoal(pose,c);
  if(probe){
    N.mode='PROBING';
    N.goal=probe;
    N.path=[];
    N.explorationTarget={x:probe[0],y:probe[1],gain:0,pathCost:0,utility:0};
    N.lastPlanReason=`EXPLORE PROBE — ${ (N.coverage*100).toFixed(1) }% KNOWN, no reachable frontier`;
    return true;
  }

  // No remaining tile can be reached with the current complete-map evidence.
  // There is no separate SEARCHING mode: REPLAN has finished its only job and
  // the robot stops in a clear BLOCKED state until new map evidence arrives.
  N.mode='BLOCKED';
  N.path=[]; N.goal=null;
  N.lastPlanReason=`NO REACHABLE INCOMPLETE TILE — ${N.frontierRejected}/${N.frontierEvaluated} CANDIDATES REJECTED`;
  sendStop(navigationCommandSource());
  return false;
}
function observeNavigationMode() { /* disabled by design */ }

function reportExternalTargetStatus(status) {
  const targetId = N.externalTargetId;
  if (!targetId) return N.externalTargetStatusQueue;

  // Serialize status writes. Without this, 'running' and 'blocked' requests
  // can complete out of order and the server's last-write-wins state can make
  // the browser oscillate between TARGET NAVIGATING and TARGET BLOCKED.
  N.externalTargetStatusQueue = N.externalTargetStatusQueue
    .catch(() => {})
    .then(async () => {
      // If a newer target replaced this one while the queue was waiting, do not
      // send an obsolete status update.
      if (N.externalTargetId !== targetId) return;
      try {
        await fetch('/api/navigation/target/status', {
          method: 'POST',
          headers: {'Content-Type': 'application/json'},
          body: JSON.stringify({id: targetId, status})
        });
      } catch (_) {
        // The target remains valid locally; the next sync will reconcile status.
      }
    });
  return N.externalTargetStatusQueue;
}

async function syncExternalTarget() {
  if (N.externalTargetSyncBusy) return;
  N.externalTargetSyncBusy = true;
  try {
    const response = await fetch('/api/navigation/target', {cache: 'no-store'});
    if (!response.ok) return;
    const data = await response.json();
    const t = data?.target;
    if (!data?.active || !t || data.status === 'idle') {
      if (N.externalTarget && N.externalTargetStatus !== 'reached') {
        N.externalTarget = null;
        N.externalTargetId = null;
        N.externalTargetStatus = 'idle';
        if (N.active) {
          N.goal = null;
          N.path = [];
          if (!N.targetFailureHold) N.mode = 'IDLE';
          sendStop(N.targetFailureHold ? 'terminal_target' : navigationCommandSource());
        }
        updateUI();
      }
      return;
    }

    const x = Number(t.x), y = Number(t.y), id = Number(data.id);
    if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(id)) return;

    const changed = N.externalTargetId !== id ||
      !N.externalTarget ||
      Math.abs(N.externalTarget[0] - x) > 1e-9 ||
      Math.abs(N.externalTarget[1] - y) > 1e-9;

    const incomingStatus = String(data.status || 'pending').toLowerCase();
    const terminalStatus = incomingStatus === 'reached' || incomingStatus === 'blocked';

    N.externalTarget = [x, y];
    N.externalTargetId = id;
    // Do not let a stale server-side 'pending' value demote an already-running
    // target while the browser is actively planning it.
    N.externalTargetStatus = terminalStatus ? incomingStatus : (N.externalTargetStatus === 'running' && !changed ? 'running' : 'pending');

    if (N.externalTargetStatus === 'reached') {
      if (N.targetValidation) N.targetValidation.stage = 'REACHED';
      N.goal = null;
      N.path = [];
      if (N.config.continueExplorationAfterTargetReached) {
        N.mode = 'REPLAN';
        N.lastPlanReason = 'TARGET REACHED — RESUMING FULL-MAP EXPLORATION';
      } else {
        N.mode = 'TARGET REACHED';
        sendStop(navigationCommandSource());
      }
      updateUI();
      return;
    }

    if (N.externalTargetStatus === 'blocked') {
      if (N.targetValidation) N.targetValidation.stage = 'BLOCKED';
      N.mode = 'TARGET BLOCKED';
      if (N.targetValidation) N.targetValidation.stage = 'BLOCKED';
      N.goal = null;
      N.path = [];
      sendStop(navigationCommandSource());
      updateUI();
      return;
    }

    if (changed) {
      N.goal = null;
      N.path = [];
      N.localPathInvalidated = false;
      N.lastPlanAt = 0;
      N.targetFailureHold = false;
      N.targetValidation = null;
      N.mode = 'TARGET PENDING';
      N.lastPlanReason = `TERMINAL TARGET ${x.toFixed(2)}, ${y.toFixed(2)}`;
      // Take navigation ownership immediately, before target validation/A* runs.
      // A zero-velocity lease is intentional: it prevents the old exploration
      // command owner from winning during target handover.
      claimCommandOwnership('terminal_target', 1400);
      sendVelocity(0, 0, 'terminal_target', 1400);
    }

    if (!N.active && state.ws?.readyState === WebSocket.OPEN) {
      toggleAutoPilot(true);
    }
    if (N.externalTargetStatus !== 'running' && state.ws?.readyState === WebSocket.OPEN) {
      N.externalTargetStatus = 'running';
      reportExternalTargetStatus('running');
      // Keep the terminal-target lease alive until the first planning cycle.
      claimCommandOwnership('terminal_target', 1400);
    }
    if (N.active && state.navPose) updateUI();
  } catch (_) {
    // Server may be starting/restarting; retry on the next interval.
  } finally {
    N.externalTargetSyncBusy = false;
  }
}

export function initNavigation() {
  const c = ensureGrid();
  if (!N.localMap) initLocalMap(c);
  if (!N.heartbeat) {
    N.heartbeat = setInterval(() => {
      if (!N.active || !state.navPose) return;
      const age = performance.now() - (N.lastCommand?.at || 0);
      if (age < 180) return;
      const cmd = N.lastCommand || { v: 0, omega: 0, sent: false };
      if (N.mode === 'LOCAL BLOCKED' || N.mode === 'COMPLETE' || N.mode === 'START BLOCKED' || N.mode === 'BLOCKED') return;
      if (Math.abs(cmd.v) < 0.001 && Math.abs(cmd.omega) < 0.001) return;
      const source = navigationCommandSource();
      const sent = sendVelocity(cmd.v, cmd.omega, source, 700);
      N.lastCommand = { ...cmd, sent, at: performance.now() };
      if (!sent) N.lastError = 'Heartbeat could not send velocity: WebSocket not OPEN';
    }, 100);
  }
  if (!N.externalTargetSync) {
    syncExternalTarget();
    N.externalTargetSync = setInterval(syncExternalTarget, 250);
  }
  updateUI();
}

export function invalidateKnowledgeSeed() {
  N.knowledgeSeededCloudLength = -1;
}

function ensureGTSessionAnchor(pose) {
  if (!pose || pose.source !== 'ground_truth') return false;
  if (N.areaAnchor) return true;

  const storedKey = 'pioneer.autopilot.gtAreaAnchor';
  let anchor = null;
  try {
    anchor = JSON.parse(sessionStorage.getItem(storedKey) || 'null');
  } catch (_) {
    anchor = null;
  }

  // sessionId is created on WebSocket connection. Only reuse an anchor from the
  // same live mapping session; otherwise the first GT pose becomes the new anchor.
  const sameSession = anchor &&
    (!state.sessionId || anchor.session_id === state.sessionId) &&
    anchor.source === 'ground_truth' &&
    Number.isFinite(Number(anchor.x)) &&
    Number.isFinite(Number(anchor.y));

  if (!sameSession) {
    anchor = {
      session_id: state.sessionId || null,
      source: 'ground_truth',
      x: Number(pose.x),
      y: Number(pose.y),
      z: Number(pose.z || 0),
      theta: Number(pose.theta || 0)
    };
    try { sessionStorage.setItem(storedKey, JSON.stringify(anchor)); } catch (_) {}
    log(`[GT] New mapping-session tile anchor: x=${anchor.x.toFixed(3)} y=${anchor.y.toFixed(3)}`, 'info');
  } else {
    log(`[GT] Reusing mapping-session tile anchor: x=${Number(anchor.x).toFixed(3)} y=${Number(anchor.y).toFixed(3)}`, 'info');
  }

  N.areaAnchor = {
    x: Number(anchor.x),
    y: Number(anchor.y),
    z: Number(anchor.z || 0),
    theta: Number(anchor.theta || 0)
  };
  N.home = { ...N.areaAnchor };
  N.areaAnchorSessionKey = state.sessionId || null;

  if (N.config) initGrid(N.config);
  return true;
}


export function resetAutopilotSessionAnchor() {
  N.areaAnchor = null;
  N.areaAnchorSessionKey = null;
  try {
    sessionStorage.removeItem('pioneer.autopilot.areaAnchor');
    sessionStorage.removeItem('pioneer.autopilot.gtAreaAnchor');
  } catch (_) {}
}

export function toggleAutoPilot(force) {
  const nextActive = force === undefined ? !N.active : !!force;
  if (!state.ws || state.ws.readyState !== WebSocket.OPEN) {
    toast('WebSocket not connected — Auto Pilot cannot start', 'error');
    log('[AP] Cannot toggle: WebSocket is not OPEN', 'err');
    return;
  }
  const c = ensureGrid();
  if (!c || !c.enabled) {
    toast('Navigation is disabled in YAML', 'error');
    log('[AP] Cannot start: navigation.enabled is false or config is unavailable', 'err');
    return;
  }
  if (nextActive && !state.navPose) {
    toast('Ground-truth pose not available yet — wait for GT telemetry', 'error');
    log('[AP] Cannot start: ground-truth pose is not available yet. Waiting for GT telemetry.', 'err');
    return;
  }
  if (nextActive) {
    const p = state.navPose;
    const validPose = Number.isFinite(Number(p.x)) && Number.isFinite(Number(p.y)) && Number.isFinite(Number(p.theta));
    if (!validPose) {
      toast('Invalid ground-truth pose — Auto Pilot cannot start', 'error');
      log(`[AP] Cannot start: invalid ground-truth pose ${JSON.stringify(p)}`, 'err');
      return;
    }
    log(`[AP] Ground-truth pose ready: x=${p.x.toFixed(3)} y=${p.y.toFixed(3)} z=${Number(p.z ?? 0).toFixed(3)} yaw=${(p.theta * 180 / Math.PI).toFixed(1)}°`, 'info');
  }
  if (nextActive && state.navPose?.source === 'ground_truth') {
    ensureGTSessionAnchor(state.navPose);
  }
  N.active = nextActive;
  window.dispatchEvent(new CustomEvent('autopilot-render-state', { detail: { active: N.active } }));
  if (N.active) {
    if (String(c.area?.circle?.center_mode || c.area?.center_mode || '') === 'robot_start' && N.areaAnchor) {
      initGrid(c);
      N.config = c;
    }
    N.mode = 'STARTING';
    N.goal = null;
    N.path = [];
    N.lastPlanAt = 0;
    sendStop(navigationCommandSource());
    sendScanControl(true);
    log('[AP] AUTO PILOT enabled — navigation engine ACTIVE', 'ok');
    toast('Auto Pilot enabled', 'success');
  } else {
    N.mode='IDLE'; N.goal=null; N.path=[];
    N.modeLoop.transitions=[]; N.modeLoop.lastMode=null; N.modeLoop.rebooting=false;
    sendStop(navigationCommandSource());
    sendScanControl(false);
    // Tile-map visibility is a viewer/UI state, not an Auto Pilot state. Never
    // hide or reset it when Auto Pilot is switched off.
    refreshExplorationMap();
    toast('Auto Pilot disabled', 'warn');
  }
  updateUI();
}

export function onScanFrame(points, meta) {
  const c = ensureGrid();
  N.debugNav.scanUpdateId++;
  N.localBackup.scanPoints = Array.isArray(points) ? Math.floor(points.length / 3) : 0;
  N.localBackup.cloudPoints = mapStore.length;
  const localMeta = { ...meta, __currentScanPoints: points };
  // mapStore already contains this frame because ws-client appends it before
  // calling onScanFrame. Seed only the older, not-yet-ingested stored points,
  // then ingest the current scan once. This makes knowledge/coverage use both
  // old and new points without double-counting the current frame.
  const currentCount = Array.isArray(points) ? Math.floor(points.length / 3) : 0;
  const oldCloudEnd = Math.max(0, mapStore.length - currentCount);
  if (N.knowledgeSeededCloudLength < 0) seedKnowledgeFromCompleteCloud(c, oldCloudEnd);

  if (N.active) {
    ingest(points, meta);
    N.knowledgeSeededCloudLength = mapStore.length;
  } else if (oldCloudEnd < mapStore.length) {
    seedKnowledgeFromCompleteCloud(c, mapStore.length);
  }

  // The local map is rebuilt AFTER the current scan is incorporated so it
  // contains both historical and newly received points.
  rebuildLocalMapFromCompleteCloud(localMeta, c);
  const now=performance.now();
  if (now-N.lastPlanAt>=N.config.replanMs) {
    N.lastPlanAt=now;
    const pose=state.navPose;
    if (pose) {
      const goalReached = N.goal && Math.hypot(pose.x - N.goal[0], pose.y - N.goal[1]) < N.config.goalReach;
      if (goalReached && N.explorationTarget?.tileKey) {
        const reachedTile = N.exploration.tiles.get(N.explorationTarget.tileKey);
        if (reachedTile && !reachedTile.explored && Number(reachedTile.coverage || 0) < N.config.tileCoverageCompleteRate) {
          reachedTile.unreachable = true;
          reachedTile.unreachableAtCoverage = Number(reachedTile.coverage || 0);
          N.lastPlanReason = `TILE ${N.explorationTarget.tileKey} REACHED BUT COVERAGE INCOMPLETE — MARKED UNREACHABLE`;
        }
      }
      if (!N.externalTarget && (N.missionComplete || N.mode === 'COMPLETE')) {
        sendStop(navigationCommandSource());
        updateUI();
        return;
      }
      if (N.externalTarget && N.externalTargetStatus === 'reached' &&
          !N.config.continueExplorationAfterTargetReached) {
        sendStop(navigationCommandSource());
        updateUI();
        emitNavigationDebug();
        return;
      }
      // Replan when the target is reached, the path is exhausted, or the
      // mission map changed enough to justify a new exploration target.
      if (!N.path.length || !N.goal || goalReached || N.localPathInvalidated ||
          N.mode === 'PROBING' || N.mode === 'LOCAL RECOVERY' ||
          N.mode === 'LOCAL AVOIDANCE' || N.mode === 'BOUNDARY RECOVERY' || N.mode === 'REPLAN') {
        if (N.mode === 'REPLAN') observeNavigationMode('REPLAN', N.config);
        plan(pose, N.config);
        observeNavigationMode(N.mode, N.config);
      }
      if (N.path.length >= 2) {
        commandFromPath(N.path, pose, N.config);
      } else if (N.mode === 'BOUNDARY RECOVERY' && N.goal) {
        // Boundary recovery is a safety behavior and must run even while a
        // terminal target is active — otherwise the robot sits stuck outside
        // the safe area, the target keeps failing A*, and it eventually gets
        // dropped in favor of exploration/probing.
        commandBoundaryRecovery(pose, N.config);
      } else if (N.externalTarget && N.externalTargetStatus !== 'reached') {
        // A terminal target is exclusive: never fall back to probing or
        // exploration while the target mission is pending/waiting.
        sendStop('terminal_target');
      } else if (N.mode === 'PROBING' && N.goal) {
        commandProbe(pose, N.config);
      } else if (N.mode === 'COMPLETE') {
        sendStop(navigationCommandSource());
      } else if (N.mode === 'TARGET REACHED') {
        sendStop(navigationCommandSource());
      } else if (N.mode === 'BLOCKED') {
        recordCommand(0, 0);
      } else {
        sendStop(navigationCommandSource());
      }
      updateUI();
      emitNavigationDebug();
    }
  }
}

export function onRobotPose(meta) {
  const c = ensureGrid();
  const pose = state.navPose;
  if (!pose || pose.source !== 'ground_truth') return;

  // Lock the exploration/tile-map area to the FIRST GT pose of this application
  // mapping session. The tile map is world-coordinate based; unlike the local
  // 2D map it must not recenter on the moving robot.
  ensureGTSessionAnchor(pose);

  if (N.config) seedKnowledgeFromCompleteCloud(N.config);
  if (state.explorationResumePending && N.config?.resumeLoadedExploration) {
    const restored = seedExplorationFromLoadedSnapshot(N.config, state.explorationResumeData);
    if (!restored) seedExplorationFromLoadedTrajectory(N.config, state.trajPrimitive);
    state.explorationResumePending = false;
    state.explorationResumeData = null;
  }

  const last = N.lastLocalPose;
  const moved = !last ||
    Math.hypot(pose.x - last.x, pose.y - last.y) > c.resolution * 0.25 ||
    Math.abs(Math.atan2(
      Math.sin(pose.theta - last.theta),
      Math.cos(pose.theta - last.theta)
    )) > 0.01;

  // Reproject the accumulated WORLD cloud around the current GT pose even when
  // the current telemetry frame contains no new scan points.
  if (moved && mapStore.length) rebuildLocalMapFromCompleteCloud({ robot: pose, ground_truth: state.groundTruth }, c);

  if (N.active && N.mode === 'PROBING' && N.goal &&
      Math.hypot(pose.x - N.goal[0], pose.y - N.goal[1]) < c.goalReach) {
    N.path = [];
    N.lastPlanReason = 'PROBE REACHED; RESCAN FRONTIERS';
  }
  refreshExplorationMap();
  updateUI();
}


export function getNavigationMap() {
  ensureGrid();
  return { width:N.grid.width, height:N.grid.height, resolution:N.grid.resolution, originX:N.grid.originX, originY:N.grid.originY, cells:N.grid.cells.slice(), area:resolvedArea(N.config), safety_margin:N.config.safety };
}

export function getExplorationPersistence() {
  const tiles = [];
  for (const [key, tile] of N.exploration.tiles) {
    if (tile?.explored) tiles.push({ key, coverage: Number(tile.coverage || 0) });
  }
  return tiles;
}

export function getNavigationState() { return { autopilot:N.active, mode:N.mode, externalTarget:N.externalTarget, externalTargetId:N.externalTargetId, externalTargetStatus:N.externalTargetStatus, coverage:N.coverage, knowledgeCoverage:N.knowledgeCoverage, localCoverage:N.localBackup.coverage, exploredTiles:N.exploration.exploredTiles, totalExplorationTiles:N.exploration.totalTiles, goal:N.goal, path:state.navPath || [], area:N.config ? resolvedArea(N.config) : null, areaAnchor: N.areaAnchor ? {x:N.areaAnchor.x, y:N.areaAnchor.y} : null, tileSize:N.config?.explorationTileSize || null, tileCoverageCompleteRate:N.config?.tileCoverageCompleteRate || null, tiles:Array.from(N.exploration.tiles.entries()).map(([key,t])=>({key,coverage:Number(t.coverage||0),state:t.explored?'discovered':(t.unreachable?'blocked':'clear')})), targetValidation:N.targetValidation, targetFailureHold:N.targetFailureHold, debug:state.navDebug || null }; }

// Rendering hint for the viewer. A local region is considered mature when the
// fraction of configured exploration tiles around the robot that are already
// explored reaches the YAML threshold. This is intentionally independent from
// the fine 2D occupancy grid and from raw point count.
export function getAdaptiveRenderHint(pose) {
  const c = N.config;
  if (!c || !pose || !c.explorationTileSize) return { localCoverage: 0, mature: false, tileCount: 0, explored: 0, keys: [] };
  const s = c.explorationTileSize;
  const radius = Math.max(1, Number(c.adaptiveRenderTileRadius || 2));
  const ix0 = Math.floor(pose.x / s), iy0 = Math.floor(pose.y / s);
  const keys = []; let explored = 0, total = 0;
  for (let iy = iy0-radius; iy <= iy0+radius; iy++) for (let ix = ix0-radius; ix <= ix0+radius; ix++) {
    const key = `${ix},${iy}`;
    const cx=(ix+0.5)*s, cy=(iy+0.5)*s;
    if (!insideArea(cx, cy, c, false)) continue;
    total++;
    const t=N.exploration.tiles.get(key);
    if (t?.explored) { explored++; keys.push(key); }
  }
  const localCoverage = total ? explored/total : 0;
  return { localCoverage, mature: localCoverage >= Math.min(1, Math.max(0, Number(c.adaptiveRenderLocalCoverageThreshold ?? 0.5))), tileCount: total, explored, keys };
}
