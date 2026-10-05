// Static defaults + runtime values loaded from /api/config.
export const MAX_POINTS = 800_000;
export const SURROUNDING_MAX_POINTS = 800_000;
export const MAX_RENDER_POINTS = MAX_POINTS + SURROUNDING_MAX_POINTS;
export const TRAVELLING_VISIBLE_HALF_SIZE = 20;
export const TRAVELLING_SURROUNDING_HALF_SIZE = 40;
export const TRAVELLING_CELL_SIZE = 2;
export const TRAVELLING_DOUBLE_CLICK_MS = 1500;
export const TRAVELLING_DOUBLE_CLICK_MOVE_PX = 8;
export const AUTO_BACKUP_THRESHOLD = 750_000;
export const AUTO_BACKUP_INTERVAL_MS = 60_000;
export const MAX_TRAJ = 50_000;
export const WS_URL = 'ws://localhost:8765';
export const API_URL = '/api/maps';
export const LSCN_MAGIC = 0x4E43534C;
export const LMAP_MAGIC = 0x50414D4C;
export const VERSION = '13.4-exploration-mission-v18-gt-nav';

export const runtimeConfig = {
  loaded: false,
  // Safe fallback matching the shipped YAML. Navigation uses the GT robot pose;
  // obstacle height is measured from the supporting ground plane in robot-local Z.
  navigation: {
    enabled: true,
    resolution_m: 0.10,
    local_map_resolution_m: 0.05,
    local_sensor_radius_m: 1.0,
    robot_height_m: 0.277,
    ground_reference_z_m: -0.1385,
    obstacle_positive_min_height_m: 0.05,
    obstacle_positive_max_height_m: 1.00,
    obstacle_negative_max_height_m: -0.15,
    local_min_radius_m: 0.05,
    local_voxel_size_m: 0.05,
    robot_width_m: 0.50,
    robot_length_m: 0.51,
    safety_margin_m: 0.20,
    max_linear_speed_mps: 0.15,
    max_angular_speed_rps: 1.20,
    lookahead_m: 0.35,
    goal_reach_m: 0.20,
    continue_exploration_after_target_reached: true,
    // Controller yaw is measured from a body axis 90° ahead of navigation +X.
    heading_offset_rad: Math.PI / 2,
    replan_interval_ms: 350,
    coverage_target: 0.85,
    target_known_percent: 85,
    exploration_tile_size_m: 0.50,
    exploration_min_observations: 3,
    exploration_min_view_angles: 2,
    exploration_min_view_angle_deg: 30.0,
    exploration_min_viewpoint_distance_m: 0.20,
    resume_loaded_exploration: true,
    outside_area_recovery_enabled: true,
    outside_area_recovery_mode: 'path_return',
    outside_area_warning_distance_m: 0.50,
    outside_area_recovery_waypoint_tolerance_m: 0.20,
    outside_area_recovery_max_path_search_distance_m: 5.0,
    outside_area_recovery_speed_scale: 0.45,
    navigation_mode_loop_recovery_enabled: true,
    navigation_mode_loop_threshold: 6,
    navigation_mode_loop_window_ms: 12000,
    navigation_mode_loop_reboot_cooldown_ms: 15000,
    navigation_mode_loop_off_ms: 1000,
    return_home: true,
    local_inference_enabled: true,
    local_inference_coverage: 0.70,
    local_inference_min_known_neighbors: 6,
    local_inference_passes: 3,
    area: {
      type: "circle",
      rectangle: { min_x: 0, max_x: 20, min_y: -1, max_y: 23 },
      circle: { center_mode: "robot_start", center_x: 0, center_y: 0, radius_m: 15 }
    }
  },
  websocket: { url: WS_URL },
  lidar: {
    offset_angle_deg: 0,
    angle_filter: { min_deg: 0, max_deg: 360 }
  },
  viewer: {
    visible_max_points: MAX_POINTS,
    surrounding_max_points: SURROUNDING_MAX_POINTS,
    visible_half_size_m: TRAVELLING_VISIBLE_HALF_SIZE,
    surrounding_half_size_m: TRAVELLING_SURROUNDING_HALF_SIZE,
    cell_size_m: TRAVELLING_CELL_SIZE,
    ground_visible: true,
    invert_x: false,
    invert_y: false,
    invert_z: false,
    adaptive_representation_enabled: true,
    adaptive_representation_mode: 'auto',
    adaptive_local_coverage_threshold: 0.50,
    adaptive_tile_radius: 2,
    adaptive_voxel_size_m: 0.01,
    adaptive_voxel_average_position: true,
    adaptive_max_voxels: 0,
    adaptive_overload_point_threshold: 1000000,
    adaptive_overload_visible_ratio: 0.90,
    adaptive_refresh_min_interval_ms: 120,
    adaptive_overload_refresh_ms: 35,
    show_3d_map_during_autopilot: false
  },
  backup: {
    interval_ms: AUTO_BACKUP_INTERVAL_MS,
    threshold_points: AUTO_BACKUP_THRESHOLD
  },
  trajectory: { max_render_points: MAX_TRAJ }
};

export async function loadRuntimeConfig() {
  const response = await fetch('/api/config', { cache: 'no-store' });
  if (!response.ok) throw new Error(`Runtime config HTTP ${response.status}`);
  const cfg = await response.json();

  Object.assign(runtimeConfig, cfg);
  runtimeConfig.loaded = true;
  return runtimeConfig;
}

export function getWsUrl() {
  return runtimeConfig.websocket?.url || WS_URL;
}
