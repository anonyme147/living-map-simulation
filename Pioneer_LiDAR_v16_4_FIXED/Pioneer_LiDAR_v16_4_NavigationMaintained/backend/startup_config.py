"""Startup configuration loaded from a YAML file."""
from copy import deepcopy

DEFAULTS = {
    "server": {"host": "0.0.0.0", "port": 5000},
    "websocket": {"url": "ws://localhost:8765"},
    "detection": {
        "mode": "locate",
        "interval_sec": 5.0,
        "confidence": 0.50,
        "candidate_confidence": 0.10,
        "image_size": 640,
        "object_list_file": "config/detection_objects.txt",
        "fire_smoke": {"enabled": True, "model": "models/detection/fire_smoke_yolov8n.pt", "confidence": 0.25, "image_size": 320, "interval_sec": 5.0},
        "locate_anything": {"threads": 8, "mode": "fast", "timeout_sec": 900.0},
    },
    "lidar": {
        "offset_angle_deg": 0.0,
        "angle_filter": {"min_deg": 0.0, "max_deg": 360.0},
    },
    "viewer": {
        "visible_max_points": 800000,
        "surrounding_max_points": 800000,
        "visible_half_size_m": 20.0,
        "surrounding_half_size_m": 40.0,
        "cell_size_m": 2.0,
        "ground_visible": True,
        "invert_x": False,
        "invert_y": False,
        "invert_z": False,
        "adaptive_representation_enabled": True,
        "adaptive_representation_mode": "auto",
        "adaptive_local_coverage_threshold": 0.50,
        "adaptive_tile_radius": 2,
        "adaptive_voxel_size_m": 0.10,
        "adaptive_max_voxels": 60000,
        "adaptive_overload_point_threshold": 1000000,
        "adaptive_overload_visible_ratio": 0.90,
        "adaptive_refresh_min_interval_ms": 120,
        "adaptive_overload_refresh_ms": 35,
    },
    "backup": {"interval_ms": 60000, "threshold_points": 750000},
    "trajectory": {"max_render_points": 50000},
    "navigation": {
        "enabled": True, "resolution_m": 0.10, "local_sensor_radius_m": 8.0, "global_scan_min_range_m": 0.20, "global_scan_max_range_m": 12.0,
        "robot_height_m": 0.277, "ground_reference_z_m": -0.1385,
        "obstacle_positive_min_height_m": 0.05, "obstacle_positive_max_height_m": 1.0,
        "obstacle_negative_max_height_m": -0.15, "robot_width_m": 0.50, "robot_length_m": 0.51,
        "safety_margin_m": 0.50, "max_linear_speed_mps": 0.35,
        "max_angular_speed_rps": 1.20, "lookahead_m": 0.35, "goal_reach_m": 0.20,
        "replan_interval_ms": 350, "coverage_target": 0.80, "return_home": True,
        "continue_exploration_after_target_reached": True,
        "boundary_buffer_m": 0.10, "boundary_lookahead_m": 0.45, "local_avoidance_distance_m": 0.75,
        "local_inference_neighbor_radius_cells": 1, "local_inference_tie_class": "clear",
        "local_recovery_require_real_neighbors": True,
        "local_lidar_origin_x_m": 0.254, "local_lidar_origin_y_m": 0.0,
        "local_map_circle_margin_m": 0.0, "local_ray_clip_margin_m": 0.0,
        "local_clearance_start_m": 0.08, "local_clearance_step_m": 0.025, "local_lateral_step_m": 0.05,
        "local_recovery_ray_section_cells": 2,
        "local_avoidance_heading_step_deg": 10.0, "local_avoidance_heading_min_deg": -170.0, "local_avoidance_heading_max_deg": 170.0,
        "local_avoidance_angular_gain": 1.8, "local_avoidance_linear_speed_scale": 0.45,
        "local_avoidance_heading_penalty": 1.8, "local_avoidance_turn_penalty": 0.12,
        "local_avoidance_clearance_weight": 2.0, "local_avoidance_unknown_penalty": 0.12,
        "local_avoidance_boundary_bonus": 2.0, "local_corridor_half_width_scale": 0.20,
        "local_forward_safety_distance_m": 0.45,
        "path_angular_gain": 2.2, "path_slow_heading_rad": 0.9, "path_slow_speed_scale": 0.35,
        "goal_slowdown_distance_scale": 1.5, "goal_slowdown_speed_scale": 0.5, "boundary_slow_speed_scale": 0.45,
        "boundary_sample_step_m": 0.05, "probe_heading_offset_deg": 35.0, "probe_heading_samples": 5,
        "probe_speed_scale": 0.65, "probe_angular_gain": 2.0, "probe_slow_heading_rad": 0.8, "probe_slow_speed_scale": 0.35,
        "area": {
            "type": "circle",
            "rectangle": {"min_x": 0.0, "max_x": 20.0, "min_y": -1.0, "max_y": 23.0},
            "circle": {"center_x": 0.0, "center_y": 0.0, "radius_m": 15.0},
        },
    },
}


def _merge(base, override):
    out = deepcopy(base)
    if not isinstance(override, dict):
        return out
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(out.get(key), dict):
            out[key] = _merge(out[key], value)
        else:
            out[key] = value
    return out


def load_startup_config(path=None):
    """Load and validate a YAML config. No file means defaults."""
    if path is None:
        return deepcopy(DEFAULTS)

    from pathlib import Path
    import yaml

    cfg_path = Path(path).expanduser().resolve()
    if not cfg_path.is_file():
        raise FileNotFoundError(f"Configuration file not found: {cfg_path}")

    with cfg_path.open("r", encoding="utf-8") as f:
        raw = yaml.safe_load(f) or {}

    if not isinstance(raw, dict):
        raise ValueError("The YAML root must be a mapping/object")

    cfg = _merge(DEFAULTS, raw)

    # Normalize numeric values so the frontend receives predictable JSON.
    cfg["server"]["host"] = str(cfg["server"]["host"])
    cfg["server"]["port"] = int(cfg["server"]["port"])
    cfg["websocket"]["url"] = str(cfg["websocket"]["url"])
    detection = cfg["detection"]
    detection["mode"] = str(detection.get("mode", "locate")).lower()
    if detection["mode"] not in ("yolo", "locate", "hybrid"):
        raise ValueError("detection.mode must be yolo, locate, or hybrid")
    detection["interval_sec"] = float(detection.get("interval_sec", 5.0))
    detection["confidence"] = float(detection.get("confidence", 0.50))
    detection["candidate_confidence"] = float(detection.get("candidate_confidence", 0.10))
    if not 0.0 <= detection["candidate_confidence"] <= detection["confidence"] <= 1.0:
        raise ValueError("detection confidence values must satisfy 0 <= candidate <= confidence <= 1")
    detection["image_size"] = int(detection.get("image_size", 640))
    fire_smoke = detection.setdefault("fire_smoke", {})
    fire_smoke["enabled"] = bool(fire_smoke.get("enabled", True))
    fire_smoke["model"] = str(fire_smoke.get("model", "models/detection/fire_smoke_yolov8n.pt"))
    fire_smoke["confidence"] = float(fire_smoke.get("confidence", 0.25))
    fire_smoke["image_size"] = int(fire_smoke.get("image_size", 320))
    fire_smoke["interval_sec"] = float(fire_smoke.get("interval_sec", 5.0))
    if not 0.0 <= fire_smoke["confidence"] <= 1.0:
        raise ValueError("detection.fire_smoke.confidence must be between 0 and 1")
    locate = detection.setdefault("locate_anything", {})
    locate["threads"] = max(1, int(locate.get("threads", 8)))
    locate["mode"] = str(locate.get("mode", "fast")).lower()
    if locate["mode"] not in ("fast", "hybrid", "slow"):
        raise ValueError("detection.locate_anything.mode must be fast, hybrid, or slow")
    cfg["lidar"]["offset_angle_deg"] = float(cfg["lidar"]["offset_angle_deg"])
    af = cfg["lidar"]["angle_filter"]
    af["min_deg"] = float(af["min_deg"])
    af["max_deg"] = float(af["max_deg"])

    if not -180.0 <= cfg["lidar"]["offset_angle_deg"] <= 180.0:
        raise ValueError("lidar.offset_angle_deg must be between -180 and 180")
    if not 0.0 <= af["min_deg"] <= 360.0 or not 0.0 <= af["max_deg"] <= 360.0:
        raise ValueError("lidar.angle_filter min/max must be between 0 and 360")

    viewer = cfg["viewer"]
    for key in ("visible_max_points", "surrounding_max_points"):
        viewer[key] = int(viewer[key])
        if viewer[key] < 1:
            raise ValueError(f"viewer.{key} must be > 0")
    for key in ("visible_half_size_m", "surrounding_half_size_m", "cell_size_m"):
        viewer[key] = float(viewer[key])
        if viewer[key] <= 0:
            raise ValueError(f"viewer.{key} must be > 0")
    viewer["ground_visible"] = bool(viewer["ground_visible"])
    viewer["invert_x"] = bool(viewer["invert_x"])
    viewer["invert_y"] = bool(viewer["invert_y"])
    viewer["invert_z"] = bool(viewer["invert_z"])
    viewer["adaptive_representation_enabled"] = bool(viewer.get("adaptive_representation_enabled", True))
    viewer["adaptive_representation_mode"] = str(viewer.get("adaptive_representation_mode", "auto")).lower()
    if viewer["adaptive_representation_mode"] not in ("auto", "points", "voxels"):
        raise ValueError("viewer.adaptive_representation_mode must be auto, points, or voxels")
    for key in ("adaptive_local_coverage_threshold", "adaptive_overload_visible_ratio"):
        viewer[key] = float(viewer[key])
        if not 0.0 <= viewer[key] <= 1.0:
            raise ValueError(f"viewer.{key} must be between 0 and 1")
    for key in ("adaptive_tile_radius", "adaptive_max_voxels", "adaptive_overload_point_threshold", "adaptive_refresh_min_interval_ms", "adaptive_overload_refresh_ms"):
        viewer[key] = int(viewer[key])
        # adaptive_max_voxels=0 explicitly means unlimited voxel instances.
        min_allowed = 0 if key == "adaptive_max_voxels" else 1
        if viewer[key] < min_allowed:
            op = ">= 0" if key == "adaptive_max_voxels" else "> 0"
            raise ValueError(f"viewer.{key} must be {op}")
    viewer["adaptive_voxel_size_m"] = float(viewer["adaptive_voxel_size_m"])
    if viewer["adaptive_voxel_size_m"] <= 0:
        raise ValueError("viewer.adaptive_voxel_size_m must be > 0")

    cfg["backup"]["interval_ms"] = int(cfg["backup"]["interval_ms"])
    cfg["backup"]["threshold_points"] = int(cfg["backup"]["threshold_points"])
    cfg["trajectory"]["max_render_points"] = int(cfg["trajectory"]["max_render_points"])

    nav = cfg["navigation"]
    nav["enabled"] = bool(nav["enabled"])
    for key in ("resolution_m", "local_sensor_radius_m", "global_scan_min_range_m", "global_scan_max_range_m", "robot_height_m", "obstacle_positive_min_height_m", "obstacle_positive_max_height_m", "robot_width_m", "robot_length_m", "safety_margin_m", "max_linear_speed_mps", "max_angular_speed_rps", "lookahead_m", "goal_reach_m", "boundary_buffer_m", "boundary_lookahead_m", "local_avoidance_distance_m", "local_map_circle_margin_m", "local_ray_clip_margin_m", "local_clearance_start_m", "local_clearance_step_m", "local_lateral_step_m", "local_avoidance_heading_step_deg", "local_avoidance_angular_gain", "local_avoidance_linear_speed_scale", "local_avoidance_heading_penalty", "local_avoidance_turn_penalty", "local_avoidance_clearance_weight", "local_avoidance_unknown_penalty", "local_avoidance_boundary_bonus", "local_corridor_half_width_scale", "local_forward_safety_distance_m", "path_angular_gain", "path_slow_heading_rad", "path_slow_speed_scale", "goal_slowdown_distance_scale", "goal_slowdown_speed_scale", "boundary_slow_speed_scale", "boundary_sample_step_m", "probe_heading_offset_deg", "probe_speed_scale", "probe_angular_gain", "probe_slow_heading_rad", "probe_slow_speed_scale"):
        nav[key] = float(nav[key])
        if nav[key] < 0:
            raise ValueError(f"navigation.{key} must be >= 0")
    # Obstacle heights are measured relative to the local supporting-ground plane.
    for key in ("robot_height_m", "ground_reference_z_m", "obstacle_positive_min_height_m", "obstacle_positive_max_height_m", "obstacle_negative_max_height_m", "local_lidar_origin_x_m", "local_lidar_origin_y_m", "local_avoidance_heading_min_deg", "local_avoidance_heading_max_deg"):
        nav[key] = float(nav[key])
    if nav["robot_height_m"] <= 0:
        raise ValueError("navigation.robot_height_m must be > 0")
    if nav["obstacle_positive_min_height_m"] < 0 or nav["obstacle_positive_max_height_m"] < nav["obstacle_positive_min_height_m"]:
        raise ValueError("navigation positive obstacle height thresholds are invalid")
    if nav["obstacle_negative_max_height_m"] >= 0:
        raise ValueError("navigation.obstacle_negative_max_height_m must be < 0")
    if nav["obstacle_positive_max_height_m"] <= nav["obstacle_positive_min_height_m"]:
        raise ValueError("navigation positive obstacle range must have max > min")
    if nav["local_avoidance_heading_min_deg"] >= nav["local_avoidance_heading_max_deg"]:
        raise ValueError("navigation.local_avoidance_heading_min_deg must be less than heading_max_deg")
    nav["replan_interval_ms"] = int(nav["replan_interval_ms"])
    nav["coverage_target"] = float(nav["coverage_target"])
    nav["return_home"] = bool(nav["return_home"])
    nav["local_map_resolution_m"] = float(nav.get("local_map_resolution_m", 0.05))
    for key in ("local_inference_neighbor_radius_cells", "local_inference_passes", "local_recovery_ray_section_cells", "probe_heading_samples"):
        nav[key] = int(nav.get(key, DEFAULTS["navigation"].get(key, 1)))
    nav["local_inference_tie_class"] = str(nav.get("local_inference_tie_class", "clear")).lower()
    if nav["local_inference_tie_class"] not in ("clear", "obstacle", "margin"):
        raise ValueError("navigation.local_inference_tie_class must be clear, obstacle, or margin")
    nav["local_recovery_require_real_neighbors"] = bool(nav.get("local_recovery_require_real_neighbors", True))
    if nav["local_map_resolution_m"] <= 0:
        raise ValueError("navigation.local_map_resolution_m must be > 0")
    if not 0.0 < nav["coverage_target"] <= 1.0:
        raise ValueError("navigation.coverage_target must be in (0, 1]")
    area = nav["area"]
    if not isinstance(area, dict):
        raise ValueError("navigation.area must be a mapping/object")
    area["type"] = str(area.get("type", "circle")).lower()
    if area["type"] not in ("rectangle", "circle"):
        raise ValueError("navigation.area.type must be rectangle or circle")
    area.setdefault("rectangle", {"min_x": 0.0, "max_x": 20.0, "min_y": -1.0, "max_y": 23.0})
    area.setdefault("circle", {"center_mode": "robot_start", "center_x": 0.0, "center_y": 0.0, "radius_m": 15.0})
    for key in ("min_x", "max_x", "min_y", "max_y"):
        area["rectangle"][key] = float(area["rectangle"][key])
    for key in ("center_x", "center_y", "radius_m"):
        area["circle"][key] = float(area["circle"][key])
    area["circle"]["center_mode"] = str(
        area["circle"].get("center_mode", area.get("center_mode", "world"))
    ).lower()
    if area["circle"]["center_mode"] not in ("world", "robot_start", "robot_current"):
        raise ValueError("navigation.area.circle.center_mode must be world, robot_start, or robot_current")
    if area["circle"]["radius_m"] <= 0:
        raise ValueError("navigation.area.circle.radius_m must be > 0")

    # Keep the resolved config path for diagnostics only.
    cfg["_config_file"] = str(cfg_path)
    return cfg
