from .depth.robust import robust_depth
from .geometry import pixel_to_camera, camera_to_robot, robot_to_world


def fuse_detection(detection, depth_map, calibration, robot_rotation, robot_translation,
                   detection_id, pose_source="odometry", border_fraction=0.15,
                   max_depth_m=80.0, depth_scale=1.0):
    x1, y1, x2, y2 = detection.bbox
    stats = robust_depth((x1, y1, x2, y2), depth_map, border_fraction,
                         max_depth_m=max_depth_m, depth_scale=depth_scale)
    result = {"detection_id": int(detection_id), "class": detection.class_name,
              "confidence": float(detection.confidence),
              "bbox": [x1, y1, x2, y2], "depth_m": stats["depth_m"],
              "depth_valid": stats["depth_valid"], "depth_quality": stats,
              "localization_valid": False, "pose_source": pose_source}
    if not stats["depth_valid"]:
        return result
    u, v = (x1 + x2) / 2.0, (y1 + y2) / 2.0
    p_camera = pixel_to_camera(u, v, stats["depth_m"], calibration)
    p_robot = camera_to_robot(p_camera, calibration)
    p_world = robot_to_world(p_robot, robot_rotation, robot_translation)
    result.update(position_camera=p_camera.tolist(), position_robot_frame=p_robot.tolist(),
                  position_world_frame=p_world.tolist(), localization_valid=True)
    return result
