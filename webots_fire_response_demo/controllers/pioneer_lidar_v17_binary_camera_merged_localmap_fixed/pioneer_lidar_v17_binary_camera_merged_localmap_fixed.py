"""
Pioneer 3-AT LiDAR Controller
================================
Original v7.1 binary LiDAR protocol + angle filter, refactored so that the
browser owns navigation/autopilot. This controller is deliberately limited to:
  - LiDAR / IMU / encoder acquisition
  - differential-drive motor control
  - WebSocket command reception
  - binary LSCN telemetry transmission
  - command watchdog for fail-safe stopping

The browser sends low-level commands such as:
  {"type":"velocity","v":0.35,"omega":0.0}
  {"type":"stop"}
  {"type":"scan_control","enabled":true}
  {"type":"config", ...}
"""

from controller import Supervisor
import math
import json
import asyncio
import websockets
import threading
import queue
import struct
import os
import base64
import urllib.error
import urllib.request
import numpy as np
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

from perception.camera import Camera as PerceptionCamera
from perception.calibration import load_geometry
from perception.detector import YoloDetector, DetectorUnavailable
from perception.depth.depth_anything_v2 import DepthAnythingV2Metric
from perception.depth.base import DepthModelUnavailable
from perception.fusion import fuse_detection
from perception.tracker import DetectionTracker
from perception.locate_anything import LocateAnythingDetector
from perception.ekf_pose import DifferentialDrivePoseEKF, wrap_angle

# YOLOv8 detection support
try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False
    print("[DETECTION] ultralytics not installed – detection disabled")

# Optional front-camera support
try:
    import cv2
    import numpy as np
    CAMERA_DISPLAY_AVAILABLE = True
except ImportError:
    cv2 = None
    np = None
    CAMERA_DISPLAY_AVAILABLE = False

# Values match the authoritative Webots calibration in robot_geometry.json.
# Keeping these aligned is critical for both distance and rotation accuracy.
WHEEL_RADIUS = 0.11
TRACK_WIDTH = 0.394
LIDAR_JOINT_X = 0.254
LIDAR_JOINT_Z = 0.14
LIDAR_ARM_LEN = 0.0
INITIAL_PITCH = 0.0
ROTATION_SPEED = 2.0
WS_HOST = "localhost"
WS_PORT = 8765
SEND_EVERY_N = 5
LIDAR_SUBSAMPLE = 4
COMMAND_TIMEOUT_SEC = 0.75
LSCN_MAGIC = b"LSCN"

# Stand-alone Webots companion behavior.  This remains local to the Webots
# scene: it does not emit data into The Living Map simulation pipeline.
AUTONOMOUS_SEARCH_ENABLED = True
HOME_DOCK_DEF = "HOME_DOCK"
SEARCH_SPEED_M_S = 0.60
TURN_SPEED_RAD_S = 1.15
GOAL_TOLERANCE_M = 0.55
OBSTACLE_CLEARANCE_M = 0.65
TARGET_CONFIRM_RANGE_M = 4.5
CAMERA_HALF_FOV_RAD = math.radians(36.0)

# Front camera configuration
CAMERA_NAME = "front camera"
CAMERA_SNAPSHOT_INTERVAL_SEC = 5.0
CAMERA_SNAPSHOT_PREFIX = "front_camera"
CAMERA_PREVIEW_ENABLED = False
# A responsive local browser feed: roughly 8 FPS, with light frames so the
# Webots controller continues prioritising LiDAR navigation and autonomy.
CAMERA_STREAM_INTERVAL_SEC = 0.12
CAMERA_STREAM_JPEG_QUALITY = 65
WEBOTS_BEACON_ENDPOINT = os.environ.get(
    "LIVING_MAP_WEBOTS_BEACON_ENDPOINT",
    "http://127.0.0.1:5012/api/webots/beacon",
)
# Fallback for the supplied Webots scene; the deployed value is read from YAML.
DEFAULT_DEPTH_SCALE = 0.446426


def clean(v):
    if v is None:
        return 0.0
    try:
        if math.isnan(v) or math.isinf(v):
            return 0.0
    except (TypeError, ValueError):
        pass
    return v


def clean_all(obj):
    if isinstance(obj, float):
        return clean(obj)
    if isinstance(obj, list):
        return [clean_all(item) for item in obj]
    if isinstance(obj, dict):
        return {k: clean_all(v) for k, v in obj.items()}
    return obj


def load_depth_scale(config_path):
    try:
        import yaml
        with open(config_path, "r", encoding="utf-8") as stream:
            config = yaml.safe_load(stream) or {}
        return float(config.get("depth", {}).get("scale", DEFAULT_DEPTH_SCALE))
    except Exception as exc:
        print(f"[DEPTH] using fallback calibration scale {DEFAULT_DEPTH_SCALE}: {exc}")
        return DEFAULT_DEPTH_SCALE


def load_depth_roi_border(config_path):
    """Load the inner depth ROI margin used to avoid neighboring objects."""
    try:
        import yaml
        with open(config_path, "r", encoding="utf-8") as stream:
            config = yaml.safe_load(stream) or {}
        value = config.get("fusion", {}).get("roi_border_fraction", 0.25)
        return min(0.45, max(0.0, float(value)))
    except Exception as exc:
        print(f"[DEPTH] using fallback ROI border 0.25: {exc}")
        return 0.25


def load_detection_config(config_path):
    try:
        import yaml
        with open(config_path, "r", encoding="utf-8") as stream:
            config = yaml.safe_load(stream) or {}
        return config.get("detection", {}) or {}
    except Exception as exc:
        print(f"[DETECTION] using default backend settings: {exc}")
        return {}


def merge_config(base, override):
    """Recursively merge runtime overrides without losing nested defaults."""
    result = dict(base or {})
    for key, value in (override or {}).items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = merge_config(result[key], value)
        else:
            result[key] = value
    return result


def load_object_list(project_root, detection_config):
    """Read the allowed object prompts from the configured list file."""
    configured = detection_config.get("object_list_file")
    if configured:
        path = project_root / configured
        if path.is_file():
            values = [line.strip() for line in path.read_text(encoding="utf-8").splitlines()
                      if line.strip() and not line.lstrip().startswith("#")]
            if values:
                return values
    return list(detection_config.get("classes", ["person", "fire", "smoke", "house", "tree"]))


class PioneerController:
    def __init__(self):
        self.robot = Supervisor()
        self.timestep = int(self.robot.getBasicTimeStep())
        self.robot_node = self.robot.getSelf()

        self.motors = {
            "fl": self.robot.getDevice("front left wheel"),
            "fr": self.robot.getDevice("front right wheel"),
            "rl": self.robot.getDevice("back left wheel"),
            "rr": self.robot.getDevice("back right wheel"),
        }
        for m in self.motors.values():
            m.setPosition(float("inf"))
            m.setVelocity(0.0)

        self.encoders = {
            "rl": self.robot.getDevice("back left wheel sensor"),
            "rr": self.robot.getDevice("back right wheel sensor"),
        }
        for e in self.encoders.values():
            e.enable(self.timestep)

        self.rot_motor = self.robot.getDevice("lidar pitch motor")
        self.rot_sensor = self.robot.getDevice("lidar pitch sensor")
        self.rot_sensor.enable(self.timestep)

        self.lidar = self.robot.getDevice("lidar")
        self.lidar.enable(self.timestep)

        self.gyro = self.robot.getDevice("gyro")
        self.imu = self.robot.getDevice("inertial unit")
        self.gyro.enable(self.timestep)
        self.imu.enable(self.timestep)

        # Front camera
        self.camera = None
        self.camera_adapter = None
        self.camera_width = 0
        self.camera_height = 0
        self.camera_snapshot_counter = 0
        self.next_camera_snapshot_time = 0.0
        self.next_camera_stream_time = 0.0

        try:
            self.camera = self.robot.getDevice(CAMERA_NAME)
            self.camera.enable(self.timestep)
            self.camera.recognitionEnable(self.timestep)
            self.camera_width = self.camera.getWidth()
            self.camera_height = self.camera.getHeight()
            self.camera_adapter = PerceptionCamera(self.camera)
            self.next_camera_snapshot_time = self.robot.getTime()
            print(
                f"[CAMERA] enabled: {self.camera_width}x{self.camera_height} "
                f"(device='{CAMERA_NAME}')"
            )
            if not CAMERA_DISPLAY_AVAILABLE:
                print("[CAMERA] OpenCV/numpy not available — snapshots/display disabled")
        except Exception as exc:
            self.camera = None
            print(f"[CAMERA] unavailable — continuing without camera: {exc}")

        # Keep the navigation/local-map pose in the same world frame as the
        # LiDAR points. The persistent cloud is emitted using the Webots
        # world transform, so starting odometry at (0, 0) would make the
        # local-map query search around the wrong location.
        startup_pos = self.robot_node.getPosition()
        self.x = clean(startup_pos[0]) if startup_pos else 0.0
        self.y = clean(startup_pos[1]) if startup_pos else 0.0
        self.theta = 0.0
        startup_rpy = self.imu.getRollPitchYaw()
        if startup_rpy:
            self.theta = clean(startup_rpy[2])
        self.prev_left_pos = None
        self.prev_right_pos = None
        self.ekf = DifferentialDrivePoseEKF(
            (self.x, self.y), self.theta, WHEEL_RADIUS, TRACK_WIDTH)
        self.pose_source = "ground_truth"
        self.gps_denied = False
        self._previous_gps_denied = False
        # Initial pose is the last valid GPS fix for a run that starts inside
        # the denied zone. After that, this value is updated only while GPS is
        # available; entering the zone never samples ground truth directly.
        self.last_gps_pose = (self.x, self.y, self.theta)

        # Shared geometry lives beside the controller directory, not inside it.
        try:
            controller_dir = Path(__file__).resolve().parent
            geometry_path = controller_dir.parent / "robot_geometry.json"
            self.calibration = load_geometry(geometry_path)
            self.camera_optical_R = self.calibration.base_from_camera[:3, :3]
            self.camera_optical_T = self.calibration.base_from_camera[:3, 3]
            print(f"[CAMERA] geometry loaded: {geometry_path}")
        except Exception as e:
            print(f"[CAMERA] failed to load extrinsics: {e}")
            self.calibration = None
            self.camera_optical_R = np.eye(3)
            self.camera_optical_T = np.zeros(3)

        self.scanning = False
        self.offset_angle = 0.0
        self.angle_min_deg = 0.0
        self.angle_max_deg = 360.0

        # Detection support
        self.yolo = None
        self.next_detection_time = 0.0
        self.detection_interval = 5.0
        self.detection_counter = 0
        self.detection_tracker = DetectionTracker(max_missed=4)
        self.detector = None
        self.locate_detector = None
        self.fire_smoke_detector = None
        self.active_detector = None
        self.detector_backend = "none"
        self.detection_mode = "yolo"
        self.detection_classes = []
        self.depth_model = None
        self.detection_error = None
        self.depth_error = None
        project_root = Path(__file__).resolve().parents[3]
        perception_config = project_root / "config" / "perception.yaml"
        detection_config = load_detection_config(perception_config)
        map_parameter = project_root / "map_parameter.yaml"
        map_config = {}
        try:
            import yaml
            with open(map_parameter, "r", encoding="utf-8") as stream:
                map_config = yaml.safe_load(stream) or {}
            detection_config = merge_config(detection_config, map_config.get("detection", {}))
        except Exception as exc:
            print(f"[DETECTION] map_parameter overrides unavailable: {exc}")
        localization_config = map_config.get("localization", {}) or {}
        ekf_config = localization_config.get("ekf", {}) or {}
        self.ekf_imu_yaw_offset = float(ekf_config.get("imu_yaw_offset_rad", math.pi / 2.0))
        denied_config = localization_config.get("gps_denied_zone", {}) or {}
        self.gps_denied_enabled = bool(denied_config.get("enabled", True))
        self.gps_denied_center = (
            float(denied_config.get("center_x", 18.1601)),
            float(denied_config.get("center_y", 18.1601)))
        self.gps_denied_radius = float(denied_config.get("radius_m", 21.0))
        print("[LOCALIZATION] GPS denied zone: enabled=%s center=(%.4f, %.4f) radius=%.2fm" % (
            self.gps_denied_enabled, *self.gps_denied_center, self.gps_denied_radius))
        print("[LOCALIZATION] EKF IMU yaw offset: %.6frad" % self.ekf_imu_yaw_offset)
        self.detection_interval = float(detection_config.get("interval_sec", 5.0))
        self.depth_scale = load_depth_scale(perception_config)
        self.depth_roi_border_fraction = load_depth_roi_border(perception_config)
        print(f"[DEPTH] Webots metric calibration scale: {self.depth_scale:.6f}")
        print(f"[DEPTH] inner ROI border: {self.depth_roi_border_fraction:.2f}")
        requested_mode = str(detection_config.get("mode", detection_config.get("backend", "yolo"))).lower()
        if requested_mode == "locate_anything":
            requested_mode = "locate"
        if requested_mode not in ("yolo", "locate", "hybrid"):
            print(f"[DETECTION] invalid mode {requested_mode!r}; using yolo")
            requested_mode = "yolo"
        self.detection_mode = requested_mode
        self.detection_classes = load_object_list(project_root, detection_config)
        locate_config = detection_config.get("locate_anything", {}) or {}
        if requested_mode in ("locate", "hybrid"):
            locate_cli = project_root / locate_config.get(
                "cli", "vendor/locate-anything.cpp/build/locate-anything-cli")
            locate_model = project_root / locate_config.get(
                "model", "models/detection/locate-anything-q8_0.gguf")
            try:
                self.locate_detector = LocateAnythingDetector(
                    locate_cli, locate_model,
                    locate_config.get(
                        "prompt",
                        "Locate all the instances that match the following description: "
                        + "</c>".join(self.detection_classes)),
                    threads=locate_config.get("threads", 8),
                    timeout_sec=locate_config.get("timeout_sec", 12.0),
                    mode=locate_config.get("mode", "hybrid"),
                )
                print(f"[DETECTION] LocateAnything model loaded once: {locate_cli}")
            except DetectorUnavailable as exc:
                print(f"[DETECTION] LocateAnything unavailable: {exc}")
        model_path = project_root / "models" / "detection" / "yolov8s-world.pt"
        if requested_mode in ("yolo", "hybrid"):
            try:
                self.detector = YoloDetector(
                    model_path,
                    confidence=float(detection_config.get("confidence", 0.50)),
                    candidate_confidence=float(detection_config.get("candidate_confidence", 0.10)),
                    image_size=int(detection_config.get("image_size", 640)),
                    tile_grid=int(detection_config.get("tile_grid", 2)),
                    enable_tiled_fallback=bool(detection_config.get("tiled_fallback", True)),
                    classes=self.detection_classes)
                self.yolo = self.detector.model
                print(f"[DETECTION] YOLO model ready: {model_path}")
            except DetectorUnavailable as exc:
                self.detection_error = str(exc)
                print(f"[DETECTION] YOLO ERROR: {self.detection_error}")
        if requested_mode == "yolo":
            self.active_detector = self.detector
        elif requested_mode == "locate":
            self.active_detector = self.locate_detector
        else:
            self.active_detector = self.detector or self.locate_detector
        self.detector_backend = requested_mode if self.active_detector is not None else "none"
        fire_config = detection_config.get("fire_smoke", {}) or {}
        if bool(fire_config.get("enabled", True)):
            fire_model = project_root / fire_config.get(
                "model", "models/detection/fire_smoke_yolov8n.pt")
            try:
                self.fire_smoke_detector = YoloDetector(
                    fire_model,
                    confidence=float(fire_config.get("confidence", 0.25)),
                    candidate_confidence=max(0.05, float(fire_config.get("confidence", 0.25)) * 0.5),
                    image_size=int(fire_config.get("image_size", 320)),
                    tile_grid=1,
                    enable_tiled_fallback=False,
                    classes=None)
                print(f"[DETECTION] fire/smoke auxiliary model ready: {fire_model}")
            except DetectorUnavailable as exc:
                print(f"[DETECTION] fire/smoke auxiliary model unavailable: {exc}")
        if self.active_detector is None:
            self.active_detector = self.fire_smoke_detector
            if self.active_detector is not None:
                self.detector_backend = requested_mode + "+fire_smoke"
        print(f"[DETECTION] mode={self.detection_mode} objects={self.detection_classes}")

        self._detection_jobs = queue.Queue(maxsize=1)
        self._detection_results = queue.Queue(maxsize=1)
        self._detection_stop = threading.Event()
        self._detection_worker = threading.Thread(
            target=self._detection_worker_loop, name="perception-worker", daemon=True)
        self._detection_worker.start()
        depth_checkpoint = project_root / "models" / "depth" / "depth_anything_v2_metric_vkitti_vits.pth"
        try:
            self.depth_model = DepthAnythingV2Metric(
                depth_checkpoint, encoder="vits", max_depth_m=80.0,
                implementation_root=project_root / "vendor" / "Depth-Anything-V2")
            print("[DEPTH] ready: Depth Anything V2 metric VKITTI / vits")
        except DepthModelUnavailable as exc:
            self.depth_error = str(exc)
            print(f"[DEPTH] ERROR: {self.depth_error}")

        self.cmd_v = 0.0
        self.cmd_omega = 0.0
        self.last_command_time = time.monotonic()

        # The original project delegated navigation to its browser viewer.
        # The Command Post companion launches Webots independently, so it
        # needs a self-contained, LiDAR-safe search mode as well.
        self.home_node = self.robot.getFromDef(HOME_DOCK_DEF)
        self.fire_node = self.robot.getFromDef("FIRE_TARGET")
        self.fire_animation_node = self.robot.getFromDef("FIRE_FLAME_ANIM")
        self.fire_light_node = self.robot.getFromDef("FIRE_LIGHT")
        self.victim_node = self.robot.getFromDef("VICTIM_TARGET")
        self.home_position = self._node_position(self.home_node) or (self.x, self.y)
        self.detected_targets = set()
        self.dashboard_beacons_sent = set()
        self.scene_confirmations = {}
        self.camera_detection_boxes = []
        # Beacon delivery is represented on the Command Post dashboard only.
        # The world scene remains focused on physical hazards and casualties.
        self.webots_beacon_nodes = {}
        self.search_phase = "SEARCH"
        self.search_goal_index = 0
        self.search_goals = self._build_search_goals()
        self.next_autonomy_report_time = 0.0
        self.obstacle_avoidance_active = False
        if AUTONOMOUS_SEARCH_ENABLED:
            self._reset_to_home_dock()
            print("[AUTONOMY] enabled: LiDAR-safe search started from HOME_DOCK")
        else:
            print("[AUTONOMY] disabled: waiting for browser velocity commands")

        self.clients = set()
        self.queues = set()
        self.loop = None
        self.total_points = 0
        self.frame_counter = 0

    @staticmethod
    def _node_position(node):
        """Return a Webots node's horizontal world position, if available."""
        if node is None:
            return None
        try:
            position = node.getPosition()
            return (clean(position[0]), clean(position[1])) if position else None
        except Exception:
            try:
                translation = node.getField("translation")
                position = translation.getSFVec3f() if translation is not None else None
                return (clean(position[0]), clean(position[1])) if position else None
            except Exception:
                return None

    def _reset_to_home_dock(self):
        """Begin every visual-companion run from the explicit safe dock."""
        try:
            translation = self.robot_node.getField("translation")
            if translation is not None:
                current = translation.getSFVec3f()
                translation.setSFVec3f([self.home_position[0], self.home_position[1], current[2]])
                self.robot_node.resetPhysics()
            self.x, self.y = self.home_position
            self.last_gps_pose = (self.x, self.y, self.theta)
            print("[AUTONOMY] HOME_DOCK reset: x=%.2f y=%.2f" % self.home_position)
        except Exception as exc:
            print(f"[AUTONOMY] home reset unavailable: {exc}")

    def _build_search_goals(self):
        """Create a repeatable coverage sweep around the scene targets and dock."""
        goals = []
        fire_position = self._node_position(self.fire_node)
        if fire_position is not None:
            # Confirm the fire from a safe viewing distance. The target is inside
            # the camera confirmation range, but no longer in the flame/LiDAR
            # avoidance zone where the vehicle can repeatedly pivot in place.
            goals.append((fire_position[0] + 3.2, fire_position[1] - 2.1, "FIRE SEARCH"))

        victim_position = self._node_position(self.victim_node)
        if victim_position is not None:
            # Use the original south-side approach point. It keeps the response
            # route outside the table while remaining within camera range.
            goals.append((victim_position[0] + 0.75, victim_position[1] - 0.55, "VICTIM RESPONSE"))
        goals.append((self.home_position[0], self.home_position[1], "RETURN HOME"))
        return goals

    def _lidar_sector_clearance(self, center_fraction, half_width_fraction=0.08):
        """Return the nearest valid range in one sector of the 360-degree scan."""
        try:
            ranges = self.lidar.getRangeImage()
            if not ranges:
                return float("inf")
            count = len(ranges)
            center = int(count * center_fraction) % count
            half_window = max(1, int(count * half_width_fraction))
            indices = [((center + offset) % count) for offset in range(-half_window, half_window + 1)]
            valid = [clean(ranges[index]) for index in indices if clean(ranges[index]) > 0.05]
            return min(valid) if valid else float("inf")
        except Exception:
            return float("inf")

    def _front_clearance(self):
        """Forward is the middle of Webots' full (-pi..+pi) LiDAR image."""
        return self._lidar_sector_clearance(0.5)

    def _side_clearances(self):
        """Compare left/right open space to choose a safe obstacle-avoidance turn."""
        return (
            self._lidar_sector_clearance(0.75, 0.10),
            self._lidar_sector_clearance(0.25, 0.10),
        )

    def _report_autonomy(self, goal_name, distance, clearance):
        now = self.robot.getTime()
        if now < self.next_autonomy_report_time:
            return
        self.next_autonomy_report_time = now + 1.5
        clearance_text = "CLEAR" if not math.isfinite(clearance) else f"{clearance:.2f}m"
        print(
            "[AUTONOMY] phase=%s target=%s distance=%.2fm front=%s" %
            (self.search_phase, goal_name, distance, clearance_text)
        )

    def _confirm_visible_targets(self):
        """Confirm mission targets through camera recognition, with a close-range fallback."""
        pose = self.robot_node.getPosition()
        if not pose:
            return
        recognized_models = set()
        recognized_objects = []
        if self.camera is not None:
            try:
                recognized_objects = list(self.camera.getRecognitionObjects())
                recognized_models = {item.getModel().strip().lower() for item in recognized_objects}
            except Exception:
                recognized_models = set()
                recognized_objects = []

        # Export every visible fire plus the victim box. This gives the browser
        # real camera-recognition coordinates and camera-relative distances for
        # a CV-style overlay, rather than decorative fixed labels.
        camera_boxes = []
        for item in recognized_objects:
            try:
                model = item.getModel().strip().lower()
                label = "FIRE" if model == "fire" else "VICTIM" if model in {"victim", "pedestrian"} else None
                if label is None:
                    continue
                center = item.getPositionOnImage()
                size = item.getSizeOnImage()
                relative_position = item.getPosition()
                width, height = float(size[0]), float(size[1])
                candidate = {
                    "label": label,
                    "confidence": 1.0,
                    "center": [round(float(center[0]), 1), round(float(center[1]), 1)],
                    "size": [round(width, 1), round(height, 1)],
                    "distance_m": round(math.sqrt(sum(float(axis) ** 2 for axis in relative_position)), 2),
                }
                camera_boxes.append(candidate)
            except Exception:
                continue
        self.camera_detection_boxes = camera_boxes
        yaw = self.imu.getRollPitchYaw()[2] + self.ekf_imu_yaw_offset
        for node, label in (
            (self.fire_node, "FIRE"),
            (self.victim_node, "VICTIM"),
        ):
            if label in self.detected_targets:
                continue
            target = self._node_position(node)
            if target is None:
                continue
            dx, dy = target[0] - pose[0], target[1] - pose[1]
            distance = math.hypot(dx, dy)
            bearing_error = wrap_angle(math.atan2(dy, dx) - yaw)
            expected_models = {
                "FIRE": {"fire"},
                "VICTIM": {"victim", "pedestrian"},
            }[label]
            camera_confirmed = bool(recognized_models & expected_models)
            proximity_confirmed = (
                distance <= TARGET_CONFIRM_RANGE_M and
                abs(bearing_error) <= CAMERA_HALF_FOV_RAD
            )
            if camera_confirmed or proximity_confirmed:
                self.detected_targets.add(label)
                source = "camera recognition" if camera_confirmed else "camera FOV/range"
                self.scene_confirmations[label.lower()] = {
                    "label": label,
                    "source": source,
                    "distance_m": round(distance, 2),
                    "timestamp": round(self.robot.getTime(), 2),
                }
                print("[TARGET DETECTED] %s confirmed by %s at %.2f m" % (label, source, distance))
                self._deploy_webots_beacon(label, target, yaw)
                if label == "FIRE":
                    victim_goal_index = next(
                        (index for index, goal in enumerate(self.search_goals)
                         if goal[2] == "VICTIM RESPONSE"),
                        None,
                    )
                    if victim_goal_index is not None:
                        self.search_goal_index = victim_goal_index
                        self.search_phase = "VICTIM_RESPONSE"
                        print("[AUTONOMY] fire confirmed; immediate diversion to victim response")

    def _deploy_webots_beacon(self, label, target, heading_rad):
        """Show a local beacon and submit one simulated sensor event to Living Map."""
        event_type = {
            "FIRE": "fire_detected",
            "VICTIM": "victim_detected",
        }[label]
        beacon_node = self.webots_beacon_nodes.get(label)
        if beacon_node is not None:
            try:
                scale = beacon_node.getField("scale")
                if scale is not None:
                    scale.setSFVec3f([1.0, 1.0, 1.0])
            except Exception:
                pass
        if label in self.dashboard_beacons_sent:
            return
        self.dashboard_beacons_sent.add(label)
        payload = json.dumps({
            "event_type": event_type,
            "severity": {"FIRE": 0.98, "VICTIM": 0.95}[label],
            "world_x": round(float(target[0]), 3),
            "world_y": round(float(target[1]), 3),
            "heading_deg": round(math.degrees(heading_rad) % 360.0, 1),
            "source": "WEBOTS_PIONEER_DEMO",
        }).encode("utf-8")

        def transmit():
            request = urllib.request.Request(
                WEBOTS_BEACON_ENDPOINT,
                data=payload,
                headers={"Content-Type": "application/json"},
                method="POST",
            )
            try:
                with urllib.request.urlopen(request, timeout=0.75) as response:
                    result = json.loads(response.read().decode("utf-8"))
                print("[WEBOTS BEACON] %s deployed → %s → ONA pipeline" % (
                    event_type, result.get("beacon_id", "queued")))
            except (urllib.error.URLError, TimeoutError, ValueError) as exc:
                # The visual world also runs standalone; never halt its mission
                # if the optional Living Map demo server is not running.
                print("[WEBOTS BEACON] %s local marker deployed; dashboard bridge unavailable: %s" % (event_type, exc))

        threading.Thread(target=transmit, name="WebotsBeaconTx", daemon=True).start()

    def _animate_fire(self):
        """Flicker the three-dimensional Webots flame without changing mission logic."""
        if self.fire_animation_node is None:
            return
        try:
            now = self.robot.getTime()
            scale = 0.92 + 0.12 * math.sin(now * 8.0) + 0.05 * math.sin(now * 15.0)
            scale_field = self.fire_animation_node.getField("scale")
            if scale_field is not None:
                scale_field.setSFVec3f([1.0 + 0.04 * math.sin(now * 5.0), 1.0, scale])
            if self.fire_light_node is not None:
                intensity = self.fire_light_node.getField("intensity")
                if intensity is not None:
                    intensity.setSFFloat(2.2 + 0.9 * (0.5 + 0.5 * math.sin(now * 10.0)))
        except Exception:
            # Animation is cosmetic; it must never interrupt autonomous motion.
            pass

    def _update_autonomous_search(self):
        """Drive a bounded LiDAR-safe coverage sweep and return to HOME_DOCK."""
        if not AUTONOMOUS_SEARCH_ENABLED or not self.search_goals:
            return
        self._confirm_visible_targets()
        pose = self.robot_node.getPosition()
        if not pose:
            return
        goal_x, goal_y, goal_name = self.search_goals[self.search_goal_index]
        dx, dy = goal_x - pose[0], goal_y - pose[1]
        distance = math.hypot(dx, dy)
        if distance <= GOAL_TOLERANCE_M:
            if self.search_goal_index >= len(self.search_goals) - 1:
                if self.search_phase != "COMPLETE":
                    self.search_phase = "COMPLETE"
                    self._stop()
                    print("[AUTONOMY] returned to HOME_DOCK; search complete")
            else:
                print("[AUTONOMY] reached %s" % goal_name)
                self.search_goal_index += 1
                if self.search_goal_index == len(self.search_goals) - 1:
                    self.search_phase = "RETURN_HOME"
            return

        yaw = self.imu.getRollPitchYaw()[2] + self.ekf_imu_yaw_offset
        heading_error = wrap_angle(math.atan2(dy, dx) - yaw)
        clearance = self._front_clearance()
        self._report_autonomy(goal_name, distance, clearance)
        if clearance < OBSTACLE_CLEARANCE_M:
            # Pick the more open side instead of treating a rear/side obstacle
            # as if it were in front of the robot.
            left_clearance, right_clearance = self._side_clearances()
            turn = TURN_SPEED_RAD_S if left_clearance >= right_clearance else -TURN_SPEED_RAD_S
            if not self.obstacle_avoidance_active:
                print(
                    "[AUTONOMY] obstacle ahead; turning %s (left=%.2fm right=%.2fm)" %
                    ("left" if turn > 0 else "right", left_clearance, right_clearance)
                )
            self.obstacle_avoidance_active = True
            self._set_velocity(0.0, turn)
            return
        self.obstacle_avoidance_active = False
        angular = max(-TURN_SPEED_RAD_S, min(TURN_SPEED_RAD_S, heading_error * 1.6))
        forward = SEARCH_SPEED_M_S if abs(heading_error) < 0.75 else SEARCH_SPEED_M_S * 0.35
        self._set_velocity(forward, angular)

    def _update_camera(self):
        """Display the live front-camera image and save periodic PNG snapshots."""
        if self.camera is None:
            return

        image = self.camera.getImage()
        if image is None:
            return

        # Webots camera images are BGRA, row-major, top-left origin.
        if CAMERA_PREVIEW_ENABLED and CAMERA_DISPLAY_AVAILABLE:
            try:
                frame = np.frombuffer(
                    image, dtype=np.uint8
                ).reshape((self.camera_height, self.camera_width, 4))

                cv2.imshow(
                    "Pioneer 3-AT_robot1 - front camera",
                    frame,
                )
                cv2.waitKey(1)
            except Exception as exc:
                # Do not let a local GUI problem stop robot control.
                print(f"[CAMERA] display error: {exc}")

        now = self.robot.getTime()
        if now >= self.next_camera_snapshot_time:
            name = (
                f"{CAMERA_SNAPSHOT_PREFIX}_"
                f"{self.camera_snapshot_counter:04d}.png"
            )
            try:
                result = self.camera.saveImage(name, 100)
                if result == 0:
                    print(f"[CAMERA] saved {name}")
                    self.camera_snapshot_counter += 1
            except Exception as exc:
                print(f"[CAMERA] snapshot error: {exc}")

            self.next_camera_snapshot_time = now + CAMERA_SNAPSHOT_INTERVAL_SEC

    def _broadcast_camera_stream(self):
        """Send the real front-camera image and scene confirmations to the visual panel."""
        if (self.camera is None or not self.queues or cv2 is None or np is None):
            return
        now = self.robot.getTime()
        if now < self.next_camera_stream_time:
            return
        image = self.camera.getImage()
        if image is None:
            return
        try:
            frame = np.frombuffer(image, dtype=np.uint8).reshape(
                (self.camera_height, self.camera_width, 4))
            ok, encoded = cv2.imencode(
                ".jpg", frame[:, :, :3],
                [int(cv2.IMWRITE_JPEG_QUALITY), CAMERA_STREAM_JPEG_QUALITY],
            )
            if not ok:
                return
            self._broadcast_json({
                "type": "camera_frame",
                "timestamp": round(now, 2),
                "image": "data:image/jpeg;base64," + base64.b64encode(encoded.tobytes()).decode("ascii"),
                "detections": list(self.scene_confirmations.values()),
                "vision_boxes": self.camera_detection_boxes,
                "image_size": [self.camera_width, self.camera_height],
            })
            self.next_camera_stream_time = now + CAMERA_STREAM_INTERVAL_SEC
        except Exception as exc:
            print(f"[CAMERA] stream error: {exc}")

    def _angle_in_range(self, angle_deg):
        mn, mx = self.angle_min_deg, self.angle_max_deg
        if mn <= mx:
            return mn <= angle_deg <= mx
        return angle_deg >= mn or angle_deg <= mx

    def _set_velocity(self, v, omega):
        try:
            v = float(v)
            omega = float(omega)
        except (TypeError, ValueError):
            self.cmd_v = 0.0
            self.cmd_omega = 0.0
            return

        # Conservative safety limits; navigation engine can request anything
        # below these values without risking an invalid Webots motor command.
        v = max(-0.7, min(0.7, v))
        omega = max(-2.2, min(2.2, omega))
        self.cmd_v = v
        self.cmd_omega = omega
        self.last_command_time = time.monotonic()

    def _stop(self):
        self.cmd_v = 0.0
        self.cmd_omega = 0.0

    def _handle_command(self, data):
        msg_type = data.get("type")
        if msg_type == "velocity":
            self._set_velocity(data.get("v", 0.0), data.get("omega", 0.0))
        elif msg_type == "stop":
            self._stop()
            self.last_command_time = time.monotonic()
        elif msg_type == "scan_control":
            enabled = bool(data.get("enabled", False))
            self.scanning = enabled
            if not enabled:
                self.rot_motor.setVelocity(0.0)
            print(f"[SCAN] {'ON' if enabled else 'OFF'} (viewer command)")
        elif msg_type == "config":
            if "offset_angle" in data:
                try:
                    self.offset_angle = float(data["offset_angle"])
                except (TypeError, ValueError):
                    pass
            if "angle_min" in data:
                try:
                    self.angle_min_deg = (math.degrees(float(data["angle_min"])) + 360.0) % 360.0
                except (TypeError, ValueError):
                    pass
            if "angle_max" in data:
                try:
                    self.angle_max_deg = (math.degrees(float(data["angle_max"])) + 360.0) % 360.0
                except (TypeError, ValueError):
                    pass
            print(
                f"[CONFIG] offset={math.degrees(self.offset_angle):.1f}° "
                f"angle={self.angle_min_deg:.1f}→{self.angle_max_deg:.1f}°"
            )
        elif msg_type == "ping":
            self._broadcast_json({"type": "pong", "timestamp": self.robot.getTime()})

    async def _ws_handler(self, websocket):
        q = asyncio.Queue()
        self.clients.add(websocket)
        self.queues.add(q)
        print(f"[WS] Client CONNECTED (total: {len(self.clients)})")

        async def recv_loop():
            try:
                async for msg in websocket:
                    try:
                        data = json.loads(msg)
                        if isinstance(data, dict):
                            self._handle_command(data)
                    except Exception as exc:
                        print(f"[WS] command error: {exc}")
            except websockets.exceptions.ConnectionClosed:
                pass

        recv_task = asyncio.create_task(recv_loop())
        try:
            while True:
                out_msg = await q.get()
                await websocket.send(out_msg)
        except (websockets.exceptions.ConnectionClosed, asyncio.CancelledError):
            pass
        finally:
            recv_task.cancel()
            try:
                await recv_task
            except asyncio.CancelledError:
                pass
            self.clients.discard(websocket)
            self.queues.discard(q)
            # If this was the last browser, stop immediately.
            if not self.clients:
                self._stop()
                self.scanning = False
                self.rot_motor.setVelocity(0.0)
            print(f"[WS] Client DISCONNECTED (total: {len(self.clients)})")

    def _start_ws_server(self):
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)

        async def main():
            await websockets.serve(
                self._ws_handler,
                WS_HOST,
                WS_PORT,
                ping_interval=None,
                max_size=None,
            )
            print(f"[WS] Server listening on ws://{WS_HOST}:{WS_PORT}")
            await asyncio.Future()

        self.loop.run_until_complete(main())

    def _broadcast(self, message):
        if self.loop is None or not self.queues:
            return
        for q in list(self.queues):
            asyncio.run_coroutine_threadsafe(q.put(message), self.loop)

    def _broadcast_json(self, payload):
        self._broadcast(json.dumps(clean_all(payload), separators=(",", ":")))

    @staticmethod
    def _diff_drive(v, omega):
        vl = (v - omega * TRACK_WIDTH / 2.0) / WHEEL_RADIUS
        vr = (v + omega * TRACK_WIDTH / 2.0) / WHEEL_RADIUS
        return vl, vr

    def _apply_drive(self):
        if (not AUTONOMOUS_SEARCH_ENABLED and
                time.monotonic() - self.last_command_time > COMMAND_TIMEOUT_SEC):
            self._stop()

        vl, vr = self._diff_drive(self.cmd_v, self.cmd_omega)
        self.motors["fl"].setVelocity(vl)
        self.motors["fr"].setVelocity(vr)
        self.motors["rl"].setVelocity(vl)
        self.motors["rr"].setVelocity(vr)

    def _update_odometry(self):
        raw_left = self.encoders["rl"].getValue()
        raw_right = self.encoders["rr"].getValue()

        if self.prev_left_pos is None:
            self.prev_left_pos = clean(raw_left)
            self.prev_right_pos = clean(raw_right)
            rpy = self.imu.getRollPitchYaw()
            if rpy:
                self.theta = wrap_angle(clean(rpy[2]) + self.ekf_imu_yaw_offset)
            startup_pos = self.robot_node.getPosition()
            if startup_pos:
                self.x = clean(startup_pos[0])
                self.y = clean(startup_pos[1])
            return

        left_pos = clean(raw_left)
        right_pos = clean(raw_right)
        dl = (left_pos - self.prev_left_pos) * WHEEL_RADIUS
        dr = (right_pos - self.prev_right_pos) * WHEEL_RADIUS
        self.prev_left_pos = left_pos
        self.prev_right_pos = right_pos

        if not all(math.isfinite(v) for v in (dl, dr)):
            return

        rpy = self.imu.getRollPitchYaw()
        imu_yaw = (wrap_angle(clean(rpy[2]) + self.ekf_imu_yaw_offset)
                   if rpy else None)
        # The EKF consumes wheel angle increments, not absolute encoder values.
        self.x, self.y, self.theta = self.ekf.step(dl / WHEEL_RADIUS, dr / WHEEL_RADIUS, imu_yaw)

        if not all(math.isfinite(v) for v in (self.x, self.y, self.theta)):
            print("[WARN] Odom NaN — resetting")
            self.x = self.y = self.theta = 0.0

    @staticmethod
    def _rotation_from_yaw(yaw):
        c, s = math.cos(float(yaw)), math.sin(float(yaw))
        return (c, -s, 0.0, s, c, 0.0, 0.0, 0.0, 1.0)

    def _pose_in_use(self):
        """Return the pose visible to navigation/perception.

        Ground truth is used only as the simulated GPS availability oracle and
        outside the denied area. The position returned in the denied area is
        exclusively the wheel/IMU EKF estimate.
        """
        truth = self.robot_node.getPosition()
        truth_x, truth_y = float(truth[0]), float(truth[1])
        dx = truth_x - self.gps_denied_center[0]
        dy = truth_y - self.gps_denied_center[1]
        denied = self.gps_denied_enabled and (dx * dx + dy * dy <= self.gps_denied_radius ** 2)
        if denied != self._previous_gps_denied:
            if denied:
                self.ekf.reset(self.last_gps_pose[:2], self.last_gps_pose[2])
                print("[LOCALIZATION] GPS denied: switching to EKF sensor pose")
            else:
                print("[LOCALIZATION] GPS available: switching to ground-truth pose")
            self._previous_gps_denied = denied
        self.gps_denied = denied
        if denied:
            self.pose_source = "ekf"
            return (self.ekf.state[0], self.ekf.state[1], self.ekf.state[2],
                    self._rotation_from_yaw(self.ekf.state[2]))
        self.pose_source = "ground_truth"
        truth_rot = tuple(float(value) for value in self.robot_node.getOrientation())
        truth_yaw = math.atan2(truth_rot[3], truth_rot[0])
        self.last_gps_pose = (truth_x, truth_y, truth_yaw)
        return (truth_x, truth_y, float(truth[2]), truth_rot)

    def _get_lidar_points(self):
        if not self.scanning:
            return []

        rot_motor_pos = self.rot_sensor.getValue()
        ranges = self.lidar.getRangeImage()
        if ranges is None:
            return []

        res = self.lidar.getHorizontalResolution()
        fov = self.lidar.getFov()
        effective_rot = rot_motor_pos + self.offset_angle + INITIAL_PITCH
        cr, sr = math.cos(effective_rot), math.sin(effective_rot)

        pose_x, pose_y, pose_z, rot = self._pose_in_use()
        r00, r01, r02 = rot[0], rot[1], rot[2]
        r10, r11, r12 = rot[3], rot[4], rot[5]
        r20, r21, r22 = rot[6], rot[7], rot[8]

        points = []
        for i in range(0, res, LIDAR_SUBSAMPLE):
            dist = ranges[i]
            if dist == float("inf") or not math.isfinite(dist) or dist <= 0.0 or dist > 12.0:
                continue

            phi = i * fov / res - fov / 2.0
            angle_deg = (-math.degrees(phi) + 360.0) % 360.0
            if not self._angle_in_range(angle_deg):
                continue

            px_l = dist * math.cos(phi)
            py_l = dist * math.sin(phi)
            pz_l = 0.0
            px_j, py_j, pz_j = px_l, py_l, pz_l + LIDAR_ARM_LEN
            px_r = px_j
            py_r = py_j * cr - pz_j * sr
            pz_r = py_j * sr + pz_j * cr
            px_rob = LIDAR_JOINT_X + px_r
            py_rob = py_r
            pz_rob = LIDAR_JOINT_Z + pz_r

            px_w = pose_x + r00 * px_rob + r01 * py_rob + r02 * pz_rob
            py_w = pose_y + r10 * px_rob + r11 * py_rob + r12 * pz_rob
            pz_w = pose_z + r20 * px_rob + r21 * py_rob + r22 * pz_rob
            points.append([px_w, py_w, pz_w])

        self.total_points += len(points)
        return points

    def _build_binary_packet(self):
        pose_x, pose_y, pose_z, rot = self._pose_in_use()
        pts = self._get_lidar_points()
        rpy = self.imu.getRollPitchYaw()
        imu_yaw = rpy[2] if rpy else 0.0
        truth_position = self.robot_node.getPosition()
        truth_orientation = tuple(float(value) for value in self.robot_node.getOrientation())
        metadata = {
            "type": "scan",
            "scanning": self.scanning,
            "offset_angle": self.offset_angle,
            "angle_min_deg": self.angle_min_deg,
            "angle_max_deg": self.angle_max_deg,
            "robot": {
                "x": pose_x,
                "y": pose_y,
                "theta": self.theta if self.pose_source == "ground_truth" else self.ekf.state[2],
                "orientation": list(rot),
                "pose_source": self.pose_source,
                "gps_denied": self.gps_denied,
            },
            "ground_truth": {
                "x": float(truth_position[0]),
                "y": float(truth_position[1]),
                "z": float(truth_position[2]),
                "yaw": math.atan2(truth_orientation[3], truth_orientation[0]),
            },
            "lidar": {
                "mount_x": LIDAR_JOINT_X,
                "mount_y": 0.0,
                "mount_z": LIDAR_JOINT_Z,
                "pitch_zero": INITIAL_PITCH,
            },
            "imu": {
                "roll": rpy[0] if rpy else 0.0,
                "pitch": rpy[1] if rpy else 0.0,
                "yaw": imu_yaw,
            },
            "command": {"v": self.cmd_v, "omega": self.cmd_omega},
            "total_points": self.total_points,
            "new_point_count": len(pts),
            "timestamp": self.robot.getTime(),
        }
        metadata = clean_all(metadata)
        header_bytes = json.dumps(metadata, separators=(",", ":")).encode("utf-8")
        point_bytes = struct.pack(
            f"<{len(pts) * 3}f", *[c for p in pts for c in p]
        ) if pts else b""
        frame = LSCN_MAGIC + struct.pack("<I", len(header_bytes)) + header_bytes + point_bytes
        return frame, metadata, pts

    def _run_detection(self, rgb, capture_position, capture_orientation,
                       capture_time, capture_pose_source):
        """Run detection/depth on a captured frame and its captured robot pose.
        Each dict contains a detection_id, class/confidence/bbox, explicit
        depth validity, and positions only when metric depth is available.
        """
        if self.active_detector is None:
            return []
        if self.calibration is None:
            return []
        try:
            detectors = []
            if self.detection_mode in ("yolo", "hybrid") and self.detector is not None:
                detectors.append(self.detector)
            if self.detection_mode in ("locate", "hybrid") and self.locate_detector is not None:
                detectors.append(self.locate_detector)
            if self.fire_smoke_detector is not None:
                detectors.append(self.fire_smoke_detector)
            if len(detectors) > 1:
                with ThreadPoolExecutor(max_workers=len(detectors)) as pool:
                    batches = list(pool.map(lambda detector: detector.detect(rgb), detectors))
                raw_detections = []
                for batch in batches:
                    raw_detections.extend(batch)
                # Keep the highest-confidence box when both models identify
                # the same object, while retaining distinct nearby objects.
                def same_object(left, right):
                    if left.class_name != right.class_name:
                        return False
                    iou = YoloDetector._iou(left.bbox, right.bbox)
                    if iou >= 0.30:
                        return True
                    lx = (left.x1 + left.x2) * 0.5
                    ly = (left.y1 + left.y2) * 0.5
                    rx = (right.x1 + right.x2) * 0.5
                    ry = (right.y1 + right.y2) * 0.5
                    center_distance = math.hypot(lx - rx, ly - ry)
                    left_area = max(1.0, (left.x2 - left.x1) * (left.y2 - left.y1))
                    right_area = max(1.0, (right.x2 - right.x1) * (right.y2 - right.y1))
                    return center_distance <= 0.20 * math.sqrt(min(left_area, right_area))
                merged = []
                for detection in sorted(raw_detections,
                                        key=lambda item: item.confidence, reverse=True):
                    duplicate = any(same_object(detection, existing) for existing in merged)
                    if not duplicate:
                        merged.append(detection)
                raw_detections = merged
            elif detectors:
                self.active_detector = detectors[0]
                if self.active_detector is self.detector:
                    raw_detections = self.detector.detect(rgb)
                else:
                    raw_detections = self.active_detector.detect(rgb)
        except Exception as exc:
            print(f"[DETECTION] inference failed: {exc}")
            return []
        depth_map = None
        if self.depth_model is not None:
            try:
                depth_map = self.depth_model.predict(rgb)
            except Exception as exc:
                print(f"[DEPTH] inference failed: {exc}")
        else:
            print("[DEPTH] unavailable; detections will be reported without localization")
        detections = []
        tracked_detections = self.detection_tracker.update(raw_detections)
        track_ids = {id(detection): track_id for detection, track_id in tracked_detections}
        for detection in raw_detections:
                track_id = track_ids[id(detection)]
                if depth_map is None:
                    output = {"detection_id": track_id, "track_id": track_id,
                              "class": detection.class_name,
                              "confidence": detection.confidence,
                              "bbox": list(detection.bbox), "depth_m": None,
                              "depth_valid": False, "localization_valid": False,
                              "pose_source": "odometry"}
                else:
                    R_wb = np.array([[capture_orientation[0], capture_orientation[1], capture_orientation[2]],
                                     [capture_orientation[3], capture_orientation[4], capture_orientation[5]],
                                     [capture_orientation[6], capture_orientation[7], capture_orientation[8]]])
                    output = fuse_detection(detection, depth_map, self.calibration,
                                            R_wb, capture_position,
                                            track_id, capture_pose_source,
                                            border_fraction=self.depth_roi_border_fraction,
                                            depth_scale=self.depth_scale)
                    output["track_id"] = track_id
                detections.append(output)
                if output.get("localization_valid"):
                    print("[PERCEPTION] %s confidence=%.2f world=(%.2f, %.2f, %.2f) depth=%.2fm" % (
                        output["class"], output["confidence"],
                        *output["position_world_frame"], output["depth_m"]))
                else:
                    print("[PERCEPTION] %s confidence=%.2f depth unavailable; localization invalid" % (
                        output["class"], output["confidence"]))
                self.detection_counter += 1
        if not detections:
            print("[PERCEPTION] clear: no configured objects detected in current frame")
        return detections

    def _detection_worker_loop(self):
        while not self._detection_stop.is_set():
            try:
                job = self._detection_jobs.get(timeout=0.2)
            except queue.Empty:
                continue
            if job is None:
                return
            try:
                detections = self._run_detection(
                    job["rgb"], job["position"], job["orientation"],
                    job["capture_time"], job["pose_source"])
                result = {"detections": detections, "capture_time": job["capture_time"],
                          "backend": self.detector_backend}
                try:
                    self._detection_results.put_nowait(result)
                except queue.Full:
                    try:
                        self._detection_results.get_nowait()
                    except queue.Empty:
                        pass
                    self._detection_results.put_nowait(result)
            except Exception as exc:
                print(f"[DETECTION] worker failed: {exc}")

    def _enqueue_detection(self):
        if self.active_detector is None or self.camera_adapter is None or self.calibration is None:
            return
        try:
            capture_pose = self._pose_in_use()
            job = {
                "rgb": self.camera_adapter.capture_rgb(),
                "capture_time": float(self.robot.getTime()),
                "position": capture_pose[:3],
                "orientation": capture_pose[3],
                "pose_source": self.pose_source,
            }
            try:
                self._detection_jobs.put_nowait(job)
            except queue.Full:
                print("[DETECTION] inference busy; dropping capture")
        except Exception as exc:
            print(f"[DETECTION] capture failed: {exc}")

    def _poll_detection_result(self):
        try:
            return self._detection_results.get_nowait()
        except queue.Empty:
            return None

    def _build_detection_packet(self, detections, capture_time=None, backend=None):
        """Create a JSON‑serializable packet for a list of detections."""
        return {
            "type": "detection",
            "status": "detected" if detections else "clear",
            "timestamp": self.robot.getTime() if capture_time is None else capture_time,
            "processed_at": self.robot.getTime(),
            "detection_count": len(detections),
            "detector_ready": self.active_detector is not None,
            "detector_backend": self.detector_backend if backend is None else backend,
            "depth_ready": self.depth_model is not None,
            "detections": detections,
        }

    def run(self):
        ws_thread = threading.Thread(target=self._start_ws_server, daemon=True)
        ws_thread.start()

        print("\n" + "=" * 64)
        print(" Pioneer 3-AT LiDAR Controller — VIEWER COMMAND MODE")
        print("=" * 64)
        print(" Webots keyboard driving is disabled.")
        print(" Browser owns manual driving and Auto Pilot.")
        print(" Command watchdog: %.2fs" % COMMAND_TIMEOUT_SEC)
        print(" Front camera: %s" % ("enabled" if self.camera is not None else "unavailable"))
        print(" WebSocket: ws://%s:%d" % (WS_HOST, WS_PORT))
        print("=" * 64 + "\n")

        while self.robot.step(self.timestep) != -1:
            self.frame_counter += 1
            self._update_autonomous_search()
            self._animate_fire()
            self._apply_drive()

            if self.scanning:
                self.rot_motor.setPosition(float("inf"))
                self.rot_motor.setVelocity(ROTATION_SPEED)
            else:
                self.rot_motor.setVelocity(0.0)

            self._update_odometry()
            self._update_camera()
            self._broadcast_camera_stream()

            if self.frame_counter % SEND_EVERY_N == 0:
                frame, meta, pts = self._build_binary_packet()
                self._broadcast(frame)
                completed = self._poll_detection_result()
                if completed is not None:
                    self._broadcast_json(self._build_detection_packet(
                        completed["detections"], completed["capture_time"], completed["backend"]))
                now = self.robot.getTime()
                if now >= self.next_detection_time:
                    self._enqueue_detection()
                    self.next_detection_time = now + self.detection_interval

        if CAMERA_DISPLAY_AVAILABLE:
            try:
                cv2.destroyAllWindows()
            except Exception:
                pass


if __name__ == "__main__":
    PioneerController().run()
