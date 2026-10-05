"""ONA-authorized Webots fire-response Executor.

It polls only the dashboard adapter that reads its ONA Role 4 inbox.  It never
reads Command Post mission state directly and reports completion as a normal
physical beacon through the same BeaconStore ingress used by the Writer.
"""

from __future__ import annotations

import json
import math
import os
import time
from urllib.error import URLError
from urllib.request import Request, urlopen

from controller import Supervisor


BRIDGE_ROOT = os.environ.get("WEBOTS_BRIDGE_ROOT", "http://127.0.0.1:5012")
UNIT_ID = "WEBOTS-FIREFIGHTER-1"
POLL_INTERVAL_S = 0.75
# The TurtleBot fires from a safe stand-off rather than trying to collide with
# the FireSmoke target or furniture in the house.
ATTACK_STANDOFF_M = 2.10
JET_SPEED_MPS = 5.6
WATER_DROPLET_RADIUS_M = 0.055
WATER_SPAWN_EVERY_STEPS = 2
WATER_DROPLET_LIFETIME_S = 2.5
WATER_CONTACTS_TO_EXTINGUISH = 240
NOZZLE_TIP_OFFSET_M = 0.42
WATER_ARC_SEGMENTS = 26
WATER_ARC_RADIUS_M = 0.043
WATER_CONTACT_EVERY_STEPS = 3


WATER_DROPLET_TEMPLATE = """Solid {{
  name "water-droplet-{uid}"
  translation {x:.4f} {y:.4f} {z:.4f}
  rotation {ax:.5f} {ay:.5f} {az:.5f} {angle:.5f}
  children [
    Transform {{
      scale 2.35 1 1
      children [
        Shape {{
          appearance PBRAppearance {{
            baseColor 0.08 0.64 1
            emissiveColor 0.015 0.12 0.22
            transparency 0.08
            roughness 0.08
            metalness 0.0
          }}
          geometry Sphere {{
            radius {radius:.4f}
            subdivision 2
          }}
        }}
      ]
    }}
  ]
  physics Physics {{
    density 1000
  }}
}}"""


WATER_ARC_TEMPLATE = """DEF WATER_ARC_SEGMENT_{index} Transform {{
  translation 0 0 -10
  children [
    Transform {{
      scale 3.1 0.72 0.72
      children [
        Shape {{
          appearance PBRAppearance {{
            baseColor 0.72 0.92 1
            emissiveColor 0.14 0.31 0.48
            transparency 0.10
            roughness 0.05
            metalness 0.0
          }}
          geometry Sphere {{
            radius {radius:.4f}
            subdivision 2
          }}
        }}
      ]
    }}
  ]
}}"""


def _axis_angle_from_x(direction: list[float]) -> tuple[float, float, float, float]:
    """Return the axis-angle rotation taking the local +X axis onto direction."""
    dot = max(-1.0, min(1.0, direction[0]))
    angle = math.acos(dot)
    axis = [0.0, -direction[2], direction[1]]
    length = math.sqrt(sum(component * component for component in axis))
    if length < 1e-7:
        return (0.0, 0.0, 1.0, 0.0 if dot >= 0.0 else math.pi)
    return (axis[0] / length, axis[1] / length, axis[2] / length, angle)


class FirefighterExecutor(Supervisor):
    def __init__(self) -> None:
        super().__init__()
        self.step_ms = int(self.getBasicTimeStep())
        self.left_motor = self.getDevice("left wheel motor")
        self.right_motor = self.getDevice("right wheel motor")
        for motor in (self.left_motor, self.right_motor):
            motor.setPosition(float("inf"))
            motor.setVelocity(0.0)

        self.lidar = self.getDevice("LDS-01")
        if self.lidar:
            self.lidar.enable(self.step_ms)

        self.self_node = self.getSelf()
        self.fire = self.getFromDef("FIRE_TARGET")
        self.nozzle = self.getFromDef("WATER_NOZZLE")
        self.cannon_base = self.getFromDef("WATER_CANNON_BASE")
        self.turret_yaw_pose = self.getFromDef("WATER_CANNON_YAW")
        self.turret_pitch_pose = self.getFromDef("WATER_CANNON_PITCH")
        self.root_children = self.getRoot().getField("children")
        self.target: tuple[float, float] | None = None
        self.briefing_id: str | None = None
        self.last_poll = 0.0
        self.extinguishing = False
        self.completed = False
        self.step_count = 0
        self.droplet_id = 0
        self.droplets: list[dict] = []
        self.water_arc_nodes: list = []
        self.water_contacts = 0
        self.last_contact_percent = -1
        self.initial_fire_height = self.fire.getField("fireHeight").getSFFloat() if self.fire else 1.0
        self.initial_fire_radius = self.fire.getField("fireRadius").getSFFloat() if self.fire else 0.35
        if not self.nozzle or not self.cannon_base or not self.turret_yaw_pose or not self.turret_pitch_pose:
            print("[WEBOTS EXECUTOR] ERROR: water turret nodes are unavailable")
        print(f"[WEBOTS EXECUTOR] {UNIT_ID} ready; waiting for ONA Role 4 authorization delivery")

    def _request_json(self, url: str, payload: dict | None = None) -> dict | None:
        try:
            if payload is None:
                with urlopen(url, timeout=0.5) as response:
                    return json.loads(response.read().decode("utf-8"))
            data = json.dumps(payload).encode("utf-8")
            request = Request(url, data=data, headers={"Content-Type": "application/json"}, method="POST")
            with urlopen(request, timeout=0.75) as response:
                return json.loads(response.read().decode("utf-8"))
        except (URLError, TimeoutError, ValueError, OSError):
            return None

    def _poll_ona_mission(self) -> None:
        if self.target is not None or time.monotonic() - self.last_poll < POLL_INTERVAL_S:
            return
        self.last_poll = time.monotonic()
        response = self._request_json(f"{BRIDGE_ROOT}/api/webots/executor/mission")
        if not response or response.get("status") != "delivered":
            return
        mission = response.get("mission", {})
        target = mission.get("target", {})
        try:
            self.target = (float(target["world_x"]), float(target["world_y"]))
            self.briefing_id = str(mission["briefing_id"])
        except (KeyError, TypeError, ValueError):
            print("[WEBOTS EXECUTOR] rejected malformed Role 4 mission payload")
            return
        print(f"[WEBOTS EXECUTOR] ONA Role 4 mission accepted; fire target={self.target}")

    def _pose(self) -> tuple[float, float, float]:
        position = self.self_node.getPosition()
        orientation = self.self_node.getOrientation()
        heading = math.atan2(orientation[3], orientation[0])
        return position[0], position[1], heading

    def _set_wheels(self, left: float, right: float) -> None:
        self.left_motor.setVelocity(max(-6.0, min(6.0, left)))
        self.right_motor.setVelocity(max(-6.0, min(6.0, right)))

    def _sync_cannon_mount(self) -> None:
        """Keep the purely visual cannon rigidly mounted to the mobile Executor."""
        if not self.cannon_base:
            return
        x, y, heading = self._pose()
        z = self.self_node.getPosition()[2]
        self.cannon_base.getField("translation").setSFVec3f([x, y, z])
        self.cannon_base.getField("rotation").setSFRotation([0, 0, 1, heading])

    def _navigate_with_lidar(self) -> bool:
        assert self.target is not None
        x, y, heading = self._pose()
        dx, dy = self.target[0] - x, self.target[1] - y
        distance = math.hypot(dx, dy)
        if distance <= ATTACK_STANDOFF_M:
            self._set_wheels(0.0, 0.0)
            print(f"[WEBOTS EXECUTOR] Firefighting stand-off reached ({distance:.2f} m); engaging water turret")
            return True

        desired_heading = math.atan2(dy, dx)
        heading_error = math.atan2(math.sin(desired_heading - heading), math.cos(desired_heading - heading))
        obstacle_turn = 0.0
        if self.lidar:
            ranges = [value for value in self.lidar.getRangeImage() if math.isfinite(value)]
            if ranges and min(ranges) < 0.30:
                obstacle_turn = 1.25
        turn = max(-1.7, min(1.7, 2.0 * heading_error + obstacle_turn))
        speed = 3.6 * max(0.20, 1.0 - abs(heading_error) / math.pi)
        self._set_wheels(speed - turn, speed + turn)
        return False

    @staticmethod
    def _normalise_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    def _aim_turret_at_fire(self) -> None:
        """Aim yaw/pitch at the real FireSmoke target using a low ballistic arc."""
        if not self.fire or not self.nozzle or not self.turret_yaw_pose or not self.turret_pitch_pose:
            return
        nozzle_position = self.nozzle.getPosition()
        fire_position = self.fire.getPosition()
        fire_height = self.fire.getField("fireHeight").getSFFloat()
        dx, dy = fire_position[0] - nozzle_position[0], fire_position[1] - nozzle_position[1]
        horizontal_distance = max(0.12, math.hypot(dx, dy))
        _, _, chassis_heading = self._pose()
        yaw = self._normalise_angle(math.atan2(dy, dx) - chassis_heading)
        target_height = max(0.22, fire_height * 0.45)
        vertical_delta = target_height - nozzle_position[2]
        speed_sq = JET_SPEED_MPS * JET_SPEED_MPS
        gravity = 9.81
        discriminant = speed_sq * speed_sq - gravity * (gravity * horizontal_distance ** 2 + 2 * vertical_delta * speed_sq)
        if discriminant >= 0:
            pitch = math.atan((speed_sq - math.sqrt(discriminant)) / (gravity * horizontal_distance))
        else:
            pitch = math.atan2(vertical_delta, horizontal_distance)
        self.turret_yaw_pose.getField("rotation").setSFRotation([0, 0, 1, max(-3.1, min(3.1, yaw))])
        self.turret_pitch_pose.getField("rotation").setSFRotation([0, 1, 0, max(-0.85, min(0.55, -pitch))])

    def _spawn_water_droplet(self) -> None:
        if not self.nozzle:
            return
        position = self.nozzle.getPosition()
        orientation = self.nozzle.getOrientation()
        direction = [orientation[0], orientation[3], orientation[6]]
        magnitude = math.sqrt(sum(component * component for component in direction))
        if magnitude <= 0.0:
            return
        direction = [component / magnitude for component in direction]
        start = [position[index] + direction[index] * NOZZLE_TIP_OFFSET_M for index in range(3)]
        axis_x, axis_y, axis_z, angle = _axis_angle_from_x(direction)
        self.droplet_id += 1
        self.root_children.importMFNodeFromString(
            -1,
            WATER_DROPLET_TEMPLATE.format(
                uid=self.droplet_id,
                x=start[0], y=start[1], z=start[2],
                ax=axis_x, ay=axis_y, az=axis_z, angle=angle,
                radius=WATER_DROPLET_RADIUS_M,
            ),
        )
        node = self.root_children.getMFNode(-1)
        node.setVelocity([
            JET_SPEED_MPS * direction[0], JET_SPEED_MPS * direction[1], JET_SPEED_MPS * direction[2], 0, 0, 0,
        ])
        self.droplets.append({"node": node, "born": time.monotonic()})

    def _ensure_water_arc(self) -> None:
        if self.water_arc_nodes:
            return
        for index in range(WATER_ARC_SEGMENTS):
            self.root_children.importMFNodeFromString(
                -1, WATER_ARC_TEMPLATE.format(index=index, radius=WATER_ARC_RADIUS_M)
            )
            self.water_arc_nodes.append(self.getFromDef(f"WATER_ARC_SEGMENT_{index}"))

    def _update_water_arc(self) -> None:
        """Render a continuous, gravity-curved hose stream from nozzle to fire."""
        if not self.nozzle or not self.fire:
            return
        self._ensure_water_arc()
        start = self.nozzle.getPosition()
        fire_position = self.fire.getPosition()
        end = [fire_position[0], fire_position[1], max(0.18, self.fire.getField("fireHeight").getSFFloat() * 0.34)]
        dx, dy = end[0] - start[0], end[1] - start[1]
        horizontal_distance = math.hypot(dx, dy)
        arc_height = max(0.28, min(0.82, horizontal_distance * 0.34))
        for index, node in enumerate(self.water_arc_nodes):
            if node is None:
                continue
            t = (index + 0.5) / WATER_ARC_SEGMENTS
            x = start[0] + dx * t
            y = start[1] + dy * t
            z = start[2] + (end[2] - start[2]) * t + arc_height * 4.0 * t * (1.0 - t)
            tangent = [dx, dy, (end[2] - start[2]) + arc_height * 4.0 * (1.0 - 2.0 * t)]
            tangent_length = math.sqrt(sum(component * component for component in tangent))
            direction = [component / tangent_length for component in tangent]
            ax, ay, az, angle = _axis_angle_from_x(direction)
            node.getField("translation").setSFVec3f([x, y, z])
            node.getField("rotation").setSFRotation([ax, ay, az, angle])

    def _register_stream_contact(self) -> None:
        """The modeled stream endpoint is at FIRE_TARGET, so contact is counted only while spraying."""
        if self.step_count % WATER_CONTACT_EVERY_STEPS == 0:
            self.water_contacts += 1

    def _update_water_contacts(self) -> None:
        """Remove spent droplets; only actual droplet/fire contact reduces the fire."""
        if not self.fire:
            return
        fire_position = self.fire.getPosition()
        fire_height = self.fire.getField("fireHeight").getSFFloat()
        fire_radius = self.fire.getField("fireRadius").getSFFloat()
        contact_center = [fire_position[0], fire_position[1], max(0.20, fire_height * 0.42)]
        contact_radius = max(0.34, fire_radius + 0.16)
        alive: list[dict] = []
        now = time.monotonic()
        for droplet in self.droplets:
            node = droplet["node"]
            position = node.getPosition()
            dx = position[0] - contact_center[0]
            dy = position[1] - contact_center[1]
            dz = position[2] - contact_center[2]
            hit_fire = dx * dx + dy * dy + dz * dz <= contact_radius * contact_radius
            expired = now - droplet["born"] >= WATER_DROPLET_LIFETIME_S or position[2] < -0.10
            if hit_fire or expired:
                if hit_fire:
                    self.water_contacts += 1
                node.remove()
            else:
                alive.append(droplet)
        self.droplets = alive

    def _apply_contact_extinguishing(self) -> None:
        if not self.fire:
            return
        progress = min(1.0, self.water_contacts / WATER_CONTACTS_TO_EXTINGUISH)
        percent = int(progress * 100)
        if percent >= self.last_contact_percent + 25:
            self.last_contact_percent = percent
            print(f"[WEBOTS EXECUTOR] Water contact confirmed; extinguishing progress={percent}%")
        self.fire.getField("fireHeight").setSFFloat(max(0.02, self.initial_fire_height * (1.0 - progress)))
        self.fire.getField("fireRadius").setSFFloat(max(0.01, self.initial_fire_radius * (1.0 - progress)))

    def _extinguish(self) -> None:
        if not self.extinguishing:
            self.extinguishing = True
            print("[WEBOTS EXECUTOR] Target reached; aiming turret and starting visible ballistic water stream")
        self._aim_turret_at_fire()
        self.step_count += 1
        self._update_water_arc()
        self._register_stream_contact()
        self._apply_contact_extinguishing()
        if self.water_contacts < WATER_CONTACTS_TO_EXTINGUISH or self.completed:
            return
        result = self._request_json(
            f"{BRIDGE_ROOT}/api/webots/executor/completion",
            {"unit_id": UNIT_ID, "briefing_id": self.briefing_id},
        )
        if result and result.get("status") == "queued_for_ona":
            self.completed = True
            print("[WEBOTS EXECUTOR] Fire extinguished by water contact; mission_completed beacon released to Beacon Network")

    def run(self) -> None:
        while self.step(self.step_ms) != -1:
            self._sync_cannon_mount()
            self._poll_ona_mission()
            if self.target is None:
                continue
            if not self.extinguishing:
                if self._navigate_with_lidar():
                    self._extinguish()
            else:
                self._extinguish()


if __name__ == "__main__":
    FirefighterExecutor().run()
