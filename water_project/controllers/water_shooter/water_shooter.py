"""
Water-shooting robot supervisor controller (Webots R2025a).

The robot is a fixed-base, two-axis (yaw/pitch) turret with a nozzle whose
local +X axis is the firing direction. At every simulation step this
controller:
  1. drives the yaw motor with a sine wave (sweeping motion);
  2. reads the nozzle pose and computes the firing direction;
  3. spawns a small water droplet (Solid sphere) just past the nozzle tip
     and gives it a velocity along the firing direction;
  4. deletes droplets whose AGE exceeds LIFETIME (hard cap, e.g. 10 s);
  5. deletes droplets that have settled in the pool, so the live node count
     stays bounded (a 10 s-old jet would otherwise mean >1000 solids).

CONTINUOUS JET
--------------
Droplets are spawned every physics step, so the spacing between them is

    spacing = JET_SPEED * basicTimeStep * SPAWN_INTERVAL

For a visually continuous stream, keep spacing <= droplet diameter
(2 * DROPLET_R). With the defaults below: 7.0 m/s * 0.008 s = 0.056 m,
droplet diameter 0.05 m -> near-continuous.

NOTE: immersionProperties is a field of Solid, NOT of Physics. Putting it
inside Physics triggers:
    "error: Skipped unknown 'immersionProperties' field in Physics node."
"""

from controller import Supervisor
import math

# ----------------------------- Tuning parameters -----------------------------
JET_SPEED = 7.0        # m/s, initial droplet speed
PITCH_ANGLE = 0.35     # rad, elevation angle; the nozzle is tilted UP by
                       # commanding a NEGATIVE pitch motor position
YAW_SWEEP = 0.25       # rad, amplitude of the yaw sweep
YAW_PERIOD = 6.0       # s, period of the yaw sweep
DROPLET_R = 0.025      # m, droplet radius (visual thickness of the jet)
NOZZLE_LEN = 0.25      # m, spawn offset past the nozzle origin
                       # (must be >= the barrel length defined in the world)
SPAWN_INTERVAL = 1     # spawn every N physics steps (1 = densest jet)
LIFETIME = 10.0        # s, HARD maximum age of a droplet

# Pool settling: droplets that fall below the water surface and slow down
# are removed after SETTLE_TIME. This bounds the number of live nodes
# (flight + pooling time is ~2 s instead of the full 10 s LIFETIME).
# Set REMOVE_SETTLED = False to let droplets float in the pool until
# LIFETIME expires (faithful 10 s lifetime, but heavy: ~1250 live solids).
REMOVE_SETTLED = True
SETTLE_TIME = 0.8      # s, how long a slow droplet under water may rest
SETTLE_SPEED = 0.5     # m/s, below this a submerged droplet counts as rested
WATER_SURFACE_Z = 0.3  # m, top of the Fluid box (translation z 0.15 + height 0.3/2)

DROPLET_TEMPLATE = """Solid {{
  name "droplet_{uid}"
  translation {tx:.4f} {ty:.4f} {tz:.4f}
  children [
    Shape {{
      appearance PBRAppearance {{
        baseColor 0.3 0.6 1.0
        transparency 0.25
        roughness 0.1
        metalness 0.0
      }}
      geometry Sphere {{
        radius {r}
        subdivision 2
      }}
    }}
  ]
  boundingObject Sphere {{
    radius {r}
    subdivision 2
  }}
  physics Physics {{
    density 1000
  }}
  immersionProperties [
    ImmersionProperties {{
      fluidName "water"
      dragForceCoefficients 0.5 0.5 0.5
      viscousResistanceForceCoefficient 0.5
    }}
  ]
}}"""


class WaterShooter(Supervisor):
    def __init__(self):
        super().__init__()
        self.timestep = int(self.getBasicTimeStep())
        self.yaw_motor = self.getDevice("yaw_motor")
        self.pitch_motor = self.getDevice("pitch_motor")
        self.nozzle = self.getFromDef("NOZZLE")
        self.root_children = self.getRoot().getField("children")

        if self.yaw_motor is None or self.pitch_motor is None:
            print("ERROR: 'yaw_motor'/'pitch_motor' not found.")
        if self.nozzle is None:
            print("ERROR: DEF NOZZLE not found in the world.")

        # Fixed elevation: negative position tilts the nozzle upward.
        self.pitch_motor.setPosition(-PITCH_ANGLE)

        self.droplets = []   # list of dicts: node, birth, settle
        self.sim_time = 0.0
        self.spawn_clock = 0
        self.uid = 0

    def fire_droplet(self):
        """Spawn one droplet just past the nozzle tip, along the barrel axis."""
        pos = self.nozzle.getPosition()      # world position (m)
        R = self.nozzle.getOrientation()     # 3x3 row-major rotation matrix
        direction = [R[0], R[3], R[6]]       # nozzle local +X in world coords

        offset = NOZZLE_LEN + 2.0 * DROPLET_R
        start = [pos[i] + direction[i] * offset for i in range(3)]

        self.uid += 1
        self.root_children.importMFNodeFromString(
            -1,
            DROPLET_TEMPLATE.format(uid=self.uid, tx=start[0], ty=start[1],
                                    tz=start[2], r=DROPLET_R),
        )
        node = self.root_children.getMFNode(-1)
        node.setVelocity([JET_SPEED * direction[0],
                          JET_SPEED * direction[1],
                          JET_SPEED * direction[2],
                          0, 0, 0])
        self.droplets.append({"node": node, "birth": self.sim_time,
                              "settle": 0.0})

    def remove_expired(self):
        """Remove droplets that are too old or have settled in the pool."""
        alive = []
        for d in self.droplets:
            node, birth, settle = d["node"], d["birth"], d["settle"]
            remove = self.sim_time - birth > LIFETIME
            if not remove and REMOVE_SETTLED:
                vel = node.getVelocity()
                speed = math.sqrt(vel[0]**2 + vel[1]**2 + vel[2]**2)
                pos = node.getPosition()
                if pos[2] < WATER_SURFACE_Z and speed < SETTLE_SPEED:
                    settle += self.timestep / 1000.0
                    d["settle"] = settle
                    remove = settle > SETTLE_TIME
                else:
                    d["settle"] = 0.0
            if remove:
                node.remove()
            else:
                alive.append(d)
        self.droplets = alive

    def run(self):
        while self.step(self.timestep) != -1:
            self.sim_time += self.timestep / 1000.0

            yaw = YAW_SWEEP * math.sin(2.0 * math.pi * self.sim_time / YAW_PERIOD)
            self.yaw_motor.setPosition(yaw)

            self.spawn_clock += 1
            if self.spawn_clock >= SPAWN_INTERVAL:
                self.spawn_clock = 0
                self.fire_droplet()

            self.remove_expired()


if __name__ == "__main__":
    WaterShooter().run()
