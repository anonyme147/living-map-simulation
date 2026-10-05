"""Single source of truth for simulations exposed by the demo launchpad."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class DemoDefinition:
    """One independently runnable simulation and its dashboard destination."""

    demo_id: str
    title: str
    description: str
    script: str
    dashboard_port: int
    launch_args: tuple[str, ...]
    mode: str

    @property
    def dashboard_url(self) -> str:
        return f"http://127.0.0.1:{self.dashboard_port}"

    def to_dict(self) -> dict:
        return {
            "id": self.demo_id,
            "title": self.title,
            "description": self.description,
            "dashboard_url": self.dashboard_url,
            "mode": self.mode,
        }


# Add future simulations here. The menu discovers entries from this collection
# automatically; no HTML or route changes are required for a new card.
DEMO_REGISTRY: tuple[DemoDefinition, ...] = (
    DemoDefinition(
        demo_id="mission",
        title="Live Mission Simulation",
        description="Normal Writer-to-ONA-to-Command-Post mission workflow with operator-selected Executor dispatch.",
        script="simulation/run_demo.py",
        dashboard_port=5000,
        launch_args=("--once", "--serve"),
        mode="Nominal operation",
    ),
    DemoDefinition(
        demo_id="mission_webotsim",
        title="Live Mission Simulation + Webotsim",
        description="Normal live mission with an optional, independent Pioneer LiDAR Webots visual companion.",
        script="simulation/run_demo.py",
        dashboard_port=5011,
        launch_args=("--once", "--serve", "--dashboard-port", "5011", "--webots-visualization"),
        mode="Nominal operation + physical robot visualization",
    ),
    DemoDefinition(
        demo_id="webots_fire_response",
        title="Webots Fire Response Chain",
        description="Pioneer detects fire, ONA relays the authorized mission to a LiDAR TurtleBot firefighter, then the response unit extinguishes the fire and publishes mission completion.",
        script="simulation/run_webots_fire_response_demo.py",
        dashboard_port=5012,
        launch_args=(),
        mode="Physical Writer-to-Executor response chain",
    ),
    DemoDefinition(
        demo_id="satellite_mesh_failover",
        title="Satellite Failure → RF Mesh Failover",
        description="Normal satellite CARRY fails mid-mission; ONA automatically reroutes subsequent packets through the RF mesh.",
        script="simulation/run_satellite_mesh_failover_demo.py",
        dashboard_port=5004,
        launch_args=(),
        mode="Automatic continuity failover",
    ),
    DemoDefinition(
        demo_id="mesh_dual_node_failure",
        title="RF Mesh Dual-Node Failure",
        description="Two relays fail simultaneously mid-transmission; the in-flight packet reroutes around both failures.",
        script="simulation/run_mesh_dual_node_failure_demo.py",
        dashboard_port=5006,
        launch_args=(),
        mode="Dual-failure mesh resilience",
    ),
    DemoDefinition(
        demo_id="zombie_link_heartbeat_loss",
        title="Zombie Link — Heartbeat Loss",
        description="Beacon traffic continues normally while ONA↔Command Post keepalives stop, exposing a degraded trust state.",
        script="simulation/run_heartbeat_loss_demo.py",
        dashboard_port=5007,
        launch_args=(),
        mode="Trust degradation without data loss",
    ),
    DemoDefinition(
        demo_id="satellite_no_failover",
        title="Baseline — Satellite Loss Without Mesh",
        description="Satellite fails after two arrivals; later beacons remain queued because no RF mesh continuity path is applied.",
        script="simulation/run_satellite_no_failover_demo.py",
        dashboard_port=5008,
        launch_args=(),
        mode="Comparison baseline",
    ),
    DemoDefinition(
        demo_id="zombie_link_no_detection",
        title="Baseline — Zombie Link Without Detection",
        description="Keepalives disappear while beacon traffic continues, but operators receive no trust-degradation warning.",
        script="simulation/run_zombie_link_no_detection_demo.py",
        dashboard_port=5009,
        launch_args=(),
        mode="Comparison baseline",
    ),
    DemoDefinition(
        demo_id="signal_integrity_showcase",
        title="Signal Integrity Showcase",
        description="One continuous mission proving satellite failover, heartbeat recovery, and relay rerouting with zero lost or corrupted beacon payloads.",
        script="simulation/run_signal_integrity_showcase_demo.py",
        dashboard_port=5010,
        launch_args=(),
        mode="Flagship combined resilience proof",
    ),
)


def get_demo(demo_id: str) -> DemoDefinition | None:
    """Look up a menu entry without exposing implementation details to callers."""
    return next((demo for demo in DEMO_REGISTRY if demo.demo_id == demo_id), None)
