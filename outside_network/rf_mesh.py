"""Mesh-only implementation of ONA Role 3 CARRY for the RF relay demo.

The normal satellite CARRY implementation remains in ``lora_uplink.py``.
This module is selected only by ``run_rf_mesh_demo.py`` and preserves the
same outward packet shape delivered to the Command Post's uplink queue.

Bidirectional routing
---------------------
``MeshRelayDriver.send()``          — Writer beacon uplink: entry → gateway
``MeshRelayDriver.send_downlink()`` — ONA Role 4 downlink: gateway → entry
``MeshMissionRelay``                — drop-in replacement for MissionRelay that
                                      routes the authorized mission packet
                                      through the mesh instead of sleeping.

Node placement
--------------
Nodes are scattered freely within the building-to-CP bounding box using
independent, wide lateral offsets (±0.005° lat, ±0.006° lon) that are
*uncorrelated* with the corridor progression.  This makes the Dijkstra path
visibly zigzag across the map rather than trace a straight conveyor belt.
The link-radius is widened to guarantee connectivity even with non-collinear
placement.
"""

from __future__ import annotations

import heapq
import json
import logging
import math
import queue
import random
import threading
import time
from dataclasses import dataclass
from typing import Optional

from outside_network.lora_uplink import BaseSatelliteDriver, RetryQueue, SatelliteUplink
from outside_network.mission_relay import MissionRelay


logger = logging.getLogger(__name__)

DEFAULT_MESH_NODE_COUNT = 6
DEFAULT_HOP_DURATION_S = 0.55


def _distance_km(a_lat: float, a_lon: float, b_lat: float, b_lon: float) -> float:
    """Return a short-range great-circle distance without a geo dependency."""
    radius_km = 6_371.0
    d_lat = math.radians(b_lat - a_lat)
    d_lon = math.radians(b_lon - a_lon)
    value = (
        math.sin(d_lat / 2) ** 2
        + math.cos(math.radians(a_lat))
        * math.cos(math.radians(b_lat))
        * math.sin(d_lon / 2) ** 2
    )
    return radius_km * 2 * math.atan2(math.sqrt(value), math.sqrt(1 - value))


@dataclass
class RFMeshNode:
    """One relay in the ONA Role 3 RF mesh."""

    node_id: str
    lat: float
    lon: float
    gateway: bool = False
    state: str = "idle"

    def to_dict(self) -> dict:
        return {
            "node_id": self.node_id,
            "lat": round(self.lat, 7),
            "lon": round(self.lon, 7),
            "gateway": self.gateway,
            "state": self.state,
        }


class RFMeshTopology:
    """Scattered relay topology between the ONA edge and Command Post.

    Node placement uses *wide, independent* lateral offsets so nodes are
    visibly distributed across the map area rather than strung in a line.
    Dijkstra on the undirected graph then finds the true shortest hop path,
    which visibly zigzags between the scattered positions.
    """

    def __init__(
        self,
        nodes: list[RFMeshNode],
        links: list[tuple[str, str, float]],
        entry_node_id: str,
        gateway_node_id: str,
    ):
        self._nodes = {node.node_id: node for node in nodes}
        self._links = links
        self.entry_node_id = entry_node_id
        self.gateway_node_id = gateway_node_id
        self._adjacency: dict[str, list[tuple[str, float]]] = {node.node_id: [] for node in nodes}
        for first, second, distance_km in links:
            self._adjacency[first].append((second, distance_km))
            self._adjacency[second].append((first, distance_km))
        self._lock = threading.RLock()

    @classmethod
    def generate(
        cls,
        *,
        building_lat: float,
        building_lon: float,
        command_post_lat: float,
        command_post_lon: float,
        node_count: int = DEFAULT_MESH_NODE_COUNT,
        seed: Optional[int] = None,
        resilient: bool = False,
        resilience_profile: str = "none",
    ) -> "RFMeshTopology":
        """Create a scattered, always-connected relay topology.

        Each interior node is placed at a random position within the bounding
        box of the building-to-CP corridor, with independent lateral offsets
        that are *wide* (~±0.005° lat / ±0.006° lon, roughly 550m × 400m)
        and uncorrelated with the along-corridor progression.  This ensures
        the Dijkstra path zigzags visibly across the map instead of following
        the straight line between the two endpoints.

        Entry (RF-01) and gateway (RF-{n}) are anchored close to the building
        and Command Post respectively so the relay corridor is well-defined.
        """
        if node_count < 4:
            raise ValueError("RF mesh needs at least four nodes for a meaningful relay path")
        if resilience_profile not in {"none", "single", "dual"}:
            raise ValueError(f"Unknown RF mesh resilience profile {resilience_profile!r}")
        if resilience_profile == "dual" and node_count < 6:
            raise ValueError("Dual-node resilience needs at least six primary relays")
        rng = random.Random(seed)

        route_distance = _distance_km(building_lat, building_lon, command_post_lat, command_post_lon)
        lat_delta = command_post_lat - building_lat
        lon_delta = command_post_lon - building_lon

        # Perpendicular unit vector for alternating lateral zigzag
        perp_len = math.hypot(lat_delta, lon_delta)
        perp_lat = -lon_delta / perp_len if perp_len > 0 else 0.0
        perp_lon = lat_delta / perp_len if perp_len > 0 else 0.0

        nodes: list[RFMeshNode] = []
        for index in range(node_count):
            if index == 0:
                # Entry node: anchored near building with small jitter
                lat = building_lat + rng.uniform(-0.0006, 0.0006)
                lon = building_lon + rng.uniform(-0.0006, 0.0006)
            elif index == node_count - 1:
                # Gateway node: anchored near Command Post with small jitter
                lat = command_post_lat + rng.uniform(-0.0006, 0.0006)
                lon = command_post_lon + rng.uniform(-0.0006, 0.0006)
            else:
                # Interior nodes: alternating lateral offsets (±350-500m)
                # across the corridor axis, producing a clear zigzag path
                p = index / (node_count - 1)
                progression = p + rng.uniform(-0.02, 0.02)
                side = 1.0 if (index % 2 == 1) else -1.0
                lateral_offset = side * rng.uniform(0.0035, 0.0050)
                lat = building_lat + lat_delta * progression + perp_lat * lateral_offset
                lon = building_lon + lon_delta * progression + perp_lon * lateral_offset

            nodes.append(
                RFMeshNode(
                    node_id=f"RF-{index + 1:02d}",
                    lat=lat,
                    lon=lon,
                    gateway=index == node_count - 1,
                )
            )

        # Calibrated link radius allows adjacent relays to connect while preventing
        # long-distance shortcuts that bypass intermediate nodes like RF-05.
        step_distance_km = route_distance / (node_count - 1)
        link_radius_km = step_distance_km * 1.65

        links: list[tuple[str, str, float]] = []
        # Guarantee sequential chain links between consecutive relays
        for index in range(node_count - 1):
            first = nodes[index]
            second = nodes[index + 1]
            distance_km = _distance_km(first.lat, first.lon, second.lat, second.lon)
            links.append((first.node_id, second.node_id, round(distance_km, 4)))

        # Also add any other peer links within the calibrated RF transmission range
        for index, first in enumerate(nodes):
            for second in nodes[index + 2:]:
                distance_km = _distance_km(first.lat, first.lon, second.lat, second.lon)
                if distance_km <= link_radius_km:
                    links.append((first.node_id, second.node_id, round(distance_km, 4)))

        if resilient or resilience_profile == "single":
            # Demo-only backup branch: the normal chain remains the least-cost
            # path until RF-03 fails, then it can use the added backup relay.
            second, third, fourth = nodes[1], nodes[2], nodes[3]
            backup = RFMeshNode(
                node_id=f"RF-{node_count + 1:02d}",
                lat=(second.lat + fourth.lat) / 2 + perp_lat * 0.003,
                lon=(second.lon + fourth.lon) / 2 + perp_lon * 0.003,
            )
            nodes.append(backup)
            primary_cost = _distance_km(second.lat, second.lon, third.lat, third.lon) + _distance_km(third.lat, third.lon, fourth.lat, fourth.lon)
            links.extend([
                (second.node_id, backup.node_id, round(primary_cost * 0.7, 4)),
                (backup.node_id, fourth.node_id, round(primary_cost * 0.7, 4)),
            ])

        if resilience_profile == "dual":
            # Two independent branches preserve connectivity if RF-03 and
            # RF-05 disappear together after the packet reaches RF-02.
            second, third, fourth, fifth, sixth = nodes[1], nodes[2], nodes[3], nodes[4], nodes[5]
            first_backup = RFMeshNode(
                node_id=f"RF-{node_count + 1:02d}",
                lat=(second.lat + fourth.lat) / 2 + perp_lat * 0.003,
                lon=(second.lon + fourth.lon) / 2 + perp_lon * 0.003,
            )
            second_backup = RFMeshNode(
                node_id=f"RF-{node_count + 2:02d}",
                lat=(fourth.lat + sixth.lat) / 2 - perp_lat * 0.003,
                lon=(fourth.lon + sixth.lon) / 2 - perp_lon * 0.003,
            )
            nodes.extend([first_backup, second_backup])
            first_primary_cost = _distance_km(second.lat, second.lon, third.lat, third.lon) + _distance_km(third.lat, third.lon, fourth.lat, fourth.lon)
            second_primary_cost = _distance_km(fourth.lat, fourth.lon, fifth.lat, fifth.lon) + _distance_km(fifth.lat, fifth.lon, sixth.lat, sixth.lon)
            links.extend([
                (second.node_id, first_backup.node_id, round(first_primary_cost * 0.7, 4)),
                (first_backup.node_id, fourth.node_id, round(first_primary_cost * 0.7, 4)),
                (fourth.node_id, second_backup.node_id, round(second_primary_cost * 0.7, 4)),
                (second_backup.node_id, sixth.node_id, round(second_primary_cost * 0.7, 4)),
            ])

        topology = cls(
            nodes=nodes,
            links=links,
            entry_node_id=nodes[0].node_id,
            gateway_node_id=nodes[node_count - 1].node_id,
        )
        # Verify a route exists. Any future placement change that breaks
        # connectivity will raise here rather than failing silently at runtime.
        topology.shortest_path()
        return topology

    def shortest_path(
        self,
        source_id: Optional[str] = None,
        dest_id: Optional[str] = None,
        excluded_node_ids: Optional[set[str]] = None,
    ) -> list[RFMeshNode]:
        """Distance-weighted Dijkstra between any two nodes.

        Defaults to entry → gateway (uplink direction).  Pass
        ``source_id=gateway_node_id, dest_id=entry_node_id`` for the reverse
        (downlink) direction.  The graph is undirected so either direction
        produces the shortest path on the same undirected topology.
        """
        with self._lock:
            source = source_id if source_id is not None else self.entry_node_id
            destination = dest_id if dest_id is not None else self.gateway_node_id
            excluded = excluded_node_ids or set()
            if source in excluded or destination in excluded:
                raise RuntimeError("RF mesh route endpoint is excluded")
            distances = {node_id: math.inf for node_id in self._nodes}
            previous: dict[str, Optional[str]] = {node_id: None for node_id in self._nodes}
            distances[source] = 0.0
            frontier: list[tuple[float, str]] = [(0.0, source)]

            while frontier:
                current_distance, current = heapq.heappop(frontier)
                if current == destination:
                    break
                if current_distance != distances[current]:
                    continue
                if current in excluded or self._nodes[current].state == "failed":
                    continue
                for neighbour, weight in self._adjacency[current]:
                    if neighbour in excluded or self._nodes[neighbour].state == "failed":
                        continue
                    candidate = current_distance + weight
                    if candidate < distances[neighbour]:
                        distances[neighbour] = candidate
                        previous[neighbour] = current
                        heapq.heappush(frontier, (candidate, neighbour))

            if not math.isfinite(distances[destination]):
                raise RuntimeError(
                    f"RF mesh topology has no route from {source!r} to {destination!r}"
                )

            path_ids: list[str] = []
            cursor: Optional[str] = destination
            while cursor is not None:
                path_ids.append(cursor)
                cursor = previous[cursor]
            return [self._nodes[node_id] for node_id in reversed(path_ids)]

    def set_relaying(self, node_id: Optional[str]) -> None:
        with self._lock:
            for node in self._nodes.values():
                if node.state != "failed":
                    node.state = "relaying" if node.node_id == node_id else "idle"

    def fail_node(self, node_id: str) -> None:
        """Make one relay unavailable to future Dijkstra searches."""
        with self._lock:
            if node_id not in self._nodes:
                raise KeyError(f"Unknown RF mesh node {node_id!r}")
            self._nodes[node_id].state = "failed"

    def snapshot(self) -> dict:
        with self._lock:
            return {
                "nodes": [node.to_dict() for node in self._nodes.values()],
                "links": [
                    {"from": first, "to": second, "distance_km": distance_km}
                    for first, second, distance_km in self._links
                ],
                "entry_node_id": self.entry_node_id,
                "gateway_node_id": self.gateway_node_id,
            }


class MeshRelayDriver(BaseSatelliteDriver):
    """New-demo-only Role 3/4 driver that carries one packet hop by hop.

    ``send()``          — uplink (Writer beacon → CP): entry → gateway
    ``send_downlink()`` — downlink (CP → Executor): gateway → entry
    Both directions use the same Dijkstra routing; only source/dest are swapped.
    """

    def __init__(
        self,
        uplink_queue: queue.Queue,
        downlink_queue: queue.Queue,
        topology: RFMeshTopology,
        *,
        hop_duration_s: float = DEFAULT_HOP_DURATION_S,
        failure_after_hop: Optional[int] = None,
        failure_node_id: Optional[str] = None,
        failure_node_ids: Optional[tuple[str, ...]] = None,
        on_node_failure=None,
        on_reroute=None,
        show_downlink_activity: bool = True,
    ):
        self._uplink_queue = uplink_queue
        self._downlink_queue = downlink_queue
        self.topology = topology
        self.hop_duration_s = max(0.1, float(hop_duration_s))
        self.link_disrupted = False
        self.packet_loss_prob = 0.0
        self._last_successful_contact_at = time.monotonic()
        self._active: Optional[dict] = None
        self._lock = threading.RLock()
        self._failure_after_hop = failure_after_hop
        self._failure_node_ids = failure_node_ids or ((failure_node_id,) if failure_node_id else ())
        self._node_failure_injected = False
        self._on_node_failure = on_node_failure
        self._on_reroute = on_reroute
        # Some resilience demonstrations focus only on the in-flight Writer
        # uplink.  Keep their later Role 4 delivery real, but allow its reverse
        # mesh animation to stay hidden so it is not mistaken for ONA → Writer
        # traffic.  Existing mesh demos retain the default visible downlink.
        self._show_downlink_activity = bool(show_downlink_activity)

    def set_disruption(self, duration_s: float = 12.0) -> None:
        """Keep the dashboard's existing optional failure hook compatible."""
        with self._lock:
            self.link_disrupted = True
        logger.warning("[RFMesh] Gateway disruption for %.1fs; RetryQueue buffering remains active", duration_s)

        def restore() -> None:
            time.sleep(duration_s)
            with self._lock:
                self.link_disrupted = False
            logger.info("[RFMesh] Gateway restored; queued mesh packets may resume")

        threading.Thread(target=restore, name="RFMeshRestore", daemon=True).start()

    def arm_next_uplink_node_failure(
        self,
        *,
        failure_after_hop: int = 1,
        failure_node_ids: tuple[str, ...],
    ) -> None:
        """Arm the existing mid-relay reroute for one future uplink packet.

        The normal mesh and existing node-failure demos keep their constructor
        configuration.  A showcase can call this only after an earlier phase
        has completed, without duplicating or changing the Dijkstra reroute.
        """
        if failure_after_hop < 1:
            raise ValueError("failure_after_hop must be at least 1")
        if not failure_node_ids:
            raise ValueError("failure_node_ids must contain at least one relay")
        with self._lock:
            known_node_ids = {node["node_id"] for node in self.topology.snapshot()["nodes"]}
            for node_id in failure_node_ids:
                if node_id not in known_node_ids:
                    raise KeyError(f"Unknown RF mesh node {node_id!r}")
            self._failure_after_hop = int(failure_after_hop)
            self._failure_node_ids = tuple(failure_node_ids)
            self._node_failure_injected = False
        logger.info(
            "[RFMesh] Mid-relay node failure armed for next uplink; after_hop=%s nodes=%s",
            failure_after_hop,
            ", ".join(failure_node_ids),
        )

    def _hop_through(
        self,
        path: list[RFMeshNode],
        packet_id: str,
        direction: str,
        *,
        beacon: Optional[dict] = None,
    ) -> None:
        """Walk ``path`` hop-by-hop, lighting up each relay node in sequence.

        ``direction`` is a human-readable label ("UPLINK" or "DOWNLINK") used
        in log messages only; it does not affect routing behaviour.
        """
        serialized_path = [node.to_dict() for node in path]
        show_activity = direction != "DOWNLINK" or self._show_downlink_activity
        if show_activity:
            with self._lock:
                self._active = {
                    "active": True,
                    "direction": direction,
                    "packet_id": packet_id,
                    "beacon_id": beacon.get("beacon_id") if beacon else None,
                    "event_type": beacon.get("event", {}).get("type") if beacon else None,
                    "source_unit_id": beacon.get("writer_id") if beacon else None,
                    "path": serialized_path,
                    "current_hop_index": 0,
                    "hop_count": max(0, len(path) - 1),
                    "started_at": time.time(),
                }

        logger.info(
            "[RFMesh] %s CARRY started; packet=%s route=%s",
            direction,
            packet_id,
            " -> ".join(node.node_id for node in path),
        )
        index = 0
        while index < len(path) - 1:
            source, destination = path[index], path[index + 1]
            if show_activity:
                self.topology.set_relaying(source.node_id)
                with self._lock:
                    if self._active is not None:
                        self._active["current_hop_index"] = index
                        self._active["current_from"] = source.node_id
                        self._active["current_to"] = destination.node_id
            logger.info(
                "[RFMesh] %s hop %s/%s; packet=%s %s -> %s",
                direction,
                index + 1,
                len(path) - 1,
                packet_id,
                source.node_id,
                destination.node_id,
            )
            time.sleep(self.hop_duration_s)
            # Enabled only by the dedicated node-failure demo.  The packet has
            # completed this hop and therefore reroutes from ``destination``.
            if (
                direction == "UPLINK"
                and not self._node_failure_injected
                and self._failure_after_hop is not None
                and index + 1 == self._failure_after_hop
                and self._failure_node_ids
            ):
                previous_path = [node.to_dict() for node in path]
                for failed_node_id in self._failure_node_ids:
                    self.topology.fail_node(failed_node_id)
                self._node_failure_injected = True
                logger.warning(
                    "[RFMesh] NODE FAILURE: %s offline simultaneously after packet=%s reached %s; recalculating from current relay",
                    ", ".join(self._failure_node_ids),
                    packet_id,
                    destination.node_id,
                )
                if self._on_node_failure is not None:
                    try:
                        self._on_node_failure(
                            packet_id=packet_id,
                            current_node_id=destination.node_id,
                            failed_node_ids=tuple(self._failure_node_ids),
                        )
                    except Exception:
                        logger.exception("[RFMesh] Node-failure observer failed")
                try:
                    path = self.topology.shortest_path(source_id=destination.node_id)
                except RuntimeError:
                    with self._lock:
                        if self._active is not None:
                            self._active.update({
                                "no_route": True,
                                "reroute_from": destination.node_id,
                                "failed_node_ids": list(self._failure_node_ids),
                                "previous_path": previous_path,
                            })
                    logger.error(
                        "[RFMesh] NO VIABLE ROUTE: packet=%s retained by RetryQueue after simultaneous failures %s",
                        packet_id,
                        ", ".join(self._failure_node_ids),
                    )
                    raise
                with self._lock:
                    if self._active is not None:
                        self._active.update({
                            "path": [node.to_dict() for node in path],
                            "current_hop_index": 0,
                            "hop_count": max(0, len(path) - 1),
                            "rerouted": True,
                            "reroute_from": destination.node_id,
                            "failed_node_id": self._failure_node_ids[0],
                            "failed_node_ids": list(self._failure_node_ids),
                            "previous_path": previous_path,
                        })
                logger.warning(
                    "[RFMesh] REROUTE: packet=%s now follows %s",
                    packet_id,
                    " -> ".join(node.node_id for node in path),
                )
                if self._on_reroute is not None:
                    try:
                        self._on_reroute(
                            packet_id=packet_id,
                            current_node_id=destination.node_id,
                            route_node_ids=tuple(node.node_id for node in path),
                        )
                    except Exception:
                        logger.exception("[RFMesh] Reroute observer failed")
                index = 0
                continue
            index += 1

        if show_activity:
            self.topology.set_relaying(path[-1].node_id)
        logger.info(
            "[RFMesh] %s delivered; packet=%s final-node=%s",
            direction,
            packet_id,
            path[-1].node_id,
        )

    def send(self, data: bytes) -> bool:
        """ONA Role 3 CARRY uplink: Writer edge (entry) → Command Post (gateway)."""
        with self._lock:
            if self.link_disrupted:
                logger.warning("[RFMesh] Gateway unavailable; packet remains in RetryQueue")
                return False

        try:
            packet = json.loads(data.decode("utf-8"))
            packet_id = str(packet.get("packet_id", "MESH-PACKET"))
            beacon = packet.get("beacon_data", {})
            path = self.topology.shortest_path()   # entry → gateway (default)
            self._hop_through(path, packet_id, "UPLINK", beacon=beacon)
            self._uplink_queue.put(data)
            with self._lock:
                self._last_successful_contact_at = time.monotonic()
            logger.info(
                "[RFMesh] Gateway delivered; packet=%s node=%s -> Command Post",
                packet_id,
                path[-1].node_id,
            )
            return True
        except Exception:
            logger.exception("[RFMesh] CARRY failed before Command Post delivery")
            return False
        finally:
            self.topology.set_relaying(None)
            with self._lock:
                self._active = None

    def send_downlink(self, data: bytes, packet_id: str) -> bool:
        """ONA Role 4 CARRY downlink: Command Post (gateway) → Executor edge (entry).

        Routes gateway → entry using the same Dijkstra topology in reverse.
        Called only by MeshMissionRelay; does not affect the normal MissionRelay
        used by the three other demos.
        """
        try:
            path = self.topology.shortest_path(
                source_id=self.topology.gateway_node_id,
                dest_id=self.topology.entry_node_id,
            )
            self._hop_through(path, packet_id, "DOWNLINK")
            self._downlink_queue.put(data)
            with self._lock:
                self._last_successful_contact_at = time.monotonic()
            logger.info(
                "[RFMesh] Executor downlink delivered; packet=%s node=%s -> field edge",
                packet_id,
                path[-1].node_id,
            )
            return True
        except Exception:
            logger.exception("[RFMesh] Downlink CARRY failed before Executor delivery")
            return False
        finally:
            self.topology.set_relaying(None)
            with self._lock:
                self._active = None

    def receive(self, timeout_s: float = 1.0) -> Optional[bytes]:
        try:
            return self._downlink_queue.get(timeout=timeout_s)
        except queue.Empty:
            return None

    def is_available(self) -> bool:
        with self._lock:
            return not self.link_disrupted

    def link_quality(self) -> float:
        with self._lock:
            age_s = max(1.0, time.monotonic() - self._last_successful_contact_at)
            return max(0.0, min(1.0, 0.98 - 0.08 * math.log10(age_s)))

    def mesh_snapshot(self) -> dict:
        with self._lock:
            activity = dict(self._active) if self._active else {"active": False}
        return {"enabled": True, **self.topology.snapshot(), "activity": activity}


class MeshSatelliteUplink(SatelliteUplink):
    """SatelliteUplink-compatible facade used only by the mesh scenario."""

    def __init__(
        self,
        driver: MeshRelayDriver,
        retry_queue: RetryQueue,
        *,
        visual_mode: str = "mesh",
        show_initial_blue_link: bool = False,
    ):
        super().__init__(driver, retry_queue)
        self.mesh_driver = driver
        self._visual_mode = visual_mode
        self._show_initial_blue_link = show_initial_blue_link
        self._initial_blue_packet_id: Optional[str] = None

    def mesh_snapshot(self) -> dict:
        return {"mode": self._visual_mode, **self.mesh_driver.mesh_snapshot()}

    def transmission_activity(self) -> dict:
        """Expose only the requested first blue ingress pulse for the mesh demo.

        The actual RF hop animation remains sourced from ``mesh_snapshot`` and
        continues for every packet.  This affects dashboard presentation only;
        it never changes the relay queue or routing behaviour.
        """
        if self._show_initial_blue_link:
            snapshot = self.mesh_driver.mesh_snapshot()
            activity = snapshot.get("activity", {})
            packet_id = activity.get("packet_id")
            if activity.get("active") and activity.get("direction") == "UPLINK":
                if self._initial_blue_packet_id is None:
                    self._initial_blue_packet_id = packet_id
                if packet_id == self._initial_blue_packet_id:
                    return {**activity, "mesh": True}

        # The normal dashboard direct CARRY line stays hidden after that first
        # ingress pulse; the mesh renderer owns all per-hop visuals.
        return {"active": False, "mesh": True}


class MeshMissionRelay(MissionRelay):
    """MissionRelay variant used exclusively by the RF mesh demo.

    Overrides only ``_deliver_after_downlink()`` to route the authorized
    mission packet through the RF mesh (gateway -> entry) hop-by-hop instead
    of sleeping for a fixed duration.  All authorization logic, inbox
    targeting, and dispatch tracking are inherited unchanged from MissionRelay
    so the hard architectural rules (CP -> ONA -> targeted Executor inbox only)
    remain fully enforced.

    The three other demos instantiate MissionRelay directly and are unaffected.
    """

    def __init__(
        self,
        uplink: MeshSatelliteUplink,
        executor_inboxes: dict[str, queue.Queue],
        mesh_driver: MeshRelayDriver,
    ):
        super().__init__(uplink, executor_inboxes)
        self._mesh_driver = mesh_driver

    def _deliver_after_downlink(
        self, target_unit_id: str, payload: dict, relay_id: str
    ) -> None:
        """Route the mission packet through the mesh, then place it in the inbox."""
        try:
            data = json.dumps(payload).encode("utf-8")
            logger.info(
                "[MeshMissionRelay] ONA Role 4 downlink via RF mesh; relay_id=%s target=%s",
                relay_id,
                target_unit_id,
            )
            delivered = self._mesh_driver.send_downlink(data, relay_id)
            if delivered:
                # ONA Role 4 is the only code path that puts a mission in this inbox.
                self.executor_inboxes[target_unit_id].put(payload)
                logger.info(
                    "[MeshMissionRelay] Briefing delivered via mesh; relay_id=%s target=%s",
                    relay_id,
                    target_unit_id,
                )
            else:
                logger.error(
                    "[MeshMissionRelay] Mesh downlink failed; relay_id=%s target=%s — executor inbox not populated",
                    relay_id,
                    target_unit_id,
                )
        finally:
            import threading as _t
            with self._lock:
                dispatch = self._active_dispatches.get(target_unit_id)
                if dispatch and dispatch["relay_id"] == relay_id:
                    del self._active_dispatches[target_unit_id]
