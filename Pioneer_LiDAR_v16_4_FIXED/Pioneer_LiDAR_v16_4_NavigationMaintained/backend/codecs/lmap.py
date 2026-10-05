"""Binary .lmap codec and session-archive helpers."""

import gzip
import json
import struct


from backend.config import LMAP_MAGIC


_HEADER_SIZE = 25
_LEGACY_LMAP_MAGIC = b"LAPM"  # v9.3 browser encoder wrote reversed bytes


def _decode_one(data: bytes, offset: int = 0):
    if offset + _HEADER_SIZE > len(data):
        raise ValueError("Truncated LMAP header")

    magic = data[offset:offset + 4]
    if magic not in (LMAP_MAGIC, _LEGACY_LMAP_MAGIC):
        raise ValueError("Invalid LMAP file (bad magic)")

    version, flags, point_count, traj_count, meta_len, reserved = struct.unpack(
        "<BIIIII",
        data[offset + 4:offset + _HEADER_SIZE],
    )

    if version != 1:
        raise ValueError(f"Unsupported LMAP version {version}")

    offset += _HEADER_SIZE

    meta_end = offset + meta_len
    if meta_end > len(data):
        raise ValueError("Truncated LMAP metadata")

    metadata = json.loads(data[offset:meta_end].decode("utf-8"))
    offset = meta_end

    point_values = point_count * 3
    point_bytes = point_values * 4
    point_end = offset + point_bytes

    if point_end > len(data):
        raise ValueError("Truncated LMAP point data")

    positions = list(
        struct.unpack(
            f"<{point_values}f",
            data[offset:point_end],
        )
    )
    offset = point_end

    traj_values = traj_count * 3
    traj_bytes = traj_values * 4
    traj_end = offset + traj_bytes

    if traj_end > len(data):
        raise ValueError("Truncated LMAP trajectory data")

    trajectory = list(
        struct.unpack(
            f"<{traj_values}f",
            data[offset:traj_end],
        )
    )

    return {
        "metadata": metadata,
        "positions": positions,
        "trajectory": trajectory,
        "next_offset": traj_end,
    }


def _decompress_if_needed(data: bytes) -> bytes:
    if data[:2] == b"\x1f\x8b":
        # gzip.decompress supports concatenated gzip members, which is exactly
        # how session snapshots are appended to one .lmap.gz file.
        return gzip.decompress(data)
    return data


def decode_lmap(data: bytes) -> dict:
    """
    Decode a normal LMAP or a session archive made from multiple LMAP blocks.

    For session archives, the newest snapshot for each generation is used.
    The newest trajectory is global and is therefore used once.
    """
    data = _decompress_if_needed(data)

    first = _decode_one(data, 0)

    if first["next_offset"] == len(data):
        return {
            "metadata": first["metadata"],
            "positions": first["positions"],
            "trajectory": first["trajectory"],
        }

    chunks = [first]
    offset = first["next_offset"]

    while offset < len(data):
        chunk = _decode_one(data, offset)
        chunks.append(chunk)
        offset = chunk["next_offset"]

    if not any(c["metadata"].get("session_id") for c in chunks):
        raise ValueError("Unexpected concatenated LMAP data")

    latest_by_generation = {}

    for chunk in chunks:
        meta = chunk["metadata"]
        generation = int(meta.get("generation", 0))
        old = latest_by_generation.get(generation)

        if old is None or int(meta.get("snapshot_id", 0)) > int(
            old["metadata"].get("snapshot_id", 0)
        ):
            latest_by_generation[generation] = chunk

    ordered = [
        chunk
        for _, chunk in sorted(
            latest_by_generation.items(),
            key=lambda item: item[0],
        )
    ]

    positions = []
    for chunk in ordered:
        positions.extend(chunk["positions"])

    newest = max(
        chunks,
        key=lambda c: int(c["metadata"].get("snapshot_id", 0)),
    )

    metadata = dict(newest["metadata"])
    metadata["name"] = metadata.get("session_id", metadata.get("name", "session"))
    metadata["total_points"] = len(positions) // 3
    metadata["total_traj_points"] = len(newest["trajectory"]) // 3
    metadata["session_archive"] = True
    metadata["generations"] = len(ordered)

    return {
        "metadata": metadata,
        "positions": positions,
        "trajectory": newest["trajectory"],
    }


def encode_lmap(metadata: dict, positions: list, trajectory: list) -> bytes:
    """Encode metadata + points + trajectory into raw version-1 LMAP bytes."""
    meta_bytes = json.dumps(metadata, separators=(",", ":")).encode("utf-8")
    point_count = len(positions) // 3
    traj_count = len(trajectory) // 3

    header = struct.pack(
        "<4sBIIIII",
        LMAP_MAGIC,
        1,
        0,
        point_count,
        traj_count,
        len(meta_bytes),
        0,
    )

    pos_bytes = struct.pack(f"<{len(positions)}f", *positions)
    traj_bytes = struct.pack(f"<{len(trajectory)}f", *trajectory)

    return header + meta_bytes + pos_bytes + traj_bytes
