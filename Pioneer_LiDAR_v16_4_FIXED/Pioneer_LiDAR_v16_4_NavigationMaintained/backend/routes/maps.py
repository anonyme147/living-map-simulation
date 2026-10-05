from datetime import datetime
import glob
import json
import os
import threading

from flask import Blueprint, Response, jsonify, request

from backend.codecs.lmap import decode_lmap, encode_lmap
from backend.config import MAPS_DIR
from backend.exporters.laz import HAS_LASPY, export_to_laz


maps_bp = Blueprint("maps", __name__, url_prefix="/api/maps")

# Flask is threaded in run.py. Serialize append/read operations so a GET can
# never observe a half-written gzip member.
_SESSION_FILE_LOCK = threading.RLock()


def _map_files():
    files = glob.glob(str(MAPS_DIR / "*.json"))
    files += glob.glob(str(MAPS_DIR / "*.lmap*"))
    return sorted(files, key=os.path.getmtime, reverse=True)


def _safe_path(filename):
    """Resolve a map filename while preventing path traversal."""
    candidate = (MAPS_DIR / filename).resolve()
    maps_root = MAPS_DIR.resolve()

    if candidate.parent != maps_root:
        return None

    return candidate


@maps_bp.get("")
def list_maps():
    """List saved maps with metadata."""
    maps = []

    for filepath in _map_files():
        name = os.path.basename(filepath)
        size_kb = round(os.path.getsize(filepath) / 1024, 1)

        try:
            if name.endswith(".json"):
                with open(filepath, "r", encoding="utf-8") as fp:
                    data = json.load(fp)

                maps.append({
                    "filename": name,
                    "name": data.get("name", name),
                    "timestamp": data.get("timestamp", ""),
                    "point_count": data.get("metadata", {}).get(
                        "total_points", 0
                    ),
                    "size_kb": size_kb,
                })
            else:
                with _SESSION_FILE_LOCK, open(filepath, "rb") as fp:
                    raw = fp.read()

                info = decode_lmap(raw)
                meta = info.get("metadata", {})

                maps.append({
                    "filename": name,
                    "name": meta.get("name", name),
                    "timestamp": meta.get("timestamp", ""),
                    "point_count": meta.get(
                        "total_points",
                        len(info.get("positions", [])) // 3,
                    ),
                    "size_kb": size_kb,
                })

        except Exception:
            maps.append({
                "filename": name,
                "name": name,
                "timestamp": "",
                "point_count": 0,
                "size_kb": size_kb,
            })

    return jsonify(maps)


@maps_bp.post("")
def save_map():
    """Save a binary .lmap.gz upload or a legacy JSON map."""
    content_type = request.content_type or ""

    if content_type in ("application/octet-stream", "application/gzip"):
        raw = request.get_data()
        name_header = request.headers.get("X-Map-Name", "")

        if name_header:
            base_name = os.path.splitext(
                os.path.splitext(os.path.basename(name_header))[0]
            )[0]
        else:
            base_name = f"map_{datetime.now().strftime('%Y%m%d_%H%M%S')}"

        filename = f"{base_name}.lmap.gz"
        filepath = _safe_path(filename)

        if filepath is None:
            return jsonify({"error": "Invalid filename"}), 400

        with open(filepath, "wb") as fp:
            fp.write(raw)

        return jsonify({"success": True, "filename": filename})

    data = request.get_json()
    if not data or "name" not in data:
        return jsonify({"error": "Missing 'name' field"}), 400

    filename = f"map_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
    filepath = _safe_path(filename)

    with open(filepath, "w", encoding="utf-8") as fp:
        json.dump(data, fp)

    return jsonify({"success": True, "filename": filename})


@maps_bp.post("/session/<filename>")
def append_session_snapshot(filename):
    """
    Append one compressed LMAP snapshot as a new gzip member.

    A session file is therefore one .lmap.gz file containing many independent
    snapshots. The decoder selects the newest snapshot per generation, so
    periodic backups replace older snapshots logically without rewriting a
    potentially huge archive on every 60-second save.
    """
    if not filename.endswith(".lmap.gz"):
        return jsonify({"error": "Session filename must end in .lmap.gz"}), 400

    if os.path.basename(filename) != filename:
        return jsonify({"error": "Invalid session filename"}), 400

    raw = request.get_data()
    if not raw:
        return jsonify({"error": "Empty snapshot"}), 400

    if raw[:2] != b"\x1f\x8b":
        return jsonify({"error": "Snapshot must be gzip-compressed"}), 400

    filepath = _safe_path(filename)
    if filepath is None:
        return jsonify({"error": "Invalid filename"}), 400

    # IMPORTANT: do not decode/re-encode a live snapshot here.
    #
    # The browser sends one gzip-compressed LMAP block per generation. The
    # session archive is intentionally a concatenation of those gzip members.
    # Python/browser gzip implementations can differ in how they expose
    # concatenated members, so validating the complete archive on every POST
    # can incorrectly reject an otherwise valid individual snapshot.
    #
    # We only validate the transport envelope here. The complete archive is
    # decoded once, authoritatively, when it is loaded or when Save-As is
    # requested.
    with _SESSION_FILE_LOCK:
        with open(filepath, "ab") as fp:
            fp.write(raw)
            fp.flush()
            os.fsync(fp.fileno())

    return jsonify({
        "success": True,
        "filename": filename,
        "bytes": len(raw),
    })



@maps_bp.delete("/session/<filename>")
def clear_session(filename):
    """Delete a live session backup archive during a map reset."""
    if not filename.endswith(".lmap.gz"):
        return jsonify({"error": "Session filename must end in .lmap.gz"}), 400

    if os.path.basename(filename) != filename:
        return jsonify({"error": "Invalid session filename"}), 400

    filepath = _safe_path(filename)
    if filepath is None:
        return jsonify({"error": "Invalid filename"}), 400

    with _SESSION_FILE_LOCK:
        if filepath.exists():
            filepath.unlink()
            return jsonify({"success": True, "deleted": True})

    return jsonify({"success": True, "deleted": False})

@maps_bp.post("/session/<filename>/save-as")
def save_session_as_map(filename):
    """
    Create a normal user-saved .lmap.gz from the complete session archive.

    The browser only sends the desired name. All historical point segments
    already stored in the session archive are reconstructed server-side, so
    points that have fallen out of the Three.js travelling-area render buffer
    are preserved. The complete primitive navigation path is copied as well.
    """
    if not filename.endswith(".lmap.gz"):
        return jsonify({"error": "Session filename must end in .lmap.gz"}), 400

    if os.path.basename(filename) != filename:
        return jsonify({"error": "Invalid session filename"}), 400

    source = _safe_path(filename)
    if source is None or not source.exists():
        return jsonify({"error": "Session archive not found"}), 404

    data = request.get_json(silent=True) or {}
    name = str(data.get("name", "")).strip()
    if not name:
        name = f"map_{datetime.now().strftime('%Y%m%d_%H%M%S')}"

    # Use the same basename sanitisation rules as normal map saving.
    name = os.path.basename(name)
    if name.endswith(".lmap.gz"):
        base_name = name[:-8]
    elif name.endswith(".lmap"):
        base_name = name[:-5]
    else:
        base_name = name

    if not base_name:
        return jsonify({"error": "Invalid map name"}), 400

    target = _safe_path(f"{base_name}.lmap.gz")
    if target is None:
        return jsonify({"error": "Invalid map filename"}), 400

    try:
        with _SESSION_FILE_LOCK, open(source, "rb") as fp:
            raw = fp.read()

        decoded = decode_lmap(raw)
        source_meta = dict(decoded.get("metadata", {}))
        positions = decoded.get("positions", [])
        trajectory = decoded.get("trajectory", [])

        # The trajectory is intentionally stored as primitive line vertices:
        # consecutive [x,y,z] samples form one THREE.Line path when loaded.
        source_meta.update({
            "name": base_name,
            "timestamp": datetime.now().isoformat(),
            "total_points": len(positions) // 3,
            "total_traj_points": len(trajectory) // 3,
            "trajectory_type": "primitive_lines",
            "trajectory_format": "xyz_vertices_connected_in_order",
            "saved_from_session": filename,
            "session_archive": False,
        })

        # Re-encode as one ordinary LMAP. This makes the saved map independent
        # from the live session archive while preserving every historical point
        # and the complete primitive navigation path.
        import gzip
        lmap_bytes = encode_lmap(source_meta, positions, trajectory)
        compressed = gzip.compress(lmap_bytes)

        with _SESSION_FILE_LOCK, open(target, "wb") as fp:
            fp.write(compressed)
            fp.flush()
            os.fsync(fp.fileno())

        return jsonify({
            "success": True,
            "filename": target.name,
            "point_count": len(positions) // 3,
            "trajectory_points": len(trajectory) // 3,
        })
    except Exception as exc:
        import traceback
        traceback.print_exc()
        return jsonify({"error": f"Save-As failed: {type(exc).__name__}: {exc}"}), 500


@maps_bp.get("/<filename>")
def load_map(filename):
    """Load a stored map by filename."""
    filepath = _safe_path(filename)

    if filepath is None or not filepath.exists():
        return jsonify({"error": "Map not found"}), 404

    if filename.endswith(".lmap") or filename.endswith(".lmap.gz"):
        with _SESSION_FILE_LOCK, open(filepath, "rb") as fp:
            raw = fp.read()

        return Response(
            raw,
            mimetype="application/octet-stream",
        )

    with open(filepath, "r", encoding="utf-8") as fp:
        return jsonify(json.load(fp))


@maps_bp.get("/<filename>/export.laz")
def export_laz(filename):
    """Export a stored LMAP/session archive to compressed LAZ."""
    filepath = _safe_path(filename)

    if filepath is None or not filepath.exists():
        return jsonify({"error": "Map not found"}), 404

    if not HAS_LASPY:
        return jsonify({
            "error": "laspy not installed. Run: pip install laspy[lazrs]"
        }), 501

    try:
        with _SESSION_FILE_LOCK, open(filepath, "rb") as fp:
            raw = fp.read()

        decoded = decode_lmap(raw)
        laz_bytes = export_to_laz(decoded)

        return Response(
            laz_bytes,
            mimetype="application/octet-stream",
            headers={
                "Content-Disposition":
                    f'attachment; filename="{filename}.laz"'
            },
        )
    except Exception as exc:
        return jsonify({"error": str(exc)}), 500


@maps_bp.delete("/<filename>")
def delete_map(filename):
    """Delete a map or session archive."""
    filepath = _safe_path(filename)

    if filepath is not None and filepath.exists():
        filepath.unlink()
        return jsonify({"success": True})

    return jsonify({"error": "Map not found"}), 404
