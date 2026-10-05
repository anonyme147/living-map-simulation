# backend/routes/detections.py

from flask import Blueprint, jsonify, request
import json
import os
from datetime import datetime

# In-memory list of current detections (updated by controller via WebSocket, not directly here)
CURRENT_DETECTIONS = []

detections_bp = Blueprint('detections', __name__, url_prefix='/api/detections')

@detections_bp.get('/')
def get_detections():
    """Return the current list of detections as JSON."""
    return jsonify(CURRENT_DETECTIONS)

@detections_bp.post('/save')
def save_detections():
    """Save the current detections to a timestamped file under data/.
    Expected JSON body: {"detections": [...]} (optional, otherwise uses in-memory list).
    """
    data = request.get_json()
    if data and isinstance(data.get('detections'), list):
        dets = data['detections']
    else:
        dets = CURRENT_DETECTIONS

    if not dets:
        return jsonify({"error": "No detections to save"}), 400

    ts = datetime.now().strftime('%Y%m%d_%H%M%S')
    filename = f"detections_{ts}.json"
    data_dir = os.path.join(os.path.dirname(__file__), '..', '..', 'data')
    os.makedirs(data_dir, exist_ok=True)
    filepath = os.path.abspath(os.path.join(data_dir, filename))
    try:
        with open(filepath, 'w', encoding='utf-8') as fp:
            json.dump({"timestamp": ts, "detections": dets}, fp, indent=2)
    except Exception as e:
        return jsonify({"error": str(e)}), 500

    return jsonify({"success": True, "filename": filename})
