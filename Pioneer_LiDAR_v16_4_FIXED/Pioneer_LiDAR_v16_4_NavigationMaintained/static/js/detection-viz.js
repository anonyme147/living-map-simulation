// detection-viz.js – visualize world detections as colored spheres
import * as THREE from 'three';
import { state } from './state.js';
import { enuToThree } from './utils.js';

// Map of detection_id -> sprite object. These packets are detections, not tracks.
const spriteMap = new Map();
const CLASS_COLORS = {
  person: 0x2196f3,
  fire: 0xff3d00,
  house: 0x9c27b0,
  tree: 0x22aa44,
};

function colorForClass(className) {
  return CLASS_COLORS[String(className).toLowerCase()] ?? 0xffd000;
}

export function updateDetections(detections) {
  // Remove sprites that are no longer present (by detection_id)
  const currentIds = new Set(detections.filter(d => d.localization_valid !== false && Array.isArray(d.position_world_frame)).map(d => d.detection_id ?? d.track_id));
  for (const [id, sprite] of spriteMap.entries()) {
    if (!currentIds.has(id)) {
      if (state.scene && sprite.parent) state.scene.remove(sprite);
      spriteMap.delete(id);
    }
  }

  // Add / update detections
  detections.forEach(det => {
    if (det.localization_valid === false || !Array.isArray(det.position_world_frame)) return;
    const id = det.detection_id ?? det.track_id;
    const { class: cls, position_world_frame } = det;
    let sprite = spriteMap.get(id);
    if (!sprite) {
      const material = new THREE.MeshBasicMaterial({ color: colorForClass(cls) });
      sprite = new THREE.Mesh(new THREE.SphereGeometry(0.18, 20, 14), material);
      sprite.userData.isDetectionSphere = true;
      sprite.userData.detectionClass = cls;
      spriteMap.set(id, sprite);
      if (state.areaGroup) state.areaGroup.add(sprite);
    }
    sprite.material.color.setHex(colorForClass(cls));
    // update position
    const [x, y, z] = position_world_frame;
    // Detection packets use Webots ENU world coordinates. Keep them in the
    // same Three.js frame as the LiDAR cloud and robot: (x, z, -y).
    sprite.position.copy(enuToThree(x, y, z));
  });
}
