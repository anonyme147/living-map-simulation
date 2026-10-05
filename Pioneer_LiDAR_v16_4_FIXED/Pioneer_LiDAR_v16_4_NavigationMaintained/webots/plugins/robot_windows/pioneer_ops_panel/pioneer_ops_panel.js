import RobotWindow from 'https://cyberbotics.com/wwi/R2025a/RobotWindow.js';

window.robotWindow = new RobotWindow();
const canvas = document.getElementById('scan');
const context = canvas.getContext('2d');

function text(id, value) {
  document.getElementById(id).textContent = value;
}

function drawScan(samples, maxRange) {
  const width = canvas.clientWidth || 320;
  const height = canvas.clientHeight || 194;
  const ratio = window.devicePixelRatio || 1;
  canvas.width = width * ratio;
  canvas.height = height * ratio;
  context.setTransform(ratio, 0, 0, ratio, 0, 0);
  context.clearRect(0, 0, width, height);
  const centerX = width / 2;
  const centerY = height * 0.57;
  const radius = Math.min(width * 0.42, height * 0.76);
  context.strokeStyle = 'rgba(56,189,248,.22)';
  context.lineWidth = 1;
  [0.33, 0.66, 1].forEach(scale => { context.beginPath(); context.arc(centerX, centerY, radius * scale, 0, Math.PI * 2); context.stroke(); });
  context.beginPath(); context.moveTo(centerX, centerY - radius); context.lineTo(centerX, centerY + radius); context.moveTo(centerX - radius, centerY); context.lineTo(centerX + radius, centerY); context.stroke();
  context.fillStyle = '#38bdf8'; context.beginPath(); context.arc(centerX, centerY, 4, 0, Math.PI * 2); context.fill();
  if (!samples.length) return;
  context.fillStyle = '#4ade80';
  samples.forEach((distance, index) => {
    const angle = (index / samples.length) * Math.PI * 2 - Math.PI / 2;
    const normalized = Math.max(0, Math.min(1, distance / maxRange));
    const x = centerX + Math.cos(angle) * radius * normalized;
    const y = centerY + Math.sin(angle) * radius * normalized;
    context.fillRect(x - 1.5, y - 1.5, 3, 3);
  });
}

function renderBoxes(boxes) {
  const layer = document.getElementById('boxes');
  layer.replaceChildren();
  boxes.forEach(box => {
    if (!box.center || !box.size) return;
    const node = document.createElement('div');
    const isFire = box.label === 'FIRE';
    node.className = 'box' + (isFire ? ' fire' : '');
    const left = 100 * (box.center[0] - box.size[0] / 2) / 640;
    const top = 100 * (box.center[1] - box.size[1] / 2) / 480;
    node.style.left = left + '%'; node.style.top = top + '%';
    node.style.width = 100 * box.size[0] / 640 + '%'; node.style.height = 100 * box.size[1] / 480 + '%';
    const tag = document.createElement('span'); tag.className = 'tag';
    tag.textContent = `${box.label} · ${Number(box.distance_m || 0).toFixed(2)} m`;
    node.appendChild(tag); layer.appendChild(node);
  });
}

window.robotWindow.receive = message => {
  let data;
  try { data = JSON.parse(message); } catch (_) { return; }
  if (data.type !== 'operator_panel') return;
  const lidar = data.lidar || {};
  drawScan(lidar.samples || [], lidar.max_range_m || 12);
  text('nearest', lidar.nearest_m == null ? 'CLEAR' : `${lidar.nearest_m.toFixed(2)} m`);
  text('forward', Number.isFinite(lidar.front_clearance_m) ? `${lidar.front_clearance_m.toFixed(2)} m` : 'CLEAR');
  text('points', String(lidar.valid_points || 0)); text('nav', data.navigation || 'SEARCH');
  if (data.camera) { const image = document.getElementById('camera'); image.src = data.camera; image.style.display = 'block'; document.getElementById('empty').style.display = 'none'; }
  renderBoxes(data.boxes || []);
  const detections = data.detections || [];
  document.getElementById('events').innerHTML = detections.length ? detections.map(item => `<div class="event ${item.label === 'FIRE' ? 'fire' : ''}">${item.label} CONFIRMED · ${Number(item.distance_m || 0).toFixed(2)} m</div>`).join('') : 'No target confirmed yet.';
  text('state', `LIVE · ${Number(data.time || 0).toFixed(1)} s`);
};
