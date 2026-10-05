import { state } from './state.js';
import { initScene, startRenderLoop, handleResize } from './scene.js';
import { initPointCloud } from './point-cloud.js';
import { initRobotViz } from './robot-viz.js';
import { initUI } from './ui-controller.js';
import { initMapManager } from './map-manager.js';
import { connect } from './ws-client.js';
import { initTravellingArea } from './travelling-area.js';
import { loadRuntimeConfig, runtimeConfig } from './config.js';
import { initNavigation } from './navigation.js';

async function main() {
  try {
    state.appConfig = await loadRuntimeConfig();
    console.log('[CONFIG] startup parameters loaded', state.appConfig);
  } catch (e) {
    console.warn('[CONFIG] failed to load startup parameters; using built-in safe defaults', e);
    console.error('[CONFIG] /api/config failed; navigation is using circle(robot_start, 15m) fallback.');
  }
  console.log('[AREA DEBUG] Pioneer viewer startup');
  initScene();
  initPointCloud();
  initRobotViz();

  initUI();
  initTravellingArea();
  initMapManager();
  initNavigation();

  console.log('[AREA DEBUG] scene graph after init', {
    areaGroup: state.areaGroup,
    pointCloudParent: state.pointCloud?.parent?.name || state.pointCloud?.parent?.type,
    robotParent: state.robotGroup?.parent?.name || state.robotGroup?.parent?.type,
    trajectoryParent: state.trajectory?.parent?.name || state.trajectory?.parent?.type
  });

  connect();

  console.log('[AREA DEBUG] final scene graph', {
    areaGroup: state.areaGroup,
    children: state.areaGroup?.children.map(c => c.name || c.type),
    pointCloudParent: state.pointCloud?.parent?.name,
    robotParent: state.robotGroup?.parent?.name,
    trajectoryParent: state.trajectory?.parent?.name
  });

  startRenderLoop(fps => {
    state.hud.fps.textContent = fps;
  });

  window.addEventListener('resize', handleResize);
}

main().catch(err => console.error('[CONFIG] viewer startup failed', err));
