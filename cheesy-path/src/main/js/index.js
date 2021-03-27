import { Translation2d, Rotation2d, Pose2d } from './geometry'
import Canvas from './canvas'
import WaypointRow from './waypointRow'

window.customElements.define('waypoint-row', WaypointRow, { extends: 'tr' });

export const config = {
  fieldWidth: 360, // inches
  fieldHeight: 180, // inches

  xOffset: 0, // inches
  yOffset: 0, // inches

  width: 980, // pixels
  height: 490, // pixels

  robotWidth: 34, // inches
  robotHeight: 34, // inches

  waypointRadius: 7, // px
  splineWidth: 3, // px
}

const kEps = 1E-9;
const pi = Math.PI;

const el = {
  titleInput: document.getElementById("title"),
  isReversedCheckbox: document.getElementById('isReversed'),
  waypointsDialog: document.getElementById('waypointsDialog'),
  waypointsOutput: document.getElementById('waypointsOutput'),
  clipboardToast: document.getElementById('clipboardToast'),
};

let canvas;
let fileHandling;
let state = {
  waypoints: [
    new Pose2d(
      new Translation2d(0, 0),
      Rotation2d.fromDegrees(0)
    ),
    new Pose2d(
      new Translation2d(50, 50),
      Rotation2d.fromDegrees(0)
    ),
  ],
}

function init() {
  console.log('Initializing cheesy-path...');
  canvas = new Canvas(document.querySelector('#canvases'), config);

  document.addEventListener('keydown', (e) => {
    if (e.code === 'KeyS' && (e.ctrlKey || e.metaKey)) {
      e.preventDefault();
      saveFile();
    }
  })

  canvas.update(state.waypoints);
  rebind();
}

document.addEventListener('DOMContentLoaded', init);
