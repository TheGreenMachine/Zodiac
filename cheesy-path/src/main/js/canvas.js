import { Translation2d } from './geometry'

function svg(tagName, attrs) {
  const svgNs = "http://www.w3.org/2000/svg";
  let element = document.createElementNS(svgNs, tagName);
  if (attrs && typeof attrs === 'object') {
      for (const [key, value] of Object.entries(attrs)) {
          element.setAttribute(key, value);
      }
  }
  return element;
}

const WAYPOINT_FILL = "#2CFF2C";
const pi = Math.PI;

export default class Canvas {
  constructor(canvases, config) {
    this.config = config;
    const { width, height } = config;
    this.width = width;
    this.height = height;

    let field = canvases.querySelector('#field');
    let background = canvases.querySelector('#background');
    this.interactive = canvases.querySelector('#interactive');
    const widthString = `${width / 1.5}px`;
    const heightString = `${height / 1.5}px`;

    field.style.width = widthString;
    field.style.height = heightString;
    background.style.width = widthString;
    background.style.height = heightString;
    this.interactive.style.width = widthString;
    this.interactive.style.height = heightString;
    canvases.style.width = widthString;
    canvases.style.height = heightString;

    this.ctx = field.getContext('2d');
    this.ctx.canvas.width = width;
    this.ctx.canvas.height = height;
    this.ctx.clearRect(0, 0, width, height);
    this.ctx.fillStyle = "#FF0000";

    this.ctxBackground = background.getContext('2d');
    this.ctxBackground.canvas.width = width;
    this.ctxBackground.canvas.height = height;
    this.ctxBackground.clearRect(0, 0, width, height);

    this.interactive.setAttribute("width", width);
    this.interactive.setAttribute("height", height);
    this.interactive.setAttribute("viewBox", `0 0 ${width} ${height}`);
    // interactive.addEventListener('click', onCanvasClick);

    this.robotRadius = Math.sqrt(Math.pow(config.robotWidth, 2) + Math.pow(config.robotHeight, 2)) / 2;
    this.robotTan = Math.atan2(config.robotHeight, config.robotWidth);

    this.setFieldImage('6_field1');
  }

  setFieldImage(name) {
    let image = new Image();
    this.backgroundImage = image;
    image.src = `/resources/img/${name}.jpg`;
    image.onload = () => {
      this.ctxBackground.drawImage(image, 0, 0, this.width, this.height);
      update(false);
    };
  }

  clearSplines() {
    this.ctx.clearRect(0, 0, this.width, this.height);
    this.ctx.fillStyle = "#FF0000";
  }

  clear() {
    this.clearSplines();

    this.ctxBackground.clearRect(0, 0, this.width, this.height);
    this.ctxBackground.fillStyle = "#FF0000";
    this.ctxBackground.drawImage(this.backgroundImage, 0, 0, this.width, this.height);

    while (this.interactive.lastChild) {
      this.interactive.removeChild(this.interactive.lastChild);
    }
  }

  update(waypoints) {
    this.clear();
    this.drawWaypoints(waypoints);
  }

  drawWaypoints(waypoints) {
    waypoints.forEach((waypoint, i) => {
          this.drawWaypoint(waypoint, i);
          this.drawRobot(waypoint);
      });
  }

  drawWaypoint(waypoint, index) {
    let point = svg('circle', {
      fill: WAYPOINT_FILL,
      cx: waypoint.translation.drawX,
      cy: waypoint.translation.drawY,
      r: this.config.waypointRadius,
      'data-index': index,
    });

    // point.addEventListener('mousedown', handleWaypointDragStart);
    // point.addEventListener('click', handleWaypointClick);

    this.interactive.appendChild(point);
  }

  drawRobot(position) {
    const h = position.rotation.getRadians();
    const r = this.robotRadius;
    const t = this.robotTan;
    let angles = [h + (pi / 2) + t, h - (pi / 2) + t, h + (pi / 2) - t, h - (pi / 2) - t];

    let points = [];

    angles.forEach((angle) => {
        let point = new Translation2d(position.translation.x + (r * Math.cos(angle)),
            position.translation.y + (r * Math.sin(angle)));
        points.push(point);
        point.draw(this.ctx, Math.abs(angle - h) < pi / 2 ? "#00AAFF" : "#0066FF", this.config.splineWidth);
    });
  }
}
