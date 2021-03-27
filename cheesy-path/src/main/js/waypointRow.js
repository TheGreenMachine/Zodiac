export default class WaypointRow extends HTMLTableRowElement {
  constructor() {
    super();
    let template = document.getElementById('waypoint-row-template');
    let templateContent = template.content;

    this.appendChild(templateContent.cloneNode(true));
    this.xInput = this.querySelector('.x input');
    this.yInput = this.querySelector('.y input');
    this.headingInput = this.querySelector('.heading input');
    this.enabledInput = this.querySelector('.enabled input');
  }

  get x() {
    return this.xInput.value;
  }

  /**
   * @param {number} val
   */
  set x(val) {
    this.xInput.value = val;
  }

  get y() {
    return this.yInput.value;
  }

  /**
   * @param {number} val
   */
  set y(val) {
    this.yInput.value = val;
  }

  get heading() {
    return this.headingInput.value;
  }

  /**
   * @param {number} val
   */
  set heading(val) {
    this.headingInput.value = val;
  }

  get enabled() {
    return this.enabledInput.checked;
  }

  /**
   * @param {boolean} val
   */
  set enabled(val) {
    this.enabledInput.checked = val;
  }
}
