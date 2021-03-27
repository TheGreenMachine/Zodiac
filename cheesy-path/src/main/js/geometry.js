import { config } from './index'

function d2r(d) {
  return d * (Math.PI / 180);
}

function r2d(r) {
  return r * (180 / Math.PI);
}

export class Translation2d {
  constructor(x, y) {
    this.x = x;
    this.y = y;
  }

  norm() {
    return Math.hypot(this.x, this.y);
  }

  norm2() {
    return this.x * this.x + this.y * this.y;
  }

  translateBy(other) {
    return new Translation2d(this.x + other.x, this.y + other.y);
  }

  rotateBy(rotation) {
    return new Translation2d(this.x * rotation.cos - this.y * rotation.sin, this.x * rotation.sin + this.y * rotation.cos);
  }

  direction() {
    return new Rotation2d(this.x, this.y, true);
  }

  inverse() {
    return new Translation2d(-this.x, -this.y);
  }

  interpolate(other, x) {
    if (x <= 0) {
      return new Translation2d(this.x, this.y);
    } else if (x >= 1) {
      return new Translation2d(other.x, other.y);
    }
    return this.extrapolate(other, x);
  }

  extrapolate(other, x) {
    return new Translation2d(x * (other.x - this.x) + this.x, x * (other.y - this.y) + this.y);
  }

  scale(s) {
    return new Translation2d(this.x * s, this.y * s);
  }

  static dot(a, b) {
    return a.x * b.x + a.y * b.y;
  }

  static getAngle(a, b) {
    let cos_angle = this.dot(a, b) / (a.norm() * b.norm());
    if (Double.isNaN(cos_angle)) {
      return new Rotation2d(1, 0, false);
    }

    return Rotation2d.fromRadians(Math.acos(Math.min(1.0, Math.max(cos_angle, -1.0))));
  }

  static cross(a, b) {
    return a.x * b.y - a.y * b.x;
  }

  distance(other) {
    return this.inverse().translateBy(other).norm();
  }

  draw(ctx, color, radius) {
    color = color || "#2CFF2C";
    ctx.beginPath();
    ctx.arc(this.drawX, this.drawY, radius, 0, 2 * Math.PI, false);
    ctx.fillStyle = color;
    ctx.strokeStyle = color;
    ctx.fill();
    ctx.lineWidth = 0;
    ctx.stroke();
  }

  get drawX() {
    return (this.x + config.xOffset) * (config.width / config.fieldWidth);
  }

  get drawY() {
    return config.height - (this.y + config.yOffset) * (config.height / config.fieldHeight);
  }
}

export class Rotation2d {
  constructor(x, y, normalize) {
    this.cos = x;
    this.sin = y;
    this.normalize = normalize;
    if (normalize) {
      this.normalizeFunc();
    }
  }

  static fromRadians(angle_radians) {
    return new Rotation2d(Math.cos(angle_radians), Math.sin(angle_radians), false);
  }

  static fromDegrees(angle_degrees) {
    return this.fromRadians(d2r(angle_degrees));
  }

  normalizeFunc() {
    let magnitude = Math.hypot(this.cos, this.sin);
    if (magnitude > kEps) {
      this.cos /= magnitude;
      this.sin /= magnitude;
    } else {
      this.sin = 0;
      this.cos = 1;
    }
  }

  tan() {
    if (Math.abs(this.cos) < kEps) {
      if (this.sin >= 0.0) {
        return Number.POSITIVE_INFINITY;
      } else {
        return Number.NEGATIVE_INFINITY;
      }
    }
    return this.sin / this.cos;
  }

  getRadians() {
    return Math.atan2(this.sin, this.cos);
  }

  getDegrees() {
    return r2d(this.getRadians());
  }

  rotateBy(other) {
    return new Rotation2d(this.cos * other.cos - this.sin * other.sin,
      this.cos * other.sin + this.sin * other.cos, true);
  }

  normal() {
    return new Rotation2d(-this.sin, this.cos, false);
  }

  inverse() {
    return new Rotation2d(this.cos, -this.sin, false);
  }

  interpolate(other, x) {
    if (x <= 0) {
      return new Rotation2d(this.cos, this.sin, this.normalize);
    } else if (x >= 1) {
      return new Rotation2d(other.cos, other.sin, other.normalize);
    }
    let angle_diff = this.inverse().rotateBy(other).getRadians();
    return this.rotateBy(Rotation2d.fromRadians(angle_diff * x));
  }

  distance(other) {
    return this.inverse().rotateBy(other).getRadians();
  }
}

export class Pose2d {
  constructor(translation, rotation, comment) {
    this.translation = translation;
    this.rotation = rotation;
    this.comment = comment || "";
  }

  static exp(delta) {
    let sin_theta = Math.sin(delta.dtheta);
    let cos_theta = Math.cos(delta.dtheta);
    let s, c;

    if (Math.abs(delta.dtheta) < kEps) {
      s = 1.0 - 1.0 / 6.0 * delta.dtheta * delta.dtheta;
      c = .5 * delta.dtheta;
    } else {
      s = sin_theta / delta.dtheta;
      c = (1.0 - cos_theta) / delta.dtheta;
    }

    return new Pose2d(new Translation2d(delta.dx * s - delta.dy * c, delta.dx * c + delta.dy * s),
      new Rotation2d(cos_theta, sin_theta, false));
  }

  static log(transform) {
    let dtheta = transform.getRotation().getRadians();
    let half_dtheta = 0.5 * dtheta;
    let cos_minus_one = transform.getRotation().cos() - 1.0;
    let halftheta_by_tan_of_halfdtheta;
    if (Math.abs(cos_minus_one) < kEps) {
      halftheta_by_tan_of_halfdtheta = 1.0 - 1.0 / 12.0 * dtheta * dtheta;
    } else {
      halftheta_by_tan_of_halfdtheta = -(half_dtheta * transform.getRotation().sin()) / cos_minus_one;
    }
    let translation_part = transform.getTranslation()
      .rotateBy(new Rotation2d(halftheta_by_tan_of_halfdtheta, -half_dtheta, false));
    return new Twist2d(translation_part.x(), translation_part.y(), dtheta);
  }

  get getTranslation() {
    return this.translation;
  }

  get getRotation() {
    return this.rotation;
  }

  transformBy(other) {
    return new Pose2d(this.translation.translateBy(other.translation.rotateBy(this.rotation)),
      this.rotation.rotateBy(other.rotation));
  }

  inverse() {
    let rotation_inverted = this.rotation.inverse();
    return new Pose2d(this.translation.inverse().rotateBy(rotation_inverted), rotation_inverted);
  }

  normal() {
    return new Pose2d(this.translation, this.rotation.normal());
  }

  interpolate(other, x) {
    if (x <= 0) {
      return new Pose2d(this.translation, this.rotation, this.comment);
    } else if (x >= 1) {
      return new Pose2d(other.translation, other.rotation, other.comment);
    }
    let twist = Pose2d.log(this.inverse().transformBy(other));
    return this.transformBy(Pose2d.exp(twist.scaled(x)));
  }

  distance(other) {
    return Pose2d.log(this.inverse().transformBy(other)).norm();
  }

  heading(other) {
    return Math.atan2(this.translation.y - other.translation.y, this.translation.x - other.translation.x);
  }

  draw(drawHeading, radius) {
    this.translation.draw(null, radius);

    if (!drawHeading) {
      return;
    }

    let x = this.translation.drawX;
    let y = this.translation.drawY;

    ctx.beginPath();
    ctx.moveTo(x, y);
    ctx.lineTo(x + 25 * Math.cos(-this.rotation.getRadians()), y + 25 * Math.sin(-this.rotation.getRadians()));
    ctx.lineWidth = 3;
    ctx.stroke();
    ctx.closePath();
  }

  toString() {
    return "new Pose2d(new Translation2d(" + this.translation.x + ", " + this.translation.y + "), new Rotation2d(" + this.rotation.cos + ", " + this.rotation.sin + ", " + this.rotation.normalize + "))";
  }

  transform(other) {
    other.position.rotate(this.rotation);
    this.translation.translate(other.translation);
    this.rotation.rotate(other.rotation);
  }
}
