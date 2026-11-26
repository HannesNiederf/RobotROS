// Map drawing on a canvas. The transform helpers are pure functions (tested with node --test).

/** Yaw from a geometry_msgs/Quaternion. */
export function yawOf(q) {
  return Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
}

/** Quaternion (z/w only) for a yaw angle. */
export function quatOf(yaw) {
  return { x: 0, y: 0, z: Math.sin(yaw / 2), w: Math.cos(yaw / 2) };
}

/**
 * Fit the map (meta: resolution, origin {x,y}, width, height in cells) into a canvas of w x h pixels,
 * keeping the aspect ratio and a margin. Returns {scale (px per m), offX, offY, meta}.
 */
export function fitTransform(meta, w, h, margin = 10) {
  const mw = meta.width * meta.resolution;
  const mh = meta.height * meta.resolution;
  const scale = Math.max(1e-6, Math.min((w - 2 * margin) / mw, (h - 2 * margin) / mh));
  return { scale, offX: (w - mw * scale) / 2, offY: (h - mh * scale) / 2, meta };
}

/** World (map frame, m) -> canvas pixel. Map image top = highest y. */
export function worldToCanvas(tf, x, y) {
  const { meta, scale, offX, offY } = tf;
  const mh = meta.height * meta.resolution;
  return { px: offX + (x - meta.origin.x) * scale, py: offY + (mh - (y - meta.origin.y)) * scale };
}

export function canvasToWorld(tf, px, py) {
  const { meta, scale, offX, offY } = tf;
  const mh = meta.height * meta.resolution;
  return { x: meta.origin.x + (px - offX) / scale, y: meta.origin.y + mh - (py - offY) / scale };
}

/** True if the canvas point lies on the map image. */
export function insideMap(tf, px, py) {
  const { meta, scale, offX, offY } = tf;
  return px >= offX && py >= offY && px <= offX + meta.width * meta.resolution * scale &&
    py <= offY + meta.height * meta.resolution * scale;
}

export class MapView {
  constructor(canvas) {
    this.canvas = canvas;
    this.ctx = canvas.getContext("2d");
    this.image = null;
    this.meta = null;
    this.pose = null;
    this.plan = [];
    this.goal = null;
    this.stations = [];
    this.tf = null;
    this.onTap = null;
    canvas.addEventListener("pointerup", (ev) => this._tap(ev));
    new ResizeObserver(() => this.resize()).observe(canvas);
  }

  setMap(dataUrl, meta) {
    const img = new Image();
    img.onload = () => {
      this.image = img;
      this.meta = meta;
      this.resize();
    };
    img.src = dataUrl;
  }

  setPose(pose) {
    this.pose = pose;
    this.draw();
  }

  setPlan(points) {
    this.plan = points;
    this.draw();
  }

  setStations(stations) {
    this.stations = stations || [];
    this.draw();
  }

  setGoal(goal) {
    this.goal = goal;
    this.draw();
  }

  resize() {
    const r = this.canvas.getBoundingClientRect();
    const dpr = window.devicePixelRatio || 1;
    this.canvas.width = Math.max(1, Math.round(r.width * dpr));
    this.canvas.height = Math.max(1, Math.round(r.height * dpr));
    this.ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    this.tf = this.meta ? fitTransform(this.meta, r.width, r.height) : null;
    this.draw();
  }

  _tap(ev) {
    if (!this.tf || !this.onTap) return;
    const r = this.canvas.getBoundingClientRect();
    const px = ev.clientX - r.left;
    const py = ev.clientY - r.top;
    if (insideMap(this.tf, px, py)) this.onTap(canvasToWorld(this.tf, px, py));
  }

  draw() {
    const { ctx, canvas } = this;
    const dpr = window.devicePixelRatio || 1;
    ctx.clearRect(0, 0, canvas.width / dpr, canvas.height / dpr);
    if (!this.tf || !this.image) return;
    const { meta, scale, offX, offY } = this.tf;
    ctx.imageSmoothingEnabled = false;
    ctx.drawImage(this.image, offX, offY, meta.width * meta.resolution * scale, meta.height * meta.resolution * scale);
    if (this.plan.length > 1) {
      ctx.strokeStyle = "#1a9e3a";
      ctx.lineWidth = 3;
      ctx.beginPath();
      this.plan.forEach((p, i) => {
        const c = worldToCanvas(this.tf, p.x, p.y);
        if (i === 0) ctx.moveTo(c.px, c.py);
        else ctx.lineTo(c.px, c.py);
      });
      ctx.stroke();
    }
    for (const st of this.stations) {
      // Square = taught station pose, short line = direction the front points (out of the box, ÄM-01 D-33).
      const c = worldToCanvas(this.tf, st.pose.x, st.pose.y);
      const h = Math.max(7, 0.15 * scale);
      ctx.fillStyle = st.kind === "DOCK" ? "#7b1fa2" : st.occupied ? "#8d6e63" : "#ffffff";
      ctx.strokeStyle = st.kind === "DOCK" ? "#4a148c" : "#4e342e";
      ctx.lineWidth = 2;
      ctx.fillRect(c.px - h, c.py - h, 2 * h, 2 * h);
      ctx.strokeRect(c.px - h, c.py - h, 2 * h, 2 * h);
      ctx.beginPath();
      ctx.moveTo(c.px, c.py);
      ctx.lineTo(c.px + 2 * h * Math.cos(st.pose.theta), c.py - 2 * h * Math.sin(st.pose.theta));
      ctx.stroke();
      ctx.fillStyle = "#263238";
      ctx.font = "16px system-ui, sans-serif";
      ctx.fillText(st.name, c.px + h + 4, c.py - h - 2);
    }
    if (this.goal) {
      const c = worldToCanvas(this.tf, this.goal.x, this.goal.y);
      ctx.strokeStyle = "#d48a00";
      ctx.lineWidth = 3;
      ctx.beginPath();
      ctx.arc(c.px, c.py, 10, 0, 2 * Math.PI);
      ctx.stroke();
    }
    if (this.pose) {
      const c = worldToCanvas(this.tf, this.pose.x, this.pose.y);
      const r = Math.max(10, 0.2 * scale);
      ctx.save();
      ctx.translate(c.px, c.py);
      ctx.rotate(-this.pose.yaw); // canvas y points down
      ctx.fillStyle = "#1f5fd1";
      ctx.beginPath();
      ctx.moveTo(r, 0);
      ctx.lineTo(-0.7 * r, 0.6 * r);
      ctx.lineTo(-0.4 * r, 0);
      ctx.lineTo(-0.7 * r, -0.6 * r);
      ctx.closePath();
      ctx.fill();
      ctx.restore();
    }
  }
}
