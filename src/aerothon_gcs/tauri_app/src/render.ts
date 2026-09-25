// GCS canvas renderers — monochrome (graphite/steel), fully data-driven.
// Nothing here hardcodes a map, path, or target: the map auto-frames the live
// GPS trail, obstacles come from the live lidar/SLAM, and the SLAM view renders
// the real slam_toolbox OccupancyGrid when the aggregator forwards it (falling
// back to accumulated live-lidar points until the first /map arrives).

export interface OccGrid { res: number; w: number; h: number; ox: number; oy: number; data: number[]; }

const C = {
  ink: "#0c0e11", grid: "rgba(255,255,255,.05)", gridText: "#3f454e",
  track: "205,214,224", drone: "#eef1f4", droneEdge: "#cdd6e0", heading: "#cdd6e0",
  obst: "#8b929c", occ: "#cdd6e0", free: "rgba(255,255,255,.03)", inflate: "rgba(205,214,224,.06)",
  home: "#7fb894", scanRay: "rgba(205,214,224,.10)", scanHit: "#aeb7c2", label: "#5c636d",
};

function fit(c: HTMLCanvasElement): [CanvasRenderingContext2D, number, number] {
  const g = c.getContext("2d")!; g.setTransform(1, 0, 0, 1, 0, 0); return [g, c.width, c.height];
}
function proj(w: number, h: number, b: any, pad: number) {
  const s = Math.min((w - pad * 2) / (b.x1 - b.x0), (h - pad * 2) / (b.y1 - b.y0));
  const ox = (w - (b.x1 - b.x0) * s) / 2 - b.x0 * s, oy = (h - (b.y1 - b.y0) * s) / 2 + b.y1 * s;
  return { s, fx: (x: number) => ox + x * s, fy: (y: number) => oy - y * s };
}
function drone(g: CanvasRenderingContext2D, x: number, y: number, ang: number, r: number) {
  g.save(); g.translate(x, y);
  g.fillStyle = "rgba(205,214,224,.14)"; g.beginPath(); g.arc(0, 0, r + 7, 0, 7); g.fill();
  g.rotate(ang);
  g.fillStyle = C.drone; g.strokeStyle = C.droneEdge; g.lineWidth = 1.4; g.beginPath();
  g.moveTo(r, 0); g.lineTo(-r * 0.7, r * 0.62); g.lineTo(-r * 0.32, 0); g.lineTo(-r * 0.7, -r * 0.62);
  g.closePath(); g.fill(); g.stroke(); g.restore();
}
function tape(g: CanvasRenderingContext2D, x: number, y: number, txt: string, label?: string) {
  g.save(); g.font = "600 13px 'IBM Plex Mono',monospace"; const w = 48, h = 20; g.translate(x, y);
  g.fillStyle = "rgba(10,11,13,.85)"; g.fillRect(-w / 2, -h / 2, w, h);
  g.strokeStyle = "#3a3e45"; g.lineWidth = 1; g.strokeRect(-w / 2, -h / 2, w, h);
  g.fillStyle = "#eceef1"; g.textAlign = "center"; g.fillText(txt, 0, 4);
  if (label) { g.fillStyle = "#5c636d"; g.font = "700 8px Inter,sans-serif"; g.fillText(label, 0, -h / 2 - 4); }
  g.restore();
}

export function drawHUD(c: HTMLCanvasElement, S: any) {
  const [g, W, H] = fit(c); g.clearRect(0, 0, W, H);
  const roll = (S ? S.flight.roll_deg : 0) * Math.PI / 180, pitch = S ? S.flight.pitch_deg : 0;
  const cx = W / 2, cy = H / 2, ppd = H / 60;
  g.save(); g.beginPath(); g.rect(0, 0, W, H); g.clip();
  g.translate(cx, cy); g.rotate(roll); g.translate(0, pitch * ppd);
  g.fillStyle = "#2b3038"; g.fillRect(-W, -H * 2, W * 2, H * 2);        // sky (steel)
  g.fillStyle = "#15181d"; g.fillRect(-W, 0, W * 2, H * 2);             // ground (graphite)
  g.strokeStyle = "#eceef1"; g.lineWidth = 1.6; g.beginPath(); g.moveTo(-W, 0); g.lineTo(W, 0); g.stroke();
  g.strokeStyle = "rgba(255,255,255,.55)"; g.fillStyle = "rgba(255,255,255,.6)";
  g.font = "10px 'IBM Plex Mono',monospace"; g.textAlign = "center"; g.lineWidth = 1.2;
  for (let d = -30; d <= 30; d += 10) { if (!d) continue; const y = -d * ppd; const w = d % 20 ? 22 : 42;
    g.beginPath(); g.moveTo(-w, y); g.lineTo(w, y); g.stroke();
    g.fillText(String(Math.abs(d)), w + 11, y + 3); g.fillText(String(Math.abs(d)), -w - 11, y + 3); }
  g.restore();
  // fixed aircraft symbol
  g.strokeStyle = C.droneEdge; g.lineWidth = 2.5; g.beginPath();
  g.moveTo(cx - 42, cy); g.lineTo(cx - 13, cy); g.lineTo(cx - 13, cy + 8);
  g.moveTo(cx + 42, cy); g.lineTo(cx + 13, cy); g.lineTo(cx + 13, cy + 8); g.stroke();
  g.fillStyle = C.droneEdge; g.beginPath(); g.arc(cx, cy, 2.5, 0, 7); g.fill();
  const yaw = S ? Math.round(S.flight.yaw_deg) : 0, alt = S ? S.flight.alt : 0, gs = S ? S.flight.gs : 0;
  tape(g, cx, 12, `${yaw}°`, "HDG"); tape(g, W - 36, cy, alt.toFixed(1), "ALT"); tape(g, 34, cy, gs.toFixed(1), "SPD");
}

function drawOcc(g: CanvasRenderingContext2D, P: any, grid: OccGrid) {
  const cell = grid.res * P.s;
  for (let r = 0; r < grid.h; r++) for (let col = 0; col < grid.w; col++) {
    const v = grid.data[r * grid.w + col]; if (v < 0) continue;   // unknown -> skip
    const wx = grid.ox + col * grid.res, wy = grid.oy + r * grid.res;
    if (v >= 50) g.fillStyle = C.occ;
    else if (v === 0) g.fillStyle = C.free;
    else continue;
    g.fillRect(P.fx(wx) - cell / 2 - .5, P.fy(wy) - cell / 2 - .5, cell + 1, cell + 1);
  }
}

export function drawMap(c: HTMLCanvasElement, S: any, trail: [number, number][],
                        cells: Map<string, [number, number]>, grid?: OccGrid | null) {
  const [g, W, H] = fit(c); g.fillStyle = C.ink; g.fillRect(0, 0, W, H);
  let x0 = -5, x1 = 5, y0 = -5, y1 = 5;
  trail.forEach(([tx, ty]) => { x0 = Math.min(x0, tx - 6); x1 = Math.max(x1, tx + 6); y0 = Math.min(y0, ty - 6); y1 = Math.max(y1, ty + 6); });
  if (S) { x0 = Math.min(x0, S.flight.x - 6); x1 = Math.max(x1, S.flight.x + 6); y0 = Math.min(y0, S.flight.y - 6); y1 = Math.max(y1, S.flight.y + 6); }
  const P = proj(W, H, { x0, x1, y0, y1 }, 22);
  // grid + coords
  g.strokeStyle = C.grid; g.lineWidth = 1; g.font = "9px 'IBM Plex Mono',monospace";
  for (let x = Math.ceil(x0 / 5) * 5; x <= x1; x += 5) { g.beginPath(); g.moveTo(P.fx(x), 0); g.lineTo(P.fx(x), H); g.stroke();
    g.fillStyle = C.gridText; g.fillText(x + "m", P.fx(x) + 3, 12); }
  for (let y = Math.ceil(y0 / 5) * 5; y <= y1; y += 5) { g.beginPath(); g.moveTo(0, P.fy(y)); g.lineTo(W, P.fy(y)); g.stroke();
    g.fillStyle = C.gridText; g.fillText(y + "m", 3, P.fy(y) - 3); }
  // real occupancy grid if available, else discovered lidar points + inflation
  if (grid && grid.data && grid.data.length) drawOcc(g, P, grid);
  else {
    g.fillStyle = C.inflate; cells.forEach(([ox, oy]) => { g.beginPath(); g.arc(P.fx(ox), P.fy(oy), .5 * P.s, 0, 7); g.fill(); });
    g.fillStyle = C.obst; cells.forEach(([ox, oy]) => g.fillRect(P.fx(ox) - 1.4, P.fy(oy) - 1.4, 2.8, 2.8));
  }
  // home
  g.strokeStyle = C.home; g.lineWidth = 1.5; g.beginPath(); g.arc(P.fx(0), P.fy(0), 8, 0, 7); g.stroke();
  g.fillStyle = C.home; g.font = "9px Inter,sans-serif"; g.fillText("HOME", P.fx(0) + 11, P.fy(0) + 3);
  // breadcrumb trail (steel, fading)
  for (let i = 1; i < trail.length; i++) { const a = i / trail.length;
    g.strokeStyle = `rgba(${C.track},${0.12 + 0.7 * a})`; g.lineWidth = 1.4 + 1.2 * a; g.beginPath();
    g.moveTo(P.fx(trail[i - 1][0]), P.fy(trail[i - 1][1])); g.lineTo(P.fx(trail[i][0]), P.fy(trail[i][1])); g.stroke(); }
  // drone + heading
  if (S) { drone(g, P.fx(S.flight.x), P.fy(S.flight.y), -S.flight.yaw_deg * Math.PI / 180, 10);
    const hR = -S.flight.yaw_deg * Math.PI / 180, vL = Math.max(14, (S.flight.gs || 1) * 16);
    g.strokeStyle = C.heading; g.lineWidth = 1.5; g.beginPath();
    g.moveTo(P.fx(S.flight.x), P.fy(S.flight.y)); g.lineTo(P.fx(S.flight.x) + vL * Math.cos(hR), P.fy(S.flight.y) + vL * Math.sin(hR)); g.stroke(); }
  // scale bar
  g.fillStyle = C.label; g.font = "10px 'IBM Plex Mono',monospace"; g.textAlign = "left";
  const bx = W - 92, by = H - 14; g.strokeStyle = C.label; g.lineWidth = 1.5;
  g.beginPath(); g.moveTo(bx, by); g.lineTo(bx + 10 * P.s, by); g.stroke(); g.fillText("10 m", bx, by - 5);
}

export function drawSlam(c: HTMLCanvasElement, S: any, cells: Map<string, [number, number]>, grid?: OccGrid | null) {
  const [g, W, H] = fit(c); g.fillStyle = "#08090b"; g.fillRect(0, 0, W, H);
  const cx = S ? S.flight.x : 0, cy = S ? S.flight.y : 0;
  let b: any;
  const src = (grid && grid.data && grid.data.length) ? null : cells;
  if (grid && grid.data && grid.data.length) {
    b = { x0: grid.ox, x1: grid.ox + grid.w * grid.res, y0: grid.oy, y1: grid.oy + grid.h * grid.res };
  } else if (src && src.size > 25) {
    let x0 = cx, x1 = cx, y0 = cy, y1 = cy;
    src.forEach((p) => { x0 = Math.min(x0, p[0]); x1 = Math.max(x1, p[0]); y0 = Math.min(y0, p[1]); y1 = Math.max(y1, p[1]); });
    b = { x0: x0 - 2, x1: x1 + 2, y0: y0 - 2, y1: y1 + 2 };
    if (b.x1 - b.x0 < 12) { const m = (b.x0 + b.x1) / 2; b.x0 = m - 6; b.x1 = m + 6; }
    if (b.y1 - b.y0 < 8) { const m = (b.y0 + b.y1) / 2; b.y0 = m - 4; b.y1 = m + 4; }
  } else b = { x0: cx - 12, x1: cx + 12, y0: cy - 12, y1: cy + 12 };
  const P = proj(W, H, b, 8);
  g.strokeStyle = "rgba(255,255,255,.04)"; g.lineWidth = 1;
  for (let x = Math.ceil(b.x0 / 2) * 2; x < b.x1; x += 2) { g.beginPath(); g.moveTo(P.fx(x), 0); g.lineTo(P.fx(x), H); g.stroke(); }
  for (let y = Math.ceil(b.y0 / 2) * 2; y < b.y1; y += 2) { g.beginPath(); g.moveTo(0, P.fy(y)); g.lineTo(W, P.fy(y)); g.stroke(); }
  if (grid && grid.data && grid.data.length) drawOcc(g, P, grid);
  else if (src) { g.fillStyle = C.inflate; src.forEach((p) => { g.beginPath(); g.arc(P.fx(p[0]), P.fy(p[1]), .5 * P.s, 0, 7); g.fill(); });
    g.fillStyle = C.occ; src.forEach((p) => g.fillRect(P.fx(p[0]) - 1.4, P.fy(p[1]) - 1.4, 2.8, 2.8)); }
  // live lidar rays
  const sc = S?.scan;
  if (sc && sc.ranges) { const R = sc.ranges, n = R.length;
    for (let k = 0; k < n; k++) { const r = R[k]; if (r == null) continue; const a = -Math.PI + 2 * Math.PI * k / n;
      const ex = cx + r * Math.cos(a), ey = cy + r * Math.sin(a);
      g.strokeStyle = C.scanRay; g.lineWidth = 1; g.beginPath(); g.moveTo(P.fx(cx), P.fy(cy)); g.lineTo(P.fx(ex), P.fy(ey)); g.stroke();
      g.fillStyle = C.scanHit; g.fillRect(P.fx(ex) - 1.6, P.fy(ey) - 1.6, 3.2, 3.2); } }
  if (S) drone(g, P.fx(cx), P.fy(cy), -S.flight.yaw_deg * Math.PI / 180, 9);
  g.fillStyle = C.label; g.font = "10px 'IBM Plex Mono',monospace"; g.textAlign = "left";
  const lbl = (grid && grid.data && grid.data.length) ? `occupancy ${grid.w}×${grid.h} @ ${grid.res}m` : `lidar points: ${cells.size}`;
  g.fillText(lbl, 8, H - 8);
}
