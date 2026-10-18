import type { Telemetry } from "./types";

export type GraphGroup =
  "flight" | "navigation" | "perception" | "payload" | "sensors";
interface Trace {
  key: string;
  label: string;
}
export interface GraphDefinition {
  title: string;
  unit: string;
  group: GraphGroup;
  traces: Trace[];
  note?: string;
}
export const graphs: GraphDefinition[] = [
  {
    title: "Altitude",
    unit: "m",
    group: "flight",
    traces: [{ key: "alt", label: "Reported altitude" }],
    note: "Production: altitude relative to the local origin. Standalone: altitude above FCU home.",
  },
  {
    title: "Speed",
    unit: "m/s",
    group: "flight",
    traces: [
      { key: "speed", label: "Measured ground speed" },
      { key: "forward", label: "Commanded body forward speed" },
    ],
    note: "Ground speed is a magnitude; the forward command is a signed body-axis component.",
  },
  {
    title: "Roll and pitch",
    unit: "°",
    group: "flight",
    traces: [
      { key: "roll", label: "Roll" },
      { key: "pitch", label: "Pitch" },
    ],
  },
  {
    title: "Heading",
    unit: "°",
    group: "flight",
    traces: [{ key: "yaw", label: "Yaw" }],
    note: "A gap marks a crossing of the heading wrap boundary.",
  },
  {
    title: "Battery voltage",
    unit: "V",
    group: "flight",
    traces: [{ key: "voltage", label: "Voltage" }],
  },
  {
    title: "Battery remaining",
    unit: "%",
    group: "flight",
    traces: [{ key: "battery", label: "FCU remaining estimate" }],
  },
  {
    title: "Front lidar range",
    unit: "m",
    group: "navigation",
    traces: [{ key: "front", label: "Front range" }],
    note: "Controller-reported range, not minimum aircraft clearance.",
  },
  {
    title: "Gap bearing",
    unit: "°",
    group: "navigation",
    traces: [{ key: "bearing", label: "Selected gap bearing" }],
    note: "Angle of the selected opening relative to the aircraft nose.",
  },
  {
    title: "Banner angle",
    unit: "°",
    group: "navigation",
    traces: [{ key: "squareAngle", label: "Angle to square on" }],
  },
  {
    title: "Banner standoff",
    unit: "m",
    group: "navigation",
    traces: [{ key: "squareDistance", label: "Measured standoff" }],
  },
  {
    title: "Detector processing rate",
    unit: "Hz",
    group: "perception",
    traces: [
      { key: "qrHz", label: "QR" },
      { key: "bannerHz", label: "Banner" },
      { key: "redHz", label: "Red zone" },
    ],
    note: "Rates use capture time in the vehicle clock.",
  },
  {
    title: "Capture to processing",
    unit: "ms",
    group: "perception",
    traces: [
      { key: "qrDelay", label: "QR" },
      { key: "bannerDelay", label: "Banner" },
      { key: "redDelay", label: "Red zone" },
    ],
    note: "Processing stamp minus capture stamp, both in the same ROS clock.",
  },
  {
    title: "Sensor receipt age",
    unit: "s",
    group: "perception",
    traces: [
      { key: "poseAge", label: "Pose" },
      { key: "lidarAge", label: "Lidar" },
      { key: "powerAge", label: "Battery" },
    ],
    note: "Time since the Pi received each source. This is not capture age.",
  },
  {
    title: "Gateway receipt rate",
    unit: "Hz",
    group: "perception",
    traces: [
      { key: "cameraHz", label: "Camera" },
      { key: "lidarHz", label: "Lidar" },
    ],
    note: "Remote gateway only. Rates use steady receipt time, not vehicle time.",
  },
  {
    title: "Gateway capture age",
    unit: "s",
    group: "perception",
    traces: [
      { key: "cameraAge", label: "Camera" },
      { key: "scanAge", label: "Lidar" },
    ],
    note: "Vehicle clock minus the original capture timestamp.",
  },
  {
    title: "Winch travel",
    unit: "m",
    group: "payload",
    traces: [
      { key: "payoutMeasured", label: "Measured travel" },
      { key: "payoutEstimate", label: "Command-integrated estimate" },
    ],
    note: "Travel and command acceptance do not confirm payload release.",
  },
  {
    title: "Vibration",
    unit: "m/s²",
    group: "sensors",
    traces: [
      { key: "vibrationX", label: "X" },
      { key: "vibrationY", label: "Y" },
      { key: "vibrationZ", label: "Z" },
    ],
    note: "FCU vibration levels, not raw acceleration or camera shake.",
  },
  {
    title: "Accelerometer clipping",
    unit: "count",
    group: "sensors",
    traces: [
      { key: "clip1", label: "IMU 1" },
      { key: "clip2", label: "IMU 2" },
      { key: "clip3", label: "IMU 3" },
    ],
    note: "Cumulative FCU counters. A gap marks a counter reset.",
  },
  {
    title: "Magnetic field strength",
    unit: "µT",
    group: "sensors",
    traces: [
      { key: "magStrength", label: "Measured magnitude" },
      { key: "magReference", label: "Captured reference" },
    ],
  },
  {
    title: "Magnetic field axes",
    unit: "µT",
    group: "sensors",
    traces: [
      { key: "magX", label: "X" },
      { key: "magY", label: "Y" },
      { key: "magZ", label: "Z" },
    ],
    note: "Components in the reported sensor frame; they change as the aircraft rotates.",
  },
  {
    title: "Magnetic interference indicator",
    unit: "%",
    group: "sensors",
    traces: [{ key: "magDeviation", label: "Field-strength change" }],
    note: "Change from the disarmed reference. Nearby magnetic sources and motion also affect this indicator; it is not a CompassMot calibration result.",
  },
  {
    title: "Battery current",
    unit: "A",
    group: "sensors",
    traces: [{ key: "current", label: "Reported battery current" }],
    note: "Compare current changes with field strength to investigate electrical interference. No current sensor means unavailable.",
  },
];

export interface HistoryRow {
  elapsed: number;
  receivedAt: number;
  serverTime: number | null;
  stage: string;
  magneticFrame?: string;
  values: Record<string, number | null>;
}
export const traceKeys = graphs.flatMap((g) => g.traces.map((s) => s.key));
const uniqueKeys = [...new Set(traceKeys)];
const finite = (v: unknown): number | null =>
  typeof v === "number" && Number.isFinite(v) ? v : null;
const nonnegative = (v: unknown) => {
  const n = finite(v);
  return n !== null && n >= 0 ? n : null;
};
const fresh = (t: Telemetry, key: string) => {
  const age = nonnegative(t.sample_age_s?.[key]);
  return age !== null && age < 3;
};

export function readValues(
  t: Telemetry,
  serverTime: number,
): Record<string, number | null> {
  const v: Record<string, number | null> = Object.fromEntries(
    uniqueKeys.map((k) => [k, null]),
  );
  // Explicit clock failure also covers paused simulation. Health graphs remain readable.
  const clockValid =
    !t.devices?.clock || (fresh(t, "devices") && t.devices.clock.ok === true);
  if (clockValid && fresh(t, "pose")) {
    v.alt = finite(t.flight?.alt);
  }
  if (
    clockValid &&
    fresh(t, t.sample_age_s?.attitude === undefined ? "pose" : "attitude")
  ) {
    v.roll = finite(t.flight?.roll_deg);
    v.pitch = finite(t.flight?.pitch_deg);
    v.yaw = finite(t.flight?.yaw_deg);
  }
  if (clockValid && fresh(t, "velocity")) v.speed = nonnegative(t.flight?.gs);
  if (clockValid && fresh(t, "avoidance")) {
    v.forward = finite(t.nav?.cmd_vx);
    if (fresh(t, "lidar")) {
      v.front = nonnegative(t.nav?.front_m);
      const bearing = finite(t.nav?.centering_err);
      v.bearing = bearing === null ? null : (bearing * 180) / Math.PI;
    }
  }
  if (fresh(t, "power")) {
    const volt = finite(t.power?.volt);
    v.voltage = volt !== null && volt > 0 ? volt : null;
    const pct = finite(t.power?.pct);
    v.battery = pct !== null && pct >= 0 && pct <= 100 ? pct : null;
    v.current = finite(t.power?.current_a);
  }
  if (clockValid && fresh(t, "square") && fresh(t, "lidar")) {
    v.squareAngle = finite(t.percep?.square_angle_deg);
    v.squareDistance = nonnegative(t.percep?.square_standoff_m);
  }
  for (const key of ["qr", "banner", "red"]) {
    const rate = t.detector_rates?.[key];
    const received = finite(rate?.received);
    if (
      !clockValid ||
      finite(serverTime) === null ||
      received === null ||
      serverTime - received < 0 ||
      serverTime - received >= 3
    )
      continue;
    v[`${key}Hz`] = nonnegative(rate?.hz);
    const capture = finite(rate?.capture_stamp),
      processed = finite(rate?.processed_stamp);
    v[`${key}Delay`] =
      capture !== null && processed !== null && processed >= capture
        ? (processed - capture) * 1000
        : null;
  }
  for (const [key, source] of [
    ["poseAge", "pose"],
    ["lidarAge", "lidar"],
    ["powerAge", "power"],
  ])
    v[key] = nonnegative(t.sample_age_s?.[source]);
  if (fresh(t, "devices")) {
    for (const [source, hz, age] of [
      ["camera", "cameraHz", "cameraAge"],
      ["lidar", "lidarHz", "scanAge"],
    ] as const) {
      const d = t.devices?.[source];
      const receiptAge = nonnegative(d?.receipt_age_s);
      if (receiptAge !== null && receiptAge < 3) {
        v[hz] = nonnegative(d?.hz);
        v[age] = nonnegative(d?.capture_age_s);
      }
    }
  }
  if (clockValid && fresh(t, "winch")) {
    v.payoutEstimate = nonnegative(t.winch?.payout_estimate_m);
    const age = nonnegative(t.winch?.measurement_age_s);
    if (age !== null && age < 3)
      v.payoutMeasured = nonnegative(t.winch?.measured_payout_m);
  }
  if (clockValid && fresh(t, "vibration")) {
    for (const [i, axis] of ["X", "Y", "Z"].entries())
      v[`vibration${axis}`] = nonnegative(
        t.sensor_diagnostics?.vibration_mps2?.[i],
      );
    for (let i = 0; i < 3; i++)
      v[`clip${i + 1}`] = nonnegative(t.sensor_diagnostics?.clipping?.[i]);
  }
  if (clockValid && fresh(t, "magnetic")) {
    for (const [i, axis] of ["X", "Y", "Z"].entries())
      v[`mag${axis}`] = finite(t.sensor_diagnostics?.magnetic_ut?.[i]);
    v.magStrength = nonnegative(t.sensor_diagnostics?.magnetic_strength_ut);
  }
  return v;
}

// At most five snapshots per second, ten minutes, independent of React renders.
export class TelemetryHistory {
  rows: HistoryRow[] = [];
  magneticReference: { strength: number; frame: string } | null = null;
  private latestTelemetry: Telemetry | null = null;
  private origin: number | null = null;
  private lastReceipt: number | null = null;
  private disconnected = false;
  ingest(
    t: Telemetry,
    serverTime: number,
    monotonicMs: number,
    wallMs: number,
  ) {
    const resumed =
      this.disconnected ||
      (this.lastReceipt !== null && monotonicMs - this.lastReceipt >= 3000);
    if (resumed) this.gap(monotonicMs - 1, wallMs);
    this.disconnected = false;
    this.lastReceipt = monotonicMs;
    this.latestTelemetry = t;
    if (
      this.magneticReference &&
      t.sensor_diagnostics?.magnetic_frame !== this.magneticReference.frame
    )
      this.magneticReference = null;
    if (this.origin === null) this.origin = monotonicMs;
    const elapsed = (monotonicMs - this.origin) / 1000;
    const last = this.rows[this.rows.length - 1];
    if (!resumed && last && elapsed - last.elapsed < 0.199) return;
    const values = readValues(t, serverTime);
    if (values.magStrength !== null && this.magneticReference) {
      values.magReference = this.magneticReference.strength;
      values.magDeviation =
        (values.magStrength / this.magneticReference.strength - 1) * 100;
    }
    this.append({
      elapsed,
      receivedAt: wallMs,
      serverTime: finite(serverTime),
      stage: String(t.mission?.state || "Unknown"),
      magneticFrame: t.sensor_diagnostics?.magnetic_frame || "",
      values,
    });
  }
  disconnect() {
    this.disconnected = true;
  }
  canCaptureMagneticReference(monotonicMs: number): boolean {
    const t = this.latestTelemetry;
    if (
      !t ||
      this.disconnected ||
      this.lastReceipt === null ||
      monotonicMs - this.lastReceipt >= 1000
    )
      return false;
    const clockValid =
      !t.devices?.clock || (fresh(t, "devices") && t.devices.clock.ok === true);
    const strength = finite(t.sensor_diagnostics?.magnetic_strength_ut);
    return (
      clockValid &&
      fresh(t, "magnetic") &&
      (t.sample_age_s?.magnetic ?? Infinity) < 1 &&
      fresh(t, "state") &&
      t.safety?.fcu_connected === true &&
      t.mission?.armed === false &&
      strength !== null &&
      strength > 0 &&
      !!t.sensor_diagnostics?.magnetic_frame
    );
  }
  captureMagneticReference(monotonicMs: number): boolean {
    if (!this.canCaptureMagneticReference(monotonicMs)) return false;
    const d = this.latestTelemetry!.sensor_diagnostics!;
    this.magneticReference = {
      strength: d.magnetic_strength_ut!,
      frame: d.magnetic_frame!,
    };
    return true;
  }
  tick(monotonicMs: number, wallMs: number) {
    if (this.lastReceipt !== null && monotonicMs - this.lastReceipt >= 3000)
      this.gap(monotonicMs, wallMs);
  }
  private gap(monotonicMs: number, wallMs: number) {
    if (this.origin === null) return;
    this.append({
      elapsed: (monotonicMs - this.origin) / 1000,
      receivedAt: wallMs,
      serverTime: null,
      stage: "Telemetry unavailable",
      values: Object.fromEntries(uniqueKeys.map((k) => [k, null])),
    });
  }
  private append(row: HistoryRow) {
    this.rows.push(row);
    const cutoff = row.elapsed - 600;
    const first = this.rows.findIndex((r) => r.elapsed >= cutoff);
    if (first > 0) this.rows.splice(0, first);
    if (this.rows.length > 3100) this.rows.splice(0, this.rows.length - 3100);
  }
}

export function seriesPoints(
  rows: HistoryRow[],
  key: string,
): [number, number | null][] {
  let previous: number | null = null;
  return rows.map((r) => {
    const current = r.values[key] ?? null;
    const crossing =
      current !== null &&
      previous !== null &&
      ((key === "yaw" && Math.abs(current - previous) > 180) ||
        (key.startsWith("clip") && current < previous));
    previous = current;
    return [r.elapsed, crossing ? null : current];
  });
}
export function historyCsv(rows: HistoryRow[]): string {
  const quote = (s: string) =>
    `"${(/^[=+@\-\t\r\n]/.test(s) ? "'" + s : s).split('"').join('""')}"`;
  const columns = graphs.flatMap((g) =>
    g.traces.map((s) => ({
      key: s.key,
      label: `${g.title}: ${s.label} [${g.unit}]`,
    })),
  );
  return [
    [
      "Browser receipt UTC",
      "Elapsed steady seconds",
      "Pi snapshot Unix seconds",
      "Mission stage",
      "Magnetic sensor frame",
      ...columns.map((c) => c.label),
    ]
      .map(quote)
      .join(","),
    ...rows.map((r) =>
      [
        quote(new Date(r.receivedAt).toISOString()),
        r.elapsed.toFixed(3),
        r.serverTime ?? "",
        quote(r.stage),
        quote(r.magneticFrame || ""),
        ...columns.map((c) => r.values[c.key] ?? ""),
      ].join(","),
    ),
  ].join("\r\n");
}
