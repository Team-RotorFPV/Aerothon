import { test } from "node:test";
import assert from "node:assert/strict";
import {
  TelemetryHistory,
  readValues,
  seriesPoints,
  historyCsv,
} from "../src/telemetryHistory.ts";

test("stale gateway clock health cannot revive vehicle traces", () => {
  const t = sample();
  t.devices.clock.ok = false;
  t.sample_age_s.devices = 4;
  assert.equal(readValues(t, 100).alt, null);
  t.devices.clock.ok = true;
  assert.equal(readValues(t, 100).alt, null);
});

function sensors() {
  const t = sample();
  t.sample_age_s.vibration = 0.1;
  t.sample_age_s.magnetic = 0.1;
  t.sample_age_s.state = 0.1;
  t.safety = { fcu_connected: true };
  t.mission.armed = false;
  t.power.current_a = 4;
  t.sensor_diagnostics = {
    vibration_mps2: [2, 3, 4],
    clipping: [0, 1, 2],
    magnetic_ut: [30, 40, 0],
    magnetic_strength_ut: 50,
    magnetic_frame: "base_link",
  };
  return t;
}

test("vibration, clipping and magnetic field retain units and independent freshness", () => {
  const t = sensors(),
    v = readValues(t, 100);
  assert.equal(v.vibrationZ, 4);
  assert.equal(v.clip1, 0);
  assert.equal(v.magStrength, 50);
  assert.equal(v.magY, 40);
  assert.equal(v.current, 4);
  t.sample_age_s.magnetic = 4;
  assert.equal(readValues(t, 100).magStrength, null);
  assert.equal(readValues(t, 100).vibrationX, 2);
  t.sample_age_s.vibration = 4;
  assert.equal(readValues(t, 100).clip1, null);
});

test("magnetic reference requires fresh explicitly disarmed FCU data, and never sends a command", () => {
  const h = new TelemetryHistory(),
    t = sensors();
  t.mission.armed = true;
  h.ingest(t, 100, 0, 100000);
  assert.equal(h.captureMagneticReference(0), false);
  t.mission.armed = false;
  t.safety.fcu_connected = false;
  h.ingest(t, 100, 200, 100200);
  assert.equal(h.captureMagneticReference(200), false);
  t.safety.fcu_connected = true;
  delete t.sample_age_s.state;
  h.ingest(t, 100, 400, 100400);
  assert.equal(h.captureMagneticReference(400), false);
  t.sample_age_s.state = 0.1;
  h.ingest(t, 100, 600, 100600);
  assert.equal(h.captureMagneticReference(600), true);
  assert.deepEqual(h.magneticReference, { strength: 50, frame: "base_link" });
  assert.equal(h.canCaptureMagneticReference(1600), false);
});

test("magnetic deviation starts unknown, preserves signed change and clears on sensor-frame change", () => {
  const h = new TelemetryHistory(),
    t = sensors();
  h.ingest(t, 100, 0, 100000);
  assert.equal(h.rows[0].values.magDeviation, null);
  h.captureMagneticReference(0);
  t.sensor_diagnostics.magnetic_strength_ut = 45;
  h.ingest(t, 100, 200, 100200);
  assert.ok(Math.abs(h.rows[1].values.magDeviation + 10) < 0.001);
  assert.equal(h.rows[1].values.magReference, 50);
  t.sensor_diagnostics.magnetic_frame = "other_imu";
  h.ingest(t, 100, 400, 100400);
  assert.equal(h.magneticReference, null);
  assert.equal(h.rows[2].values.magDeviation, null);
  assert.ok(historyCsv(h.rows).includes("Magnetic sensor frame"));
});

test("clipping counter resets and both heading conventions break plotted lines", () => {
  const rows = [4, 5, 0, 1].map((n, i) => ({
    elapsed: i,
    values: { clip1: n },
  }));
  assert.deepEqual(seriesPoints(rows, "clip1"), [
    [0, 4],
    [1, 5],
    [2, null],
    [3, 1],
  ]);
  assert.deepEqual(
    seriesPoints(
      [
        { elapsed: 0, values: { yaw: 359 } },
        { elapsed: 1, values: { yaw: 1 } },
      ],
      "yaw",
    ),
    [
      [0, 359],
      [1, null],
    ],
  );
});

test("fresh controller heartbeats cannot refresh old lidar geometry", () => {
  const t = sample();
  t.sample_age_s.lidar = 4;
  const v = readValues(t, 100);
  for (const key of ["front", "bearing", "squareAngle", "squareDistance"])
    assert.equal(v[key], null);
  assert.equal(v.forward, 0.5);
});

test("CSV stage text cannot turn into a spreadsheet formula", () => {
  const h = new TelemetryHistory(),
    t = sample();
  t.mission.state = '=HYPERLINK("example")';
  h.ingest(t, 100, 0, 100000);
  assert.ok(historyCsv(h.rows).includes("\"'=HYPERLINK"));
});

const sample = () => ({
  mission: { state: "RETURN" },
  flight: { alt: 5, gs: 1.2, roll_deg: 3, pitch_deg: -2, yaw_deg: 10 },
  nav: { front_m: 2.1, centering_err: Math.PI / 6, cmd_vx: 0.5 },
  power: { volt: 12, pct: 75 },
  percep: { square_angle_deg: 4, square_standoff_m: 2 },
  sample_age_s: {
    pose: 0.1,
    velocity: 0.1,
    avoidance: 0.1,
    power: 0.1,
    square: 0.1,
    winch: 0.1,
    devices: 0.1,
    lidar: 0.2,
  },
  detector_rates: {
    qr: { hz: 8, capture_stamp: 30, processed_stamp: 30.1, received: 100 },
  },
  devices: {
    clock: { ok: true },
    camera: { ok: true, hz: 10, receipt_age_s: 0.1, capture_age_s: 0.04 },
  },
  winch: {
    payout_estimate_m: 2.2,
    measured_payout_m: 2,
    measurement_age_s: 0.1,
  },
});

test("bootstrap and absent source stamps never become zero-valued measurements", () => {
  const t = sample();
  t.sample_age_s = {};
  t.detector_rates = {};
  assert.ok(Object.values(readValues(t, 100)).every((v) => v === null));
});
test("each flight trace uses its own freshness and stale measured winch stays unknown", () => {
  const t = sample();
  t.sample_age_s.velocity = 4;
  t.sample_age_s.avoidance = 4;
  t.winch.measurement_age_s = 4;
  const v = readValues(t, 100);
  assert.equal(v.alt, 5);
  assert.equal(v.speed, null);
  assert.equal(v.front, null);
  assert.equal(v.forward, null);
  assert.equal(v.payoutMeasured, null);
  assert.equal(v.payoutEstimate, 2.2);
});
test("bearing converts radians; timing stays in matching source clock, regardless of browser clock", () => {
  const t = sample(),
    v = readValues(t, 100);
  assert.ok(Math.abs(v.bearing - 30) < 0.001);
  assert.ok(Math.abs(v.qrDelay - 100) < 0.001);
  const h = new TelemetryHistory();
  h.ingest(t, 100, 10, 9999999999999);
  assert.equal(h.rows[0].values.qrHz, 8);
  assert.equal(readValues(t, 104).qrHz, null);
  assert.equal(readValues(t, NaN).qrHz, null);
});
test("paused vehicle clock gaps movement and perception; health evidence remains", () => {
  const t = sample();
  t.devices.clock.ok = false;
  const v = readValues(t, 100);
  for (const k of [
    "alt",
    "yaw",
    "speed",
    "front",
    "qrHz",
    "qrDelay",
    "squareAngle",
    "payoutMeasured",
  ])
    assert.equal(v[k], null);
  assert.equal(v.poseAge, 0.1);
  assert.equal(v.voltage, 12);
});
test("invalid battery, delay and non-finite values remain unavailable", () => {
  const t = sample();
  t.power.pct = -1;
  t.power.volt = NaN;
  t.flight.alt = Infinity;
  t.detector_rates.qr.processed_stamp = 29;
  const v = readValues(t, 100);
  assert.equal(v.battery, null);
  assert.equal(v.voltage, null);
  assert.equal(v.alt, null);
  assert.equal(v.qrDelay, null);
});
test("network silence and quick reconnect insert gaps without joining old and new values", () => {
  const h = new TelemetryHistory();
  h.ingest(sample(), 100, 1000, 100000);
  h.tick(5000, 104000);
  assert.equal(h.rows[1].values.alt, null);
  h.ingest(sample(), 100, 6000, 105000);
  assert.equal(h.rows[h.rows.length - 2].values.alt, null);
  h.disconnect();
  h.ingest(sample(), 100, 6100, 105100);
  assert.equal(h.rows[h.rows.length - 2].values.alt, null);
  assert.equal(h.rows[h.rows.length - 1].values.alt, 5);
});
test("higher-rate telemetry is bounded to 5Hz and ten minutes; monotonic time survives wall-clock jumps", () => {
  const h = new TelemetryHistory();
  for (let i = 0; i < 20000; i++)
    h.ingest(sample(), 100, i * 50, 100000 - i * 1000);
  assert.ok(h.rows.length <= 3100);
  assert.ok(h.rows.length >= 2900);
  assert.ok(h.rows[h.rows.length - 1].elapsed - h.rows[0].elapsed <= 600);
  assert.ok(h.rows.every((r, i) => !i || r.elapsed > h.rows[i - 1].elapsed));
});
test("heading wrap breaks the plotted line, ordinary heading changes do not", () => {
  const rows = [179, -179, -177].map((n, i) => ({
    elapsed: i,
    values: { yaw: n },
  }));
  assert.deepEqual(seriesPoints(rows, "yaw"), [
    [0, 179],
    [1, null],
    [2, -177],
  ]);
});
test("CSV preserves units, missing values and quoted stage text", () => {
  const h = new TelemetryHistory(),
    t = sample();
  t.mission.state = 'RETURN,"gate"';
  h.ingest(t, 100, 0, 100000);
  h.tick(4000, 104000);
  const csv = historyCsv(h.rows);
  assert.ok(csv.includes("Reported altitude [m]"));
  assert.ok(csv.includes('"RETURN,""gate"""'));
  assert.ok(csv.includes("Telemetry unavailable"));
  assert.ok(!csv.includes("null") && !csv.includes("NaN"));
});

test("standalone attitude freshness does not depend on a GPS position stream", () => {
  const t = sample();
  t.sample_age_s.pose = 5;
  t.sample_age_s.attitude = 0.1;
  assert.equal(readValues(t, 100).roll, t.flight.roll_deg);
  assert.equal(readValues(t, 100).alt, null);
  t.sample_age_s.pose = 0.1;
  t.sample_age_s.attitude = 5;
  assert.equal(readValues(t, 100).roll, null);
});
