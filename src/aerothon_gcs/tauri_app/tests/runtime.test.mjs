import { test } from "node:test";
import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import {
  endpoint,
  videoUrl,
  freshFcu,
  startAllowed,
  taskCommand,
  commandEnvelope,
  acknowledgement,
  commandId,
  reviewAllowed,
  tasks,
} from "../src/runtime.ts";
import { validateDraft } from "../src/arenaForm.ts";
const schema = JSON.parse(
  readFileSync(new URL("../src/arenaSchema.json", import.meta.url)),
);

test('HTTP intranet browsers without randomUUID can still send unique command IDs', () => {
  const provider = { getRandomValues: bytes => { bytes.set(Array.from({length:16}, (_,i) => i)); return bytes; } };
  assert.equal(commandId(provider), '00010203-0405-4607-8809-0a0b0c0d0e0f');
  assert.equal(commandId({ ...provider, randomUUID: () => 'secure-context-id' }), 'secure-context-id');
  assert.throws(() => commandId({}));
});
const ready = () => ({
  mission: { state: "IDLE", mode: "GUIDED", armed: false },
  safety: { ready: true, fcu_connected: true },
  runtime_health: {fresh:true},
  sample_age_s: { state: 0.1, readiness: 0.1 },
});

test("browser tasks use the shared production supervisor registry", () => {
  const source = readFileSync(new URL("../../../aerothon_runtime/aerothon_runtime/contracts.py", import.meta.url),"utf8");
  for(const task of tasks) {
    assert.ok(source.includes(`'${task.id}':`));
    const c=taskCommand(task.id,'stow',{height:5});
    assert.equal(c.cmd,'start'); assert.equal(c.args.task, task.id);
  }
  assert.deepEqual(taskCommand('mechanism','release'), {cmd:'start',args:{task:'mechanism',params:{action:'release'}}});
  assert.throws(()=>taskCommand('manual_joystick','stow'));
  assert.throws(()=>taskCommand('mechanism','unsupported'));
});
test("flight starts require fresh FCU and readiness, exclusive execution", () => {
  const t = ready();
  assert.equal(startAllowed(t, true, "mission"), true);
  assert.equal(startAllowed(t, false, "mission"), false);
  for (const key of ["state", "readiness"]) {
    const stale = ready();
    stale.sample_age_s[key] = 3;
    assert.equal(startAllowed(stale, true, "mission"), false);
  }
  for (const phase of ["STARTING", "TAKEOFF", "SEARCH", "RETURN", "LAND"]) {
    t.execution = {task:'mission',state:'Running',stage:phase};
    assert.equal(startAllowed(t, true, "mission"), false);
  }
  delete t.execution;
  t.runtime_health.fresh=false;
  assert.equal(startAllowed(t, true, "mission"), false);
  assert.equal(freshFcu(null), false);
  assert.equal(freshFcu({}), false);
});
test("mechanism bench tasks do not require flight readiness and never implicitly arm", () => {
  const t = ready();
  t.safety.ready = false;
  t.mission.mode = "STABILIZE";
  assert.equal(startAllowed(t, true, "mechanism"), false);
  t.devices={winch_actuator:{ok:true}};
  assert.equal(startAllowed(t, true, "mechanism"), true);
  t.mission.armed = true;
  assert.equal(startAllowed(t, true, "mechanism"), false);
  assert.equal(taskCommand("mechanism", "lower").cmd, "start");
});
test("acknowledgement cannot claim physical completion", () => {
  assert.equal(
    acknowledgement({ cmd_id: "id", cmd: "winch", result: "accepted" }).state,
    "Accepted",
  );
  assert.equal(
    acknowledgement({ cmd_id: "id", cmd: "winch", result: "rejected" }).state,
    "Failed",
  );
  assert.equal(
    acknowledgement({ cmd_id: "id", cmd: "winch", result: "completed" }).state,
    "Outcome unknown",
  );
});

test("review confirmation rechecks arming, takeoff, disarm and mechanism conditions", () => {
  const t = ready();
  assert.equal(reviewAllowed(t, true, "takeoff", { alt: NaN }), false);
  assert.equal(reviewAllowed(t, true, "takeoff", { alt: 5 }), true);
  t.mission.armed = true;
  assert.equal(reviewAllowed(t, true, "winch", { action: "release" }), false);
  assert.equal(reviewAllowed(t, true, "disarm", {}), false);
  t.safety.on_ground = true;
  t.sample_age_s.extended_state = 0.1;
  assert.equal(reviewAllowed(t, true, "disarm", {}), true);
  t.sample_age_s.state = 4;
  assert.equal(reviewAllowed(t, true, "arm", {}), false);
  assert.equal(reviewAllowed(t, true, "abort", {}), true);
  assert.equal(reviewAllowed(null, true, "abort", {}), true);
});
test("changed addresses are shared by telemetry and video with no embedded credentials", () => {
  assert.equal(
    endpoint(" ws://192.168.0.101:8765/ "),
    "ws://192.168.0.101:8765",
  );
  assert.equal(
    videoUrl("ws://192.168.0.100:8765", "/percep/overlay"),
    "/video/stream?topic=%2Fpercep%2Foverlay&type=mjpeg",
  );
  assert.throws(() => endpoint("http://localhost:8899"));
  assert.throws(() => endpoint("ws://user:password@localhost:8765"));
});
test("all twenty-one production arena dimensions and built-in presets validate offline", () => {
  // Twenty plus the clarified exit-corridor obstacle reach.
  assert.equal(schema.fields.length, 21);
  assert.ok(schema.fields.some((f) => f.group === "obstacle" && f.key === "reach_frac"));
  assert.deepEqual(validateDraft(schema.defaults, schema.fields), []);
  for (const preset of Object.values(schema.presets))
    assert.deepEqual(validateDraft(preset, schema.fields), []);
  const d = structuredClone(schema.defaults);
  d.corridor.nav_height_m = d.corridor.wall_height_m;
  assert.ok(validateDraft(d, schema.fields).some((s) => s.includes("walls")));
  d.payload.mass_kg = "";
  assert.ok(
    validateDraft(d, schema.fields).some((s) => s.includes("Payload mass")),
  );
  d.banner.border_m = 1;
  assert.ok(validateDraft(d, schema.fields).some((s) => s.includes("panel")));
});
