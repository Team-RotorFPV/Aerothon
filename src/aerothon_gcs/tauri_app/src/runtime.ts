import type { Telemetry } from "./types";

export interface ConnectionProfile {
  id:string; name:string; transport:'tcp'|'udp'|'serial'; host:string; port:number; bind_port:number;
  system_id:number; component_id:number; gateway:string; device:string; baud:number;
  camera_device:string; lidar_port:string; lidar_yaw_deg:number; lidar_mirrored:boolean;
  camera_hfov:number; marker_m:number; rc_config:Record<string,unknown>; slam:boolean;
  hardware_reviewed:boolean; delivery_calibration:string;
  vehicle?:'simulation'|'drone'; winch_backend?:'mavlink'|'gpio'; gcs_host?:string;
  autoconnect?:boolean; slam_odometry?:'lidar'|'fcu';
}
export const vehicleLabel = (p?: ConnectionProfile | null) =>
  !p ? 'No vehicle selected'
    : (p.vehicle ?? (p.transport === 'serial' ? 'drone' : 'simulation')) === 'drone'
      ? 'Real drone' : 'Laptop simulation';
export interface HostState {profiles:ConnectionProfile[]; profile_id:string|null; runtime:'Stopped'|'Running'; can_disconnect:boolean; can_edit_profiles:boolean;connection_error?:string;autoconnect_suspended?:boolean;}
export class ApiError extends Error { status:number; constructor(status:number, message:string) { super(message); this.status=status; } }
export async function api(path:string, data?:unknown):Promise<any> {
  const r=await fetch(path, data === undefined ? undefined : {method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(data)});
  // A proxy or crash can answer with plain text; show it, not a JSON parse error.
  const text=await r.text(); let body:any;
  try { body=text ? JSON.parse(text) : {}; } catch { body={error:text.slice(0,300)}; }
  if(!r.ok) throw new ApiError(r.status, body.error || `Request failed (${r.status})`); return body;
}

export function commandId(provider: {
  randomUUID?: () => string;
  getRandomValues: (bytes: Uint8Array) => Uint8Array;
} = globalThis.crypto): string {
  if (typeof provider?.randomUUID === 'function') return provider.randomUUID();
  if (typeof provider?.getRandomValues !== 'function') throw new Error('This browser cannot generate safe command IDs');
  // getRandomValues is available on HTTP intranet pages; randomUUID requires
  // a secure context. Keep the same UUID protocol without a weak fallback.
  const bytes = new Uint8Array(16);
  provider.getRandomValues(bytes);
  bytes[6] = (bytes[6] & 15) | 64;
  bytes[8] = (bytes[8] & 63) | 128;
  const hex = Array.from(bytes, byte => byte.toString(16).padStart(2, '0')).join('');
  return `${hex.slice(0,8)}-${hex.slice(8,12)}-${hex.slice(12,16)}-${hex.slice(16,20)}-${hex.slice(20)}`;
}
export type CommandState =
  | "Submitted"
  | "Accepted"
  | "Running"
  | "Completed"
  | "Failed"
  | "Outcome unknown";
export interface CommandOutcome {
  command_id: string;
  cmd: string;
  state: CommandState;
  reason: string;
}
export const tasks = [
  {id:'mission',label:'Full mission',movement:true,description:'Complete production workflow: takeoff, detection, delivery, return and disarm.'},
  {id:'slam',label:'SLAM mapping',movement:false,description:'Map using the production lidar while the transmitter controls flight. Enable SLAM in the connection profile.'},
  {id:'qr',label:'QR detection',movement:false,description:'Run production QR detection using the current camera pose. No automatic arming or camera movement.'},
  {id:'banner',label:'Banner alignment',movement:true,description:'Take off and align using the production banner behaviour, then return and land.'},
  {id:'avoidance_manual',label:'Avoidance with transmitter',movement:true,description:'Use configured RC channels as pilot intent through the production obstacle controller.'},
  {id:'avoidance_route',label:'Avoidance with GPS route',movement:true,description:'Follow GPS waypoints through the production obstacle controller, then return and land.'},
  {id:'mechanism',label:'Mechanism only',movement:false,description:'Disarmed bench lower, release, stow or stop. Measured device feedback is required.'},
  {id:'delivery_here',label:'Payload delivery here',movement:true,description:'Take off at the current location, centre on the specified QR and run production delivery.'},
  {id:'delivery_gps',label:'Payload delivery at GPS',movement:true,description:'Fly to one GPS location and height, verify the specified QR and deliver through production behaviours.'},
] as const;

export function endpoint(value: string): string {
  const u = new URL(value.trim());
  if (
    !["ws:", "wss:"].includes(u.protocol) ||
    !u.hostname ||
    u.username ||
    u.password ||
    u.hash
  )
    throw new Error(
      "Use a WebSocket URL without credentials, such as ws://192.168.0.101:8765",
    );
  return u.toString().replace(/\/$/, "");
}
export function defaultProfile(host = ''): ConnectionProfile {
  return {id:'vehicle',name:'Vehicle',transport:'tcp',host,port:5762,bind_port:14555,system_id:1,component_id:1,
    gateway:host ? `http://${host}:8090` : '',device:'/dev/ttyAMA0',baud:115200,camera_device:'/dev/video0',lidar_port:'/dev/ttyUSB0',
    lidar_yaw_deg:0,lidar_mirrored:false,camera_hfov:.851919,marker_m:1.0,rc_config:{},slam:false,hardware_reviewed:false,delivery_calibration:'',vehicle:'simulation'};
}
export function videoUrl(_value:string, topic:string):string {
  return `/video/stream?topic=${encodeURIComponent(topic)}&type=mjpeg`;
}

export function age(t: Telemetry | null, source: string): number {
  const n = t?.sample_age_s?.[source];
  return typeof n === "number" && Number.isFinite(n) && n >= 0 ? n : Infinity;
}
export function freshFcu(t: Telemetry | null): boolean {
  return t?.safety?.fcu_connected === true && age(t, "state") < 3;
}
export function missionActive(t: Telemetry | null): boolean {
  const phase = t?.mission?.state?.toUpperCase();
  return (
    !!phase &&
    !["IDLE", "WAITING", "COMPLETED", "FAILED", "RESET"].includes(phase)
  );
}
export function startAllowed(t:Telemetry|null, connected:boolean, task:string):boolean {
  if(!connected || t?.runtime_health?.fresh !== true || (t.execution && ['Submitted','Accepted','Running'].includes(t.execution.state))) return false;
  if(task === 'qr' || task === 'slam') return t.devices?.[task === 'qr' ? 'camera' : 'lidar']?.ok === true;
  if(!freshFcu(t)) return false;
  if(task === 'mechanism') return t.mission.armed === false && t.devices?.winch_actuator?.ok === true;
  return age(t,'readiness') < 3 && t.safety.ready === true;
}
export function taskCommand(task:string, action:string, params:Record<string,unknown> = {}):{cmd:string;args:Record<string,unknown>} {
  if(!tasks.some(t=>t.id===task)) throw new Error('Unknown production task');
  if(task==='mechanism' && !['lower','release','stow','stop'].includes(action)) throw new Error('Select a mechanism action');
  return {cmd:'start',args:{task,params:task==='mechanism' ? {action} : params}};
}

export function reviewAllowed(
  t: Telemetry | null,
  connected: boolean,
  cmd: string,
  args: Record<string, unknown>,
): boolean {
  if (!connected) return false;
  if (
    ["abort", "land", "stop"].includes(cmd) ||
    (cmd === "set_mode" && args.mode === "RTL")
  )
    return true;
  if (cmd === "start") return startAllowed(t, connected, String(args.task));
  if (!freshFcu(t)) return false;
  if (["start_mission", "start_test"].includes(cmd))
    return startAllowed(t, connected, "mission");
  if (["arm", "takeoff"].includes(cmd))
    return (
      startAllowed(t, connected, "mission") &&
      (cmd !== "takeoff" ||
        (typeof args.alt === "number" &&
          Number.isFinite(args.alt) &&
          args.alt >= 1 &&
          args.alt <= 10))
    );
  if (cmd === "disarm")
    return (
      t?.mission.armed === true &&
      t.safety.on_ground === true &&
      age(t, "extended_state") < 3
    );
  if (
    [
      "lidar_cal_apply",
      "set_banner_colours",
      "set_geofence",
      "set_delivery_zone",
    ].includes(cmd)
  )
    return t?.mission.armed === false && !missionActive(t);
  if (cmd === "winch") return startAllowed(t, connected, "mechanism");
  return !missionActive(t);
}
export function commandEnvelope(
  id: string,
  cmd: string,
  args: unknown,
  now = Date.now(),
): string {
  return JSON.stringify({
    v: 1,
    kind: "command",
    t: now / 1000,
    data: { cmd_id: id, cmd, args, confirm: true },
  });
}
export function acknowledgement(data: {
  cmd_id: string;
  cmd: string;
  result: string;
  reason?: string;
}): CommandOutcome {
  return {
    command_id: data.cmd_id,
    cmd: data.cmd,
    state:
      data.result === "accepted"
        ? "Accepted"
        : data.result === "rejected"
          ? "Failed"
          : "Outcome unknown",
    reason:
      data.reason ||
      (data.result === "accepted"
        ? "Request accepted by the runtime; physical outcome is not confirmed."
        : ""),
  };
}
