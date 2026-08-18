// Mirrors the aggregator schema (docs/ARCHITECTURE.md §7.1).
// In production, GENERATE this from the Rust structs with ts-rs so it never drifts.

// One line of the Q27 arming interlock (gcs_aggregator/readiness.py).
export interface ReadinessItem {
  key: string;
  label: string;
  ok: boolean;
  value: unknown;
  reason: string;
}

export interface Telemetry {
  mission: {
    selected: string | null; state: string; armed: boolean; mode: string;
    elapsed?: number;
    // Delivery accuracy is 15 rulebook marks and was measured nowhere; it is
    // carried as a NUMBER so the panel can compare and threshold it, not just
    // print a sentence.
    delivery_offset_m?: number | null;
    landing_precision?: string | null;
    result?: string | null;
    result_reason?: string | null;
  };
  flight: {
    x: number; y: number; alt: number;
    gs?: number; roll_deg?: number; pitch_deg?: number; yaw_deg?: number;
  };
  gps: { lat: number; lon: number; sats: number; fix?: string };
  power: { volt: number; pct: number };
  nav: { front_m: number; centering_err: number; cmd_vx: number };
  percep: {
    start_qr: string; target_match: boolean; banner: boolean;
    redzone_visible: boolean;
    // Tri-state. "CLEAR" means the camera looked at the ground and saw none;
    // "NOT_VISIBLE" means it could not see the ground at all. The old boolean
    // reported both as safe.
    redzone_status?: "UNKNOWN" | "NOT_VISIBLE" | "CLEAR" | "RED";
    redzone_reason?: string;
    redzone_exclusions?: number[][];
    redzone_area_m2?: number;
  };
  safety: {
    ready: boolean; fcu_connected?: boolean; ekf?: boolean; geofence?: string;
    ready_items?: ReadinessItem[];
    ready_reasons?: string[];
    ready_waived?: string[];
  };
  checklist: Record<string, boolean>;
  scan?: { yaw_deg: number; ranges: (number | null)[] };
  scans?: ScanRow[];
  gimbal?: { pitch_deg: number };
}

// One observation the aircraft made: a marker it decoded, a banner it read, or
// something it looked at and REFUSED. De-duplicated by the aggregator, so a
// marker held in frame for 200 frames is one row with count 200 -- the count
// separates a solid read from a single-frame blip.
export interface ScanRow {
  key: string;
  seq: number;
  kind: "qr" | "banner";
  // What was read. Empty on a rejection, where `reason` carries the detail.
  payload: string;
  // Decoding is not matching. Only a payload equal to the delivery target
  // named by the start QR is a match, and only a match is tagged.
  matched: boolean;
  status: "DECODED" | "MATCHED" | "IDENTIFIED" | "REJECTED";
  reason: string;
  // Which lettering path read it: "brightness" or "stroke". A rescued read
  // should look different from a clean one.
  via: string;
  stage: string;
  t: number;
  t_last: number;
  count: number;
}

export interface Envelope {
  v: number;
  kind: "telemetry" | "event" | "ack" | "map";
  t: number;
  data: any;
}
