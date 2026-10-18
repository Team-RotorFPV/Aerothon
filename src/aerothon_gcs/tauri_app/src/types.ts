// Browser contract for gcs_aggregator. Arena fields are exported directly
// from mission_bt; compatibility is checked by the GCS regression tests.

// One line of the Q27 arming interlock (gcs_aggregator/readiness.py).
export interface ReadinessItem {
  key: string;
  label: string;
  ok: boolean;
  value: unknown;
  reason: string;
}

export interface Telemetry {
  banner_colours?: Record<string,string>;
  geofence_vertices?: [number, number][];
  execution?: {task:string; state:string; stage?:string; control_owner?:string};
  runtime_health?: {fresh:boolean};
  sensor_diagnostics?: {
    vibration_mps2: (number | null)[] | null;
    clipping: (number | null)[] | null;
    magnetic_ut: (number | null)[] | null;
    magnetic_strength_ut: number | null;
    magnetic_units_basis?: string | null;
    magnetic_frame: string | null;
    magnetic_capture_stamp: number | null;
  };
  devices?: {
    [key:string]: DeviceMetric | undefined;
    camera?: DeviceMetric;
    lidar?: DeviceMetric;
    clock?: { ok: boolean; reason?: string };
  };
  winch?: {
    payout_estimate_m?: number | null;
    measured_payout_m?: number | null;
    measurement_age_s?: number | null;
    state?: string;
    release_requested?: boolean;
  };
  detector_rates?: Record<
    string,
    {
      hz: number | null;
      capture_stamp: number;
      processed_stamp: number | null;
      received: number;
    }
  >;
  sample_age_s?: Record<string, number>;
  mission: {
    selected: string | null;
    state: string;
    armed: boolean;
    mode: string;
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
    x: number;
    y: number;
    alt: number;
    gs?: number;
    roll_deg?: number;
    pitch_deg?: number;
    yaw_deg?: number;
  };
  gps: { lat: number; lon: number; sats: number; fix?: string };
  power: { volt: number; pct: number; current_a?: number | null };
  nav: { front_m: number; centering_err: number; cmd_vx: number };
  percep: {
    start_qr: string;
    target_match: boolean;
    mission_target?: string;
    qr_reference?: boolean;
    qr_match_basis?: string[];
    qr_appearance_score?: number | null;
    banner: boolean;
    redzone_visible: boolean;
    // Tri-state. "CLEAR" means the camera looked at the ground and saw none;
    // "NOT_VISIBLE" means it could not see the ground at all. The old boolean
    // reported both as safe.
    redzone_status?: "UNKNOWN" | "NOT_VISIBLE" | "CLEAR" | "RED";
    redzone_reason?: string;
    redzone_exclusions?: number[][];
    redzone_area_m2?: number;
    // Square on: perpendicular to the banner's FACE, measured by the lidar.
    // Distinct from `banner`, which only says the nose points at it. null
    // means not measured -- never render it as zero.
    square_ok?: boolean | null;
    square_angle_deg?: number | null;
    square_standoff_m?: number | null;
    square_reason?: string;
  };
  safety: {
    ready: boolean;
    on_ground?: boolean;
    fcu_connected?: boolean;
    ekf?: boolean;
    geofence?: string;
    ready_items?: ReadinessItem[];
    ready_reasons?: string[];
    ready_waived?: string[];
  };
  checklist: Record<string, boolean>;
  // Arena dimensions (mission_bt/arena_dimensions.py): the field list, the
  // presets, and what mission_bt reports it actually applied.
  dimension_fields?: DimensionField[];
  dimension_presets?: { builtin: Record<string, Dimensions>; saved: Record<string, Dimensions> };
  arena_dimensions?: {
    sent: Dimensions | null; applied: Dimensions | null;
    params?: Record<string, unknown> | null; ok?: boolean; reason: string;
  } | null;
  // Lidar calibration mode (gcs_aggregator/lidar_calibration.py): bearings
  // in degrees CCW from the nose (corrected) or the lidar's own 0 (raw).
  lidar_cal?: {
    raw_fresh: boolean;
    raw_nearest: [number, number] | null;
    body_nearest: [number, number] | null;
    geometry?: Record<string, number> | null;
    background_ready?: boolean;
    samples?: number;
    capture_details?: Record<string, {samples:number; beams:number; spread_m:number; spread_deg:number}>;
    scan?: import('./lidarGeometry').Scan;
    captures: { ahead?: [number, number] | null; port?: [number, number] | null;
                far?: [number, number] | null; starboard?: [number, number] | null };
    result: { yaw_deg: number; mirrored: boolean; error_deg: number;
              validation_error_m?:number; range_scale?:number; range_bias_m?:number;
              ok: boolean; reason: string } | null;
    applied: { yaw_deg: number; mirrored: boolean; source: string;
               reason: string; geometry_measured?:boolean } | null;
  };
  slam_status?: {source?:string;state?:string;reason?:string};
  scan?: { yaw_deg: number; ranges: (number | null)[];
           angle_min?: number; angle_step?: number; range_min?:number;range_max?:number;
           frame?:string;stamp?:number };
  scans?: ScanRow[];
  gimbal?: { pitch_deg: number };
}

export interface DeviceMetric {
  ok: boolean;
  reason?:string;
  hz?: number | null; // Gateway receipt rate, measured in steady time.
  receipt_age_s?: number | null;
  capture_age_s?: number | null; // Vehicle clock minus capture timestamp.
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
  kind: "telemetry" | "event" | "ack" | "map" | "execution" | "receipt";
  t: number;
  data: any;
}

// group -> field -> value. Lengths in metres; the two choice fields are strings.
export type Dimensions = Record<string, Record<string, number | string>>;

export interface DimensionField {
  group: string; key: string; label: string;
  default: number; min: number; max: number;
  // What the algorithms do with it; null means recorded only.
  used_by: string | null;
}

