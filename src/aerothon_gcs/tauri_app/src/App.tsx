import { lazy, Suspense, useEffect, useRef, useState } from "react";
import maplibregl from "maplibre-gl";
import "maplibre-gl/dist/maplibre-gl.css";
import type { Telemetry } from "./types";
import { TelemetryHistory } from "./telemetryHistory";
import {
  acknowledgement,
  commandId,
  age,
  freshFcu,
  missionActive,
  startAllowed,
  reviewAllowed,
  taskCommand,
  tasks,
  videoUrl,
  type CommandOutcome,
  api,
  ApiError,
  type HostState,
  vehicleLabel,
} from "./runtime";
import Connections from "./Connections";
import ArenaDimensions from "./ArenaDimensions";
import LidarCalibration from "./LidarCalibration";
import Slam, { type Grid } from "./LidarView";
import { decodeGrid } from "./lidarGeometry";
import coverage from "./satelliteCoverage.json";
import { satelliteStyle } from "./mapSources";
import "./gcs.css";
const Graphs = lazy(() => import("./Graphs"));

const value = (n: unknown, unit = "", precision = 1) =>
  typeof n === "number" && Number.isFinite(n)
    ? `${n.toFixed(precision)}${unit}`
    : "Unavailable";
// Readiness values are numbers, text or, for detectors, {name: age seconds}.
const readinessValue = (v: unknown): string =>
  v && typeof v === "object"
    ? Object.entries(v as Record<string, unknown>)
        .map(([name, age]) => `${name} ${value(age, " s")}`)
        .join(", ")
    : String(v ?? "Unavailable");
// The fence as GeoJSON for the map: [lat, lon] pairs, closed ring.
const EMPTY = { type: "FeatureCollection", features: [] } as const;
const fenceGeo = (pts: [number, number][]) =>
  pts.length < 3
    ? EMPTY
    : {
        type: "FeatureCollection",
        features: [
          {
            type: "Feature",
            properties: {},
            geometry: {
              type: "Polygon",
              coordinates: [
                [...pts, pts[0]].map(([lat, lon]) => [lon, lat]),
              ],
            },
          },
        ],
      };
const boundary = (text: string) =>
  text
    .trim()
    .split("\n")
    .filter(Boolean)
    .map((line) => {
      const [lat, lon, ...extra] = line.split(",").map(Number);
      if (extra.length || !Number.isFinite(lat) || !Number.isFinite(lon))
        throw new Error("Enter latitude, longitude on each line");
      return { lat, lon };
    });
export default function App() {
  const [mode, setMode] = useState<"flight" | "testing">("flight"),
    [theme, setTheme] = useState(localStorage.getItem("gcs-theme") || "light");
  const [view, setView] = useState<"camera" | "slam" | "map" | "graphs">("map");
  const activeEndpoint = `${location.protocol === 'https:' ? 'wss' : 'ws'}://${location.host}/ws`;
  const [hostState, setHostState] = useState<HostState | null>(null);
  const [waypoints, setWaypoints] = useState('');
  async function refreshHost() {
    try { setHostState(await api('/api/state')); } catch { setHostState(null); }
  }
  useEffect(() => { void refreshHost(); const timer = window.setInterval(() => void refreshHost(), 2000); return () => window.clearInterval(timer); }, []);
  const [linked, setLinked] = useState(false),
    [received, setReceived] = useState(0),
    [clock, setClock] = useState(Date.now()),
    [telemetry, setTelemetry] = useState<Telemetry | null>(null);
  const [events, setEvents] = useState<string[]>([]),
    [error, setError] = useState("");
  const [settings, setSettings] = useState(false),
    [arenaOpen, setArenaOpen] = useState(false),
    [lidarOpen, setLidarOpen] = useState(false);
  const [task, setTask] = useState("mission"),
    [action, setAction] = useState("stow"),
    [height, setHeight] = useState(5);
  const [review, setReview] = useState<{
      cmd: string;
      args: Record<string, unknown>;
      label: string;
    } | null>(null),
    [busy, setBusy] = useState(false),
    [starting, setStarting] = useState(false);
  const [outcome, setOutcome] = useState<CommandOutcome | null>(null);
  const [target, setTarget] = useState(""),
    [fenceText, setFenceText] = useState(""),
    [mapReady, setMapReady] = useState(false);
  const [colours, setColours] = useState({
    green: "",
    orange: "",
    red: "",
  });
  const [topic, setTopic] = useState("/percep/overlay"),
    [videoError, setVideoError] = useState(false);
  const [grid, setGrid] = useState<Grid | null>(null),
    [gridReceived, setGridReceived] = useState(0),
    [onlineMap, setOnlineMap] = useState(false),
    [mapError, setMapError] = useState(false);
  const history = useRef(new TelemetryHistory()),
    socket = useRef<WebSocket | null>(null),
    mapHost = useRef<HTMLDivElement>(null),
    map = useRef<maplibregl.Map | null>(null),
    marker = useRef<maplibregl.Marker | null>(null),
    centered = useRef(false);
  const pending = useRef(
    new Map<
      string,
      { timer: number; resolve: (result: CommandOutcome) => void; cmd: string }
    >(),
  );
  const fresh = linked && received > 0 && clock - received < 3000,
    t = fresh ? telemetry : null,
    fcu = freshFcu(t),
    running = missionActive(t) || starting;
  const selectedTask = mode === "flight" ? "mission" : task,
    taskInfo = tasks.find((item) => item.id === selectedTask)!;
  const canStart = !busy && !starting && startAllowed(t, fresh, selectedTask);
  const record = (text: string) =>
    setEvents((list) => [
      ...list.slice(-149),
      `${new Date().toLocaleTimeString()} ${text}`,
    ]);
  useEffect(() => {
    document.documentElement.dataset.theme = theme;
    localStorage.setItem("gcs-theme", theme);
  }, [theme]);
  useEffect(() => {
    const timer = window.setInterval(() => {
      setClock(Date.now());
      history.current.tick(performance.now(), Date.now());
    }, 500);
    return () => window.clearInterval(timer);
  }, []);
  useEffect(() => {
    let disposed = false,
      retry = 0;
    const open = () => {
      const streamUrl = new URL(activeEndpoint);
      streamUrl.searchParams.set('map_encoding', 'rle');
      const ws = new WebSocket(streamUrl);
      socket.current = ws;
      ws.onopen = () => {
        if (disposed || socket.current !== ws) return;
        setLinked(true);
        record(`Connected to ${activeEndpoint}`);
      };
      ws.onmessage = (event) => {
        if (disposed || socket.current !== ws) return;
        try {
          const envelope = JSON.parse(event.data);
          if (envelope.kind === "telemetry") {
            history.current.ingest(
              envelope.data,
              envelope.t,
              performance.now(),
              Date.now(),
            );
            setTelemetry(envelope.data);
            setReceived(Date.now());
            if (
              age(envelope.data, "mission") < 3 &&
              envelope.data.mission?.state !== "STARTING"
            )
              setStarting(false);
          } else if (envelope.kind === "execution" || envelope.kind === "receipt") {
            setOutcome({ ...envelope.data, cmd: "execution" });
            setStarting(false);
          } else if (envelope.kind === "map") {
            setGrid(decodeGrid(envelope.data));
            setGridReceived(Date.now());
          } else if (envelope.kind === "event")
            record(envelope.data.msg || "Runtime event");
          else if (envelope.kind === "ack") {
            const result = acknowledgement(envelope.data),
              entry = pending.current.get(result.command_id);
            if (entry) {
              window.clearTimeout(entry.timer);
              pending.current.delete(result.command_id);
              entry.resolve(result);
            }
            setOutcome(result);
            record(
              `${result.cmd}: ${result.state}${result.reason ? `, ${result.reason}` : ""}`,
            );
          }
        } catch {
          setError("The runtime sent an invalid message.");
        }
      };
      ws.onclose = () => {
        if (disposed || socket.current !== ws) return;
        setLinked(false);
        history.current.disconnect();
        for (const [id, entry] of pending.current) {
          window.clearTimeout(entry.timer);
          const result: CommandOutcome = {
            command_id: id,
            cmd: entry.cmd,
            state: "Outcome unknown",
            reason: "Connection lost; this command will not be replayed.",
          };
          entry.resolve(result);
          setOutcome(result);
        }
        pending.current.clear();
        if (!disposed) retry = window.setTimeout(open, 1500);
      };
    };
    open();
    return () => {
      disposed = true;
      window.clearTimeout(retry);
      socket.current?.close();
      for (const [id, entry] of pending.current) {
        window.clearTimeout(entry.timer);
        entry.resolve({
          command_id: id,
          cmd: entry.cmd,
          state: "Outcome unknown",
          reason: "Connection changed; no retry was sent.",
        });
      }
      pending.current.clear();
    };
  }, [activeEndpoint]);
  async function send(
    cmd: string,
    args: Record<string, unknown> = {},
  ): Promise<CommandOutcome> {
    const ws = socket.current;
    if (!ws || ws.readyState !== WebSocket.OPEN)
      throw new Error("Production runtime is disconnected.");
    const id = commandId(),
      submitted: CommandOutcome = {
        command_id: id,
        cmd,
        state: "Submitted",
        reason: "Awaiting runtime acknowledgement.",
      };
    setOutcome(submitted);
    record(`${cmd}: Submitted`);
    try {
      const result = await api('/api/commands', {command_id:id, cmd, args, confirm:true});
      setOutcome({...result, cmd});
      return {...result, cmd};
    } catch (e) {
      const result: CommandOutcome = {...submitted, state: e instanceof ApiError ? 'Failed' : 'Outcome unknown', reason: e instanceof Error ? e.message : String(e)};
      setOutcome(result);
      return result;
    }
  }

  async function perform(cmd: string, args: Record<string, unknown> = {}) {
    setBusy(true);
    setError("");
    try {
      const result = await send(cmd, args);
      if (result.state === "Failed")
        throw new Error(result.reason || "Runtime rejected the request");
      if (cmd === "start") setStarting(true);
      if (cmd === "abort" && result.state === "Accepted") setStarting(false);
    } catch (reason) {
      setError(reason instanceof Error ? reason.message : String(reason));
    } finally {
      setBusy(false);
    }
  }
  const reviewedSend = (
    cmd: string,
    args: Record<string, unknown> = {},
    danger = false,
  ) => {
    if (danger) {
      setArenaOpen(false);
      setLidarOpen(false);
      setReview({ cmd, args, label: `Review ${cmd.replace(/_/g, " ")}` });
    } else void perform(cmd, args);
  };
  useEffect(() => {
    if (view !== "map" || !mapHost.current) return;
    const satellite = satelliteStyle(coverage, onlineMap, location.origin);
    const m = new maplibregl.Map({
      container: mapHost.current,
      center: coverage.initial.center as [number, number],
      zoom: coverage.initial.zoom,
      maxZoom: 19,
      style: {
        version: 8,
        sources: satellite.sources,
        layers: [
          {
            id: "background",
            type: "background",
            paint: {
              "background-color": theme === "dark" ? "#252627" : "#ecece8",
            },
          },
          ...(satellite.layers as maplibregl.LayerSpecification[]),
        ],
      },
    });
    map.current = m;
    // Diagnostic handle for browser checks of the map's sources and layers.
    (mapHost.current as any).aerothonMap = m;
    marker.current = new maplibregl.Marker({
      color: theme === "dark" ? "#eee" : "#242424",
    });
    centered.current = false;
    setMapError(false);
    m.addControl(new maplibregl.NavigationControl(), "top-right");
    m.on("error", () => setMapError(true));
    setMapReady(false);
    // The judges' geofence: applied (filled) and the typed draft (dashed).
    // Added once the style is parsed ("style.load"): "load" and
    // isStyleLoaded() also wait for every tile, which may never finish.
    const addFence = () => {
      if (m.getSource("fence-applied")) return;
      try {
        m.addSource("fence-applied", { type: "geojson", data: EMPTY as any });
        m.addSource("fence-draft", { type: "geojson", data: EMPTY as any });
        m.addLayer({ id: "fence-applied-fill", type: "fill", source: "fence-applied",
          paint: { "fill-color": "#e8a33d", "fill-opacity": 0.15 } });
        m.addLayer({ id: "fence-applied-line", type: "line", source: "fence-applied",
          paint: { "line-color": "#e8a33d", "line-width": 2.5 } });
        m.addLayer({ id: "fence-draft-line", type: "line", source: "fence-draft",
          paint: { "line-color": "#3d8be8", "line-width": 2, "line-dasharray": [2, 2] } });
        setMapReady(true);
      } catch {
        /* Style not parsed yet; the next style event adds them. */
      }
    };
    // An inline style can finish parsing inside the constructor, before
    // any listener exists, so try now as well as on every style event.
    addFence();
    m.on("style.load", addFence);
    m.on("styledata", addFence);
    m.on("load", addFence);
    return () => {
      marker.current?.remove();
      m.remove();
      map.current = null;
    };
  }, [view, onlineMap, theme]);
  // Draw the fence: the applied polygon and whatever is typed in the box.
  let draftFence: [number, number][] = [];
  let draftError = "";
  try {
    draftFence = fenceText.trim()
      ? boundary(fenceText).map((p) => [p.lat, p.lon] as [number, number])
      : [];
  } catch (reason) {
    draftError = String(reason).replace(/^Error: /, "");
  }
  const draftKey = JSON.stringify(draftFence);
  const appliedFence = t?.geofence_vertices ?? [];
  const appliedKey = JSON.stringify(appliedFence);
  useEffect(() => {
    const m = map.current;
    if (!m || !mapReady) return;
    (m.getSource("fence-applied") as maplibregl.GeoJSONSource | undefined)
      ?.setData(fenceGeo(appliedFence) as any);
    (m.getSource("fence-draft") as maplibregl.GeoJSONSource | undefined)
      ?.setData(fenceGeo(draftFence) as any);
    if (mapHost.current) {
      mapHost.current.dataset.fenceDraft = String(draftFence.length);
      mapHost.current.dataset.fenceApplied = String(appliedFence.length);
    }
    const show = draftFence.length >= 3 ? draftFence : appliedFence;
    if (show.length >= 3) {
      const lons = show.map((p) => p[1]), lats = show.map((p) => p[0]);
      m.fitBounds([[Math.min(...lons), Math.min(...lats)],
                   [Math.max(...lons), Math.max(...lats)]],
                  { padding: 40, maxZoom: 19, duration: 0 });
      centered.current = true;
    }
  }, [mapReady, draftKey, appliedKey]);
  useEffect(() => {
    const gps = t?.gps;
    if (
      view !== "map" ||
      !map.current ||
      !gps ||
      age(t, "gps") >= 3 ||
      gps.fix === "NO" ||
      !Number.isFinite(gps.lat) ||
      !Number.isFinite(gps.lon)
    ) {
      marker.current?.remove();
      return;
    }
    const ll: [number, number] = [gps.lon, gps.lat];
    marker.current?.setLngLat(ll).addTo(map.current);
    if (!centered.current) {
      map.current.jumpTo({ center: ll, zoom: 17 });
      centered.current = true;
    }
  }, [t, view]);
  return (
    <div className="app">
      <header>
        <div className="wordmark">
          AeroTHON <span>GCS</span>
        </div>
        <nav aria-label="Operating mode">
          <button
            aria-pressed={mode === "flight"}
            onClick={() => {
              setMode("flight");
              setReview(null);
            }}
          >
            Flight operations
          </button>
          <button
            aria-pressed={mode === "testing"}
            onClick={() => {
              setMode("testing");
              setReview(null);
            }}
          >
            Testing
          </button>
        </nav>
        <div className="header-end">
          <span className="vehicle-kind">
            {(() => {
              const active = hostState?.profiles.find((x) => x.id === hostState?.profile_id);
              return active ? `${active.name} · ${vehicleLabel(active)}` : vehicleLabel(null);
            })()}
          </span>
          <button onClick={() => setArenaOpen(true)}>Arena</button>
          <button onClick={() => setLidarOpen(true)}>Lidar calibration</button>
          <button onClick={() => setSettings(!settings)}>Connections</button>
          <button
            aria-label="Switch theme"
            onClick={() => setTheme(theme === "dark" ? "light" : "dark")}
          >
            {theme === "dark" ? "Light" : "Dark"}
          </button>
        </div>
      </header>
      <section className="status-strip" aria-label="Aircraft status">
        <Datum
          label="Connection"
          text={
            fcu
              ? "Live FCU"
              : fresh
                ? "FCU unavailable"
                : linked
                  ? "Waiting for telemetry"
                  : "Disconnected"
          }
        />
        <Datum
          label="Aircraft mode"
          text={fcu ? t?.mission.mode || "Unavailable" : "Unavailable"}
        />
        <Datum
          label="Control owner"
          text={
            fcu
              ? running
                ? t?.execution?.control_owner || "Production runtime"
                : "Transmitter / FCU"
              : "Unavailable"
          }
        />
        <Datum
          label="Armed"
          text={
            fcu && typeof t?.mission.armed === "boolean"
              ? t.mission.armed
                ? "Yes"
                : "No"
              : "Unavailable"
          }
        />
        <Datum
          label="Battery"
          text={value(age(t, "power") < 3 ? t?.power.pct : null, "%", 0)}
        />
        <Datum
          label="GPS satellites"
          text={value(age(t, "sats") < 3 ? t?.gps.sats : null, "", 0)}
        />
        <Datum
          label="Height"
          text={value(age(t, "pose") < 3 ? t?.flight.alt : null, " m")}
        />
        <Datum label="Execution" text={t?.execution ? `${t.execution.task}: ${t.execution.state} ${t.execution.stage || ''}` : t?.mission.state || "Unavailable"} />
      </section>
      {error && (
        <p className="error" role="alert">
          {error}
        </p>
      )}
      {settings && <Connections state={hostState} refresh={refreshHost} />}
      <main>
        <section className="workspace">
          <div
            className="view-tabs"
            role="tablist"
            aria-label="Monitoring view"
          >
            {(["camera", "slam", "map", "graphs"] as const).map((tab) => (
              <button
                role="tab"
                key={tab}
                id={`tab-${tab}`}
                aria-selected={view === tab}
                aria-controls={`view-${tab}`}
                onClick={() => setView(tab)}
              >
                {
                  {
                    camera: "Camera",
                    slam: "SLAM",
                    map: "Map",
                    graphs: "Graphs",
                  }[tab]
                }
              </button>
            ))}
          </div>
          <div
            role="tabpanel"
            id={`view-${view}`}
            aria-labelledby={`tab-${view}`}
          >
            {view === "map" && (
              <>
                <div ref={mapHost} className="map-host" />
                <label className="check">
                  <input
                    type="checkbox"
                    checked={onlineMap}
                    onChange={(e) => setOnlineMap(e.target.checked)}
                  />
                  Use online satellite imagery
                </label>
                <p className="view-note">
                  {age(t, "gps") < 3
                    ? `${value(t?.gps.lat, "°", 7)}, ${value(t?.gps.lon, "°", 7)}`
                    : "Cached coverage overview; aircraft position is unavailable."}{" "}
                  {onlineMap
                    ? "Online imagery needs an internet connection."
                    : `Offline imagery: ${coverage.regions.map((r) => `${r.name} (zoom ${r.minzoom}-${r.maxzoom})`).join(", ")}.`}
                </p>
                {mapError && (
                  <p className="view-note">
                    Imagery unavailable for this view. Change zoom or use online
                    imagery; no substitute map is shown.
                  </p>
                )}
              </>
            )}
            {view === "camera" && (
              <>
                <div className="camera-tools">
                  <label>
                    Camera view
                    <select
                      value={topic}
                      onChange={(e) => {
                        setTopic(e.target.value);
                        setVideoError(false);
                      }}
                    >
                      <option value="/percep/overlay">
                        Composite detections
                      </option>
                      <option value="/image_raw">Raw camera</option>
                      <option value="/percep/qr/annotated">QR</option>
                      <option value="/percep/banner/annotated">Banner</option>
                      <option value="/percep/redzone/annotated">
                        Red zone
                      </option>
                    </select>
                  </label>
                </div>
                {fresh && !videoError ? (
                  <img
                    className="camera-feed"
                    src={videoUrl(activeEndpoint, topic)}
                    alt="Live aircraft camera"
                    onError={() => setVideoError(true)}
                  />
                ) : (
                  <div className="empty-view camera-feed">
                    Camera unavailable
                  </div>
                )}
                <p className="view-note">
                  Changing the displayed feed does not move the physical camera.
                  Camera tilt is a separate reviewed command.
                </p>
                <button onClick={() => setVideoError(false)}>
                  Retry video
                </button>
              </>
            )}
            {view === "slam" && (
              <Slam
                status={telemetry?.slam_status}
                grid={grid}
                current={gridReceived > 0 && clock - gridReceived < 3000}
                scan={telemetry?.scan}
                lidarFresh={age(telemetry, "lidar") < 2}
                poseFresh={age(telemetry, "pose") < 2}
                slamEnabled={
                  hostState?.runtime === "Running" &&
                  hostState.profiles.find((x) => x.id === hostState.profile_id)?.slam === true
                }
              />
            )}
            {view === "graphs" && (
              <Suspense fallback={<p>Loading graphs…</p>}>
                <Graphs history={history.current} theme={theme} live={fresh} />
              </Suspense>
            )}
          </div>
          <details>
            <summary>Flight and mission details</summary>
            <div className="details-grid">
              <Datum
                label="Roll / pitch"
                text={
                  age(t, "pose") < 3
                    ? `${value(t?.flight.roll_deg, "°")} / ${value(t?.flight.pitch_deg, "°")}`
                    : "Unavailable"
                }
              />
              <Datum
                label="Heading"
                text={value(age(t, "pose") < 3 ? t?.flight.yaw_deg : null, "°")}
              />
              <Datum
                label="Ground speed"
                text={value(
                  age(t, "velocity") < 3 ? t?.flight.gs : null,
                  " m/s",
                )}
              />
              <Datum
                label="Front lidar range"
                text={value(age(t, "lidar") < 3 ? t?.nav.front_m : null, " m")}
              />
              <Datum
                label="Gap bearing"
                text={value(
                  age(t, "avoidance") < 3 && age(t, "lidar") < 3
                    ? ((t?.nav.centering_err ?? NaN) * 180) / Math.PI
                    : null,
                  "°",
                )}
              />
              <Datum
                label="EKF"
                text={
                  age(t, "ekf") < 3
                    ? String(t?.safety.ekf ?? "Unavailable")
                    : "Unavailable"
                }
              />
              <Datum
                label="FC fence"
                text={
                  age(t, "fence") < 3
                    ? t?.safety.geofence || "Unavailable"
                    : "Unavailable"
                }
              />
              <Datum
                label="Delivery error"
                text={value(t?.mission.delivery_offset_m, " m")}
              />
              <Datum
                label="Landing"
                text={t?.mission.landing_precision || "Unavailable"}
              />
              <Datum
                label="Mission result"
                text={
                  t?.mission.result
                    ? `${t.mission.result}: ${t.mission.result_reason || ""}`
                    : "Unavailable"
                }
              />
            </div>
            <p className="view-note">
              Front lidar range is not aircraft clearance. Mission completion,
              disarm and physical delivery are separate observations.
            </p>
            {t?.checklist && (
              <p>
                {Object.entries(t.checklist)
                  .map(
                    ([step, done]) =>
                      `${step}: ${done ? "observed" : "not confirmed"}`,
                  )
                  .join(" · ")}
              </p>
            )}
          </details>
          <section className="observations">
            <h2>Detections</h2>
            <p className="observation-line">
              Start QR: {t?.percep.start_qr || "No payload read"} · Delivery
              target:{" "}
              {t?.percep.mission_target === "<start marker appearance>"
                ? "start marker pattern"
                : t?.percep.mission_target || "Not set"}{" "}
              · Pattern: {t?.percep.qr_reference ? "captured" : "not captured"}{" "}
              · Pad:{" "}
              {t?.percep.target_match
                ? `Matched (${(t.percep.qr_match_basis ?? []).join(", ") || "payload"})`
                : "Unconfirmed"}{" "}
              · Banner:{" "}
              {t?.percep.banner ? "Detected" : "Unconfirmed"} · Red zone:{" "}
              {t?.percep.redzone_status || "UNKNOWN"}
            </p>
            <p className="view-note">
              Square on:{" "}
              {t?.percep.square_angle_deg == null
                ? t?.percep.square_reason || "Unavailable"
                : `${value(t.percep.square_angle_deg, "°")} at ${value(t.percep.square_standoff_m, " m")}`}{" "}
              · {t?.percep.redzone_exclusions?.length || 0} mapped red
              exclusions
            </p>
            <p className="view-note">
              {["qr", "banner", "red"]
                .map(
                  (kind) =>
                    `${kind}: ${value(t?.detector_rates?.[kind]?.hz, " Hz")}`,
                )
                .join(" · ")}
            </p>
            <table>
              <thead>
                <tr>
                  <th>Kind</th>
                  <th>Payload</th>
                  <th>Status</th>
                  <th>Reason</th>
                  <th>Count</th>
                </tr>
              </thead>
              <tbody>
                {t?.scans?.map((row) => (
                  <tr key={row.key}>
                    <td>{row.kind}</td>
                    <td>{row.payload || "Undecoded"}</td>
                    <td>{row.matched ? "Matched" : row.status}</td>
                    <td>{row.reason}</td>
                    <td>{row.count}</td>
                  </tr>
                ))}
              </tbody>
            </table>
          </section>
        </section>
        <aside className="task-area">
          <h2>{mode === "flight" ? "Normal mission" : "Test task"}</h2>
          {mode === "testing" && (
            <label>
              Task
              <select
                value={task}
                disabled={running}
                onChange={(e) => {
                  setTask(e.target.value);
                  setReview(null);
                }}
              >
                {tasks.map((item) => (
                  <option key={item.id} value={item.id}>
                    {item.label}
                  </option>
                ))}
              </select>
            </label>
          )}
          <p>{taskInfo.description}</p>
          {selectedTask === "mechanism" && (
            <label>
              Mechanism action
              <select
                value={action}
                onChange={(e) => setAction(e.target.value)}
              >
                {["lower", "release", "stow", "stop"].map((name) => (
                  <option key={name}>{name}</option>
                ))}
              </select>
            </label>
          )}
          {taskInfo.movement && selectedTask !== 'mission' && <label>Test height above home (m)<input type="number" min={1} max={10} value={height} onChange={e => setHeight(Number(e.target.value))}/></label>}
          {selectedTask.startsWith('delivery_') && <label>QR payload to verify<input value={target} onChange={e => setTarget(e.target.value)} /></label>}
          {['delivery_gps','avoidance_route'].includes(selectedTask) && <label>GPS waypoints (latitude, longitude per line)<textarea value={waypoints} onChange={e => setWaypoints(e.target.value)} /></label>}
          <p className="task-note">
            {selectedTask === "mechanism"
              ? "Disarmed bench task. Accepted release does not prove detachment."
              : "Flight tasks use production behaviours and readiness checks. Passive QR and SLAM tasks observe without arming. No browser joystick."}
          </p>
          <button
            disabled={!canStart}
            onClick={() => {
              const request = taskCommand(selectedTask, action, {height, target, waypoints: waypoints.trim() ? boundary(waypoints) : []});
              setReview({ ...request, label: `Start ${taskInfo.label}` });
            }}
          >
            Review start
          </button>
          <button
            disabled={!linked || busy}
            onClick={() =>
              setReview({
                cmd: "stop",
                args: {},
                label: "Stop and recover using the production abort procedure",
              })
            }
          >
            Stop and return
          </button>
          {review && (
            <section className="review">
              <h3>{review.label}</h3>
              <p>Vehicle: {hostState?.profile_id ?? 'None'}</p>
              <p>
                Mode: {fcu ? t?.mission.mode : "Unavailable"} · Armed:{" "}
                {fcu ? (t?.mission.armed ? "Yes" : "No") : "Unavailable"}
              </p>
              <p>
                Runtime acceptance does not confirm the aircraft outcome. Stop
                uses the current stage-aware LAND/RTL procedure.
              </p>
              <button
                disabled={
                  busy || !reviewAllowed(t, linked, review.cmd, review.args)
                }
                onClick={() => {
                  const request = review;
                  setReview(null);
                  void perform(request.cmd, request.args);
                }}
              >
                Confirm{" "}
                {review.cmd === "start"
                  ? "Start"
                  : "command"}
              </button>
              <button onClick={() => setReview(null)}>Cancel</button>
            </section>
          )}
          {outcome && (
            <p role="status">
              {outcome.cmd}: {outcome.state}. {outcome.reason}
            </p>
          )}
          <details open>
            <summary>Readiness</summary>
            {!fresh && <p>Readiness unavailable</p>}
            {t?.safety.ready_reasons?.map((reason) => (
              <p key={reason}>{reason}</p>
            ))}
            {t?.safety.ready_items?.map((item) => (
              <div className="readiness-row" key={item.key}>
                <span>
                  {item.label}: {item.ok ? "Ready" : "Blocked"}
                </span>
                <span>
                  {readinessValue(item.value)} · {item.reason}
                </span>
              </div>
            ))}
            {fresh && !t?.safety.ready && (
              <p>
                {t?.mission.mode !== "GUIDED"
                  ? "GUIDED mode is required for flight tasks."
                  : "Correct the named inputs before starting."}
              </p>
            )}
          </details>
          <details>
            <summary>Organiser inputs</summary>
            <button onClick={() => setArenaOpen(true)}>Arena dimensions</button>
            <button onClick={() => setLidarOpen(true)}>
              Lidar calibration
            </button>
            <Field label="Target QR payload" text={target} change={setTarget} />
            <button
              disabled={!fresh || busy}
              onClick={() => void perform("set_target", { target })}
            >
              Set target
            </button>
            {(["green", "orange", "red"] as const).map((name) => (
              <Field
                key={name}
                label={name === "red" ? "red zone HEX" : `${name} banner HEX`}
                text={colours[name]}
                change={(shade) => setColours({ ...colours, [name]: shade })}
              />
            ))}
            <button
              disabled={!fcu || busy || t?.mission.armed !== false || !Object.values(colours).every(c => /^#[0-9a-fA-F]{6}$/.test(c))}
              onClick={() =>
                reviewedSend("set_banner_colours", { colours }, true)
              }
            >
              Apply banner colours
            </button>
            <p>Enter all three organiser codes as #RRGGBB. Applied: {t?.banner_colours ? Object.entries(t.banner_colours).map(([name,hex]) => `${name} ${hex}`).join(' · ') : 'Not configured'}</p>
            <label>
              Geofence: the judges&apos; coordinates, one
              &quot;latitude, longitude&quot; per line
              <textarea
                rows={7}
                value={fenceText}
                placeholder={["-35.3634507, 149.1650815", "-35.3634507, 149.1659792", "..."].join("\n")}
                onChange={(e) => setFenceText(e.target.value)}
              />
            </label>
            <p className="view-note">
              {draftError
                ? draftError
                : draftFence.length
                  ? `${draftFence.length} points${draftFence.length === 6 ? "" : " (the judges give 6)"} · drawn dashed on the Map tab`
                  : "Paste the coordinates; the polygon is drawn on the Map tab."}
            </p>
            <button
              disabled={!fcu || busy || t?.mission.armed !== false || draftFence.length < 3 || Boolean(draftError)}
              onClick={() => {
                try {
                  reviewedSend(
                    "set_geofence",
                    { vertices: boundary(fenceText) },
                    true,
                  );
                } catch (reason) {
                  setError(String(reason));
                }
              }}
            >
              Apply geofence
            </button>
            <p className="view-note">
              Applied: {appliedFence.length ? `${appliedFence.length} points (filled on the map)` : "none"}.
              If the points surround take-off they are the arena fence and
              the aircraft searches the fenced field beyond the corridor exit;
              if take-off lies outside them they are the delivery zone and the
              aircraft enforces a fence 10 m round the zone and take-off.
              Readiness shows which. No separate delivery-zone corners are
              needed. Aircraft validation remains authoritative.
            </p>
          </details>
          <details>
            <summary>Aircraft controls</summary>
            <p>
              Transmitter takeover and FCU failsafes retain priority. Recovery
              remains available when readiness is blocked.
            </p>
            <div className="actions">
              <button
                disabled={!fcu || busy || running}
                onClick={() =>
                  reviewedSend("set_mode", { mode: "GUIDED" }, true)
                }
              >
                GUIDED
              </button>
              <button
                disabled={
                  !fcu ||
                  !t?.safety.ready ||
                  t?.mission.mode !== "GUIDED" ||
                  busy ||
                  running
                }
                onClick={() => reviewedSend("arm", {}, true)}
              >
                Arm
              </button>
              <button
                disabled={
                  !fcu ||
                  busy ||
                  t?.mission.armed !== true ||
                  t?.safety.on_ground !== true ||
                  age(t, "extended_state") >= 3
                }
                onClick={() => reviewedSend("disarm", {}, true)}
              >
                Disarm
              </button>
              <button
                disabled={!linked || busy}
                onClick={() => reviewedSend("land", {}, true)}
              >
                Land
              </button>
              <button
                disabled={!linked || busy}
                onClick={() => reviewedSend("set_mode", { mode: "RTL" }, true)}
              >
                RTL
              </button>
              <button
                disabled={!linked || busy}
                onClick={() => reviewedSend("abort", {}, true)}
              >
                Abort
              </button>
            </div>
            <label>
              Takeoff height above home, metres
              <input
                type="number"
                min="1"
                max="10"
                value={height}
                onChange={(e) => setHeight(Number(e.target.value))}
              />
            </label>
            <button
              disabled={
                !fcu ||
                !t?.safety.ready ||
                t?.mission.mode !== "GUIDED" ||
                busy ||
                running
              }
              onClick={() => reviewedSend("takeoff", { alt: height }, true)}
            >
              Review takeoff
            </button>
            <div className="actions">
              <button
                disabled={!fcu || busy}
                onClick={() =>
                  reviewedSend("gimbal_pitch", { degrees: 0 }, true)
                }
              >
                Camera forward
              </button>
              <button
                disabled={!fcu || busy}
                onClick={() =>
                  reviewedSend("gimbal_pitch", { degrees: -90 }, true)
                }
              >
                Camera down
              </button>
            </div>
            <p className="view-note">
              Winch controls are under Testing → Mechanism only. Current
              hardware has no GPIO encoder acquisition.
            </p>
          </details>
        </aside>
      </main>
      <footer>
        <details open>
          <summary>Events</summary>
          <div className="event-log">
            {events.length
              ? events.map((event, index) => <div key={index}>{event}</div>)
              : "No events yet"}
          </div>
        </details>
      </footer>
      {arenaOpen && (
        <ArenaDimensions
          S={t}
          connected={fcu}
          send={reviewedSend}
          onClose={() => setArenaOpen(false)}
        />
      )}
      {lidarOpen && (
        <LidarCalibration
          S={t}
          connected={fcu}
          send={reviewedSend}
          onClose={() => setLidarOpen(false)}
        />
      )}
    </div>
  );
}
function Datum({ label, text }: { label: string; text: string }) {
  return (
    <dl>
      <dt>{label}</dt>
      <dd>{text}</dd>
    </dl>
  );
}
function Field({
  label,
  text,
  change,
}: {
  label: string;
  text: string;
  change: (text: string) => void;
}) {
  return (
    <label>
      {label}
      <input value={text} onChange={(event) => change(event.target.value)} />
    </label>
  );
}
