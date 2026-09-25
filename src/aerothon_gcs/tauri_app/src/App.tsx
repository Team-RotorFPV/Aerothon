import { useEffect, useRef, useState } from "react";
import { listen } from "@tauri-apps/api/event";
import { invoke } from "@tauri-apps/api/core";
import maplibregl, { type GeoJSONSource } from "maplibre-gl";
import "maplibre-gl/dist/maplibre-gl.css";
import { drawHUD, drawMap, drawSlam, type OccGrid } from "./render";
import type { Telemetry, Envelope } from "./types";
import "./gcs.css";

const CL: [string, string][] = [
  ["takeoff", "Auto Takeoff (5m)"], ["start_qr", "Scan & Decode Start QR"],
  ["banner", "Corridor Banner Align"], ["corridor", "Corridor Nav & Avoidance"],
  ["target_id", "Target QR Search & Match"], ["drop", "Winch Drop & Ground Release"],
  ["return", "Corridor Return Navigation"], ["land", "Precision Landing at Origin"],
];
const STEP: Record<string, number> = {
  TAKEOFF: 0, GOTO_CORRIDOR: 1, CORRIDOR_NAV: 3, ENTER_ZONE: 3, SEARCH_QR: 4,
  WINCH_DROP: 5, RETURN: 6, RETURN_CORRIDOR: 6, LAND: 7,
};

export default function App() {
  const [S, setS] = useState<Telemetry | null>(null);
  const [log, setLog] = useState<string[]>([]);
  const [linked, setLinked] = useState(false);
  const [endpoint, setEndpoint] = useState("ws://127.0.0.1:8765");
  const [activeEndpoint, setActiveEndpoint] = useState("ws://127.0.0.1:8765");
  const [connectionNonce, setConnectionNonce] = useState(0);
  const [view, setView] = useState<"map" | "cam" | "slam">("map");
  // The composite overlay is the default: the live camera with whatever is
  // currently detected drawn on it. The per-detector streams are kept
  // selectable because they are what proved the banner lettering bug.
  const [videoTopic, setVideoTopic] = useState("/percep/overlay");
  const webSocket = useRef<WebSocket | null>(null);
  const inTauri = "__TAURI_INTERNALS__" in window;
  const mapHost = useRef<HTMLDivElement>(null);
  const geoMap = useRef<maplibregl.Map | null>(null);
  const gpsMarker = useRef<maplibregl.Marker | null>(null);
  const gpsTrail = useRef<[number, number][]>([]);
  const mapCentered = useRef(false);

  const hud = useRef<HTMLCanvasElement>(null);
  const map = useRef<HTMLCanvasElement>(null);
  const slam = useRef<HTMLCanvasElement>(null);
  const trail = useRef<[number, number][]>([]);
  const cells = useRef<Map<string, [number, number]>>(new Map());
  const grid = useRef<OccGrid | null>(null);
  const latest = useRef<Telemetry | null>(null);
  const logRef = useRef<HTMLDivElement>(null);

  useEffect(() => {
    if (!inTauri) return;
    const un = listen<string>("ws", (e) => {
      setLinked(true);
      let env: Envelope; try { env = JSON.parse(e.payload); } catch { return; }
      if (env.kind === "telemetry") onTelemetry(env.data as Telemetry);
      else if (env.kind === "map") grid.current = env.data as OccGrid;
      else if (env.kind === "event") pushLog(`● ${env.data.msg}`);
      else if (env.kind === "ack") pushLog(`⤷ ack ${env.data.cmd}: ${env.data.result}`);
    });
    const unConn = listen<{ connected: boolean; endpoint: string }>("connection", (e) => {
      setLinked(e.payload.connected);
      setEndpoint(e.payload.endpoint);
      pushLog(e.payload.connected ? `● connected ${e.payload.endpoint}` : "○ disconnected");
    });
    let raf = 0;
    const loop = () => {
      if (hud.current) drawHUD(hud.current, latest.current);
      if (map.current) drawMap(map.current, latest.current, trail.current, cells.current, grid.current);
      if (slam.current) drawSlam(slam.current, latest.current, cells.current, grid.current);
      raf = requestAnimationFrame(loop);
    };
    raf = requestAnimationFrame(loop);
    return () => { un.then((f) => f()); unConn.then((f) => f()); cancelAnimationFrame(raf); };
  }, [inTauri]);

  // Browser build: communicate directly with the ROS WebSocket aggregator.
  // The Tauri desktop build continues to use its native command bridge.
  useEffect(() => {
    if (inTauri) return;
    let stopped = false;
    let retry: number | undefined;
    let raf = 0;
    const draw = () => {
      if (hud.current) drawHUD(hud.current, latest.current);
      if (map.current) drawMap(map.current, latest.current, trail.current, cells.current, grid.current);
      if (slam.current) drawSlam(slam.current, latest.current, cells.current, grid.current);
      raf = requestAnimationFrame(draw);
    };
    raf = requestAnimationFrame(draw);
    const open = () => {
      if (stopped) return;
      const ws = new WebSocket(activeEndpoint);
      webSocket.current = ws;
      ws.onopen = () => {
        if (stopped || webSocket.current !== ws) return;
        setLinked(true);
        pushLog(`● connected ${activeEndpoint}`);
      };
      ws.onmessage = (e) => {
        if (stopped || webSocket.current !== ws) return;
        let env: Envelope; try { env = JSON.parse(String(e.data)); } catch { return; }
        if (env.kind === "telemetry") onTelemetry(env.data as Telemetry);
        else if (env.kind === "map") grid.current = env.data as OccGrid;
        else if (env.kind === "event") pushLog(`● ${(env.data as any).msg}`);
        else if (env.kind === "ack") {
          const d = env.data as any;
          pushLog(`⤷ ack ${d.cmd}: ${d.result}${d.reason ? ` · ${d.reason}` : ""}`);
        }
      };
      ws.onerror = () => {
        if (!stopped && webSocket.current === ws) ws.close();
      };
      ws.onclose = () => {
        if (stopped || webSocket.current !== ws) return;
        setLinked(false);
        retry = window.setTimeout(open, 2000);
      };
    };
    open();
    return () => {
      stopped = true;
      cancelAnimationFrame(raf);
      if (retry !== undefined) window.clearTimeout(retry);
      const ws = webSocket.current;
      if (ws) {
        ws.onclose = null;
        ws.close();
        if (webSocket.current === ws) webSocket.current = null;
      }
    };
  }, [activeEndpoint, connectionNonce, inTauri]);

  useEffect(() => {
    if (!mapHost.current) return;
    const gm = new maplibregl.Map({
      container: mapHost.current,
      center: [149.1652, -35.36324],
      zoom: 18,
      attributionControl: { compact: true },
      style: {
        version: 8,
        sources: {
          satellite: {
            type: "raster",
            // Satellite arena tiles are bundled for offline competition use.
            tiles: ["/satellite/{z}/{x}/{y}.jpg"],
            tileSize: 256,
            minzoom: 14,
            maxzoom: 19,
            attribution: "Tiles © Esri — Esri, Maxar, Earthstar Geographics, GIS Community",
          },
        },
        layers: [{ id: "satellite", type: "raster", source: "satellite" }],
      },
    });
    gm.addControl(new maplibregl.NavigationControl({ showCompass: true }), "top-right");
    gm.on("load", () => {
      gm.addSource("flight-trail", {
        type: "geojson",
        data: { type: "Feature", properties: {}, geometry: { type: "LineString", coordinates: gpsTrail.current } },
      });
      gm.addLayer({
        id: "flight-trail", type: "line", source: "flight-trail",
        paint: { "line-color": "#e8eef5", "line-width": 4, "line-opacity": 0.9 },
      });
    });
    geoMap.current = gm;
    gpsMarker.current = new maplibregl.Marker({ color: "#d47a72" });
    return () => { gpsMarker.current?.remove(); gm.remove(); geoMap.current = null; };
  }, []);

  function onTelemetry(d: Telemetry) {
    latest.current = d;
    trail.current.push([d.flight.x, d.flight.y]);
    if (trail.current.length > 2000) trail.current.shift();
    if (Number.isFinite(d.gps?.lat) && Number.isFinite(d.gps?.lon) &&
        Math.abs(d.gps.lat) > 0.0001 && Math.abs(d.gps.lon) > 0.0001) {
      const ll: [number, number] = [d.gps.lon, d.gps.lat];
      gpsTrail.current.push(ll);
      if (gpsTrail.current.length > 2000) gpsTrail.current.shift();
      gpsMarker.current?.setLngLat(ll).addTo(geoMap.current!);
      if (!mapCentered.current) {
        geoMap.current?.jumpTo({ center: ll, zoom: 19 });
        mapCentered.current = true;
      }
      const src = geoMap.current?.getSource("flight-trail") as GeoJSONSource | undefined;
      src?.setData({ type: "Feature", properties: {}, geometry: { type: "LineString", coordinates: gpsTrail.current } });
    }
    const sc = (d as any).scan;
    if (sc && sc.ranges) {
      const R = sc.ranges, n = R.length;
      for (let k = 0; k < n; k++) {
        const r = R[k]; if (r == null) continue;
        const a = -Math.PI + 2 * Math.PI * k / n;
        const ex = d.flight.x + r * Math.cos(a), ey = d.flight.y + r * Math.sin(a);
        cells.current.set(Math.round(ex / 0.15) + "," + Math.round(ey / 0.15), [ex, ey]);
      }
      if (cells.current.size > 15000) cells.current.clear();
    }
    setS(d);
  }

  function pushLog(l: string) {
    setLog((x) => [...x.slice(-150), `${new Date().toLocaleTimeString()}  ${l}`]);
    setTimeout(() => logRef.current?.scrollTo(0, 1e9), 0);
  }
  async function send(cmd: string, args: any = {}, danger = false) {
    if (danger && !confirm(`Confirm command: ${cmd.toUpperCase()}?`)) return;
    try {
      if (inTauri) await invoke("send_command", { cmd, args });
      else {
        const ws = webSocket.current;
        if (!ws || ws.readyState !== WebSocket.OPEN) throw new Error("drone WebSocket not connected");
        ws.send(JSON.stringify({
          v: 1, kind: "command", t: Date.now() / 1000,
          data: { cmd_id: crypto.randomUUID(), cmd, args, confirm: true },
        }));
      }
      pushLog(`→ ${cmd}`);
    }
    catch (e) { pushLog(`✗ ${cmd}: ${e}`); }
  }
  async function connect() {
    try {
      if (inTauri) await invoke("set_endpoint", { endpoint });
      else {
        setActiveEndpoint(endpoint.trim().replace(/\/$/, ""));
        // Allow CONNECT to force a clean retry even when the URL is unchanged.
        setConnectionNonce((value) => value + 1);
      }
      setLinked(false); pushLog(`→ connect ${endpoint}`);
    }
    catch (e) { pushLog(`✗ connection: ${e}`); }
  }

  const videoUrl = (() => {
    try {
      const u = new URL(endpoint);
      u.protocol = u.protocol === "wss:" ? "https:" : "http:";
      // The composite feed: the live camera with WHATEVER is currently
      // detected drawn on it. This was pinned to /percep/qr/annotated -- the
      // QR detector's private copy of the frame -- so during banner alignment
      // the pane showed a nadir QR view with no banner box on it.
      u.port = "8080"; u.pathname = "/stream"; u.search = `?topic=${videoTopic}&type=mjpeg`;
      return u.toString();
    } catch { return ""; }
  })();

  const f = S?.flight, m = S?.mission, p = S?.percep, sa = S?.safety;
  const scans = S?.scans ?? [];
  const cur = STEP[m?.state ?? ""] ?? -1;
  const pill = (g: boolean | undefined, a: string, b: string, inv = false) =>
    <span className={`pill ${g ? "ok" : inv ? "bad" : "warn"}`}>{g ? a : b}</span>;

  // Red zone is a TRI-state, not a boolean. This panel used to render
  // !redzone_visible as "CLEAR", so a detector that had never published
  // anything — or one that could not see the ground at all — read as safe.
  const redPill = (s: string | undefined) => {
    const cls = s === "CLEAR" ? "ok" : s === "RED" ? "bad" : "warn";
    const txt = s === "CLEAR" ? "CLEAR" : s === "RED" ? "RESTRICTED"
      : s === "NOT_VISIBLE" ? "NO GROUND VIEW" : "UNKNOWN";
    return <span className={`pill ${cls}`}>{txt}</span>;
  };

  const fmtVal = (v: unknown): string => {
    if (v === null || v === undefined) return "—";
    if (typeof v === "boolean") return v ? "yes" : "no";
    if (typeof v === "number") return String(v);
    if (typeof v === "object") return Object.entries(v as object)
      .map(([k, a]) => `${k} ${a ?? "—"}`).join(" ");
    return String(v);
  };

  return (
    <div className="gcs">
      <header>
        <div className="brand"><span className="logo-dot" /> AEROTHON <small>GCS</small></div>
        <span className="badge mode">{m?.mode || "GUIDED"}</span>
        <span className={`badge ${m?.armed ? "armed" : "disarmed"}`}>{m?.armed ? "ARMED" : "DISARMED"}</span>
        <span className={`badge ${sa?.fcu_connected ? "connected" : "disconnected"}`}>
          FCU {sa?.fcu_connected ? "CONNECTED" : "DISCONNECTED"}
        </span>
        <span className="badge">{m?.state || "STANDBY"}</span>
        <div className="view-nav">
          <button className={`view-btn ${view === "map" ? "active" : ""}`} onClick={() => setView("map")}>Flight Map</button>
          <button className={`view-btn ${view === "cam" ? "active" : ""}`} onClick={() => setView("cam")}>Video</button>
          <button className={`view-btn ${view === "slam" ? "active" : ""}`} onClick={() => setView("slam")}>SLAM</button>
        </div>
        <div className="spacer" />
        <div className="stat"><span className="k">GPS</span><span className="v mono">{(S as any)?.gps?.fix || "—"} · {S?.gps?.sats || 0}</span></div>
        <div className="stat"><span className="k">Pos</span><span className="v mono">{f ? `${f.x.toFixed(1)}, ${f.y.toFixed(1)}` : "0, 0"}</span></div>
        <div className="stat"><span className="k">EKF</span><span className="v">{(sa as any)?.ekf ? "OK" : "—"}</span></div>
        <div className="stat"><span className="k">Time</span><span className="v mono">{(m as any)?.elapsed?.toFixed?.(1) || "0.0"}s</span></div>
        <div className="stat connection-control">
          <span className="k">GCS WebSocket · {linked ? "CONNECTED" : "DISCONNECTED"}</span>
          <div className="endpoint-row">
            <input aria-label="Drone WebSocket endpoint" value={endpoint}
              onChange={(e) => setEndpoint(e.target.value)}
              onKeyDown={(e) => { if (e.key === "Enter") connect(); }} />
            <button onClick={connect}>CONNECT</button>
          </div>
        </div>
        <div className="batt-container">
          <div className="stat"><span className="k">Batt</span><span className="v mono">{S ? S.power.volt.toFixed(1) : 0}V</span></div>
          <div className="batt-bar"><div className="batt-fill" style={{ width: `${S?.power.pct || 0}%` }} /></div>
          <span className="batt-text mono">{S?.power.pct || 0}%</span>
        </div>
      </header>

      <main>
        {/* LEFT RAIL — HUD on top, all telemetry values stacked below (Mission Planner style) */}
        <aside className="rail">
          <div className="card hud-card">
            <div className="hd">Primary Flight Display <span className="tag">HUD</span></div>
            <div className="hud-body"><canvas ref={hud} width={380} height={300} /></div>
          </div>
          <div className="card values">
            <div className="sec">Flight Data</div>
            <div className="tele">
              <Cell k="Altitude" v={f?.alt?.toFixed(2)} u="m" />
              <Cell k="Ground Speed" v={(f as any)?.gs?.toFixed(2)} u="m/s" />
              <Cell k="Heading" v={Math.round((f as any)?.yaw_deg || 0)} u="°" />
              <Cell k="Front Lidar" v={S?.nav.front_m?.toFixed(2)} u="m" />
              <Cell k="Roll / Pitch" v={f ? `${(f as any).roll_deg?.toFixed(0)} / ${(f as any).pitch_deg?.toFixed(0)}` : "—"} small />
              <Cell k="Centering" v={(S?.nav.centering_err || 0).toFixed(2)} u="m" small />
            </div>
            <div className="sec">Perception &amp; Interlock</div>
            <div className="kv"><span className="dim">Start QR</span><span className="mono" style={{ color: "var(--accent)" }}>{p?.start_qr || "—"}</span></div>
            <div className="kv"><span className="dim">Target Match</span>{pill(p?.target_match, "MATCHED", "SEARCHING")}</div>
            <div className="kv"><span className="dim">Green Banner</span>{pill(p?.banner, "ALIGNED", "SCANNING")}</div>
            <div className="kv"><span className="dim">Red Zone</span>{redPill(p?.redzone_status)}</div>
            {/* ALIGNED is where the nose points; SQUARE ON is where the
                aircraft stands. The stage will not advance through the gate
                without the second, so the operator has to be able to watch
                it converge rather than guess. */}
            <div className="kv"><span className="dim">Square On</span>
              <span className="mono">
                {p?.square_angle_deg == null
                  ? (p?.square_reason || "—")
                  : `${p.square_angle_deg > 0 ? "+" : ""}${p.square_angle_deg}° @ ${p.square_standoff_m ?? "—"} m`}
              </span></div>
            {!!p?.redzone_exclusions?.length &&
              <div className="kv"><span className="dim">Exclusions</span>
                <span className="mono">{p.redzone_exclusions.length} mapped · {p.redzone_area_m2 ?? 0} m²</span></div>}
            <div className="kv"><span className="dim">Interlock</span>{pill(sa?.ready, "GO", "STANDBY")}</div>
            {/* Which of the eleven checks is holding, and what it measured. A
                greyed-out ARM button with no reason is what this replaces. */}
            {!!sa?.ready_items?.length &&
              <div className="interlock">
                {sa.ready_items.map((it) =>
                  <div key={it.key} className={`ilk ${it.ok ? "ok" : "bad"}`}
                       title={it.reason || `${it.label}: ok`}>
                    <span className="ico">{it.ok ? "✓" : "✕"}</span>
                    <span className="lbl">{it.label}</span>
                    <span className="val mono">{fmtVal(it.value)}</span>
                  </div>)}
                {!!sa.ready_waived?.length &&
                  <div className="ilk warn"><span className="ico">!</span>
                    <span className="lbl">Waived</span>
                    <span className="val mono">{sa.ready_waived.join(", ")}</span></div>}
              </div>}
            {/* WHY arming is blocked, in words, not just which row is red. */}
            {!!sa?.ready_reasons?.length &&
              <div className="kv reasons"><span className="dim">Blocking</span>
                <span className="mono">{sa.ready_reasons.join(" · ")}</span></div>}
            {/* Payload Delivery Accuracy is 15 rulebook marks; landing is 5.
                Showing only the landing figure put the smaller number in
                front of the operator and hid the larger one. */}
            <div className="kv"><span className="dim">Delivery</span>
              <span className="mono">
                {m?.delivery_offset_m == null ? "—"
                  : `${m.delivery_offset_m.toFixed(2)} m from pad`}</span></div>
            <div className="kv"><span className="dim">Landing</span>
              <span className="mono">{m?.landing_precision ?? "—"}</span></div>

            <div className="sec">Mission Sequence</div>
            <div className="checklist">
              {CL.map(([k, label], i) => {
                const done = S?.checklist[k]; const active = i === cur && !done;
                return <div key={k} className={`ci ${done ? "done" : ""} ${active ? "active" : ""}`}>
                  <span className="ico">{done ? "✓" : active ? "●" : ""}</span>{label}</div>;
              })}
            </div>
          </div>
        </aside>

        {/* RIGHT STAGE — big map / SLAM / video, switched by the header view tabs */}
        <section className="card stage">
          <div className="hd">
            {view === "cam" ? "Continuous Camera Feed" : view === "slam" ? "SLAM Occupancy & Costmap" : "Flight Map · Live"}
            <span className="tag">{view === "cam" ? "web_video_server MJPEG" : view === "slam" ? "slam_toolbox · RPLidar C1" : "Satellite · live GPS track"}</span>
          </div>
          <div className="stage-body">
            <div ref={mapHost} className="maplibre-host" style={{ display: view === "map" ? "block" : "none" }} />
            <canvas ref={slam} width={960} height={620} style={{ display: view === "slam" ? "block" : "none" }} />
            {view === "cam" && (
              <div className="cam-wrap">
                <img src={videoUrl} alt="Live camera feed"
                  onError={(e) => { (e.target as HTMLElement).style.display = "none"; }} />
                <div className="cam-tag tl">REC · {videoUrl || "set drone endpoint"}</div>
                <div className="cam-tag br">
                  <select value={videoTopic}
                    onChange={(e) => setVideoTopic(e.target.value)}
                    aria-label="camera feed">
                    <option value="/percep/overlay">ALL DETECTIONS</option>
                    <option value="/camera/image">RAW CAMERA</option>
                    <option value="/percep/qr/annotated">QR ONLY</option>
                    <option value="/percep/banner/annotated">BANNER ONLY</option>
                    <option value="/percep/redzone/annotated">RED ZONE ONLY</option>
                  </select>
                </div>
                <div className="gimbal-control">
                  <span>CAMERA SERVO · {S?.gimbal?.pitch_deg?.toFixed(0) ?? 0}°</span>
                  <button onClick={() => send("gimbal_pitch", { degrees: 0 })}>FORWARD</button>
                  <button onClick={() => send("gimbal_pitch", { degrees: -45 })}>45° DOWN</button>
                  <button onClick={() => send("gimbal_pitch", { degrees: -90 })}>DOWN</button>
                </div>
              </div>
            )}
          </div>
        </section>

        {/* RIGHT RAIL — the scan ledger. Everything decoded, identified or
            refused, in the order it happened, with matches tagged. Before
            this, a scan existed only as a line in a log nobody reads while
            flying, and the operator could not tell afterwards what the
            aircraft had actually read. */}
        <aside className="rail scanrail">
          <div className="card scans">
            <div className="hd">Scanned &amp; Detected
              <span className="tag">{scans.length} observation{scans.length === 1 ? "" : "s"}</span>
            </div>
            <div className="scanlist">
              {scans.length === 0 && <div className="scanempty">nothing scanned yet</div>}
              {scans.map((r) => (
                <div key={r.key} className={"scanrow " + r.status.toLowerCase()}>
                  <div className="scanhead">
                    <span className="scankind">{r.kind.toUpperCase()}</span>
                    <span className="scanpayload mono">
                      {r.payload || <em>refused</em>}
                    </span>
                    {r.matched && <span className="scantag match">MATCH</span>}
                    {!r.matched && r.status === "IDENTIFIED" &&
                      <span className="scantag ident">ID</span>}
                    {r.status === "REJECTED" &&
                      <span className="scantag rej">REJECTED</span>}
                  </div>
                  {r.reason && <div className="scanreason">{r.reason}</div>}
                  <div className="scanmeta mono">
                    {r.stage || "—"} · {r.count}x
                    {r.via ? ` · via ${r.via}` : ""}
                    {r.t ? ` · t+${r.t.toFixed(0)}s` : ""}
                  </div>
                </div>
              ))}
            </div>
          </div>
        </aside>
      </main>

      <footer>
        <div className="btns">
          <button className="primary" onClick={() => send("set_mode", { mode: "GUIDED" })}>GUIDED</button>
          <button disabled={!sa?.ready} onClick={() => send("arm", {}, true)}>ARM</button>
          <button disabled={!sa?.ready} onClick={() => send("takeoff", { alt: 5.0 })}>TAKEOFF</button>
          <button disabled={!sa?.ready} className="primary" onClick={() => send("start_mission", {}, true)}>START M2</button>
          <button onClick={() => send("disarm", {}, true)}>DISARM</button>
          <button className="warn" onClick={() => send("land", {})}>LAND</button>
          <button className="warn" onClick={() => send("set_mode", { mode: "RTL" }, true)}>RTL</button>
          <button className="danger" onClick={() => send("abort", {}, true)}>ABORT</button>
        </div>
        <div className="log mono" ref={logRef}>{log.map((l, i) => <div key={i}>{l}</div>)}</div>
      </footer>
    </div>
  );
}

function Cell({ k, v, u, small }: { k: string; v: any; u?: string; small?: boolean }) {
  return <div className="cell"><div className="k">{k}</div>
    <div className="v mono" style={small ? { fontSize: 14 } : undefined}>{v ?? "—"}{u && <small> {u}</small>}</div></div>;
}
