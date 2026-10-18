import { useEffect, useState } from 'react';
import type { ConnectionProfile, HostState } from './runtime';
import { defaultProfile, api } from './runtime';

// Choosing the vehicle fills the link it uses; every field stays editable.
function asVehicle(p: ConnectionProfile, vehicle: 'simulation' | 'drone'): ConnectionProfile {
  return vehicle === 'drone'
    ? { ...p, vehicle, transport: 'serial', device: '/dev/ttyAMA2', baud: p.baud || 115200, gateway: '', autoconnect:true, slam:true, slam_odometry:'lidar' }
    : { ...p, vehicle, transport: 'tcp', port: 5762, hardware_reviewed: false, winch_backend: 'mavlink', autoconnect:false, slam_odometry:'fcu',
        gateway: p.gateway || (p.host ? `http://${p.host}:8090` : '') };
}

export default function Connections({ state, refresh }: { state: HostState | null; refresh: () => Promise<void> }) {
  const [p, setP] = useState<ConnectionProfile>(() => state?.profiles.find(x => x.id === state.profile_id) ?? state?.profiles[0] ?? defaultProfile());
  const [message, setMessage] = useState('');
  const [busy, setBusy] = useState(false);
  const [items, setItems] = useState<Array<Record<string, unknown>>>([]);
  const [rc, setRc] = useState('{}');
  useEffect(() => { setRc(JSON.stringify(p.rc_config, null, 2)); }, [p.id]);
  const change = (key: string, value: unknown) => setP(old => ({ ...old, [key]: value }));
  const vehicle = p.vehicle ?? (p.transport === 'serial' ? 'drone' : 'simulation');
  const field = (key: keyof ConnectionProfile, label: string, type = 'text') =>
    <label key={key}>{label}<input type={type} value={String(p[key] ?? '')}
      onChange={e => change(key, type === 'number' ? Number(e.target.value) : e.target.value)} /></label>;
  async function run(path: string, data?: unknown) {
    setBusy(true); setMessage('');
    try {
      const result = await api(path, data);
      if (path === '/api/check') setItems(result.items);
      else setMessage(path === '/api/profiles' ? 'Profile saved on the Pi.' : 'Request accepted. Check live connection status.');
      await refresh();
    } catch (e) { setMessage(e instanceof Error ? e.message : String(e)); }
    finally { setBusy(false); }
  }
  return <section className="connections">
    <h2>Vehicle connections</h2>
    <p>The Pi always runs the flight code and this GCS. Laptop simulation connects it over the intranet to Gazebo and SITL on the laptop (FCU TCP port and sensor gateway). Real drone uses the flight controller UART and the Pi camera and lidar.</p>
    <p>Runtime: {state?.runtime ?? 'Unavailable'} · Active profile: {state?.profile_id ?? 'None'}</p>
    {state?.connection_error&&<p role="alert">Automatic connection: {state.connection_error}</p>}
    <fieldset disabled={busy || !state?.can_edit_profiles}>
      <div className="connection-fields">
        <label>Saved profile<select value={p.id} onChange={e => setP(state?.profiles.find(x => x.id === e.target.value) ?? defaultProfile())}>
          <option value="vehicle">New profile</option>{state?.profiles.map(x => <option key={x.id} value={x.id}>{x.name}</option>)}
        </select></label>
        {field('id', 'Profile ID')}{field('name', 'Name')}
        <label>Vehicle<select value={vehicle} onChange={e => setP(old => asVehicle(old, e.target.value as 'simulation' | 'drone'))}>
          <option value="simulation">Laptop simulation (Gazebo + SITL over intranet)</option><option value="drone">Real drone</option>
        </select></label>
        <label>FCU transport<select value={p.transport} onChange={e => change('transport', e.target.value)}>
          <option value="tcp">TCP</option><option value="udp">UDP</option><option value="serial">Local hardware UART</option>
        </select></label>
        {p.transport === 'serial' ? <>{field('device', 'FCU device (router owns UART)')}{field('baud', 'Baud', 'number')}
          {field('gcs_host', 'Extra Mission Planner IP (optional; the router already broadcasts to the LAN on UDP 14550)')}</> :
          <>{field('host', 'Laptop or router hostname / IP')}{field('port', 'FCU port', 'number')}{p.transport === 'udp' && field('bind_port', 'Pi UDP listen port', 'number')}</>}
        {field('system_id', 'Vehicle system ID', 'number')}{field('component_id', 'FCU component ID', 'number')}
        {vehicle === 'simulation' && field('gateway', 'Laptop sensor gateway (HTTP origin)')}
        {vehicle === 'drone' && <>{field('camera_device', 'Camera device')}{field('lidar_port', 'Lidar device')}
          <label>Winch<select value={p.winch_backend ?? 'mavlink'} onChange={e => change('winch_backend', e.target.value)}>
            <option value="mavlink">Flight controller (DO_WINCH)</option><option value="gpio">Pi GPIO: BTS7960 + N20 encoder (measured values required)</option>
          </select></label>
          <label><input type="checkbox" checked={p.autoconnect !== false} onChange={e => change('autoconnect', e.target.checked)} /> Connect automatically at Pi startup</label></>}
      </div>
      <details><summary>Calibration and transmitter configuration</summary><div className="connection-fields">
        {field('camera_hfov', 'Camera horizontal field of view (rad)', 'number')}{field('marker_m', 'Launch QR side (m); Arena dimensions replace it', 'number')}
        {field('lidar_yaw_deg', 'Lidar mount yaw (deg)', 'number')}{field('delivery_calibration', 'Measured delivery calibration file')}
        <label><input type="checkbox" checked={p.lidar_mirrored} onChange={e => change('lidar_mirrored', e.target.checked)} /> Lidar mirrored</label>
        <label><input type="checkbox" checked={p.slam} onChange={e => change('slam', e.target.checked)} /> Enable SLAM mapping</label>
        <label>SLAM pose source<select value={p.slam_odometry??(vehicle==='drone'?'lidar':'fcu')} onChange={e=>change('slam_odometry',e.target.value)}>
          <option value="lidar">LiDAR scan matching, no GPS required</option><option value="fcu">Flight-controller odometry</option></select></label>
      </div><label>RC channel mapping (JSON)<textarea rows={6} value={rc} onChange={e => setRc(e.target.value)} /></label>
      <p>Map forward, left, yaw, climb and takeover channels, min/trim/max, takeover active_above and speed limits. Transmitter takeover has priority.</p></details>
      <button onClick={() => { try { void run('/api/profiles', { ...p, rc_config: JSON.parse(rc) }); } catch { setMessage('RC mapping must be valid JSON.'); } }}>Save profile</button>
    </fieldset>
    <div className="actions">
      <button disabled={busy || state?.runtime !== 'Stopped' || !state?.profiles.some(x => x.id === p.id)} onClick={() => void run('/api/connect', { profile_id: p.id })}>Connect vehicle</button>
      <button disabled={busy} onClick={() => void run('/api/check')}>Connection check</button>
      {/* After a reboot the last profile stays selected with no runtime; Disconnect
          releases it (the host still refuses while an armed session is unresolved). */}
      <button disabled={busy || (state?.runtime === 'Stopped' && !state?.profile_id) || !state?.can_disconnect} onClick={() => void run('/api/disconnect', {})}>Disconnect runtime</button>
    </div>
    <p>The hardware router starts at boot and sends Mission Planner UDP on port 14550. Automatic connection prepares telemetry and sensors without a props checkbox. Start controls execution. Disconnect pauses automatic connection until the next Pi/web-host restart or an explicit Connect.</p>
    {message && <p role="status">{message}</p>}
    {items.map(item => <div className="readiness-row" key={String(item.key)}><span>{String(item.key)}</span><span>{item.ok === true ? 'Ready' : 'Unavailable'}</span><span>{String(item.reason ?? '')} {item.key === 'identity' ? `system ${item.value ?? 'unknown'}` : ''} {typeof item.hz === 'number' ? `${item.hz.toFixed(1)} Hz delivered` : ''} {typeof item.vehicle_hz === 'number' ? ` · ${item.vehicle_hz.toFixed(1)} Hz sensor clock` : ''} {typeof item.capture_age_s === 'number' ? ` · capture age ${item.capture_age_s.toFixed(3)} s` : ''}</span></div>)}
  </section>;
}
