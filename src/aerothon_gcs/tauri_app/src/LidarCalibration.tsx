import { useEffect, useState } from 'react';
import type { Telemetry } from './types';
import { MetricPlot } from './LidarView';

type Props = {S:Telemetry|null;connected:boolean;send:(cmd:string,args?:any,danger?:boolean)=>void;onClose:()=>void};
const fields = [['x_m','Forward X, m'],['y_m','Left Y, m'],['z_m','Up Z, m'],['roll_deg','Roll, degrees'],['pitch_deg','Pitch, degrees']];
const targets = [['ahead','Ahead, parallel to nose, 0°'],['far','Farther ahead, same line, 0°'],['port','Left, +90°'],['starboard','Right, −90°, independent check']];

export default function LidarCalibration({S,connected,send,onClose}:Props) {
  const cal=S?.lidar_cal, result=cal?.result, applied=cal?.applied;
  const disabled=!connected||S?.mission?.armed!==false;
  const [geometry,setGeometry]=useState<Record<string,string>>(()=>Object.fromEntries(fields.map(([key])=>[key,cal?.geometry?.[key]?.toString()??''])));
  const [distances,setDistances]=useState<Record<string,string>>({});
  useEffect(()=>{const key=(e:KeyboardEvent)=>{if(e.key==='Escape')onClose();};window.addEventListener('keydown',key);return()=>window.removeEventListener('keydown',key);},[onClose]);
  return <div className="dims-overlay" role="dialog" aria-label="Lidar calibration">
    <div className="card dims lidar calibration-wizard">
      <div className="hd">Lidar mounting calibration
        <span className={`tag ${cal?.raw_fresh?'ok':''}`}>{cal?.raw_fresh?`${cal.samples??0} recent raw scans`:'Waiting for raw scans'}</span>
        <button className="dims-close" onClick={onClose} aria-label="Close">×</button></div>
      <p>Keep the aircraft disarmed and stationary. Use a matte flat panel, at least 30 cm wide, with its centre in the laser plane. GPS is unnecessary.</p>
      <fieldset disabled={disabled} className="calibration-stage">
        <legend>1. Measure the optical centre relative to the airframe</legend>
        <p>The reference is the airframe base_link origin used by the aircraft model, at the centre of its frame. Measure forward X, left Y and up Z to the lidar optical centre. Use a level to measure roll and pitch. Negative values mean backward, right and down. Level the scanner within 3° before continuing.</p>
        <div className="connection-fields">{fields.map(([key,label])=><label key={key}>{label}<input type="number" step="0.001" value={geometry[key]} onChange={e=>setGeometry(g=>({...g,[key]:e.target.value}))}/></label>)}</div>
        <button disabled={fields.some(([key])=>geometry[key].trim()==='')} onClick={()=>send('lidar_cal_geometry',Object.fromEntries(fields.map(([key])=>[key,Number(geometry[key])])))}>Record measured mounting geometry</button>
        <span>{cal?.geometry?'Measurements recorded.':'All five measurements are required. These values cannot be inferred from an unknown room.'}</span>
      </fieldset>
      <fieldset disabled={disabled||!cal?.geometry} className="calibration-stage">
        <legend>2. Record the background</legend>
        <p>Remove the panel. Keep people still and leave space around all four target positions. Wait 2 seconds, then capture. The panel will be identified by its difference from this background.</p>
        <button disabled={!cal?.raw_fresh} onClick={()=>send('lidar_cal_background')}>Capture empty background</button>
        <span>{cal?.background_ready?'Background recorded.':'Needs at least 15 distinct, fresh scans.'}</span>
      </fieldset>
      <fieldset disabled={disabled||!cal?.background_ready} className="calibration-stage">
        <legend>3. Measure four target positions</legend>
        <p>Place the panel centre on each direction from the lidar optical centre, parallel to the airframe axes. Its face must be square to that direction. Measure the distance from the optical centre to the panel face. Hold still for 2 seconds before each capture. Put the far panel at least 0.5 m beyond the first panel.</p>
        {targets.map(([key,label])=>{
          const capture=cal?.captures?.[key as keyof typeof cal.captures];const detail=cal?.capture_details?.[key];
          return <div className="calibration-target" key={key}>
            <strong>{label}</strong><label>Measured distance, m<input type="number" min="0.4" max="4" step="0.001" value={distances[key]??''} onChange={e=>setDistances(d=>({...d,[key]:e.target.value}))}/></label>
            <button disabled={!distances[key]||!cal?.raw_fresh} onClick={()=>send('lidar_cal_capture',{where:key,distance_m:Number(distances[key])})}>Capture {key}</button>
            <span>{capture?`${capture[0].toFixed(2)}° raw, ${capture[1].toFixed(3)} m`:'Pending'}{detail?` · ${detail.samples} scans, ${detail.beams} beams · spread ${(1000*detail.spread_m).toFixed(1)} mm`:''}</span>
          </div>;
        })}
      </fieldset>
      <div className="calibration-preview"><h3>Full raw scan</h3><MetricPlot scan={cal?.scan} fresh={cal?.raw_fresh}/></div>
      <div className="calibration-stage"><h3>4. Review the independent checks</h3>
        {result?<p role="status">{result.ok?`Checks passed. Yaw ${result.yaw_deg.toFixed(3)}°, ${result.mirrored?'mirrored':'counter-clockwise'}. Maximum direction error ${result.error_deg}°. Independent range error ${((result.validation_error_m??0)*1000).toFixed(1)} mm. Range scale ${result.range_scale?.toFixed(5)}, bias ${((result.range_bias_m??0)*1000).toFixed(1)} mm.`:result.reason}</p>:<p>Complete all four captures. A target must contain at least five adjacent returns in each of 15 scans. Independent checks must be within 2° and 30 mm.</p>}
        <p>Applying updates measured translation, tilt, yaw, scan direction and range correction for the scan consumers. Reconnect the runtime afterwards to start a new map with the new geometry.</p>
        {applied&&<p>Applied yaw {applied.yaw_deg}°, {applied.mirrored?'mirrored':'counter-clockwise'} · {applied.geometry_measured?'full measured geometry':'legacy yaw only, position unmeasured'} · {applied.source}{applied.reason?` · ${applied.reason}`:''}</p>}
      </div>
      <div className="dims-actions"><span>{disabled?'Connect telemetry and disarm to calibrate.':'Measured checks establish repeatability within the displayed errors. They do not remove measurement uncertainty.'}</span>
        <button disabled={disabled} onClick={()=>send('lidar_cal_reset')}>Reset measurements</button>
        <button className="primary" disabled={disabled||!result?.ok} onClick={()=>send('lidar_cal_apply',{},true)}>Apply measured calibration</button></div>
    </div>
  </div>;
}
