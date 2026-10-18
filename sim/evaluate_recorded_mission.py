import bisect, csv, json, math, sys
from pathlib import Path
out=Path(sys.argv[1])
if (out/'track.csv').exists():
    with (out/'track.csv').open() as f:
        reader=csv.DictReader(f); fields=reader.fieldnames; rows=list(reader)
else:
    # A crashed runner may not flush its in-memory track. The passive onboard
    # journal remains evidence; a missing mission outcome still fails acceptance.
    fields='t_sim,x,y,z,armed,state,roll_deg,pitch_deg,mode,payload_wx,payload_wy,payload_wz'.split(',')
    rows=[]
end=float(rows[-1]['t_sim']) if rows else -1
truth=[json.loads(line) for line in (out/'evaluation_truth.jsonl').open() if line.strip()]
truth.sort(key=lambda r:r['stamp']); ts=[r['stamp'] for r in truth]
onboard=[]
for line in (out/'onboard.jsonl').open():
    r=json.loads(line); t=r['stamp']
    onboard.append(r)
    if t<=end or 'pose' not in r or 'flight' not in r: continue
    p=r['pose']['data']['pose']['position']; q=r['pose']['data']['pose']['orientation']; f=r['flight']['data']
    x,y,z,w=(q[k] for k in ('x','y','z','w'))
    roll=math.degrees(math.atan2(2*(w*x+y*z),1-2*(x*x+y*y)))
    pitch=math.degrees(math.asin(max(-1,min(1,2*(w*y-z*x)))))
    row=dict(t_sim=f'{t:.3f}',x=p['x'],y=p['y'],z=p['z'],armed=int(f['armed']),state=r.get('mission',''),roll_deg=roll,pitch_deg=pitch,mode=f['mode'])
    i=bisect.bisect_right(ts,t)-1
    if i>=0: row.update(payload_wx=truth[i]['x'],payload_wy=truth[i]['y'],payload_wz=truth[i]['z'])
    rows.append(row)
with (out/'full_track.csv').open('w',newline='') as f:
    writer=csv.DictWriter(f,fieldnames=fields); writer.writeheader(); writer.writerows(rows)
sys.path.insert(0,str(Path('sim').resolve()))
from check_track import load_track,grade,red_polygons,rect_gap,lanes,to_lane
from recording_metrics import recording_metrics
track=load_track(str(out/'full_track.csv'))
layout=json.loads((out/'layout.json').read_text())
target=json.loads((out/'evaluation_target.json').read_text())['target']
(out/'grade_onboard_pose.json').write_text(json.dumps(grade(track,layout,target),indent=2)+'\n')
vehicle_truth=[json.loads(line) for line in (out/'evaluation_vehicle_truth.jsonl').open() if line.strip()]
vehicle_truth.sort(key=lambda r:r['stamp'])
meta_times=[r['t'] for r in track]
evaluated=[]
for pose in vehicle_truth:
    t=pose['stamp']
    if not meta_times or not meta_times[0]<=t<=meta_times[-1]:continue
    i=bisect.bisect_right(meta_times,t)-1
    row=dict(track[i])
    hx,hy=layout.get('home_world',[0,0])
    row.update(t=t,x=pose['x']-hx,y=pose['y']-hy,z=pose['z'])
    w,x,y,z=pose['quaternion']
    row['roll']=math.degrees(math.atan2(2*(w*x+y*z),1-2*(x*x+y*y)))
    row['pitch']=math.degrees(math.asin(max(-1,min(1,2*(w*y-z*x)))))
    j=bisect.bisect_right(ts,t)-1
    if j>=0:row['pay']=(truth[j]['x'],truth[j]['y'],truth[j]['z'])
    evaluated.append(row)
(out/'evaluation_track.json').write_text(json.dumps(evaluated)+'\n')
res=grade(evaluated,layout,target)
from native_contact_evidence import qualify_vehicle_contacts
res['aircraft_contact_evidence']=qualify_vehicle_contacts(out,onboard,vehicle_truth,layout)
res['checks']['no obstacle contact']=bool(res['checks']['no obstacle contact']
    and res['aircraft_contact_evidence']['valid'] and res['aircraft_contact_evidence']['contact_free'])
res['checks']['independent aircraft truth recorded']=len(evaluated)>100
track=evaluated
flying=[r for r in track if r['armed'] and r['z']>.3]
red_clearance=min((rect_gap(poly,r['x'],r['y']) for r in flying for _,poly in red_polygons(layout)),default=math.inf)
corridor_clearances=[]
for r in flying:
    for lane in lanes(layout):
        u,v=to_lane(r,layout,lane)
        if 0<=u<=lane[3] and abs(v)<=lane[4]/2 and r['z']<lane[5]:
            corridor_clearances.append(lane[4]/2-abs(v))
    for obstacle in layout.get('obstacles',[]):
        if r['z']<obstacle['h']+.3:
            hx,hy=layout.get('home_world',[0,0])
            poly=[(x-hx,y-hy) for x,y in obstacle['poly']]
            corridor_clearances.append(rect_gap(poly,r['x'],r['y']))
res['required_red_clearance_m']=1.5
res['required_corridor_clearance_m']=.70
res['measured_red_clearance_m']=red_clearance if math.isfinite(red_clearance) else None
res['measured_corridor_clearance_m']=min(corridor_clearances,default=None)
res['checks']['restricted-zone clearance at least 1.50 m']=red_clearance>=1.5
res['checks']['corridor clearance at least 0.70 m']=bool(corridor_clearances) and min(corridor_clearances)>=.70
res['detector_rates']=recording_metrics(out)
res['checks']['distinct detector rates recorded']=all(r['distinct_captures']>=4 for r in res['detector_rates']['search'].values())
from qualification_evidence import motor_delivery_confirmed,camera_through_disarm
profile=json.loads((out/'frozen_calibration.json').read_text())
motor_qualified=motor_delivery_confirmed(onboard,profile)
res['checks']['motor delivery confirmed with fresh recorded FCU outputs']=motor_qualified
audits=[out/name for name in ('decision_inputs.json','decision_inputs_navigation.json')]
res['checks']['initial and navigation-time decision input audits passed']=all(
    path.exists() and json.loads(path.read_text()).get('pass') is True for path in audits)
res['checks']['complete camera timeline verified through disarm']=bool(
    res.get('final_armed') is False and camera_through_disarm(out,onboard))
res['checks']['runtime configuration and source hashes preserved']=all(
    (out/name).exists() for name in ('runtime_configuration_manifest.json','source_manifest.json','qualification_tools_manifest.json'))
res['pass']=all(res['checks'].values())
(out/'grade.json').write_text(json.dumps(res,indent=2)+'\n')
(out/'grade.txt').write_text(json.dumps(res,indent=2)+'\n'+'\n'.join(f"[{'PASS' if ok else 'FAIL'}] {name}" for name,ok in res['checks'].items())+'\n')
result=json.loads((out/'recorded_result.json').read_text())
summary=dict(target=target,mission_result=result,grade='GRADE_PASS' if res['pass'] else 'GRADE_FAIL',failures=[name for name,ok in res['checks'].items() if not ok],payload_error_m=res.get('payload_error_m'),final_armed=res.get('final_armed'),flight_time_s=res.get('flight_time_s'),landed_from_home_m=res.get('landed_from_home_m'))
summary['acceptance_pass']=res['pass'] and isinstance(result,dict) and result.get('state')=='COMPLETED'
summary['detector_rates']=res['detector_rates']
summary['configuration']=json.loads((out/'configuration.json').read_text())
summary['motor_delivery_qualified']=motor_qualified
(out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
print(json.dumps(summary,indent=2))
raise SystemExit(0 if summary['acceptance_pass'] else 1)
