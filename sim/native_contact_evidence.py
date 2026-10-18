"""Qualify live native vehicle contacts independently of geometric proxies."""
import bisect
import json
import math
from pathlib import Path
from record_vehicle_contacts import sensor_channels


def tilt_deg(quaternion):
    """Angle between the airframe's own up axis and world up.

    Independent truth records orientation as [w,x,y,z] (delivery_truth.py).
    """
    _,x,y,z=quaternion
    return math.degrees(math.acos(max(-1.,min(1.,1-2*(x*x+y*y)))))


def settled_touchdown(poses,index):
    """Independent truth showing a gentle level touchdown, not an impact."""
    if index is None:return False
    speeds=[]
    for j in (index-1,index+1):
        if not 0<=j<len(poses):continue
        a,b=sorted((poses[index],poses[j]),key=lambda r:r['stamp'])
        gap=b['stamp']-a['stamp']
        if not 0<gap<=.3:continue
        speeds.append(math.dist((a['x'],a['y'],a['z']),(b['x'],b['y'],b['z']))/gap)
    quaternion=poses[index].get('quaternion')
    if (not speeds or not isinstance(quaternion,list) or len(quaternion)!=4
            or not all(isinstance(v,(int,float)) and math.isfinite(v) for v in quaternion)):
        return False
    return max(speeds)<=1. and tilt_deg(quaternion)<=10.


def landing_mode(state,stamp):
    """Fresh onboard FCU evidence that a contact belongs to the landing."""
    flight=state.get('flight') or {}
    data=flight.get('data') or {}
    receipt=flight.get('received_stamp')
    return bool(data.get('connected') is True and data.get('mode')=='LAND'
                and isinstance(receipt,(int,float)) and math.isfinite(receipt)
                and 0<=stamp-receipt<=1.5           # MAVROS publishes state at 1 Hz
                and 0<=stamp-state.get('stamp',float('-inf'))<=.3)


def commanded_landing_contract(out,home):
    """The land-in-place a focused trial declared before it touched down.

    A full mission lands at home and writes no contract, so it keeps the home
    rule alone. A focused trial that deliberately lands where its own segment
    ends records the commanded spot from its onboard FCU pose, before any
    touchdown; this returns it in world coordinates for the independent
    check. A contract carrying no usable anchor earns no exemption at all,
    and a malformed one fails the whole channel closed.
    """
    path=Path(out)/'landing_contract.json'
    if not path.exists():return None
    contract=json.loads(path.read_text())
    if contract.get('anchor') is None:return None
    anchor=contract['anchor'];radius=contract.get('radius_m');commanded=contract.get('commanded_stamp')
    if (contract.get('frame')!='home_local_enu' or not isinstance(anchor,list) or len(anchor)!=2
            or any(isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v)
                   for v in (*anchor,radius,commanded))
            or not 0<radius<=2. or commanded<0):
        raise ValueError('Malformed commanded landing contract')
    return dict(anchor_world=(anchor[0]+home[0],anchor[1]+home[1]),
                radius_m=radius,commanded_stamp=commanded)


def qualify_vehicle_contacts(out,onboard,vehicle_truth,layout):
    result=dict(valid=False,contact_free=False,failures=[],unexpected_contacts=[],ground_canary=False)
    try:
        out=Path(out);channels=sensor_channels(out/'flight_vehicle.sdf')
        expected={c['topic'] for c in channels};inventory={c['link']:c for c in channels}
        ready=json.loads((out/'vehicle_contact_ready.json').read_text())
        if ready['channels']!=channels or ready['all_topics_advertised'] is not True:
            raise ValueError('Readiness channel inventory does not match flown vehicle')
        rows=[json.loads(line) for line in (out/'evaluation_vehicle_contacts.jsonl').read_text().splitlines()]
        armed=[r['stamp'] for r in onboard if r.get('flight',{}).get('data',{}).get('armed') is True]
        if not armed:raise ValueError('No actual armed flight interval')
        first,last=min(armed),max(armed)
        heartbeats=[r for r in rows if r['kind']=='heartbeat']
        stamps=[r['stamp'] for r in heartbeats]
        if (len(stamps)<2 or any(not math.isfinite(t) or t<0 for t in stamps)
                or any(b<=a or b-a>1.5 for a,b in zip(stamps,stamps[1:]))
                or stamps[0]>first or stamps[-1]<last):
            raise ValueError('Native clock heartbeats do not cover flight continuously')
        if any(r['channels']!=channels or set(r['advertised_topics'])!=expected for r in heartbeats):
            raise ValueError('Missing native vehicle contact publisher')
        poses=sorted(vehicle_truth,key=lambda r:r['stamp']);times=[r['stamp'] for r in poses]
        telemetry=sorted(onboard,key=lambda r:r['stamp']);telemetry_times=[r['stamp'] for r in telemetry]
        target=json.loads((out/'evaluation_target.json').read_text())['target'] if (out/'evaluation_target.json').exists() else None
        pad=layout.get('pads',{}).get(target)
        deployment_pairs=0;landing_pairs=0
        home=layout.get('home_world',[0,0]);allowed=0
        landing=commanded_landing_contract(out,home)
        for row in rows:
            if row['kind']=='heartbeat':continue
            if row['kind']!='contacts':raise ValueError('Unknown evidence record')
            stamp=row['stamp'];channel=inventory[row['link']]
            if not math.isfinite(stamp) or stamp<0 or row['topic']!=channel['topic']:
                raise ValueError('Invalid native contact channel or stamp')
            if not isinstance(row['contacts'],list):raise ValueError('Malformed contact list')
            own={f'aerothon_iris::{row["link"]}::{name}' for name in channel['collisions']}
            for pair in row['contacts']:
                a,b=pair['collision1'],pair['collision2']
                if (not isinstance(a,str) or not isinstance(b,str) or not a or not b
                        or not isinstance(pair['depth'],list)
                        or any(not math.isfinite(d) for d in pair['depth'])):
                    raise ValueError('Malformed native contact pair')
                if a in own:vehicle,other=a,b
                elif b in own:vehicle,other=b,a
                else:raise ValueError('Contact does not belong to recorded vehicle collider')
                i=bisect.bisect_left(times,stamp)
                choices=[j for j in (i-1,i) if 0<=j<len(poses)]
                index=min(choices,key=lambda j:abs(times[j]-stamp)) if choices else None
                pose=poses[index] if index is not None else None
                fresh_pose=bool(pose and abs(pose['stamp']-stamp)<=.3)
                skid=vehicle in {'aerothon_iris::base_link::skid_0','aerothon_iris::base_link::skid_1'}
                resting_surface=other in {'takeoff_landing_zone::pad::collision','site_ground::ground::collision'}
                grounded=bool(fresh_pose and pose['z']<=.45
                    and math.hypot(pose['x']-home[0],pose['y']-home[1])<=1.)
                normal_ground=grounded and skid and resting_surface
                expected_grip=(row['link'] in ('claw_jaw_a','claw_jaw_b')
                    and other.startswith('aerothon_payload::'))
                positions=pair.get('positions',[])
                if not isinstance(positions,list) or any(not all(math.isfinite(p[k]) for k in ('x','y','z')) for p in positions):
                    raise ValueError('Malformed native contact positions')
                j=bisect.bisect_right(telemetry_times,stamp)-1
                state=telemetry[j] if j>=0 else {}
                winch=state.get('winch_capture',{})
                winch_data=winch.get('data',{})
                receipt=winch.get('received_stamp',float('-inf'))
                # The deployed winch is meant to put its hook and cargo on
                # the target surface while the airframe remains airborne.
                # Retain these contacts as evidence; never exempt aircraft
                # colliders, walls, stale/stowed winches or other locations.
                deployment_surface=bool(row['link'] in ('claw_jaw_a','claw_jaw_b')
                    and other in ('delivery_zone_40x30::field::collision','site_ground::ground::collision')
                    and state.get('mission')=='WINCH_DROP' and pose and pose['z']>=2.
                    and abs(pose['stamp']-stamp)<=.3
                    and 0<=stamp-state.get('stamp',float('-inf'))<=.3
                    and math.isfinite(receipt) and 0<=stamp-receipt<=.3
                    and winch_data.get('state') in ('LOWERING','IDLE','STOWING','RELEASING')
                    and not winch_data.get('fault') and winch_data.get('payout_estimate_m',0)>=1.
                    and pad and positions and all(abs(p['z'])<=.2 and math.hypot(p['x']-pad[0],p['y']-pad[1])<=1. for p in positions))
                if deployment_surface:deployment_pairs+=1
                # A focused trial lands where its own segment ends instead of
                # flying home, and declares that spot from the onboard FCU
                # before touching down. Accept the skids settling onto the
                # declared spot; a contact on any other collider or surface,
                # away from it, before the command, outside the commanded
                # landing mode, or at impact speed or attitude, stays
                # unexpected. Where the aircraft landed remains the mission
                # grader's own check, not an exemption here.
                commanded_landing=bool(landing and skid and resting_surface and fresh_pose
                    and pose['z']<=.45 and stamp>=landing['commanded_stamp']
                    and math.hypot(pose['x']-landing['anchor_world'][0],
                                   pose['y']-landing['anchor_world'][1])<=landing['radius_m']
                    and settled_touchdown(poses,index) and landing_mode(state,stamp))
                if commanded_landing:landing_pairs+=1
                if normal_ground or expected_grip or deployment_surface or commanded_landing:
                    allowed+=1
                    if normal_ground and stamp<first:result['ground_canary']=True
                else:result['unexpected_contacts'].append(dict(stamp=stamp,vehicle=vehicle,other=other,depth=pair['depth']))
        if not result['ground_canary']:raise ValueError('Missing preflight native skid contact canary')
        result.update(valid=True,contact_free=not result['unexpected_contacts'],channels=len(channels),
                      heartbeat_count=len(heartbeats),allowed_ground_or_grip_pairs=allowed,
                      intended_deployed_hook_target_surface_pairs=deployment_pairs,
                      declared_landing_pairs=landing_pairs,
                      declared_landing=landing,
                      first_armed_stamp=first,last_armed_stamp=last)
    except (OSError,ValueError,KeyError,TypeError,IndexError) as e:
        result['failures'].append(str(e))
    return result
