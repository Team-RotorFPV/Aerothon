import json,sys
from pathlib import Path
import pytest
import xml.etree.ElementTree as ET
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'sim'))
from record_vehicle_contacts import sensor_channels
from native_contact_evidence import qualify_vehicle_contacts


@pytest.fixture
def recording(tmp_path):
    sdf=ET.Element('sdf',version='1.9');model=ET.SubElement(sdf,'model',name='aerothon_iris')
    for name in ('base_link','rotor_0','rotor_1','rotor_2','rotor_3','claw_jaw_a','claw_jaw_b'):
        link=ET.SubElement(model,'link',name=name)
        for collider in ('core','arm_0','skid_0','skid_1') if name=='base_link' else ('collider',):
            ET.SubElement(link,'collision',name=collider)
        sensor=ET.SubElement(link,'sensor',name='qualification_vehicle_contact',type='contact')
        contact=ET.SubElement(sensor,'contact')
        for c in link.findall('collision'):ET.SubElement(contact,'collision').text=c.get('name')
    ET.ElementTree(sdf).write(tmp_path/'flight_vehicle.sdf')
    channels=sensor_channels(tmp_path/'flight_vehicle.sdf')
    (tmp_path/'vehicle_contact_ready.json').write_text(json.dumps(dict(channels=channels,all_topics_advertised=True)))
    rows=[dict(kind='heartbeat',stamp=i/2,channels=channels,advertised_topics=[c['topic'] for c in channels]) for i in range(2,11)]
    body=next(c for c in channels if c['link']=='base_link')
    rows.append(dict(kind='contacts',stamp=1.,link='base_link',topic=body['topic'],contacts=[dict(collision1='aerothon_iris::base_link::skid_0',collision2='takeoff_landing_zone::pad::collision',depth=[.001])]))
    poses=[dict(stamp=i/10,x=0,y=0,z=.25 if i<20 or i>=48 else 3.,
                quaternion=[1.,0.,0.,0.]) for i in range(5,56)]
    onboard=[dict(stamp=t,flight=dict(data=dict(armed=t<5))) for t in (2.,3.,4.,5.)]
    return tmp_path,rows,onboard,poses,dict(home_world=[0,0])


def evaluate(recording):
    out,rows,onboard,poses,layout=recording
    (out/'evaluation_vehicle_contacts.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
    return qualify_vehicle_contacts(out,onboard,poses,layout)


def test_native_ground_canary_and_live_empty_airborne_channels_pass(recording):
    assert evaluate(recording)['contact_free']


@pytest.mark.parametrize('collider',['core','arm_0','skid_1'])
def test_board_contact_overrides_geometry_for_each_body_collider(recording,collider):
    row=dict(recording[1][-1]);row.update(stamp=3.,contacts=[dict(collision1=f'aerothon_iris::base_link::{collider}',collision2='return_banner::board::collision',depth=[.001])])
    recording[1].append(row)
    result=evaluate(recording)
    assert result['valid'] and not result['contact_free']
    assert result['unexpected_contacts'][0]['stamp']==3.


@pytest.mark.parametrize('fault',['missing_publisher','gap','missing_canary','wrong_collider','bad_stamp','late_start'])
def test_incomplete_or_malformed_native_evidence_fails_closed(recording,fault):
    rows=recording[1]
    if fault=='missing_publisher':rows[2]['advertised_topics']=[]
    elif fault=='gap':rows[:]=[rows[0],rows[-2],rows[-1]]
    elif fault=='missing_canary':rows.pop()
    elif fault=='wrong_collider':rows[-1]['contacts'][0]['collision1']='other_aircraft::base_link::skid_0'
    elif fault=='bad_stamp':rows[-1]['stamp']=float('nan')
    elif fault=='late_start':rows[:]=[r for r in rows if r['stamp']>=3.]
    result=evaluate(recording)
    assert not result['valid'] and not result['contact_free']


def test_ground_contact_in_corridor_is_not_normal_home_landing(recording):
    for pose in recording[3]:pose.update(x=8,z=.25)
    result=evaluate(recording)
    assert not result['contact_free']


def test_claw_payload_contact_is_expected_but_core_payload_impact_is_not(recording):
    row=dict(recording[1][-1]);row.update(stamp=3.)
    row['contacts']=[dict(collision1='aerothon_iris::base_link::core',collision2='aerothon_payload::body::body_collision',depth=[.001])]
    recording[1].append(row)
    assert not evaluate(recording)['contact_free']


def deployed_hook(recording):
    out,rows,onboard,poses,layout=recording
    (out/'evaluation_target.json').write_text(json.dumps(dict(target='d')))
    layout['pads']={'d':[0,0]}
    onboard[1].update(mission='WINCH_DROP',winch_capture=dict(received_stamp=2.95,
        data=dict(state='LOWERING',payout_estimate_m=3.,fault='')))
    channel=next(c for c in rows[0]['channels'] if c['link']=='claw_jaw_a')
    row=dict(kind='contacts',stamp=3.,link='claw_jaw_a',topic=channel['topic'],
        contacts=[dict(collision1='aerothon_iris::claw_jaw_a::collider',
            collision2='delivery_zone_40x30::field::collision',depth=[.0001],positions=[dict(x=.1,y=.1,z=.08)])])
    rows.append(row)
    return row


def test_deployed_hook_target_surface_contact_is_recorded_as_intended(recording):
    deployed_hook(recording)
    result=evaluate(recording)
    assert result['valid'] and result['contact_free']
    assert result['intended_deployed_hook_target_surface_pairs']==1


@pytest.mark.parametrize('fault',['core','wall','stowed','stale','other_phase','away_from_pad','missing_positions','high_contact','stale_airframe_pose'])
def test_deployment_exception_never_hides_aircraft_or_unqualified_contacts(recording,fault):
    row=deployed_hook(recording);winch=recording[2][1]['winch_capture']
    if fault=='core':
        channel=next(c for c in recording[1][0]['channels'] if c['link']=='base_link')
        row.update(link='base_link',topic=channel['topic'])
        row['contacts'][0]['collision1']='aerothon_iris::base_link::core'
    elif fault=='wall':row['contacts'][0]['collision2']='return_banner::board::collision'
    elif fault=='stowed':winch['data']['payout_estimate_m']=0.
    elif fault=='stale':winch['received_stamp']=2.
    elif fault=='other_phase':recording[2][1]['mission']='RETURN_CORRIDOR'
    elif fault=='away_from_pad':row['contacts'][0]['positions'][0]['x']=2.
    elif fault=='missing_positions':row['contacts'][0]['positions']=[]
    elif fault=='high_contact':row['contacts'][0]['positions'][0]['z']=3.
    elif fault=='stale_airframe_pose':recording[3][:]=[p for p in recording[3] if not 2.5<=p['stamp']<=3.5]
    assert not evaluate(recording)['contact_free']


def declared_landing(recording,**overrides):
    """A focused trial that landed in place at the far end of its segment."""
    out,rows,onboard,poses,layout=recording
    # Home for the canary, airborne down the course, then a 0.5 m/s descent
    # onto the ground 8 m away and at rest there.
    def height(i):
        if i<=15:return .25             # on the home pad
        if i<=29:return 1.              # airborne, away from home
        return max(.2,1.-.05*(i-29))    # 0.5 m/s descent, then at rest
    poses[:]=[dict(stamp=i/10,x=0. if i<=15 else 8.,y=0.,quaternion=[1.,0.,0.,0.],
                   z=height(i)) for i in range(5,56)]
    contract=dict(commanded_stamp=4.5,frame='home_local_enu',anchor=[8.,0.],
                  radius_m=1.5,pose_capture_stamp=4.45,max_age_s=.7,
                  source='onboard FCU local position at the commanded land-in-place')
    contract.update(overrides)
    (out/'landing_contract.json').write_text(json.dumps(contract))
    onboard.append(dict(stamp=5.2,mission='LAND',flight=dict(received_stamp=5.1,
        data=dict(armed=False,connected=True,mode='LAND'))))
    channel=next(c for c in rows[0]['channels'] if c['link']=='base_link')
    row=dict(kind='contacts',stamp=5.3,link='base_link',topic=channel['topic'],
        contacts=[dict(collision1='aerothon_iris::base_link::skid_0',
            collision2='site_ground::ground::collision',depth=[.0004])])
    rows.append(row)
    return row


def test_declared_land_in_place_accepts_the_settled_skid_contact(recording):
    declared_landing(recording)
    result=evaluate(recording)
    assert result['valid'] and result['contact_free'] and result['ground_canary']
    assert result['declared_landing_pairs']==1
    assert result['declared_landing']['commanded_stamp']==4.5


@pytest.mark.parametrize('fault',['no_contract','no_anchor','before_command','away_from_anchor',
                                  'core_collider','wall_surface','airborne','impact_speed',
                                  'tilted','other_mode','disconnected','stale_state','stale_pose'])
def test_declared_landing_never_hides_an_unintended_ground_contact(recording,fault):
    row=declared_landing(recording,**(dict(anchor=None) if fault=='no_anchor' else
                                      dict(anchor=[4.,0.]) if fault=='away_from_anchor' else {}))
    out,rows,onboard,poses,layout=recording
    state=onboard[-1]
    if fault=='no_contract':(out/'landing_contract.json').unlink()
    elif fault=='before_command':row['stamp']=4.4
    elif fault=='core_collider':row['contacts'][0]['collision1']='aerothon_iris::base_link::core'
    elif fault=='wall_surface':row['contacts'][0]['collision2']='return_banner::board::collision'
    elif fault=='airborne':
        for pose in poses:
            if pose['stamp']>=5.:pose['z']=3.
    elif fault=='impact_speed':
        for pose in poses:
            if abs(pose['stamp']-5.2)<1e-9:pose['z']=1.
    elif fault=='tilted':
        for pose in poses:
            if pose['stamp']>=5.:pose['quaternion']=[.9239,0.,.3827,0.]
    elif fault=='other_mode':state['flight']['data']['mode']='GUIDED'
    elif fault=='disconnected':state['flight']['data']['connected']=False
    elif fault=='stale_state':state['flight']['received_stamp']=3.
    elif fault=='stale_pose':poses[:]=[p for p in poses if not 5.<=p['stamp']<=5.6]
    result=evaluate(recording)
    assert result['valid'] and not result['contact_free']
    assert result['declared_landing_pairs']==0


@pytest.mark.parametrize('contract',[dict(frame='world'),dict(radius_m=5.),dict(radius_m=0.),
                                     dict(anchor=[8.]),dict(commanded_stamp=float('nan'))])
def test_malformed_landing_contract_fails_the_channel_closed(recording,contract):
    declared_landing(recording,**contract)
    result=evaluate(recording)
    assert not result['valid'] and not result['contact_free']
