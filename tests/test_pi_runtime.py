import asyncio
import json
import math
import time
from types import SimpleNamespace

import pytest
from aiohttp.test_utils import TestClient, TestServer
from aerothon_runtime.contracts import execution, fcu_url, profile
from aerothon_runtime.device_wire import pack, unpack
from aerothon_runtime.pilot import intent, validate
from aerothon_runtime.web import Host, application


def remote(**kw):
    return profile(dict(id='test',host='laptop.local',gateway='http://laptop.local:8090',**kw))

def rc():
    p={k:{'channel':i+1,'min':1000,'trim':1500,'max':2000} for i,k in enumerate(['forward','left','yaw','climb','takeover'])}
    p['takeover']['active_above']=1700
    return validate(dict(p,max_speed=1,max_yaw_rate=.5,max_climb_rate=.5))

@pytest.mark.parametrize('transport,url',[('tcp','tcp://laptop.local:5762'),('udp','udp://0.0.0.0:14555@laptop.local:5762')])
def test_configurable_fcu_address(transport,url):
    assert fcu_url(remote(transport=transport))==url

@pytest.mark.parametrize('kwargs',[{'host':'$(touch /tmp/no)'},{'port':float('nan')},{'system_id':0},{'gateway':'http://a/b'},{'gateway':''}])
def test_invalid_connections_rejected(kwargs):
    p=dict(id='test',host='laptop.local',gateway='http://laptop.local:8090');p.update(kwargs)
    with pytest.raises(ValueError):profile(p)

def test_local_hardware_profile_uses_same_contract():
    p=profile(dict(id='hardware',transport='serial',device='/dev/ttyAMA0'))
    assert fcu_url(p)=='/dev/ttyAMA0:115200'
    assert p['gateway']==''
    assert p['vehicle']=='drone'

def test_vehicle_kind_is_explicit_and_consistent():
    assert remote()['vehicle']=='simulation'
    # A real drone over a network link (telemetry router) needs no gateway.
    drone=profile(dict(id='radio',vehicle='drone',transport='udp',host='192.168.0.20'))
    assert drone['vehicle']=='drone' and drone['gateway']==''
    with pytest.raises(ValueError):profile(dict(id='x',vehicle='simulation',transport='serial',device='/dev/ttyAMA0'))
    with pytest.raises(ValueError):remote(vehicle='drone')
    with pytest.raises(ValueError):remote(vehicle='rover')

def test_gpio_winch_is_a_real_drone_choice_only():
    assert remote()['winch_backend']=='mavlink'
    drone=profile(dict(id='hw',transport='serial',device='/dev/ttyAMA0',winch_backend='gpio'))
    assert drone['winch_backend']=='gpio'
    with pytest.raises(ValueError):remote(winch_backend='gpio')
    with pytest.raises(ValueError):profile(dict(id='hw',transport='serial',device='/dev/ttyAMA0',winch_backend='servo'))

@pytest.mark.parametrize('case',[{'task':'delivery_here','params':{}},{'task':'banner','params':{'height':math.inf}},{'task':'avoidance_route','params':{'waypoints':[]}},{'task':'mechanism','params':{'action':'arbitrary'}}])
def test_invalid_execution_cannot_reach_controller(case):
    with pytest.raises(ValueError):execution(case)

def test_device_packets_bind_session_and_reject_truth_topics():
    body=b'actual captured device bytes'
    assert unpack(pack('scan','vehicle-session',body),'vehicle-session')==('scan',body)
    with pytest.raises(ValueError):unpack(pack('scan','old-session',body),'new-session')
    with pytest.raises(ValueError):pack('payload_truth','x',body)
    with pytest.raises(ValueError):unpack(b'\0\0\xff\xff','x')

def test_transmitter_intent_and_takeover_use_configured_channels():
    p=rc();v=intent([2000,1000,1500,1500,1800],p)
    assert v==dict(takeover=True,forward=1,left=-1,yaw=0,climb=0)
    assert intent([1500]*5,p)['forward']==0
    with pytest.raises(ValueError):intent([1500],p)
    with pytest.raises(ValueError):validate({})

class Wire:
    def __init__(self):self.sent=[]
    async def send_json(self,data):self.sent.append(data);await asyncio.sleep(0)

def host(tmp_path):
    h=Host(tmp_path,tmp_path/'state')
    h.telemetry={'mission':{'armed':False},'safety':{'ready':True,'fcu_connected':True},'runtime_health':{'fresh':True},'sample_age_s':{'state':0}}
    h.received=time.monotonic();h.internal_ws=Wire()
    return h

@pytest.mark.asyncio
async def test_duplicate_start_is_durable_and_not_resubmitted(tmp_path):
    h=host(tmp_path);data={'command_id':'unique_start_1','cmd':'start','args':{'task':'qr'}}
    a,b=await asyncio.gather(h.command(data),h.command(data))
    assert a==b and len(h.internal_ws.sent)==1
    h.db.close()
    reboot=host(tmp_path)
    r=await reboot.command(data)
    assert r['state']=='Outcome unknown'
    assert not reboot.internal_ws.sent
    reboot.db.close()

@pytest.mark.asyncio
async def test_duplicate_id_with_changed_payload_rejected(tmp_path):
    h=host(tmp_path);data={'command_id':'unique_start_1','cmd':'start','args':{'task':'qr'}}
    await h.command(data)
    with pytest.raises(ValueError):await h.command({**data,'args':{'task':'slam'}})
    h.db.close()

@pytest.mark.asyncio
async def test_two_starts_have_one_owner(tmp_path):
    h=host(tmp_path)
    await h.command({'command_id':'unique_start_1','cmd':'start','args':{'task':'qr'}})
    with pytest.raises(ValueError):await h.command({'command_id':'unique_start_2','cmd':'start','args':{'task':'slam'}})
    assert len(h.internal_ws.sent)==1
    h.db.close()

@pytest.mark.asyncio
async def test_stale_link_cannot_submit_flight_command(tmp_path):
    h=host(tmp_path);h.received-=4
    with pytest.raises(ValueError):await h.command({'command_id':'arm_request_1','cmd':'arm'})
    assert not h.internal_ws.sent
    h.db.close()

@pytest.mark.asyncio
async def test_fresh_browser_heartbeat_cannot_hide_stale_fcu_or_allow_airborne_disarm(tmp_path):
    h=host(tmp_path)
    h.telemetry['sample_age_s']['state']=4
    with pytest.raises(ValueError):await h.command({'command_id':'stale_fcu_arm_1','cmd':'arm'})
    h.telemetry['sample_age_s']['state']=0
    with pytest.raises(ValueError):await h.command({'command_id':'airborne_disarm_1','cmd':'disarm'})
    assert not h.internal_ws.sent
    h.db.close()

@pytest.mark.asyncio
async def test_mutating_profile_blocked_until_live_disarm_and_runtime_stop(tmp_path):
    h=host(tmp_path)
    h.proc=SimpleNamespace(returncode=None)
    assert not h.editable()
    h.proc=None;h.profile_id='test';h.telemetry['mission']['armed']=True
    assert not h.editable()
    h.telemetry['mission']['armed']=False
    assert h.editable()
    h.received-=4
    assert not h.editable()
    h.db.close()

@pytest.mark.asyncio
async def test_connection_check_never_fabricates_missing_capabilities(tmp_path):
    h=host(tmp_path);r=await h.check()
    assert next(x for x in r['items'] if x['key']=='camera_actuator')['ok'] is False
    assert next(x for x in r['items'] if x['key']=='identity')['ok'] is False
    h.db.close()

@pytest.mark.asyncio
async def test_browser_api_static_serving_and_cross_origin_controls(tmp_path):
    (tmp_path/'index.html').write_text('<html>Pi GCS</html>')
    h=Host(tmp_path,tmp_path/'state',aggregator='http://127.0.0.1:1')
    async with TestClient(TestServer(application(h))) as client:
        r=await client.get('/');assert r.status==200 and 'Pi GCS' in await r.text()
        r=await client.get('/api/state');s=await r.json();assert s['runtime']=='Stopped'
        r=await client.post('/api/profiles',json=remote());assert r.status==200
        r=await client.post('/api/profiles',json=remote(),headers={'Origin':'http://different-site'});assert r.status==403
        r=await client.post('/api/commands',json={'command_id':'unique_start_1','cmd':'start','args':{'task':'mission'}});assert r.status==400


def test_browser_assets_use_same_origin_and_no_tauri_runtime():
    from pathlib import Path
    p=Path(__file__).parents[1]/'src/aerothon_gcs/tauri_app/src/App.tsx'
    text=p.read_text()
    runtime=(Path(__file__).parents[1]/'src/aerothon_gcs/tauri_app/src/runtime.ts').read_text()
    assert 'location.host' in text and '/video/stream' in runtime
    assert '127.0.0.1' not in text and '@tauri-apps/api' not in text

def test_process_lock_rejects_a_second_commander(tmp_path):
    from aerothon_runtime.ownership import acquire
    path=tmp_path/'owner.lock'
    first=acquire(path)
    try:
        with pytest.raises(RuntimeError):acquire(path)
    finally:first.close()
    second=acquire(path);second.close()

@pytest.mark.asyncio
async def test_start_requires_a_live_execution_service(tmp_path):
    h=host(tmp_path);h.telemetry['runtime_health']={'fresh':False}
    with pytest.raises(ValueError):await h.command({'command_id':'missing_runner_1','cmd':'start','args':{'task':'qr'}})
    assert not h.internal_ws.sent
    h.db.close()

@pytest.mark.asyncio
async def test_rejected_unready_start_never_marks_unused_connection_as_controlled(tmp_path):
    h=host(tmp_path);h.telemetry['safety']['ready']=False
    with pytest.raises(ValueError):await h.command({'command_id':'unready_start_1','cmd':'start','args':{'task':'mission'}})
    assert not h.controlled and not h.internal_ws.sent
    h.db.close()
