#!/usr/bin/env python3
"""Passive native vehicle contacts and channel liveness; no ROS or commands."""
import argparse
import json
import threading
import time
import xml.etree.ElementTree as ET
from pathlib import Path


def sensor_channels(path,world='mission2',entity='aerothon_iris'):
    model=ET.parse(path).getroot().find('model')
    channels=[]
    for link in model.findall('link'):
        collisions={c.get('name') for c in link.findall('collision')}
        if not collisions:continue
        sensor=link.find("sensor[@name='qualification_vehicle_contact']")
        if sensor is None:raise ValueError(f"Uninstrumented collidable link: {link.get('name')}")
        covered={c.text for c in sensor.findall('contact/collision')}
        if covered!=collisions:raise ValueError(f"Incomplete collider coverage: {link.get('name')}")
        # Harmonic's Contact system publishes its entity-scoped topic rather
        # than honoring Sensor.topic (verified by the native startup log).
        topic=f'/world/{world}/model/{entity}/link/{link.get("name")}/sensor/{sensor.get("name")}/contact'
        channels.append(dict(link=link.get('name'),topic=topic,collisions=sorted(collisions)))
    required={'base_link','rotor_0','rotor_1','rotor_2','rotor_3'}
    if not required.issubset({c['link'] for c in channels}):
        raise ValueError('Direct native airframe collision coverage is incomplete')
    return channels


def main():
    from gz.transport13 import Node
    from gz.msgs10.contacts_pb2 import Contacts
    from gz.msgs10.clock_pb2 import Clock
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--model',type=Path,required=True)
    p.add_argument('--out',type=Path,required=True)
    p.add_argument('--ready',type=Path,required=True)
    p.add_argument('--until-file',type=Path,required=True)
    p.add_argument('--timeout',type=float,default=7200)
    p.add_argument('--world',default='mission2')
    p.add_argument('--entity',default='aerothon_iris')
    args=p.parse_args();channels=sensor_channels(args.model,args.world,args.entity)
    native=Node();lock=threading.Lock();clock=[None];last={};seen=set();stopped=[False]
    f=args.out.open('x');callbacks=[]
    def receive(channel):
        def callback(msg):
            stamp=msg.header.stamp.sec+msg.header.stamp.nsec*1e-9
            pairs=[dict(collision1=c.collision1.name,collision2=c.collision2.name,depth=list(c.depth),
                        positions=[dict(x=p.x,y=p.y,z=p.z) for p in c.position]) for c in msg.contact]
            signature=tuple((r['collision1'],r['collision2']) for r in pairs)
            with lock:
                if stopped[0]:return
                seen.add(channel['topic'])
                previous=last.get(channel['topic'])
                if previous and 0<=stamp-previous[0]<.05 and signature==previous[1]:return
                last[channel['topic']]=(stamp,signature)
                f.write(json.dumps(dict(kind='contacts',stamp=stamp,link=channel['link'],topic=channel['topic'],contacts=pairs))+'\n');f.flush()
        return callback
    for channel in channels:
        callback=receive(channel);callbacks.append(callback)
        if not native.subscribe(Contacts,channel['topic'],callback):raise SystemExit('Native contact subscription failed')
    def receive_clock(msg):
        with lock:clock[0]=msg.sim.sec+msg.sim.nsec*1e-9
    if not native.subscribe(Clock,'/world/mission2/clock',receive_clock):raise SystemExit('Native clock subscription failed')
    started=time.monotonic();last_heartbeat=None
    try:
        while time.monotonic()-started<args.timeout and not args.until_file.exists():
            advertised=set(native.topic_list());available=[c['topic'] for c in channels if c['topic'] in advertised]
            with lock:
                stamp=clock[0]
                if stamp is not None and (last_heartbeat is None or stamp-last_heartbeat>=.5):
                    f.write(json.dumps(dict(kind='heartbeat',stamp=stamp,channels=channels,advertised_topics=available,contact_topics_seen=sorted(seen),source='Gazebo native transport; evaluation only'))+'\n');f.flush();last_heartbeat=stamp
                if stamp is not None and len(available)==len(channels) and not args.ready.exists():
                    with args.ready.open('x') as ready:json.dump(dict(stamp=stamp,channels=channels,all_topics_advertised=True),ready,indent=2)
            time.sleep(.05)
    except KeyboardInterrupt:pass
    finally:
        advertised=set(native.topic_list())
        with lock:
            stopped[0]=True
            if clock[0] is not None and (last_heartbeat is None or clock[0]>last_heartbeat):
                f.write(json.dumps(dict(kind='heartbeat',stamp=clock[0],channels=channels,
                    advertised_topics=[c['topic'] for c in channels if c['topic'] in advertised],
                    contact_topics_seen=sorted(seen),source='Gazebo native transport; final observed clock'))+'\n');f.flush()
        for channel in channels:native.unsubscribe(channel['topic'])
        native.unsubscribe('/world/mission2/clock')
        # Keep callback references alive until the native subscriptions are
        # gone, then destroy the transport node on the Python main thread.
        del native
        with lock:f.close()


if __name__=='__main__':main()
