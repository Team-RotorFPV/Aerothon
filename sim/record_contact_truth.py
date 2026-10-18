#!/usr/bin/env python3
"""Independent native Gazebo contact evidence. No ROS or publications."""
import argparse
import json
import threading
import time
from pathlib import Path
from gz.transport13 import Node
from gz.msgs10.contacts_pb2 import Contacts


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--out',type=Path,required=True)
    p.add_argument('--until-file',type=Path,required=True)
    p.add_argument('--timeout',type=float,default=7200)
    args=p.parse_args()
    guard=threading.Lock()
    stopped=[False]
    last=[-1.,None]
    f=args.out.open('x')
    native=Node()
    def contact(msg):
        stamp=msg.header.stamp.sec+msg.header.stamp.nsec*1e-9
        pairs=[dict(collision1=c.collision1.name,collision2=c.collision2.name,
                    depth=list(c.depth)) for c in msg.contact]
        signature=tuple((r['collision1'],r['collision2']) for r in pairs)
        with guard:
            if stopped[0] or (stamp-last[0]<.05 and signature==last[1]):return
            last[:]=[stamp,signature]
            f.write(json.dumps(dict(stamp=stamp,contacts=pairs,
                source='native Gazebo payload contact sensor; evaluation only'))+'\n')
            f.flush()
    topic='/world/mission2/model/aerothon_payload/link/body/sensor/payload_contact/contact'
    if not native.subscribe(Contacts,topic,contact):raise SystemExit('Contact truth subscription failed')
    started=time.monotonic()
    try:
        while time.monotonic()-started<args.timeout and not args.until_file.exists():time.sleep(.1)
    except KeyboardInterrupt:pass
    finally:
        with guard:stopped[0]=True;f.close()


if __name__=='__main__':main()
