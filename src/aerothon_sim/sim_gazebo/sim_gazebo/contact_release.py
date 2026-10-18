"""Simulator mechanics only: detach a supported payload. No ROS interface."""
import json
import time


def surface_contact(collision1, collision2):
    collision1 = getattr(collision1, 'name', collision1)
    collision2 = getattr(collision2, 'name', collision2)
    pair = (collision1, collision2)
    payload = [s for s in pair if 'aerothon_payload::body::collision' in s]
    if len(payload) != 1:
        return False
    other = pair[1] if payload[0] == pair[0] else pair[0]
    model = other.split('::')[-3] if len(other.split('::')) >= 3 else ''
    return (model in ('site_ground', 'ground_plane', 'delivery_zone', 'delivery_floor',
                     'delivery_zone_40x30', 'takeoff_landing_zone')
            or model.startswith('target_qr_') or model.startswith('qr_pad_'))


def main():
    from gz.transport13 import Node
    from gz.msgs10.contacts_pb2 import Contacts
    from gz.msgs10.empty_pb2 import Empty
    node = Node()
    pub = node.advertise('/aerothon/payload/detach', Empty)
    pending = [0]
    released = [False]

    def contact(msg):
        if not released[0] and any(surface_contact(c.collision1, c.collision2) for c in msg.contact):
            released[0] = True
            pending[0] = 5
            print(json.dumps({'event': 'simulator_contact_detach',
                              'basis': 'contact sensor; simulator-only evaluation'}), flush=True)
    node.subscribe(Contacts, '/world/mission2/model/aerothon_payload/link/body/sensor/payload_contact/contact', contact)
    try:
        while True:
            if pending[0]:
                pub.publish(Empty())
                pending[0] -= 1
            time.sleep(0.02)
    except KeyboardInterrupt:
        pass
