"""Passive native contact channels for qualification, never flight inputs."""
import xml.etree.ElementTree as ET

SENSOR_NAME='qualification_vehicle_contact'
REQUIRED_BODY_LINKS=frozenset(('base_link','rotor_0','rotor_1','rotor_2','rotor_3'))


def attach_vehicle_contact_sensors(model):
    channels=[]
    for link in model.findall('link'):
        collisions=[c.get('name') for c in link.findall('collision')]
        if not collisions:
            continue
        existing=link.find(f"sensor[@name='{SENSOR_NAME}']")
        if existing is not None:
            link.remove(existing)
        name=link.get('name')
        sensor=ET.SubElement(link,'sensor',name=SENSOR_NAME,type='contact')
        # The native Contact system derives its actual runtime topic from the
        # world/entity names; the recorder verifies those advertised topics.
        ET.SubElement(sensor,'always_on').text='true'
        # Every physics step, with repeated pairs throttled only by the
        # evidence recorder. A transient impact must not wait for a camera.
        ET.SubElement(sensor,'update_rate').text='0'
        contact=ET.SubElement(sensor,'contact')
        for collision in collisions:
            ET.SubElement(contact,'collision').text=collision
        channels.append(dict(link=name,native_topic_suffix=f'/link/{name}/sensor/{SENSOR_NAME}/contact',collisions=collisions))
    covered={c['link'] for c in channels}
    missing=sorted(REQUIRED_BODY_LINKS-covered)
    return dict(scope='Native evaluation sensors only; no ROS bridge or flight subscriptions',
                channels=channels,required_body_links_missing=missing,
                complete_body_coverage=not missing)
