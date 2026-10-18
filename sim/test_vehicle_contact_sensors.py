"""Evaluation sensors cover colliders without changing aircraft dynamics."""
import copy,sys
from pathlib import Path
import xml.etree.ElementTree as ET
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from vehicle_contact_sensors import attach_vehicle_contact_sensors,SENSOR_NAME


def test_all_body_collision_shapes_covered_without_geometry_or_mass_changes():
    model=ET.Element('model',name='test')
    for name in ('base_link','rotor_0','rotor_1','rotor_2','rotor_3'):
        link=ET.SubElement(model,'link',name=name)
        ET.SubElement(link,'pose').text='1 2 3 0 0 0'
        ET.SubElement(ET.SubElement(link,'inertial'),'mass').text='1.94'
        for collider in ('core','arm'):
            collision=ET.SubElement(link,'collision',name=collider)
            ET.SubElement(collision,'geometry').text='unchanged'
    before=ET.tostring(model)
    result=attach_vehicle_contact_sensors(model)
    assert result['complete_body_coverage']
    assert all(c['collisions']==['core','arm'] for c in result['channels'])
    without_sensors=copy.deepcopy(model)
    for link in without_sensors.findall('link'):
        link.remove(link.find(f"sensor[@name='{SENSOR_NAME}']"))
    assert ET.tostring(without_sensors)==before
    attach_vehicle_contact_sensors(model)
    assert all(len(link.findall(f"sensor[@name='{SENSOR_NAME}']"))==1 for link in model.findall('link'))


def test_unresolved_included_airframe_does_not_claim_body_coverage():
    model=ET.fromstring('<model name="iris"><include><uri>model://iris</uri></include><link name="camera"><collision name="lens"/></link></model>')
    result=attach_vehicle_contact_sensors(model)
    assert not result['complete_body_coverage']
    assert result['required_body_links_missing']
