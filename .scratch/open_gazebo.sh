#!/usr/bin/env bash
set -e
export GZ_PARTITION=aerothon_m2
export GZ_IP=127.0.0.1
export GALLIUM_DRIVER=d3d12
export QSG_RENDER_LOOP=threaded
export GZ_SIM_RESOURCE_PATH=/tmp/aerothon_vehicle_models:/home/sarthak/ardupilot_gazebo/models:/home/sarthak/ardupilot_gazebo/worlds
export GZ_SIM_SYSTEM_PLUGIN_PATH=/home/sarthak/ardupilot_gazebo/build
# This installed upstream model nests the airframe instead of merging it.
# Adapt only the generated viewing model to its actual frame names.
/usr/bin/python3 - <<'PY'
import xml.etree.ElementTree as ET
p = '/tmp/aerothon_vehicle_models/aerothon_iris_c1_webcam/model.sdf'
t = ET.parse(p)
for joint in t.findall('./model/joint'):
    parent = joint.find('parent')
    if parent is not None and parent.text == 'base_link':
        parent.text = 'iris_with_standoffs::base_link'
model = t.find('model')
for plugin in list(model.findall('plugin')):
    if any((n.text or '').startswith('gimbal::') for n in plugin.iter('joint_name')):
        model.remove(plugin)
t.write(p)
PY
exec gz sim --render-engine-gui ogre -v 2 /tmp/aerothon_mission2_runtime.sdf
