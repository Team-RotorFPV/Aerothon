"""Start Gazebo Harmonic with the Mission 2 world + ros_gz bridge.

Full sim loop (three terminals):
  1) ArduPilot SITL:   sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --console
  2) This launch:      ros2 launch sim_gazebo mission2_world.launch.py
  3) The stack:        ros2 launch mission_bringup mission2.launch.py use_sim:=true

The ardupilot_gazebo iris model (from aerothon.repos) is spawned into the world
by SITL's gazebo-iris frame; make sure GZ_SIM_RESOURCE_PATH includes the
ardupilot_gazebo models. See docs/RUN_AND_CHECK.md.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = get_package_share_directory('sim_gazebo')
    world = os.path.join(pkg, 'worlds', 'mission2.sdf')
    bridge_cfg = os.path.join(pkg, 'config', 'gz_bridge.yaml')

    gui = LaunchConfiguration('gui')

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])),
        launch_arguments={'gz_args': f'"{world}" -r'}.items(),
    )

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        parameters=[{'config_file': bridge_cfg, 'use_sim_time': True}],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        gz, bridge,
    ])
