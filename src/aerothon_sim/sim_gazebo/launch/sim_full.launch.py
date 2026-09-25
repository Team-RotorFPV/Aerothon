"""Master Simulation Launch File (Level 6)

Brings up:
  1) Gazebo Harmonic GUI with Mission 2 World (arena, corridor, obstacles, QR markers)
  2) ros_gz_bridge (bridges /clock, /scan, /camera, /camera_info)
  3) Full ROS 2 Mission Stack (MAVROS, Perception, Avoidance, Mission BT, GCS Aggregator, Video Server)
  4) slam_toolbox 2D SLAM Mapping
  5) RViz 2 with SLAM & Sensor Visualizations

Usage:
  ros2 launch sim_gazebo sim_full.launch.py
"""
import os
import shutil
import tempfile
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_sim = get_package_share_directory('sim_gazebo')
    pkg_mission = get_package_share_directory('mission_bringup')

    # SITL default parameters.
    #
    # The upstream iris.launch.py default appends dds_udp.parm and
    # dds_use_ns.parm. Those set AP_DDS parameters, and ArduPilot PANICs on a
    # defaults file naming parameters the firmware does not have:
    #
    #   PANIC: Failed to load defaults from .../dds_udp.parm,.../dds_use_ns.parm
    #
    # which crash-loops arducopter, leaving MAVROS at connected:false with no
    # obvious cause. This project reaches the FCU over MAVLink/MAVROS and never
    # uses AP_DDS, so the DDS parameter files are omitted here whether or not
    # the firmware was built with DDS support.
    pkg_ardupilot_sitl = get_package_share_directory('ardupilot_sitl')
    pkg_ardupilot_gazebo = get_package_share_directory('ardupilot_gazebo')
    # Stage our SITL defaults on a path with no spaces in it.
    #
    # ardupilot_sitl builds the arducopter command with shell=True and does not
    # quote its arguments, so every path handed to it must be space-free. This
    # workspace lives under "/mnt/d/MY DOCUMENTS/...", which split the
    # comma-separated --defaults list mid-path; SITL died on startup with
    # "PANIC: Failed to load defaults", and with no flight controller the whole
    # stack reported "no data received" on every MAVLink-derived interlock item
    # -- a failure that names the parameter file nowhere. The two upstream
    # .parm files survive only because their own install paths are clean.
    sitl_params = os.path.join(tempfile.gettempdir(), 'aerothon_sitl.parm')
    shutil.copyfile(os.path.join(pkg_sim, 'config', 'aerothon_sitl.parm'),
                    sitl_params)

    failsafe_params = os.path.join(tempfile.gettempdir(),
                                   'aerothon_failsafe.parm')
    shutil.copyfile(os.path.join(pkg_sim, 'config', 'aerothon_failsafe.parm'),
                    failsafe_params)

    sitl_defaults = ','.join([
        os.path.join(pkg_ardupilot_sitl, 'config', 'default_params', 'copter.parm'),
        os.path.join(pkg_ardupilot_gazebo, 'config', 'gazebo-iris-gimbal.parm'),
        # Last wins: competition-representative battery and GPS, so the Q27
        # interlock sees what the real aircraft presents.
        sitl_params,
        # Rulebook 5.4 item 6 fail-safes, identical to the aircraft's file.
        failsafe_params,
    ])

    world_template = os.path.join(pkg_sim, 'worlds', 'mission2.sdf')
    # Keep the ROS-launch-owned world separate from the master shell's fully
    # materialized world. When start_gz_server:=false this launch still builds
    # its configuration; sharing the same filename used to overwrite the live
    # server world after it had been generated.
    world_file = os.path.join(tempfile.gettempdir(), 'aerothon_mission2_roslaunch.sdf')
    runtime_assets = os.path.join(tempfile.gettempdir(), 'aerothon_m2_assets')
    os.makedirs(runtime_assets, exist_ok=True)
    for asset in os.listdir(os.path.join(pkg_sim, 'materials')):
        source_asset = os.path.join(pkg_sim, 'materials', asset)
        if os.path.isfile(source_asset):
            shutil.copy2(source_asset, os.path.join(runtime_assets, asset))
    asset_uri = 'file://' + runtime_assets
    with open(world_template, encoding='utf-8') as source:
        world_text = source.read().replace('@SIM_GAZEBO_ASSET_URI@', asset_uri)
    with open(world_file, 'w', encoding='utf-8') as output:
        output.write(world_text)
    bridge_cfg = os.path.join(pkg_sim, 'config', 'gz_bridge.yaml')

    gui = LaunchConfiguration('gui')
    start_gz_server = LaunchConfiguration('start_gz_server')
    rviz = LaunchConfiguration('rviz')
    slam = LaunchConfiguration('slam')
    spawn_vehicle = LaunchConfiguration('spawn_vehicle')
    use_gz_tf = LaunchConfiguration('use_gz_tf')

    args = [
        DeclareLaunchArgument('gui', default_value='true', description='Launch Gazebo Harmonic GUI'),
        DeclareLaunchArgument('start_gz_server', default_value='true', description='Launch Gazebo server (disable when master shell owns it)'),
        DeclareLaunchArgument('rviz', default_value='true', description='Launch RViz 2 visualizer'),
        DeclareLaunchArgument('slam', default_value='true', description='Launch slam_toolbox 2D SLAM'),
        DeclareLaunchArgument('spawn_vehicle', default_value='true', description='Spawn official ArduPilot Gazebo vehicle'),
        # Declare this in the outer launch scope. The upstream vehicle launch
        # evaluates it from a deferred OnProcessStart callback after spawning.
        DeclareLaunchArgument('use_gz_tf', default_value='false', description='Use upstream Gazebo TF relay'),
        DeclareLaunchArgument('fcu_url', default_value='udp://127.0.0.1:14555@127.0.0.1:14556', description='MAVROS FCU URL (via router)'),
        DeclareLaunchArgument('stream_rate_keeper', default_value='true',
                              description='Re-assert MAVLink stream rates (SITL/MAVProxy workaround)'),
    ]

    # 1. Gazebo Harmonic Simulation
    # Start server and GUI as separate processes. On this GPU stack, starting
    # the combined client/server process concurrently with RViz can stall the
    # Gazebo transport world service before the vehicle is spawned.
    gz_server = ExecuteProcess(
        cmd=['gz', 'sim', '-s', '-r', '--headless-rendering', '-v', '3', world_file],
        output='screen',
        condition=IfCondition(start_gz_server),
    )
    gz_gui = TimerAction(
        period=12.0,
        actions=[ExecuteProcess(
            cmd=['gz', 'sim', '-g'],
            output='screen',
            condition=IfCondition(gui),
        )],
    )

    # 2. ros_gz parameter bridge for clock, lidar, camera
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_cfg, 'use_sim_time': True}],
        output='screen',
    )

    # The official vehicle owns the flight dynamics, ArduPilot plugin, camera,
    # and SITL process. The course SDF must never invent a fake flight model.
    # This include is intentionally gated: preflight_stack.sh checks that the
    # official ardupilot_gz/ardupilot_gazebo packages are installed first.
    vehicle = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ardupilot_gz_bringup'), 'launch', 'robots', 'iris.launch.py'])) ,
        launch_arguments={
            'world_name': 'mission2',
            'robot_name': 'aerothon_iris',
            'x': '-2.0',
            'y': '2.0',
            'z': '0.25',
            'Y': '0.0',
            # The official Iris is preloaded by mission2.sdf so Gazebo can
            # resolve merged model frames before simulation starts.
            'spawn_robot': 'false',
            'use_gz_sim_server': 'false',
            'use_gz_sim_gui': 'false',
            'rviz': 'false',
            # topic_tools is optional in minimal ROS installs; the mission
            # stack already publishes its own TF tree.
            'use_gz_tf': use_gz_tf,
            # Give the mission's router its own SITL channel.
            #
            # SERIAL0 belongs to MAVProxy, which the upstream bringup starts
            # unconditionally and which re-requests its streams at its 4 Hz
            # default. Chaining the router behind MAVProxy's --out meant every
            # SET_MESSAGE_INTERVAL the stack issued was clobbered on MAVProxy's
            # next sweep: LOCAL_POSITION_NED sat at 4 Hz against the 50 Hz the
            # setpoint loop asks for, GPS_RAW_INT never arrived, and the Q27
            # interlock could not see HDOP. SERIAL1 is a separate channel with
            # its own stream-rate state, so the two no longer fight.
            'serial1': 'udpclient:127.0.0.1:14560',
            # Use MAVLink SITL/MAVROS. DDS agent is an optional deployment
            # path and is not required for this competition loop.
            'use_dds_agent': 'false',
            'defaults': sitl_defaults,
            'use_sim_time': 'False',
            'synthetic_clock': 'False',
        }.items(),
        condition=IfCondition(spawn_vehicle),
    )

    # 3. Full Mission 2 Stack (including MAVROS, Perception, BT, Aggregator, SLAM, RViz)
    mission_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('mission_bringup'), 'launch', 'mission2.launch.py'])),
        launch_arguments={
            'use_sim': 'true',
            # RViz is launched explicitly below; keeping it out of the nested
            # mission launch avoids a condition/evaluation race.
            'rviz': 'false',
            'slam': slam,
            # Topics emitted by the official iris_with_gimbal + lidar_2d
            # vehicle bridge.
            'image_topic': '/camera/image',
            'scan_topic': '/scan',
            'fcu_url': LaunchConfiguration('fcu_url'),
            'stream_rate_keeper': LaunchConfiguration('stream_rate_keeper'),
        }.items(),
    )

    # The `rviz` argument was declared and read but never applied to this node,
    # so `rviz:=false` launched RViz anyway — ~20-30% CPU on a simulator already
    # running at 0.55 real-time factor, which drags every sensor rate down with
    # it. Honour the argument.
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='aerothon_rviz',
        arguments=['-d', os.path.join(
            pkg_mission, 'config', 'aerothon_slam.rviz')],
        parameters=[{'use_sim_time': True}],
        output='screen',
        condition=IfCondition(rviz),
    )

    # Gazebo publishes nav_msgs/Odometry but, on minimal ROS installations,
    # the upstream optional topic_tools relay may be absent. This small bridge
    # supplies the odom -> base_link transform required by slam_toolbox/RViz.
    odom_tf = Node(
        package='sim_gazebo',
        executable='odom_tf',
        name='gazebo_odom_tf',
        parameters=[{'use_sim_time': True, 'odom_topic': '/odometry'}],
        output='screen',
    )

    # Gazebo transport must finish advertising /gazebo/worlds before the
    # upstream create node starts. Starting every graphical and ROS process at
    # t=0 can starve Ogre/Gazebo initialization on this workstation.
    delayed_vehicle = TimerAction(period=5.0, actions=[vehicle])
    delayed_ros = TimerAction(period=8.0, actions=[gz_bridge, odom_tf, mission_stack])
    delayed_rviz = TimerAction(period=15.0, actions=[rviz_node])

    return LaunchDescription(args + [
        gz_server, delayed_vehicle, delayed_ros, gz_gui, delayed_rviz,
    ])
