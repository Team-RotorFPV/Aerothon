"""Mission 2 bring-up: MAVROS + perception + avoidance + SLAM + RViz + GCS aggregator + video.

Examples:
  # SITL + Gazebo (image/scan from sim bridge, launch RViz and SLAM):
  ros2 launch mission_bringup mission2.launch.py use_sim:=true rviz:=true slam:=true

  # Real hardware (starts the LD06 and C270 drivers too). use_sim:=false
  # also defaults SLAM and RViz off (not part of the mission; they cost the
  # Pi 5 a core) and the camera and winch backends to mavlink:
  ros2 launch mission_bringup mission2.launch.py use_sim:=false \
       fcu_url:=/dev/ttyAMA0:921600 \
       lidar_yaw_deg:=<measured> lidar_mirrored:=<measured>
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (Command, EnvironmentVariable,
                                  LaunchConfiguration, PythonExpression)
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    use_sim = LaunchConfiguration('use_sim')
    fcu_url = LaunchConfiguration('fcu_url')
    image_topic = LaunchConfiguration('image_topic')
    scan_topic = LaunchConfiguration('scan_topic')
    target = LaunchConfiguration('target')
    launch_rviz = LaunchConfiguration('rviz')
    launch_slam = LaunchConfiguration('slam')
    airframe = LaunchConfiguration('airframe')
    camera_hfov = ParameterValue(LaunchConfiguration('camera_hfov'), value_type=float)
    target_marker_m = ParameterValue(LaunchConfiguration('target_marker_m'),
                                     value_type=float)

    # Defaults that follow use_sim: a real launch that forgets winch_backend
    # or camera_backend must not drive the Gazebo joints while the aircraft's
    # winch and tilt servo get nothing, nor start SLAM and RViz on the Pi.
    in_sim = ["'", use_sim, "'.lower() in ('true', '1')"]
    sim_or = lambda sim, real: PythonExpression(
        [f"'{sim}' if "] + in_sim + [f" else '{real}'"])

    pkg_bringup = get_package_share_directory('mission_bringup')
    rviz_config_file = os.path.join(pkg_bringup, 'config', 'aerothon_slam.rviz')
    params_file = LaunchConfiguration('params_file')

    # URDF Robot description for RViz & TF
    pkg_desc = get_package_share_directory('uav_description')
    xacro_file = os.path.join(pkg_desc, 'urdf', 'uav.urdf.xacro')
    robot_description = ParameterValue(
        Command(['xacro "', xacro_file, '" airframe:=', airframe]), value_type=str)

    args = [
        DeclareLaunchArgument('use_sim', default_value='true', description='Use sim time (Gazebo)'),
        DeclareLaunchArgument('fcu_url', default_value='udp://127.0.0.1:14555@127.0.0.1:14556', description='MAVROS FCU URL (via MAVLink router)'),
        DeclareLaunchArgument('image_topic', default_value='/image_raw', description='Camera image topic'),
        DeclareLaunchArgument('scan_topic', default_value='/scan', description='LaserScan topic'),
        DeclareLaunchArgument('target', default_value='', description='Pre-assigned target QR (empty=dynamic)'),
        # The airframe and its camera. The team's quad (cad) carries a Logitech
        # C270: 55 deg diagonal is 48.8 deg across a 16:9 frame. Banner range,
        # lane spacing and red-zone projection are all computed from this, so
        # it has to be the lens actually fitted.
        DeclareLaunchArgument('airframe', default_value=EnvironmentVariable(
            'AEROTHON_AIRFRAME', default_value='cad'),
            description='cad (team airframe) | iris (ArduPilot Iris variant)'),
        DeclareLaunchArgument('camera_hfov', default_value=EnvironmentVariable(
            'AEROTHON_CAMERA_HFOV', default_value='0.851919'),
            description='Camera horizontal field of view, rad (C270: 0.851919)'),
        # The delivery pads' QR edge. The sweep spaces its lanes so every pad
        # is WHOLLY in frame on at least one of them, so it has to be the size
        # laid out. The simulator exports the world's; on the field, set it.
        DeclareLaunchArgument('target_marker_m', default_value=EnvironmentVariable(
            'AEROTHON_TARGET_QR_M', default_value='2.2'),
            description='Delivery-pad QR edge length, m'),
        # What the rulebook leaves to the venue: banner and payload size, the
        # return lane's offset, the winch's rates, the payload's colour.
        DeclareLaunchArgument('params_file', default_value=os.path.join(
            pkg_bringup, 'config', 'venue.yaml'),
            description='Venue/airframe parameters for mission_bt, winch_ctrl '
                        'and perception_payload'),
        DeclareLaunchArgument('rviz', default_value=sim_or('true', 'false'),
                              description='Launch RViz 2 with SLAM/TF displays '
                                          '(default: use_sim)'),
        DeclareLaunchArgument('slam', default_value=sim_or('true', 'false'),
                              description='Launch async slam_toolbox 2D SLAM node '
                                          '(default: use_sim)'),
        DeclareLaunchArgument('stream_rate_keeper', default_value='true',
                              description='Continuously re-assert MAVLink stream '
                                          'rates (SITL/MAVProxy workaround; see '
                                          'VERIFICATION.md 2.2)'),
        DeclareLaunchArgument('winch_backend',
                              default_value=sim_or('gazebo', 'mavlink'),
                              description='winch_ctrl backend: gazebo (the Iris '
                                          'winch joint and a detachable payload), '
                                          'sim (sequence only, nothing moves) or '
                                          'mavlink (MAV_CMD_DO_WINCH); default '
                                          'gazebo in sim, mavlink on the aircraft'),
        # The flight sensors (use_sim:=false only). Mount yaw and direction
        # are measured with scripts/check_sensors.py.
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('lidar_yaw_deg', default_value='0.0',
                              description="LD06's 0 deg, CCW from the nose"),
        DeclareLaunchArgument('lidar_mirrored', default_value='false',
                              description='driver angles run clockwise'),
        DeclareLaunchArgument('camera_device', default_value='/dev/video0'),
        # Manual exposure in 100 us units: 100 = 10 ms. Auto exposure on a
        # C270 runs to 60+ ms in shade, a 20 px smear at sweep speed; the QR
        # reader's envelope assumes 10 ms (sim/test_perception_corruption.py).
        DeclareLaunchArgument('camera_exposure', default_value='100'),
        DeclareLaunchArgument('camera_backend',
                              default_value=sim_or('sim', 'mavlink'),
                              description='camera_ctrl backend: sim (Gazebo joint) '
                                          'or mavlink (MAV_CMD_DO_MOUNT_CONTROL); '
                                          'default follows use_sim'),
    ]

    # 1. Robot State Publisher (publishes TF tree and robot_description)
    rsp_node = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim}],
    )

    # 2. MAVROS Core Node
    mavros = Node(
        package='mavros', executable='mavros_node', output='screen',
        parameters=[{'fcu_url': fcu_url, 'gcs_url': '',
                     'target_system_id': 1, 'target_component_id': 1,
                     'use_sim_time': use_sim}],
    )

    # 3. Computer Vision Perception Nodes
    qr = Node(
        package='perception_qr', executable='qr_node', output='screen',
        parameters=[{'image_topic': image_topic, 'target': target,
                     'use_sim_time': use_sim}],
    )

    banner = Node(
        package='perception_banner', executable='banner_node', output='screen',
        parameters=[{'image_topic': image_topic, 'use_sim_time': use_sim,
                     'camera_hfov': camera_hfov}],
    )

    # One camera feed with every detector's findings drawn on it. The GCS
    # showed /percep/qr/annotated and nothing else, so during banner alignment
    # the operator watched a nadir QR view with no banner box on it.
    overlay = Node(
        package='perception_overlay', executable='overlay_node', output='screen',
        parameters=[{'image_topic': image_topic, 'use_sim_time': use_sim}],
    )

    redzone = Node(
        package='perception_redzone', executable='redzone_node', output='screen',
        parameters=[{'image_topic': image_topic, 'use_sim_time': use_sim,
                     'camera_hfov': camera_hfov}],
    )

    # Is the delivered payload on the ground? WinchDrop confirms every drop
    # with the camera instead of trusting the winch's own "released" flag.
    payload = Node(
        package='perception_redzone', executable='payload_node', output='screen',
        parameters=[params_file,
                    {'image_topic': image_topic, 'use_sim_time': use_sim}],
    )

    # 3a. Hold MAVLink stream rates where the guidance loop needs them.
    #
    # Without it /mavros/local_position/pose decays to ~2 Hz in SITL because
    # MAVProxy re-requests its own lower rates on the same channel
    # (VERIFICATION.md 2.2), which is not a basis for closed-loop control.
    #
    # HISTORY, because this was got wrong twice. This node was disabled after
    # two launches carrying it ended with mavros_node and robot_state_publisher
    # aborting. That inference was wrong on both counts: the same aborts occur
    # without this node, and the abort message is
    #
    #   signal_handler(SIGINT/SIGTERM)
    #   what(): failed to initialize rcl node: the given context is not valid
    #
    # i.e. the process was SIGTERMed while still initialising and then tried to
    # finish constructing nodes on a shut-down context. It is a symptom of the
    # stack being torn down during start-up, not a cause. Two intervening
    # hypotheses (stale DDS shared memory, open discovery range) were also
    # wrong. The node itself did have a real bug — it leaked one pending future
    # per command per cycle — and that is fixed.
    #
    # The remaining SITL wart is MAVProxy competing for stream rates at all;
    # removing it from the simulation is tracked for Phase 11.
    stream_rates = Node(
        package='mission_bringup', executable='stream_rate_keeper', output='screen',
        parameters=[{'use_sim_time': use_sim}],
        condition=IfCondition(LaunchConfiguration('stream_rate_keeper')),
    )

    # 3b. Camera pointing as commanded, read-back-confirmed state (Phase 2).
    # Perception stages gate on /camera/pose_state.settled, so this must be
    # running for the mission to leave the start-QR scan.
    # NOTE: the backend cannot be derived from `use_sim` with a Python
    # conditional — LaunchConfiguration is an object, so `'sim' if use_sim
    # else 'mavlink'` is always truthy and would silently select the Gazebo
    # backend on real hardware. It is an explicit argument instead.
    camera = Node(
        package='camera_ctrl', executable='camera_ctrl_node', output='screen',
        parameters=[{'backend': LaunchConfiguration('camera_backend'),
                     'use_sim_time': use_sim}],
    )

    # 3c. Winch controller. Until this existed /winch/cmd had two publishers
    # and zero subscribers, and WinchDrop "delivered" on a fixed timer.
    winch = Node(
        package='winch_ctrl', executable='winch_node', output='screen',
        parameters=[params_file,
                    {'backend': LaunchConfiguration('winch_backend'),
                     'use_sim_time': use_sim}],
    )

    # 3d. Flight sensors. In the sim Gazebo publishes both.
    real = UnlessCondition(use_sim)
    lidar = Node(
        package='ldlidar_stl_ros2', executable='ldlidar_stl_ros2_node',
        name='ld06', output='screen', condition=real,
        parameters=[{'product_name': 'LDLiDAR_LD06', 'topic_name': 'scan_raw',
                     'frame_id': 'base_scan',
                     'port_name': LaunchConfiguration('lidar_port'),
                     'port_baudrate': 230400, 'laser_scan_dir': True,
                     'enable_angle_crop_func': False}],
    )
    lidar_mount = Node(
        package='mission_bringup', executable='scan_mount', output='screen',
        condition=real,
        parameters=[{'lidar_yaw_deg': ParameterValue(
                        LaunchConfiguration('lidar_yaw_deg'), value_type=float),
                     'mirrored': ParameterValue(
                        LaunchConfiguration('lidar_mirrored'), value_type=bool)}],
    )
    webcam = Node(
        package='usb_cam', executable='usb_cam_node_exe', name='c270',
        output='screen', condition=real,
        parameters=[{'video_device': LaunchConfiguration('camera_device'),
                     'image_width': 1280, 'image_height': 720,
                     'framerate': 30.0, 'pixel_format': 'mjpeg2rgb',
                     'frame_id': 'camera_link', 'autoexposure': False,
                     'exposure': ParameterValue(
                        LaunchConfiguration('camera_exposure'), value_type=int),
                     'autofocus': False}],
        remappings=[('image_raw', image_topic)],
    )

    # 4. Reactive Obstacle Avoidance Controller
    controller = Node(
        package='avoidance', executable='velocity_controller', output='screen',
        parameters=[{'scan_topic': scan_topic, 'use_sim_time': use_sim}],
    )

    # 5. Autonomous Behavior Tree Mission Executive
    mission = Node(
        package='mission_bt', executable='mission_tree', output='screen',
        parameters=[params_file,
                    {'use_sim_time': use_sim, 'camera_hfov': camera_hfov,
                     'target_marker_m': target_marker_m}],
    )

    # 6. GCS Aggregator WebSocket Server (port 8765)
    readiness = Node(
        package='gcs_aggregator', executable='readiness', output='screen',
        parameters=[{'use_sim_time': use_sim, 'image_topic': image_topic}],
    )

    aggregator = Node(
        package='gcs_aggregator', executable='aggregator', output='screen',
        parameters=[{'use_sim_time': use_sim}],
    )

    # 7. Annotated MJPEG Video Stream Server (port 8080)
    video = Node(
        package='web_video_server', executable='web_video_server', output='screen',
        parameters=[{'port': 8080, 'use_sim_time': use_sim}],
    )

    # 8. slam_toolbox (Online 2D SLAM Mapping)
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim,
            'odom_frame': 'odom',
            'map_frame': 'map',
            'base_frame': 'base_link',
            'scan_topic': scan_topic,
            'mode': 'mapping',
            'resolution': 0.05,
            'max_laser_range': 12.0,
            'minimum_time_interval': 0.1,
            'transform_timeout': 0.2,
            'tf_buffer_duration': 30.0,
        }],
        condition=IfCondition(launch_slam),
    )

    # slam_toolbox is a lifecycle node. Without this manager it remains
    # inactive, so RViz receives LaserScan data but never receives /map.
    slam_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_slam',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim,
            'autostart': True,
            'node_names': ['slam_toolbox'],
            'bond_timeout': 0.0,
        }],
        condition=IfCondition(launch_slam),
    )

    # 9. RViz 2 Visualizer with SLAM, costmap, point cloud, camera view
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim}],
        # Mesa 25 on this Wayland session mis-links RViz's indexed map shader
        # on the Intel Vulkan-backed path. Software GL is stable for RViz and
        # does not affect Gazebo, which remains hardware rendered separately.
        additional_env={
            'LIBGL_ALWAYS_SOFTWARE': '1',
            'QT_OPENGL': 'software',
        },
        output='screen',
        condition=IfCondition(launch_rviz),
    )

    return LaunchDescription(args + [
        rsp_node, mavros, lidar, lidar_mount, webcam,
        qr, banner, redzone, payload, overlay, camera, winch, stream_rates,
        controller, mission, readiness, aggregator, video,
        # The Gazebo odometry bridge needs a few seconds to establish odom TF.
        # Activating slam_toolbox before that point leaves its initial scan
        # filter without transforms and delays the first map indefinitely.
        TimerAction(period=8.0, actions=[slam_node, slam_lifecycle_manager]),
        rviz_node
    ])
