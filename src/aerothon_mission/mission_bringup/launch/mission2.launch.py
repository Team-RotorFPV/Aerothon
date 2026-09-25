"""Mission 2 bring-up: MAVROS + perception + avoidance + SLAM + RViz + GCS aggregator + video.

Examples:
  # SITL + Gazebo (image/scan from sim bridge, launch RViz and SLAM):
  ros2 launch mission_bringup mission2.launch.py use_sim:=true rviz:=true slam:=true

  # Real hardware:
  ros2 launch mission_bringup mission2.launch.py use_sim:=false \
       fcu_url:=/dev/ttyAMA0:921600 image_topic:=/image_raw rviz:=false
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
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

    pkg_bringup = get_package_share_directory('mission_bringup')
    rviz_config_file = os.path.join(pkg_bringup, 'config', 'aerothon_slam.rviz')

    # URDF Robot description for RViz & TF
    pkg_desc = get_package_share_directory('uav_description')
    xacro_file = os.path.join(pkg_desc, 'urdf', 'uav.urdf.xacro')
    robot_description = ParameterValue(Command(['xacro "', xacro_file, '"']), value_type=str)

    args = [
        DeclareLaunchArgument('use_sim', default_value='true', description='Use sim time (Gazebo)'),
        DeclareLaunchArgument('fcu_url', default_value='udp://127.0.0.1:14555@127.0.0.1:14556', description='MAVROS FCU URL (via MAVLink router)'),
        DeclareLaunchArgument('image_topic', default_value='/image_raw', description='Camera image topic'),
        DeclareLaunchArgument('scan_topic', default_value='/scan', description='LaserScan topic'),
        DeclareLaunchArgument('target', default_value='', description='Pre-assigned target QR (empty=dynamic)'),
        DeclareLaunchArgument('rviz', default_value='true', description='Launch RViz 2 with SLAM/TF displays'),
        DeclareLaunchArgument('slam', default_value='true', description='Launch async slam_toolbox 2D SLAM node'),
        DeclareLaunchArgument('stream_rate_keeper', default_value='true',
                              description='Continuously re-assert MAVLink stream '
                                          'rates (SITL/MAVProxy workaround; see '
                                          'VERIFICATION.md 2.2)'),
        DeclareLaunchArgument('winch_backend', default_value='gazebo',
                              description='winch_ctrl backend: gazebo (the Iris '
                                          'winch joint and a detachable payload), '
                                          'sim (sequence only, nothing moves) or '
                                          'mavlink (MAV_CMD_DO_WINCH)'),
        DeclareLaunchArgument('camera_backend', default_value='sim',
                              description='camera_ctrl backend: sim (Gazebo joint) '
                                          'or mavlink (MAV_CMD_DO_MOUNT_CONTROL)'),
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
        parameters=[{'image_topic': image_topic, 'use_sim_time': use_sim}],
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
        parameters=[{'image_topic': image_topic, 'use_sim_time': use_sim}],
    )

    # Is the delivered payload on the ground? WinchDrop confirms every drop
    # with the camera instead of trusting the winch's own "released" flag.
    payload = Node(
        package='perception_redzone', executable='payload_node', output='screen',
        parameters=[{'image_topic': image_topic, 'use_sim_time': use_sim}],
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
        parameters=[{'backend': LaunchConfiguration('winch_backend'),
                     'use_sim_time': use_sim}],
    )

    # 4. Reactive Obstacle Avoidance Controller
    controller = Node(
        package='avoidance', executable='velocity_controller', output='screen',
        parameters=[{'scan_topic': scan_topic, 'use_sim_time': use_sim}],
    )

    # 5. Autonomous Behavior Tree Mission Executive
    mission = Node(
        package='mission_bt', executable='mission_tree', output='screen',
        parameters=[{'use_sim_time': use_sim}],
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
        rsp_node, mavros, qr, banner, redzone, payload, overlay, camera, winch, stream_rates,
        controller, mission, readiness, aggregator, video,
        # The Gazebo odometry bridge needs a few seconds to establish odom TF.
        # Activating slam_toolbox before that point leaves its initial scan
        # filter without transforms and delays the first map indefinitely.
        TimerAction(period=8.0, actions=[slam_node, slam_lifecycle_manager]),
        rviz_node
    ])
