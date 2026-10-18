"""Ground console only: telemetry and GCS, without mission/actuator nodes."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='mavros', executable='mavros_node', output='screen',
             parameters=[{'fcu_url': 'udp://127.0.0.1:14555@127.0.0.1:14556',
                          'gcs_url': '', 'target_system_id': 1,
                          'target_component_id': 1, 'use_sim_time': False}]),
        Node(package='gcs_aggregator', executable='readiness', output='screen',
             parameters=[{'use_sim_time': False, 'image_topic': '/image_raw'}]),
        Node(package='gcs_aggregator', executable='aggregator', output='screen',
             parameters=[{'use_sim_time': False}]),
        Node(package='mission_bringup', executable='stream_rate_keeper',
             output='screen', parameters=[{'use_sim_time': False}]),
    ])
