from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    detector_node = Node(
        package='sae_cv_utils',
        executable='landing_base_detector',
        parameters=[os.path.join(get_package_share_directory('sae_fase3'), 'launch', 'params.yaml')]
    )

    fase3_node = Node(
        package='sae_fase3',
        executable='fase3',
        parameters=[os.path.join(get_package_share_directory('sae_fase3'), 'launch', 'params.yaml')],
        output='screen'
    )

    bridge_node = Node(
        package='sae_drone_lib',
        executable='pos_to_rviz',
        output='screen'
    )

    delayed_fase3_node = TimerAction(
        period=5.0,
        actions=[fase3_node]
    )

    return LaunchDescription([
        detector_node,
        bridge_node,
        delayed_fase3_node
    ])
