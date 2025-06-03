from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_fase2   = get_package_share_directory('sae_fase2')
    params      = os.path.join(pkg_fase2, "launch", "params.yaml")
    rviz_cfg = os.path.join(pkg_fase2, 'launch', 'drone_viz.rviz')

    exec_arg = DeclareLaunchArgument(
        "mission",
        default_value="fase2",
        description="Executable that implements the mission FSM")

    detector_node = Node(
        package='sae_cv_utils',
        executable='landing_base_detector',
        parameters=[params]
    )

    fase2_node = Node(
        package='sae_fase2',
        executable=LaunchConfiguration("mission"),
        parameters=[params],
        output='screen'
    )

    bridge_node = Node(
        package='sae_drone_lib',
        executable='pos_to_rviz',
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_cfg]
    )

    delayed_fase2_node = TimerAction(period=5.0, actions=[fase2_node])

    return LaunchDescription([
        exec_arg,
        detector_node,
        bridge_node,
        rviz_node,
        delayed_fase2_node
    ])
