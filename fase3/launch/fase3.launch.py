from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    detector_node = Node(
        package='sae_cv_utils',
        executable='landing_base_detector',
        name='landing_base_detector',
        parameters=[{'params.yaml'}],
        output='screen'
    )

    fase3_node = Node(
        package='sae_fase3',
        executable='fase3',
        name='fase3_fsm',
        output='screen'
    )

    delayed_fase3_node = TimerAction(
        period=10.0,
        actions=[fase3_node]
    )

    return LaunchDescription([
        detector_node,
        delayed_fase3_node
    ])
