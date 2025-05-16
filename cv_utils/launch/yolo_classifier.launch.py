# yolo_classifier.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    model_name_arg = DeclareLaunchArgument(
        'model',
        default_value='yolo_v3',
        description='Name of the model to load'
    )
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/vertical_camera',
        description='Image topic to subscribe to'
    )

    # Use the launch configuration to get the model name and image topic
    model_name = LaunchConfiguration('model_name')
    image_topic = LaunchConfiguration('image_topic')

    return LaunchDescription([
        model_name_arg,
        image_topic_arg,
        Node(
            package='frtl_2024_cv_utils',
            executable='yolo_classifier.py',
            name='yolo_classifier',
            output='screen',
            parameters=[
                {'model': model_name},
                {'image_topic': image_topic},
            ]
        ),
    ])
