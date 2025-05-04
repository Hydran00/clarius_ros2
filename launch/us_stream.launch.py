from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "us_image_topic_name",
                default_value="us_image",
                description="Ultrasound image output topic name",
            ),
            Node(
                package="clarius_ros2",
                executable="caster",
                output="screen",
                # passing the argument to the node
                parameters=[
                    {"us_image_topic_name": LaunchConfiguration("us_image_topic_name")},
                    {"frame_id": "clarius_probe"},
                    {"ip_address": "192.168.1.1"},
                    {"port": 5828},
                ],
            ),
        ]
    )
