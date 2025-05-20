from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    ld = LaunchDescription()
        
    DeclareLaunchArgument(
        "us_image_topic_name",
        default_value="us_image",
        description="Ultrasound image output topic name",
    )
    wrapper = Node(
        package="clarius_ros2",
        executable="clarius_wrapper",
        output="screen",
        # passing the argument to the node
        parameters=[
            {"us_image_topic_name": "us_image"},
            {"frame_id": "clarius_probe"},
            # {"ip_address": "192.168.1.1"},
            {"ip_address": "10.160.50.119"},
            {"port": 5828},
        ],
    )
    contact_detector = Node(
        package="clarius_ros2",
        executable="contact_detector",
        output="screen",
        # passing the argument to the node
        parameters=[
            {"ft_sensor_topic_name": "/lbr/force_torque_broadcaster/wrench"},
            {"us_freeze_service_name": "enable_freeze"},
            {"force_threshold": 1.4},
        ],
    )

    ld.add_action(wrapper)
    ld.add_action(contact_detector)
    return ld
        
