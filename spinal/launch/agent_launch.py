from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    dev = LaunchConfiguration("dev")
    baudrate = LaunchConfiguration("baudrate")
    verbosity = LaunchConfiguration("verbosity")

    return LaunchDescription([
        DeclareLaunchArgument("dev", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("baudrate", default_value="921600"),
        DeclareLaunchArgument("verbosity", default_value="6"),

        Node(
            package="micro_ros_agent",
            executable="micro_ros_agent",
            name="micro_ros_agent",
            output="screen",
            arguments=[
                "serial",
                "--dev", dev,
                "-b", baudrate,
                "-v", verbosity,
            ],
        ),
    ])
