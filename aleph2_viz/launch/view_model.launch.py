from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("aleph2_viz")
    aleph2_description_share = get_package_share_directory("aleph2_description")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="gui",
                default_value="true",
                description="Start joint_state_publisher GUI",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [aleph2_description_share, "/launch/state_publisher.launch.py"]
                ),
            ),
            Node(
                condition=UnlessCondition(LaunchConfiguration("gui")),
                name="joint_state_publisher",
                package="joint_state_publisher",
                executable="joint_state_publisher",
                output="screen",
                parameters=[{"publish_default_efforts": True}],
            ),
            Node(
                condition=IfCondition(LaunchConfiguration("gui")),
                name="joint_state_publisher",
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                output="screen",
                parameters=[{"publish_default_efforts": True}],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([pkg_share, "/launch/rviz.launch.py"]),
                launch_arguments={"config": "robot"}.items(),
            ),
        ]
    )
