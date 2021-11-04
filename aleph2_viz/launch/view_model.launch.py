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
            DeclareLaunchArgument(
                name="model",
                default_value=[aleph2_description_share, "/urdf/aleph2.urdf.xacro"],
                description="Absolute path to robot urdf.xacro file",
            ),
            Node(
                name="robot_state_publisher",
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[
                    {
                        "robot_description": Command(
                            ["xacro ", LaunchConfiguration("model")]
                        )
                    }
                ],
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
