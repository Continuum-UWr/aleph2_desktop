from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory("aleph2_viz")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="config",
                default_value="robot",
                description="The name of the config to use with RViz",
            ),
            DeclareLaunchArgument(
                name="rviz_file",
                default_value=[
                    pkg_share,
                    "/rviz/",
                    LaunchConfiguration("config"),
                    ".rviz",
                ],
                description="Path to the config file for RViz",
            ),
            Node(
                name="rviz2",
                package="rviz2",
                executable="rviz2",
                output="screen",
                arguments=["-d", LaunchConfiguration("rviz_file")],
            ),
        ]
    )
