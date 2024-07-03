import os

import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("hdmap_server"), "conf", "hdmap_server.yaml"
    )
    map_path = os.path.join(
        get_package_share_directory("hdmap_server"), "conf", "Town01.xodr"
    )

    return launch.LaunchDescription(
        [
            # hdmap server
            launch_ros.actions.Node(
                package="hdmap_server",
                executable="hdmap_server",
                parameters=[config, {"map_path": map_path}],
                output="screen",
            ),
            # Rviz
            launch_ros.actions.Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=[
                    "-d",
                    os.path.join(
                        get_package_share_directory("hdmap_server"),
                        "rviz",
                        "hdmap_server.rviz",
                    ),
                ],
                output="screen",
            ),
        ],
    )
