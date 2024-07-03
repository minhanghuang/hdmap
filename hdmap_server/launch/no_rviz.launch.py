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
            launch_ros.actions.Node(
                package="hdmap_server",
                executable="hdmap_server",
                parameters=[config, {"map_path": map_path}],
                output="screen",
            ),
        ],
    )
