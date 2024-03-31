import launch
import launch_ros


def generate_launch_description():
    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package='hdmap_server',
                executable='hdmap_server',
                output='screen'
            ),
        ],
    )
