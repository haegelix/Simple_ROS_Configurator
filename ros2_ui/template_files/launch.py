import launch
import launch.actions
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription([
# !INSERT_LAUNCHFILE_NODES_HERE
    ])


"""
launch_ros.actions.Node(
    package='test_package',
    executable='door_sub',
    output='screen',
    emulate_tty=True,
),
launch_ros.actions.Node(
    package='test_package',
    executable='pushbutton_outside_runner_button',
),
launch_ros.actions.Node(
    package='test_package',
    executable='light_sub',
),
"""