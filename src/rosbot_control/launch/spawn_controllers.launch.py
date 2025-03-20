from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Load the diff drive controller and start it
    load_diff_drive = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"],
    )

    return LaunchDescription([
        load_diff_drive
    ])
