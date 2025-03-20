import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bumperbot_description_dir = get_package_share_directory("rosbot_description")

    model_arg = DeclareLaunchArgument(
        name="model", 
        default_value=os.path.join(
            bumperbot_description_dir, "urdf", "rosbot.urdf.xacro"
        ),
        description="Absolute path to robot urdf file"
    )

    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")]),
        value_type=str
    )
    
    # joint_state_publisher_gui_node = Node(
    #     package="joint_state_broadcaster",
    #     executable="joint_state_broadcaster",
    #     parameters=[{"use_sim_time": True}]
    # )
    
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )


    # robot_state_publisher_node = Node(
    #     package="robot_state_publisher",
    #     executable="robot_state_publisher",
    #     parameters=[{"robot_description": robot_description}]
    # )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(bumperbot_description_dir, "rviz", "display.rviz")],
    )

    return LaunchDescription([
        model_arg,
        # robot_state_publisher_node,
        # joint_state_publisher_gui_node,
        joint_state_broadcaster_spawner,
        rviz_node
    ])
