import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Path ke rosbot_description package
    rosbot_description_pkg = get_package_share_directory("rosbot_description")

    # Argument untuk path xacro model
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(rosbot_description_pkg, "urdf", "rosbot.urdf.xacro"),
        description="Path ke Xacro file ROSbot"
    )

    # Biar Ignition bisa nemu semua resource package
    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[str(Path(rosbot_description_pkg).parent.resolve())]
    )

    # Render xacro langsung ke robot_description parameter
    robot_description = ParameterValue(
        Command([
            "xacro ",
            LaunchConfiguration("model"),
            " is_ignition:=True"
        ]),
        value_type=str
    )

    # Node robot_state_publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": True
        }],
        output="screen"
    )

    # Launch Gazebo/Ignition dengan world kosong
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
        ]),
        launch_arguments={
            "gz_args": "-v 4 -r empty.sdf"
        }.items()
    )

    # Spawn robot ke Ignition via topic robot_description
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "robot_description",
            "-name", "rosbot"
        ]
    )

    # Bridge /clock dari Ignition ke ROS 2
    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen"
    )

    return LaunchDescription([
        model_arg,
        gazebo_resource_path,
        robot_state_publisher_node,
        gazebo,
        gz_spawn_entity,
        gz_ros2_bridge,
    ])