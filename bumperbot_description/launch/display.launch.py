from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration

def generate_launch_description():

    # Declare the launch argument for the robot description
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(
            get_package_share_directory("bumperbot_description"),
            "urdf",
            "bumperbot.urdf.xacro"
        ),
        description="Absolute path to robot urdf file"
    )

    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")]),
        value_type=str
    )

    rviz_config_file = DeclareLaunchArgument(
        name="rviz_config_file",
        default_value=os.path.join(
        get_package_share_directory("bumperbot_description"),
        "rviz",
        "display.rviz"
    ))

    # Robot state publisher düğümü configurations
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description,
        }]
    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui"
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen", #Çıktı mesajı ekranda gösterilir.
        arguments=["-d", LaunchConfiguration("rviz_config_file")]
    )

    return LaunchDescription([
        model_arg,
        rviz_config_file,
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz
    ])