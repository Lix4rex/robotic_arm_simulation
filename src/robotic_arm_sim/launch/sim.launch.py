from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ros_gz_bridge.actions import RosGzBridge

import os


packageName = "robotic_arm_sim"

armParamsConfigRelativePath = "config/arm_params.yaml"
worldRelativePath = "world/panda_world.sdf"


def generate_launch_description():

    pkgPath = FindPackageShare(package=packageName).find(packageName)

    armParamsConfigPath  = os.path.join(pkgPath, armParamsConfigRelativePath)

    world = os.path.join(pkgPath, worldRelativePath)

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py",
            ])
        ),
        launch_arguments={
            "gz_args": f'-r "{world}"'
        }.items(),
    )

    bridge = RosGzBridge(
        bridge_name="panda_cmd_bridge",
        config_file=PathJoinSubstitution([
            FindPackageShare("robotic_arm_sim"),
            "config",
            "panda_bridge.yaml",
        ]),
    )


    arm_controller = Node(
        package="robotic_arm_sim",
        executable="arm_controller",
        parameters=[armParamsConfigPath]
    )

    delayed_actions = TimerAction(
        period=10.0,
        actions=[bridge, arm_controller],
    )

    return LaunchDescription([
        gz_sim,
        delayed_actions,
        
    ])