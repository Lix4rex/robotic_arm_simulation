from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess, AppendEnvironmentVariable
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from ros_gz_bridge.actions import RosGzBridge
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory

import os


packageName = "robotic_arm_sim"

armParamsConfigRelativePath = "config/arm_params.yaml"
worldRelativePath = "world/panda_world.sdf"

cubes = ["blue_cube", "red_cube", "green_cube", "yellow_cube", "cyan_cube", "magenta_cube"]


def generate_launch_description():

    pkg = get_package_share_directory(packageName)

    pkgPath = FindPackageShare(package=packageName).find(packageName)

    armParamsConfigPath  = os.path.join(pkgPath, armParamsConfigRelativePath)

    world = os.path.join(pkgPath, worldRelativePath)

    # Permet a Gazebo de trouver model://panda
    gz_models_path = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH", os.path.join(pkgPath, "models")
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py",
            ])
        ),
        launch_arguments={
            # Pas de -r : la simulation est lancee par detach_cubes une fois les cubes detaches
            "gz_args": f'"{world}"'
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

    rviz = Node(
        package='rviz2', 
        executable='rviz2', 
        output='screen',
        arguments=['-d', os.path.join(pkg, 'config', 'config.rviz')],
        parameters=[{'use_sim_time': True}]
    )

    arm_controller = Node(
        package="robotic_arm_sim",
        executable="arm_controller",
        parameters=[armParamsConfigPath]
    )

    gripper_controller = Node(
        package="robotic_arm_sim",
        executable="gripper_controller",
        parameters=[{"cubes": cubes}]
    )

    cube_detector = Node(
        package="robotic_arm_sim",
        executable="cube_detector",
        parameters=[{"use_sim_time": True}]
    )

    # Le plugin DetachableJoint attache les cubes a la pince des le demarrage :
    # on attend que ses topics existent, on les detache, puis on lance la simulation
    # (sinon le bras entraine les cubes avant que le detach soit pris en compte).
    detach_cmds = " && ".join(
        f"gz topic -t /panda/{cube}/detach -m gz.msgs.Empty -p ' '" for cube in cubes
    )
    unpause_cmd = (
        "gz service -s /world/panda_world/control --reqtype gz.msgs.WorldControl "
        "--reptype gz.msgs.Boolean --timeout 3000 --req 'pause: false'"
    )
    detach_cubes = ExecuteProcess(
        cmd=["bash", "-c",
             f"until gz topic -l | grep -q /panda/{cubes[-1]}/detach; do sleep 0.2; done; "
             f"{detach_cmds} && sleep 1 && {unpause_cmd}"],
        output="screen",
    )

    delayed_actions = TimerAction(
        period=10.0,
        actions=[bridge, arm_controller, gripper_controller, cube_detector, rviz],
    )

    return LaunchDescription([
        gz_models_path,
        gz_sim,
        detach_cubes,
        delayed_actions,
    ])