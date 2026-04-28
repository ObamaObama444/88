from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("slam_toolbox"),
                    "launch",
                    "localization_launch.py",
                ])
            ),
            launch_arguments={
                "use_sim_time": "false",
                "slam_params_file": PathJoinSubstitution([
                    FindPackageShare("main"),
                    "launch",
                    "slam_localization.yaml",
                ]),
            }.items(),
        )
    ])