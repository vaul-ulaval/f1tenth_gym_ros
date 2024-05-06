from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource

import os


def generate_launch_description():
    shared_dir = get_package_share_directory("vaul_f1tenth_system")

    f1tenth_gym_ros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("f1tenth_gym_ros"),
                    "launch",
                    "f1tenth_gym_ros.launch.py",
                )
            ]
        )
    )

    pure_pursuit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(shared_dir, "launch"), "/pure_pursuit.launch.py"]
        )
    )

    waypoint_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(shared_dir, "launch"), "/waypoint_publisher.launch.py"]
        )
    )

    telemetry_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(shared_dir, "launch"), "/telemetry.launch.py"]
        )
    )

    return LaunchDescription(
        [
            f1tenth_gym_ros_launch,
            pure_pursuit_launch,
            waypoint_publisher_launch,
            telemetry_launch,
        ]
    )
