import os
import yaml
from dotenv import load_dotenv

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.substitutions import Command


def generate_launch_description():
    load_dotenv()

    ld = LaunchDescription()

    # Load config file
    config = os.path.join(
        get_package_share_directory("f1tenth_gym_ros"),
        "config",
        "f1tenth_gym_ros.yaml",
    )
    config_dict = yaml.safe_load(open(config, "r"))

    map_name = config_dict["simulator"]["ros__parameters"]["map_name"]

    path_to_ws = os.getenv("PATH_TO_WS") or "ERROR"
    map_path = os.path.join(
        path_to_ws,
        "f1tenth_ws",
        "src",
        "main",
        "vaul_f1tenth_system",
        "maps",
        map_name,
    )

    # Gym simulator
    simulator_node = Node(
        package="f1tenth_gym_ros",
        executable="gym_bridge",
        name="simulator",
        parameters=[
            config,
            {"map_path": map_path},
        ],
    )

    # Map server to load map from file
    map_server_node = Node(
        package="nav2_map_server",
        executable="map_server",
        parameters=[
            {"yaml_filename": map_path + ".yaml"},
            {"topic_name": "map"},
            {"frame_id": "map"},
            {"output": "screen"},
            {"use_sim_time": True},
        ],
    )
    nav_lifecycle_node = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"autostart": True},
            {"node_names": ["map_server"]},
        ],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="ego_robot_state_publisher",
        parameters=[
            {
                "robot_description": Command(
                    [
                        "xacro ",
                        os.path.join(
                            get_package_share_directory("f1tenth_gym_ros"),
                            "launch",
                            "ego_racecar.xacro",
                        ),
                    ]
                )
            }
        ],
        remappings=[("/robot_description", "ego_robot_description")],
    )

    # Foxglove
    rosbridge_node = Node(
        package="rosbridge_server",
        executable="rosbridge_websocket",
        name="rosbridge_websocket",
        output="screen",
    )
    rosapi_node = Node(
        package="rosapi",
        executable="rosapi_node",
        name="rosapi_node",
        output="screen",
    )
    ld.add_action(simulator_node)
    ld.add_action(map_server_node)
    ld.add_action(nav_lifecycle_node)
    ld.add_action(robot_state_publisher)
    ld.add_action(rosbridge_node)
    ld.add_action(rosapi_node)

    return ld
