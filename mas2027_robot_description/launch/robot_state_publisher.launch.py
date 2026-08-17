from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os


def generate_launch_description():
    package_name = "mas2027_robot_description"
    urdf_name = "mas2027_sentry.urdf"

    pkg_share = get_package_share_directory(package_name)
    urdf_model_path = os.path.join(pkg_share, "urdf", urdf_name)

    with open(urdf_model_path, "r", encoding="utf-8") as f:
        robot_desc = f.read()

    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")

    declare_namespace = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Top-level namespace",
    )
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock if true",
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=namespace,
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_desc,
                "use_sim_time": use_sim_time,
            }
        ],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )

    return LaunchDescription(
        [
            declare_namespace,
            declare_use_sim_time,
            robot_state_publisher_node,
        ]
    )
