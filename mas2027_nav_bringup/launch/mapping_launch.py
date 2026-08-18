import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory("mas2027_nav_bringup")
    launch_dir = os.path.join(bringup_dir, "launch")

    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    mapping_params_file = LaunchConfiguration("mapping_params_file")
    small_point_lio_params_file = LaunchConfiguration(
        "small_point_lio_params_file"
    )
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    use_robot_state_pub = LaunchConfiguration("use_robot_state_pub")
    use_rviz = LaunchConfiguration("use_rviz")

    declare_namespace = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Top-level namespace",
    )
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
        description="Use simulation clock",
    )
    declare_mapping_params_file = DeclareLaunchArgument(
        "mapping_params_file",
        default_value=os.path.join(
            bringup_dir, "config", "slam_toolbox_mapping.yaml"
        ),
        description="PointCloud-to-LaserScan and SLAM Toolbox parameter file",
    )
    declare_small_point_lio_params_file = DeclareLaunchArgument(
        "small_point_lio_params_file",
        default_value=os.path.join(
            bringup_dir, "config", "small_point_lio_params.yaml"
        ),
        description="MID360 and Small Point-LIO parameter file",
    )
    declare_rviz_config_file = DeclareLaunchArgument(
        "rviz_config_file",
        default_value=os.path.join(
            bringup_dir, "rviz", "nav2_default_view.rviz"
        ),
        description="RViz configuration file",
    )
    declare_use_robot_state_pub = DeclareLaunchArgument(
        "use_robot_state_pub",
        default_value="True",
        description="Start robot_state_publisher",
    )
    declare_use_rviz = DeclareLaunchArgument(
        "use_rviz",
        default_value="True",
        description="Start RViz with map as the fixed frame",
    )

    # Reuse the normal sensor and LIO bringup, but do not start Nav2 while mapping.
    lio_bringup = GroupAction(
        scoped=True,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        launch_dir,
                        "rm_navigation_small_point_lio_launch.py",
                    )
                ),
                launch_arguments={
                    "namespace": namespace,
                    "use_sim_time": use_sim_time,
                    "small_point_lio_params_file": (
                        small_point_lio_params_file
                    ),
                    "use_robot_state_pub": use_robot_state_pub,
                    "use_nav2": "False",
                    "use_rviz": "False",
                }.items(),
            )
        ],
    )

    pointcloud_to_laserscan = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        namespace=namespace,
        output="screen",
        parameters=[
            mapping_params_file,
            {"use_sim_time": use_sim_time},
        ],
        remappings=[
            ("cloud_in", "/cloud_registered"),
            ("scan", "/scan"),
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
        ],
    )

    slam_toolbox = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        namespace=namespace,
        output="screen",
        parameters=[
            mapping_params_file,
            {"use_sim_time": use_sim_time},
        ],
        remappings=[
            ("scan", "/scan"),
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        namespace=namespace,
        output="screen",
        condition=IfCondition(use_rviz),
        arguments=["-d", rviz_config_file, "-f", "map"],
        remappings=[
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
        ],
    )

    return LaunchDescription(
        [
            declare_namespace,
            declare_use_sim_time,
            declare_mapping_params_file,
            declare_small_point_lio_params_file,
            declare_rviz_config_file,
            declare_use_robot_state_pub,
            declare_use_rviz,
            lio_bringup,
            pointcloud_to_laserscan,
            slam_toolbox,
            rviz,
        ]
    )
