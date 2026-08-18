# Copyright 2025 Lihan Chen
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory("mas2027_nav_bringup")
    launch_dir = os.path.join(bringup_dir, "launch")

    # Create the launch configuration variables、
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    small_point_lio_params_file = LaunchConfiguration(
        "small_point_lio_params_file"
    )
    nav2_params_file = LaunchConfiguration("nav2_params_file")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    use_robot_state_pub = LaunchConfiguration("use_robot_state_pub")
    use_fake_vel_transform = LaunchConfiguration("use_fake_vel_transform")
    use_nav2 = LaunchConfiguration("use_nav2")
    use_terrain_analysis = LaunchConfiguration("use_terrain_analysis")
    use_rviz = LaunchConfiguration("use_rviz")

    # Declare the launch arguments

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
        description="Use simulation (Gazebo) clock if true",
    )
    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Top-level namespace",
    )
 
    declare_small_point_lio_params_file_cmd = DeclareLaunchArgument(
        "small_point_lio_params_file",
        default_value=os.path.join(
            bringup_dir, "config", "small_point_lio_params.yaml"
        ),
        description="Full path to the MID360 and Small Point-LIO parameter file",
    )

    declare_nav2_params_file_cmd = DeclareLaunchArgument(
        "nav2_params_file",
        default_value=os.path.join(bringup_dir, "config", "nav2_params.yaml"),
        description="Full path to the Navigation2 parameter file",
    )

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        "use_robot_state_pub",
        default_value="True",
        description="Whether to start the robot state publisher",
    )

    declare_use_fake_vel_transform_cmd = DeclareLaunchArgument(
        "use_fake_vel_transform",
        default_value="True",
        description="Publish base_link_fake and transform Nav2 velocity into base_link",
    )

    declare_use_nav2_cmd = DeclareLaunchArgument(
        "use_nav2",
        default_value="True",
        description="Start terrain processing, fake velocity transform, and Nav2",
    )

    declare_use_terrain_analysis_cmd = DeclareLaunchArgument(
        "use_terrain_analysis",
        default_value="True",
        description="Start terrain_analysis nodes inside the Nav2 launch",
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        "rviz_config_file",
        default_value=os.path.join(bringup_dir, "rviz", "nav2_default_view.rviz"),
        description="Full path to the RVIZ config file to use",
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        "use_rviz", default_value="True", description="Whether to start RVIZ"
    )

    # Create our own temporary YAML files that include substitutions

    configured_small_point_lio_params = ParameterFile(
        RewrittenYaml(
            source_file=small_point_lio_params_file,
            root_key=namespace,
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )

    start_robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "robot_state_publisher_launch.py")
        ),
        # NOTE: This startup file is only used when the navigation module is standalone
        condition=IfCondition(use_robot_state_pub),
        launch_arguments={
            "namespace": namespace,
            "use_sim_time": use_sim_time,
        }.items(),
    )
    # mid360启动
    start_mid360_driver_node = Node(
        package="mid360_driver",
        executable="mid360_driver_node",
        name="mid360_driver",
        output="screen",
        parameters=[configured_small_point_lio_params],
    )

    start_small_point_lio_node = Node(
        package="small_point_lio",
        executable="small_point_lio_node",
        name="small_point_lio",
        output="screen",
        parameters=[configured_small_point_lio_params],
    )

    start_nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "nav2_launch.py")),
        condition=IfCondition(use_nav2),
        launch_arguments={
            "namespace": namespace,
            "use_sim_time": use_sim_time,
            "nav2_params_file": nav2_params_file,
            "perception_params_file": small_point_lio_params_file,
            "use_terrain_analysis": use_terrain_analysis,
            "use_fake_vel_transform": use_fake_vel_transform,
        }.items(),
    )

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "rviz_launch.py")),
        condition=IfCondition(use_rviz),
        launch_arguments={
            "namespace": namespace,
            "use_sim_time": use_sim_time,
            "rviz_config": rviz_config_file,
        }.items(),
    )


    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_nav2_params_file_cmd)
    ld.add_action(declare_small_point_lio_params_file_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_use_nav2_cmd)
    ld.add_action(declare_use_terrain_analysis_cmd)
    ld.add_action(declare_use_fake_vel_transform_cmd)
    ld.add_action(declare_use_rviz_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_mid360_driver_node)
    ld.add_action(start_small_point_lio_node)
    ld.add_action(start_nav2_cmd)
    ld.add_action(rviz_cmd)

    return ld
