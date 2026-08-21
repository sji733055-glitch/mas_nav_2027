from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_comm',
            executable='ros2_comm_node',
            name='ros2_comm_node',
            output='screen',
            emulate_tty=True,  # 让日志带颜色
            parameters=[
                # 这里可以放参数，目前我们不需要
            ]
        )
    ])