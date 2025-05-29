from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='siyi_a8mini',
            executable='gimbal_control_node',
            name='gimbal_control_node',
            output='screen',
            parameters=['config/gimbal_params.yaml']
        ),
        Node(
            package='siyi_a8mini',
            executable='gimbal_stream_node',
            name='gimbal_stream_node',
            output='screen',
            parameters=['config/gimbal_params.yaml']
        )
    ])
