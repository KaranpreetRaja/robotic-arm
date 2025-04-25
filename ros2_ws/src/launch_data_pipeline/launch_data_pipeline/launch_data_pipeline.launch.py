from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joint_angle_processor',
            executable='joint_angle_processor',
            name='screen'
        ),
         Node(
            package='joint_angle_processor',
            executable='joint_angle_stringify',
            name='screen'
        ),
        Node(
            package='communication_server',
            executable='communication_server',
            name='screen'
        ),
        Node(
            package='desired_pose_processor',
            executable='desired_pose_processor',
            name='screen',
        )
    ])