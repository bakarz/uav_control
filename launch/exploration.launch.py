from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='uav_control', executable='tf_broadcaster',
             name='px4_tf_broadcaster', output='screen'),
        Node(package='uav_control', executable='exploration_ctrl',
             name='exploration_ctrl', output='screen'),
        Node(package='uav_control', executable='frontier_explorer',
             name='frontier_explorer', output='screen'),
    ])
