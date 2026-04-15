from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory




pkg_path = get_package_share_directory('vint_ros')
config_path = os.path.join(pkg_path, 'conf/voice_command_evaluation.rviz')

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vint_ros',
            executable='vosk_node.py',
            name='vosk_node',
            output='log',
            arguments=['--ros-args', '--log-level', 'info']
        ),
        Node(
            package='vint_ros',
            executable='phase1_trig_vint.py',
            name='p1_trig_vint_node',
            output='screen',
            arguments=['--ros-args', '--log-level', 'info']
        ),
    ])
