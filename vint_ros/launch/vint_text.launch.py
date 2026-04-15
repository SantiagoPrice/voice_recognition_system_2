from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory




pkg_path = get_package_share_directory('vint_ros')
config_path = os.path.join(pkg_path, 'conf/voice_command_evaluation.rviz')

def generate_launch_description():
    return LaunchDescription([
        # Node(
        #     package='vint_ros',
        #     executable='vosk_node.py',
        #     name='vosk_node',
        #     output='log',
        #     arguments=['--ros-args', '--log-level', 'warn']
        # ),
        # Node(
        #     package='vint_ros',
        #     executable='Task_Manager_node.py',
        #     name='task_manager_node',
        #     output='log',
        #     arguments=['--ros-args', '--log-level', 'warn']
        # ),
        Node(
            package='vint_ros',
            executable='Edge_Labeling_node.py',
            name='Edge_labeling',
            output='log',
            arguments=['--ros-args', '--log-level', 'warn']
        ),
        Node(
            package='vint_ros',
            executable='Perception_node.py',
            name='Perception',
            output='log',
            arguments=['--ros-args', '--log-level', 'warn']
        ),
        Node(
            package='vint_ros',
            executable='Assesment_node.py',
            name='Assessment_node',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', config_path,'--ros-args', '--log-level', 'error'],
        ),
    ])
