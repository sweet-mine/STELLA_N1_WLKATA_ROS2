#!/usr/bin/python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import LifecycleNode

def generate_launch_description():
    
    wlkata_node = Node(package='stella_wlkata_sm',
                                executable='stella_wlkata_node',
                                name='stella_wlkata_node',
                                output='screen',
                                emulate_tty=True,
                                namespace='/',
                                )

    
    return LaunchDescription([
       wlkata_node,
    ])



# Node(
#            package='stella_md',
#            executable='stella_md_node',
#            name='stella_md_node',
#            output='screen'
#        )
