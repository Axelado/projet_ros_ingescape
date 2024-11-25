from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    teleop_keyboard_node = Node(
            package='teleop_twist_keyboard_for_azerty',
            executable='teleop_twist_keyboard_for_azerty',
            remappings=[('/cmd_vel', '/cmd_vel_key')],
            output='screen',
            
            prefix='xterm -e' 
         )

    return LaunchDescription([
        teleop_keyboard_node,
    ])
