from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

import os


from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_name = "ingescape_ros2_interface"

    network_device_arg = DeclareLaunchArgument(
        'network_device',
        default_value='wlo1',
        description='Network device name for ingescape agents'
    )
    
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='5670',
        description='Port number for ingescape agents'
    )
    
    network_device = LaunchConfiguration('network_device')
    port = LaunchConfiguration('port')
    
    # Construire le chemin du fichier de lancement
    full_sim_launch_path = os.path.join(get_package_share_directory("pizibot_navigation"), 'launch', 'full_sim_localization.launch.py')
    
    # Vérifier que le fichier existe
    if not os.path.isfile(full_sim_launch_path):
        print(f"Error: {full_sim_launch_path} does not exist")
    
    # Inclure le fichier de lancement
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(full_sim_launch_path),
    )
    
    # Définir les nœuds
    image_on_whiteboard = Node(
        package='ingescape_ros2_interface',
        executable='image_on_whiteboard',
        name='image_on_whiteboard',
        parameters=[
            {'agent_name': 'image_on_whiteboard'},
            {'network_device': network_device},
            {'port': port}
        ]
    )
    
    tf_listener_node = Node(
        package='ingescape_ros2_interface',
        executable='tf_listener_node',
        name='tf_listener_node'
    )
    
    pose_publish_from_room_number = Node(
        package='ingescape_ros2_interface',
        executable='pose_publish_from_room_number',
        name='pose_publish_from_room_number',
        parameters=[
            {'agent_name': 'room_number_ros'},
            {'network_device': network_device},
            {'port': port}
        ]
    )
    
    joy_publisher_node = Node(
        package='ingescape_ros2_interface',
        executable='joy_publisher_node',
        name='joy_publisher_node',
        parameters=[
            {'agent_name': 'joystick_ros'},
            {'network_device': network_device},
            {'port': port}
        ]
    )
    
    initial_pose_publisher = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub', '--once', '/initialpose',
            'geometry_msgs/msg/PoseWithCovarianceStamped',
            '{header: {frame_id: "map"}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}}'
        ],
        output='screen'
    )

    delayed_initial_pose_publisher = TimerAction(
        period=10.0,
        actions=[initial_pose_publisher]
    )
    
    return LaunchDescription([
        network_device_arg,
        port_arg,
        simulation,
        image_on_whiteboard,
        tf_listener_node,
        joy_publisher_node,
        pose_publish_from_room_number,
        # delayed_initial_pose_publisher
    ])
