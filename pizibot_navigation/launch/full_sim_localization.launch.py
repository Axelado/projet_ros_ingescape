from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

import os


from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_name = "pizibot_navigation"
    
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(get_package_share_directory(package_name), 'map', 'my_world_map_save.yaml'),
        description='Full path to the map to use for localization'
    )
    
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(get_package_share_directory("pizibot_gazebo"), 'worlds', 'my_world.world'),
        description='Full path to the world to use'
    )
    
    map = LaunchConfiguration('map')
    world = LaunchConfiguration('world')
    
    # Construire les chemins des fichiers de lancement
    launch_sim_path = os.path.join(get_package_share_directory("pizibot_gazebo"), 'launch', 'launch_sim.launch.py')
    joystick_launch_path = os.path.join(get_package_share_directory("pizibot_teleop"), 'launch', 'joystick_teleop.launch.py')
    localization_launch_path = os.path.join(get_package_share_directory(package_name), 'launch', 'localization_launch.py')
    navigation_launch_path = os.path.join(get_package_share_directory(package_name), 'launch', 'navigation_launch.py')
    #rviz_launch_path = os.path.join(get_package_share_directory(package_name), 'launch', 'rviz2.launch.py')
    
    # Chemin du fichier de configuration RViz
    rviz_config_file = os.path.join(get_package_share_directory(package_name), 'rviz', 'localization.rviz')

    # Imprimer les chemins pour vérifier leur exactitude
    print(f"launch_sim_path: {launch_sim_path}")
    print(f"joystick_launch_path: {joystick_launch_path}")
    print(f"slam_launch_path: {localization_launch_path}")
    print(f"navigation_launch_path: {navigation_launch_path}")
    # print(f"rviz_slaunch_path: {rviz_launch_path}")
    
    # Vérifier que les fichiers existent
    if not os.path.isfile(launch_sim_path):
        print(f"Error: {launch_sim_path} does not exist")
    if not os.path.isfile(joystick_launch_path):
        print(f"Error: {joystick_launch_path} does not exist")
    if not os.path.isfile(localization_launch_path):
        print(f"Error: {localization_launch_path} does not exist")
    if not os.path.isfile(navigation_launch_path):
        print(f"Error: {navigation_launch_path} does not exist")
    # if not os.path.isfile(rviz_launch_path):
    #     print(f"Error: {rviz_launch_path} does not exist")
    if not os.path.isfile(rviz_config_file):
        print(f"Error: {rviz_config_file} does not exist")
    
    # Inclure les fichiers de lancement
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_sim_path),
        launch_arguments={'world': world}.items()
    )
    
    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(joystick_launch_path), 
        launch_arguments={'use_sim_time': 'true'}.items()
    )
    
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(localization_launch_path), 
        launch_arguments={'use_sim_time': 'true',
                          'map' : map}.items()
    )
    
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_launch_path), 
        launch_arguments={'use_sim_time': 'true',
                          'map_subscribe_trasient_local': 'true'}.items()
    )
    
    # rviz2 = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(rviz_launch_path), 
    #     launch_arguments={'use_sim_time': 'true',
    #                       'rviz_config_file': rviz_config_file}.items()
    # )
    
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}]
    )
    
    return LaunchDescription([
        world_arg,
        map_arg,
        navigation,
        localization,
        joystick,
        rviz2,
        simulation, 
    ])
