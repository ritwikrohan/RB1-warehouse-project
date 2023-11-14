import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="True", description="Choose according to the use(True for simulation and False for Robot)")
    use_sim_time_arg_f = LaunchConfiguration('use_sim_time')
    map_type_arg = DeclareLaunchArgument("map_file", default_value="warehouse_map_sim.yaml", description="Choose map")
    map_type_arg_f = LaunchConfiguration('map_file')
    map_file = PathJoinSubstitution([FindPackageShare('map_server'), 'config', map_type_arg_f])
    rviz_config_dir = os.path.join(get_package_share_directory('map_server'), 'rviz', 'map_display.rviz')

    map_server_node = Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time_arg_f}, 
                        {'yaml_filename':map_file} 
                       ])

    lifecycle_manager_node = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_mapper',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time_arg_f},
                        {'autostart': True},
                        {'node_names': ['map_server']}])

    rviz2_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            output='screen',
            parameters=[{'use_sim_time': use_sim_time_arg_f}])  

    return LaunchDescription([
        use_sim_time_arg,
        map_type_arg,
        map_server_node,
        lifecycle_manager_node,
        rviz2_node           
        ])