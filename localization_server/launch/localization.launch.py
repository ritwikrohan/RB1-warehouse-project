# import os
# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
# from launch_ros.actions import Node
# from launch_ros.substitutions import FindPackageShare
# from nav2_common.launch import RewrittenYaml

# def generate_launch_description():
    
#     nav2_yaml = os.path.join(get_package_share_directory('localization_server'), 'config', 'amcl_config.yaml')
#     # map_file = os.path.join(get_package_share_directory('map_server'), 'config', 'turtlebot_area.yaml')

#     use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="False", description="Choose according to the use(True for simulation and False for Robot)")
#     use_sim_time_arg_f = LaunchConfiguration('use_sim_time')
#     map_type_arg = DeclareLaunchArgument("map_file", default_value="warehouse_map_real.yaml", description="Choose map")
#     map_type_arg_f = LaunchConfiguration('map_file')
#     map_file = PathJoinSubstitution([FindPackageShare('map_server'), 'config', map_type_arg_f])

#     rviz_config_dir = os.path.join(get_package_share_directory('localization_server'), 'rviz', 'config.rviz')

#     param_substitutions = {'use_sim_time': use_sim_time_arg_f}

#     urdf_file = '/home/user/simulation_ws/src/warehouse_robot_lab/rb1_base_common/rb1_base_description/robots/rb1_base.urdf'

#     with open(urdf_file, 'r') as info:
#         robot_desc = info.read()

#     robot_state_publisher_node = Node(
#         package='robot_state_publisher',
#         executable='robot_state_publisher',
#         output='screen',
#         parameters=[{'robot_description': robot_desc}]
#     )

#     map_server_node = Node(
#             package='nav2_map_server',
#             executable='map_server',
#             name='map_server',
#             output='screen',
#             parameters=[{'use_sim_time': use_sim_time_arg_f}, 
#                         {'yaml_filename':map_file}]
#         )
    
#     amcl_node_config = RewrittenYaml(
#         source_file=nav2_yaml,
#         root_key='',
#         param_rewrites=param_substitutions,
#         convert_types=True
#     )

#     amcl_node = Node(
#             package='nav2_amcl',
#             executable='amcl',
#             name='amcl',
#             output='screen',
#             parameters=[amcl_node_config]
#         )

#     lifecycle_node = Node(
#             package='nav2_lifecycle_manager',
#             executable='lifecycle_manager',
#             name='lifecycle_manager_localization',
#             output='screen',
#             parameters=[{'use_sim_time': use_sim_time_arg_f},
#                         {'autostart': True},
#                         {'node_names': ['map_server', 'amcl']}]
#         )
    
#     rviz2_node = Node(
#             package='rviz2',
#             executable='rviz2',
#             name='rviz2',
#             arguments=['-d', rviz_config_dir],
#             output='screen',
#             parameters=[{'use_sim_time': use_sim_time_arg_f}])

#     return LaunchDescription([
#         # rviz2_node,
#         robot_state_publisher_node,
#         use_sim_time_arg,
#         map_type_arg,
#         map_server_node,
#         amcl_node,
#         lifecycle_node
#     ])



import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    nav2_yaml = os.path.join(get_package_share_directory('localization_server'), 'config', 'amcl_config.yaml')
    map_file = os.path.join(get_package_share_directory('map_server'), 'config', 'warehouse_map_real.yaml')

    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': False}, 
                        {'yaml_filename':map_file}]
        ),
            
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_yaml]
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': False},
                        {'autostart': True},
                        {'node_names': ['map_server', 'amcl']}]
        )
    ])