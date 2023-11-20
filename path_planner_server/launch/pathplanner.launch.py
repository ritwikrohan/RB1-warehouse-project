import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation clock if true')

    param_substitutions = {
        'use_sim_time': use_sim_time
    }

    controller_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'controller.yaml')
    bt_navigator_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'bt.yaml')
    planner_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'planner_server.yaml')
    recovery_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'recovery.yaml')
    filters_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'filters.yaml')
    rviz_config = os.path.join(get_package_share_directory('path_planner_server'), 'rviz', 'pathplanning.rviz')

    controller_server_node_config = RewrittenYaml(
        source_file=controller_yaml,
        root_key='',
        param_rewrites=param_substitutions,
        convert_types=True
    )

    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[controller_server_node_config],
        remappings=[
            ('/cmd_vel', '/robot/cmd_vel'),
        ]
    )

    planner_server_node_config = RewrittenYaml(
        source_file=planner_yaml,
        root_key='',
        param_rewrites=param_substitutions,
        convert_types=True
    )

    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[planner_server_node_config]
    )
    
    recoveries_server_node_config = RewrittenYaml(
        source_file=recovery_yaml,
        root_key='',
        param_rewrites=param_substitutions,
        convert_types=True
    )

    recoveries_server_node = Node(
        package='nav2_recoveries',
        executable='recoveries_server',
        name='recoveries_server',
        output='screen',
        parameters=[recoveries_server_node_config]
    )

    bt_navigator_node_config = RewrittenYaml(
        source_file=bt_navigator_yaml,
        root_key='',
        param_rewrites=param_substitutions,
        convert_types=True
    )

    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[bt_navigator_node_config]
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    filter_mask_config = RewrittenYaml(
        source_file=filters_yaml,
        root_key='',
        param_rewrites=param_substitutions,
        convert_types=True
    )

    filter_mask = Node(
        package='nav2_map_server',
        executable='map_server',
        name='filter_mask_server',
        output='screen',
        emulate_tty=True,
        parameters=[filter_mask_config])

    costmap_node_config = RewrittenYaml(
        source_file=filters_yaml,
        root_key='',
        param_rewrites=param_substitutions,
        convert_types=True
    )

    costmap_node = Node(
        package='nav2_map_server',
        executable='costmap_filter_info_server',
        name='costmap_filter_info_server',
        output='screen',
        emulate_tty=True,
        parameters=[costmap_node_config])

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_pathplanner',
        output='screen',
        parameters=[{'autostart': True},
                    {'node_names': ['planner_server',
                                    'controller_server',
                                    'recoveries_server',
                                    'bt_navigator',
                                    'filter_mask_server',
                                    'costmap_filter_info_server']}]
    )

    approach_service_server = Node(
        package='attach_shelf',
        executable='approach_service_server_node',
        output='screen',
        name='approach_service_server',
        emulate_tty=True,
        
    )
    
    return LaunchDescription([
        approach_service_server,
        declare_use_sim_time_cmd,
        controller_server_node,
        planner_server_node,
        recoveries_server_node,
        bt_navigator_node,
        rviz2_node,
        filter_mask,
        costmap_node,
        lifecycle_manager_node
    ])