import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    base_dir = get_package_share_directory('robot_global_localization')
    map_dir = os.path.join(base_dir , "assets", "depot.yaml")
    amcl_params_dir = os.path.join(base_dir , "config", "amcl_params.yaml")
    use_sim_time = True
    
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename' : map_dir}]
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[amcl_params_dir]
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': True},
                    {'autostart': True},
                    {'node_names': ['map_server', 'amcl']}]
    )

    map_odom_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )

    planning_node = Node(
        package="robot_global_localization",
        executable="planning_node",
        name="planning_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )
    
    pos_publisher_node = Node(
        package="robot_global_localization",
        executable="initial_pos_publisher_node",
        name="initial_pos_publisher_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )
            
    return LaunchDescription([
        map_server,
        amcl,
        lifecycle_manager,
        map_odom_tf_publisher,
        planning_node,
        # pos_publisher_node
    ])
