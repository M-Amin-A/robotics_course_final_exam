import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    base_dir = get_package_share_directory('robot_control')
    use_sim_time = True
    
    world_control_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='world_control_bridge',
        output='screen',
        arguments=['/world/depot/control@ros_gz_interfaces/srv/ControlWorld']
    )
    
    rl_controller_node = Node(
        package="robot_control",
        executable="rl_controller_node",
        name="rl_controller_node",
        output="screen",
    )
            
    pid_controller_node = Node(
        package="robot_control",
        executable="pid_controller_node",
        name="pid_controller_node",
        output="screen",
    )
            
    return LaunchDescription([
        # world_control_bridge,
        # rl_controller_node,
        pid_controller_node
    ])
