import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_name_in_model = 'fishbot'
    package_name = 'fishbot_description'
    urdf_name = "fishbot_gazebo.urdf"
    world_file = "test_site.world"
    local_domain_id = '1'
    remote_domain_id = '0'

    ld = LaunchDescription()
    pkg_share = FindPackageShare(package=package_name).find(package_name) 
    urdf_model_path = os.path.join(pkg_share, f'urdf/{urdf_name}')
    world_file_path = os.path.join(pkg_share, f'world/{world_file}')

    # Start Gazebo server
    start_gazebo_cmd =  ExecuteProcess(
        cmd=['gazebo', world_file_path, '--verbose','-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )
    
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
    )

    # Launch the robot
    spawn_entity_in_local_gazebo = Node(
        package='fishbot_description', 
        executable='spawn_entity',
        arguments=[
            '-entity', robot_name_in_model,  
            '-file', urdf_model_path, 
            '-b', 
            '-ros_domain', local_domain_id, 
            '-node_name', 'spawn_entity_local'
        ]
        , output='screen'
    )
    
    spawn_entity_in_remote_gazebo = Node(
        package='fishbot_description', 
        executable='spawn_entity',
        arguments=[
            '-entity', robot_name_in_model,  
            '-file', urdf_model_path, 
            '-b', 
            '-ros_domain', remote_domain_id, 
            '-node_name', 'spawn_entity_remote'
        ]
        , output='screen'
    )
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        arguments=[urdf_model_path]
    )
    
    state_sync_node = Node(
        package='fishbot_description',
        executable='state_sync',
        name='state_sync',
        output='screen',
    )
    
    # ld.add_action(robot_state_publisher_node)
    # ld.add_action(rviz2_node)
    # ld.add_action(start_gazebo_cmd)
    ld.add_action(spawn_entity_in_local_gazebo)
    ld.add_action(spawn_entity_in_remote_gazebo)
    ld.add_action(state_sync_node)


    return ld
