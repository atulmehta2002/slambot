import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command

# from launch.actions import RegisterEventHandler
# from launch.event_handlers import OnProcessExit


def generate_launch_description():

    package_name='slambot'
    gazebo_params_file = os.path.join(get_package_share_directory(package_name),'config','gazebo_params.yaml')
    world_path = os.path.join(get_package_share_directory(package_name),'worlds','empty.world')    # maze, obstacles, turtlebot, house


    robot_state_publisher_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','slambot_rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )

    gazebo_startup_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
            launch_arguments={'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file,'world': world_path}.items(),
            )

    spawn_entity = Node(
                    package='gazebo_ros', executable='spawn_entity.py',
                    arguments=['-topic', 'robot_description', '-entity', 'my_slambot'],
                    output='screen',
                    )
    
    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params, {'use_sim_time': True}],
            remappings=[('/cmd_vel_out','/diffbot_base_controller/cmd_vel_unstamped')]
        )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diffbot_base_controller"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    # Run the node
    return LaunchDescription([
        robot_state_publisher_launch,
        gazebo_startup_node,
        spawn_entity,
        twist_mux,
        diff_drive_spawner,
        joint_broad_spawner,
    ])
