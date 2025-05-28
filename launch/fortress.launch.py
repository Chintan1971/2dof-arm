from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription,SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess, TimerAction

import os

def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_my_robot_arm = get_package_share_directory('my_robot_arm') # Corrected variable name

    urdf_file = os.path.join(pkg_my_robot_arm, 'urdf', 'arm.urdf')
    config_file = os.path.join(pkg_my_robot_arm, 'config', 'controller.yaml')

    with open(urdf_file, 'r') as f:
        robot_description_content = f.read() # Renamed to avoid conflict if publishing

    # It's good practice to publish the robot_description
    # so tools like RViz2 and the Gazebo spawner can use it.
    robot_description_publisher = Node(
        package='robot_state_publisher', # This node can publish from string param
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content,
                     'use_sim_time': True}] # Important for Gazebo Sim
    )
    
    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        # launch_arguments={'gz_args': '-r empty.sdf'}.items(), # Example: Load an empty world or your specific world
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', urdf_file, 
            '-name', 'my_robot_arm'
            # '-allow_renaming', 'true',
            # '-z', '0.1' # Example initial height
        ],
        output='screen'
    )

    # ros2_control node and controller spawners (should remain largely the same)
    # Ensure use_sim_time is True for nodes that need it with Gazebo
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[ config_file,
                    {'use_sim_time': True}],
        output='screen'
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', 
                   '/controller_manager'],
        parameters=[{'use_sim_time': True}],
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--controller-manager', 
                   '/controller_manager'],
        parameters=[{'use_sim_time': True}],
    )

    gz_plugin_path = os.path.join(
        os.getenv('HOME'),
        'ros2_ws',
        'install', 'gz_ros2_control',
        'lib')
    
    return LaunchDescription([

        SetEnvironmentVariable(
            name='LD_LIBRARY_PATH',
            value=gz_plugin_path + ':' + os.environ.get('LD_LIBRARY_PATH', '')
        ),
        robot_description_publisher, # Publish robot description first
        gz_sim_launch,
        ExecuteProcess(
            cmd=['ign','gazebo','-r','--verbose'],
            output='screen'),
        spawn_robot,
        ros2_control_node,
        
        joint_state_broadcaster_spawner,
        arm_controller_spawner
        # Any other nodes like RViz2 if desired
    ])