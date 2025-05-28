from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources  import PythonLaunchDescriptionSource
import os


def generate_launch_description():

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_your_robot = get_package_share_directory('my_robot_arm')

    urdf_file = os.path.join(pkg_your_robot, 'urdf', 'arm.urdf')
    config_file = os.path.join(pkg_your_robot, 'config', 'controller.yaml')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
            ),
        launch_arguments={
            'verbose': 'true',
            'pause': 'false',
            'extra_gazebo_args': '--verbose'
        }.items()
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-topic', 'robot_description',
                '-entity', 'my_robot_arm',
                '-x', '0.0',
                '-y', '0.0',
                '-z', '0.0'
            ],
            output='screen'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[config_file],
            output='screen',
            remappings=[
                ('/robot_description', '/robot_state_publisher/robot_description')]
        ),

        ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active','joint_state_broadcaster'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'arm_controller'],
            output='screen'
        )
        # Node(
            # package='controller_manager',
            # executable='spawner.py',
            # arguments=['joint_state_broadcaster'],
            # output='screen'
        # ),
        # Node(
            # package='controller_manager',
            # executable='spawner.py',
            # arguments=['joint_trajectory_controller'],
            # output='screen'
        # )
    ])