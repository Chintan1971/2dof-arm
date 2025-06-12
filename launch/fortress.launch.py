from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription,DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration

import os

def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_my_robot_arm = get_package_share_directory('my_robot_arm') # Corrected variable name

    gz_args = LaunchConfiguration('gz_args', default='')
    urdf_file = os.path.join(pkg_my_robot_arm, 'urdf', 'arm.xacro.urdf')
    config_file = os.path.join(pkg_my_robot_arm, 'config', 'controller.yaml')
    cube_urdf = os.path.join(pkg_my_robot_arm, 'urdf', 'movable_object.xacro.urdf')

    with open(urdf_file, 'r') as f:
        robot_description_content = f.read() # Renamed to avoid conflict if publishing

    # It's good practice to publish the robot_description
    # so tools like RViz2 and the Gazebo spawner can use it.

    robot_controllers = os.path.join(pkg_my_robot_arm, 'config', 'controller.yaml')

    robot_description_publisher = Node(
        package='robot_state_publisher', # This node can publish from string param
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description':robot_description_content}] # Important for Gazebo Sim
    )

    world = LaunchConfiguration('world')

    default_world = os.path.join(
        get_package_share_directory('my_robot_arm'),
        'worlds', 
        'empty.world'
    )

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='world to load'
    )
    
    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': ['-r -v4 ',world],'on_exit_shutdown': 'true'}.items()
        )
            # [('gz_args',[gz_args, ' -r -v 1 empty.sdf'])])
        # launch_arguments={'gz_args': '-r empty.sdf'}.items(), # Example: Load an empty world or your specific world

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description', 
            '-name', 'my_robot_arm',
            '-allow_renaming', 'true'
        ],
        output='screen'
    )

    spawn_cube = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name','movable_object',
            '-file', cube_urdf,
            '-allow_renaming', 'true',
            '-x', '5.0', '-y', '0.0', '-z', '0.5',
        ]
    )

    move_object = Node(
        package='my_robot_arm',
        executable='move_object.py',
        name='move_object',
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
        arguments=['joint_state_broadcaster'],
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--param-file', 
                   robot_controllers,],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                   '/world/empty/set_pose@ros_gz_interfaces/srv/SetEntityPose',
                   '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                   '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
                   ],
        output='screen'
    )

    ros_gz_image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/camera/image_raw']
    )
    return LaunchDescription([

        # SetEnvironmentVariable(
        #     name='LD_LIBRARY_PATH',
        #     value=gz_plugin_path + ':' + os.environ.get('LD_LIBRARY_PATH', '')
        # ),
        world_arg,  # Declare the world argument
        gz_sim_launch,
        # ExecuteProcess(
        #     cmd=['ign','gazebo','-r','--verbose'],
        #     output='screen'),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_robot,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[arm_controller_spawner],
            )
        ),
        bridge,
        ros_gz_image_bridge,
        robot_description_publisher, # Publish robot description first
        spawn_robot,
        TimerAction(
            period=1.0,  # Wait for Gazebo to start
            actions=[spawn_cube]  # Spawn the cube after a delay
        ),
        TimerAction(
            period=3.0,  
            actions=[move_object]  # Start moving the object after a delay
        )
    
        # ros2_control_node
        
        # joint_state_broadcaster_spawner,
        # arm_controller_spawner
        # Any other nodes like RViz2 if desired
    ])