from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.actions import RegisterEventHandler, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression

import os


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value=use_sim_time,
        description='If true, use simulated clock')

    frame_name_args = DeclareLaunchArgument(
        'frame_name',
        default_value='',
        description='Frame name included in the SDF world file')
    
    drone_type = LaunchConfiguration('drone_type', default='x500_tilting')
    drone_type_args = DeclareLaunchArgument('drone_type', default_value=drone_type,
                                            description='Sim Models (x500, rc_cessna, ...)')

    world_name = LaunchConfiguration('world_name', default='default')
    world_name_arg = DeclareLaunchArgument('world_name',
                                           default_value=world_name,
                                           description='World name (without .sdf)')

    os.environ['GZ_SIM_RESOURCE_PATH'] = ':' + os.path.join(get_package_share_directory('high_fidelity_tilting_quadrotor'), 'models')
    os.environ['GZ_SIM_RESOURCE_PATH'] += ':' + os.path.join(get_package_share_directory('high_fidelity_tilting_quadrotor'), 'worlds')


    wait_spawn = ExecuteProcess(cmd=["sleep", "5"])

    model_sdf_filename = [
        get_package_share_directory('high_fidelity_tilting_quadrotor'),
        '/models/',
        LaunchConfiguration('drone_type'),
        '/model.sdf']

    model_name = [LaunchConfiguration('drone_type'), "_0"]

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-file', model_sdf_filename,
                   '-name', model_name,
                   '-allow_renaming', 'true',
                   '-x', "0.0",
                   '-y', "0.0",
                   '-z', "0.3",
                   '-R', "0.0",
                   '-P', "0.0",
                   '-Y', "0.0"])

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        #parameters=[{'use_sim_time': use_sim_time}],
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                   '/model/x500_tilting_0/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                   '/world/default/model/x500_tilting_0/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
                   '/x500_tilting_0/command/motor_speed@actuator_msgs/msg/Actuators]gz.msgs.Actuators',
                   '/model/x500_tilting_0/command/servo_pos@actuator_msgs/msg/Actuators]gz.msgs.Actuators',
                   ],
        remappings=[
            ('/world/default/model/x500_tilting_0/joint_state', '/x500_tilting_0/joint_states'),
            ('/x500_tilting_0/command/motor_speed', '/x500_tilting_0/motor_speed'),
            ('/model/x500_tilting_0/command/servo_pos', '/x500_tilting_0/servo_pos'),
            ('/model/x500_tilting_0/odometry', '/x500_tilting_0/odometry'),
        ],
        output='screen'
    )
    return LaunchDescription([
        # Launch gazebo environment
        use_sim_time_arg,
        drone_type_args,
        world_name_arg,
        frame_name_args,
        spawn_entity,
        bridge,
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('ros_gz_sim'),
                              'launch', 'gz_sim.launch.py')]),
            launch_arguments=[('gz_args', [' -r -v 1 ', LaunchConfiguration('world_name'), '.sdf'])]
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[wait_spawn],
            )
        ),
    ])
