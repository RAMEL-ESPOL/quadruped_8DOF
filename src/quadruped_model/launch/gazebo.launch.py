import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    model_arg = DeclareLaunchArgument(name='model', description='Absolute path to robot urdf file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    package_name = 'quadruped_model'
    pkg_share = FindPackageShare(package=package_name).find(package_name)

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    robot_name_in_model = 'quadruped_model'

    # Get URDF file path
    urdf_file_name = 'quadruped_SimpleModel.urdf'
    urdf = os.path.join(
        get_package_share_directory('quadruped_model'),
        'urdf',
        urdf_file_name
    )
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    robot_description = {"robot_description": robot_desc}

    # Define nodes
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
    )

    start_joint_state_publisher_cmd = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
        name='joint_state_publisher',
    )

    # Spawn the robot in Gazebo
    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            "-topic", "/robot_description", 
            "-entity", robot_name_in_model,
            "-x", '0.0',
            "-y", '0.0',
            "-z", '0.05',
            "-Y", '0.0'
        ]
    )

    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', 
             '-s', 'libgazebo_ros_init.so'], output='screen'
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        gazebo,
        RegisterEventHandler(
            OnProcessExit(
                target_action=gazebo,
                on_exit=[spawn]
            )
        ),
        start_joint_state_publisher_cmd, 
        robot_state_publisher_node,
        rviz2,
    ])
