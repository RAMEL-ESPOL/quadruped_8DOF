import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitution import LaunchConfiguration
from launch_ros.actions import Node
  
def generate_launch_description():
 
 
    model_arg = DeclareLaunchArgument(name='model', description='Absolute path to robot urdf file')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    use_sim_time = LaunchConfiguration('use_sim_time') 
    package_name = 'quadruped_model'
    
    #world_file_path = 'world.world'
    #world = LaunchConfiguration('world')
    #world_path = os.path.join(pkg_share, 'worlds',  world_file_path)

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
        )

    robot_name_in_model = 'quadruped_model'

    # Get URDF via xacro

    urdf_file_name = 'quadruped_SimpleModel.urdf'
    urdf = os.path.join(
        get_package_share_directory('quadruped_model'),
        'urdf',
        urdf_file_name
        )
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()
 
    #rivz2
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
        parameters= [{'use_sim_time': use_sim_time, 'robot_description': robot_desc}] #[{'use_sim_time': use_sim_time, "robot_description": robot_description_content}],
    )

    start_joint_state_publisher_cmd = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
        name='joint_state_publisher',
    )
 
    #spawn the robot 
    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=["-topic", "/robot_description", 
                    "-entity", robot_name_in_model,
                    "-x", '0.0',
                    "-y", '0.0',
                    "-z", '0.5',
                    "-Y", '0.0']
    )

    world = ""

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        rviz2,
        spawn,
        start_joint_state_publisher_cmd, 
        robot_state_publisher_node,
        gzserver_cmd,
        gzclient_cmd
])
