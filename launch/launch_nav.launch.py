import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():

    DeclareLaunchArgument(name="use_jsp", default_value="jsp",
                              description="gui (default): use jsp_gui, jsp: use joint_state_publisher, none: no joint states published"),

    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    package_name='vander_one' #<--- CHANGE ME
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    rviz_config_dir = os.path.join(get_package_share_directory(package_name),
                                   'rviz', 'slam_config.rviz')

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    gazebo_params_file = os.path.join(get_package_share_directory(package_name),'config','gazebo_params.yaml')

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                    launch_arguments={'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file}.items()
             )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'my_bot'],
                        output='screen')

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui"
        #condition= LaunchConfigurationEquals("use_jsp", "gui")
    )

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        condition= LaunchConfigurationEquals("use_jsp", "jsp")
    )

    #teleop_node = Node(
     #   package='teleop_twist_keyboard',
      #  executable='teleop_twist_keyboard',
       # name='teleop_node',
        #remappings=[('/cmd_vel','/diff_cont/cmd_vel_unstamped')]
    #)

    # Include the SLAM Toolbox launch file
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py'
        )]),
        launch_arguments={
            'params_file': os.path.join(get_package_share_directory(package_name), 'config', 'nav2_params.yaml'),
            'use_sim_time': use_sim_time
        }.items()
    )

    # Launch them all!
    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        diff_drive_spawner,
        joint_broad_spawner,
        navigation_launch
    ])
