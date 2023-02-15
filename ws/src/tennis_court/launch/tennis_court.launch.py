import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python import get_package_share_directory
from launch_ros.actions import Node


ROS_DISTRO_ELOQUENT = "eloquent"
ROS_DISTRO_FOXY = "foxy"
ROS_DISTRO_HUMBLE = "humble"
ROS_DISTRO = os.environ.get("ROS_DISTRO")


def generate_launch_description():
    executable = "executable" if (ROS_DISTRO == ROS_DISTRO_FOXY) or (ROS_DISTRO == ROS_DISTRO_HUMBLE) else "node_executable"
    pkg_share = get_package_share_directory("tennis_court")
    gazebo_ros_share = get_package_share_directory("gazebo_ros")
    model_path = os.path.join(pkg_share, 'urdf/fledj_bot_description.urdf')
    world_path = os.path.join(pkg_share, 'worlds/court.world')


    # Gazebo Server
    gzserver_launch_file = os.path.join(gazebo_ros_share, "launch", "gzserver.launch.py")
    world_file = os.path.join(pkg_share, "worlds", "court.world")
    gzserver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gzserver_launch_file),
        launch_arguments={
            "world": world_file,
            "verbose": "false",
            "pause": LaunchConfiguration("paused")
        }.items()
    )

    # Gazebo Client
    gzclient_launch_file = os.path.join(gazebo_ros_share, "launch", "gzclient.launch.py")
    gzclient_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gzclient_launch_file),
        condition=IfCondition(LaunchConfiguration("gui"))
    )

    static_tf_node = Node(
        package="tf2_ros",
        arguments=["0", "0", "8", "3.14159", "1.57079", "3.14159", "map", "zenith_camera_link"],
        **{executable: "static_transform_publisher"}
    )

    # Ball Manager
    ball_manager_node = Node(
        package="tennis_court",
        condition=IfCondition(LaunchConfiguration("manager")),
        parameters=[{"use_sim_time": True}],
        output="screen",
        emulate_tty=True,
        **{executable: "ball_manager.py"}
    )

    # Rviz
    rviz_config_file = os.path.join(pkg_share, "config", "config.rviz")
    rviz_node = Node(
        package="rviz2",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(LaunchConfiguration("rviz")),
        parameters=[{"use_sim_time": True}],
        **{executable: "rviz2"}
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )

    spawn_entity = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-entity', 'fledj_bot', '-topic', 'robot_description'],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(name="gui", default_value="true"),
        DeclareLaunchArgument(name="paused", default_value="false"),
        DeclareLaunchArgument(name="rviz", default_value="false"),
        DeclareLaunchArgument(name="manager", default_value="true"),
        DeclareLaunchArgument(name='model', default_value=model_path),
        ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_path], output='screen'),
        gzserver_launch,
        gzclient_launch,
        static_tf_node,
        ball_manager_node,
        rviz_node,
        robot_state_publisher_node,
        spawn_entity
    ])
