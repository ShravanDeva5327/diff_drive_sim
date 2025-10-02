from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
import xacro

def generate_launch_description():
    pkg_path = get_package_share_directory('diff_drive_sim')
    ros_gz_sim_pkg_path = get_package_share_directory('ros_gz_sim')  # Package used to launch Gazebo

    # Process the xacro file to get the robot description
    xacro_path = os.path.join(pkg_path, 'description/diff_drive.xacro')
    robot_description_raw = xacro.process_file(xacro_path).toxml()

    # Publish the robot state
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_raw}],
        output='screen',
    )

    # Launch Gazebo with the specified world
    launch_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_pkg_path, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': os.path.join(pkg_path, 'worlds', 'obstacles.sdf'),
            'on_exit_shutdown': 'true',
        }.items(),
    )

    # Spawn the robot entity in Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'robot',
            '-topic', '/robot_description',
            '-x', '0', '-y', '0', '-z', '0',
        ],
    )

    # Run the ROS-Gazebo bridge with the specified configuration
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': os.path.join(pkg_path, 'config', 'ros_gz_bridge.yaml')}],
    )

    return LaunchDescription([
        robot_state_publisher_node,
        launch_gazebo,
        spawn_entity,
        ros_gz_bridge,
    ])