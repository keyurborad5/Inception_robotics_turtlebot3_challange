import os
import sys

import launch
from launch.conditions import IfCondition
import launch.launch_description_sources
from launch.substitutions import PythonExpression
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    launch_file_dir = os.path.join(get_package_share_directory('aws_robomaker_bookstore_world'), 'launch')
    world_file_name = "bookstore.world"
    world = os.path.join(get_package_share_directory('aws_robomaker_bookstore_world'), 'worlds', world_file_name)

    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time', default='true')
    x_pose = launch.substitutions.LaunchConfiguration('x_pose', default='0.00')
    y_pose = launch.substitutions.LaunchConfiguration('y_pose', default='0.0')


    gazebo_ros = get_package_share_directory('gazebo_ros')
    gazebo_client = launch.actions.IncludeLaunchDescription(
	launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros, 'launch', 'gzclient.launch.py')),
        condition=launch.conditions.IfCondition(launch.substitutions.LaunchConfiguration('gui'))
     )
    gazebo_server = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros, 'launch', 'gzserver.launch.py'))
    )

    robot_state_publisher_cmd = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    spawn_turtlebot_cmd = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose
        }.items()
    )


    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
          'world',
          default_value=[world, ''],
          description='SDF world file'),
        launch.actions.DeclareLaunchArgument(
            name='gui',
            default_value='false'
        ),
        gazebo_server,
        gazebo_client,
        robot_state_publisher_cmd,
        spawn_turtlebot_cmd
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
