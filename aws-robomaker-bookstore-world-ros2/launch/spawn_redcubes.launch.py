import random
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Define the path to the SDF file
    model_path = os.path.join(
        get_package_share_directory('aws_robomaker_bookstore_world'),
        'models', 'red_cube', 'model.sdf'
    )
    launch_file_dir = os.path.join(get_package_share_directory('aws_robomaker_bookstore_world'), 'launch')

    free_space_file_path = os.path.join(launch_file_dir,'tuples.txt')
    tuples_list = []

    with open(free_space_file_path, 'r') as file:
        for line in file:
            line = line.strip()
            tup = eval(line)
            tuples_list.append(tup)
   

    # List to hold spawn nodes
    spawn_nodes = []

    # Number of cubes to spawn
    num_cubes = 4
    pos = random.sample(tuples_list, num_cubes)

    for i in range(num_cubes):
        x_pos = random.uniform(-7.5, 7.5)
        y_pos = random.uniform(-6.5, 7.0)
        z_pos = 0.0

        spawn_cube = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name=f'spawn_red_cube_{i}',
            output='screen',
            arguments=[
                '-file', model_path,
                '-entity', f'red_cube_{i}',
                '-x', str(pos[i][0]),
                '-y', str(pos[i][1]),
                '-z', str(z_pos)
            ]
        )
        spawn_nodes.append(spawn_cube)

    return LaunchDescription(spawn_nodes)
