import random
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Define the path to the SDF file
    model_path = os.path.join(
        get_package_share_directory('aws_robomaker_bookstore_world'),
        'models', 'human', 'model.sdf'
    )
    launch_file_dir = os.path.join(get_package_share_directory('aws_robomaker_bookstore_world'), 'launch')

    free_space_file_path = os.path.join(launch_file_dir,'tuples.txt')
    tuples_list = []

    with open(free_space_file_path, 'r') as file:
        for line in file:
            line = line.strip()
            tup = eval(line)
            tuples_list.append(tup)
    pos = random.sample(tuples_list, 1)


   
    z_pos=0.0
    # Create a Node to spawn the human model
    spawn_human = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_human',
        output='screen',
        arguments=[
            '-file', model_path,
            '-entity', 'human_model',
            '-x', str(pos[0][0]),
            '-y', str(pos[0][1]),
            '-z', str(z_pos)
        ]
    )

    # Launch the node
    return LaunchDescription([
        spawn_human
    ])
