import os
import random
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']

    launch_file_dir = os.path.join(get_package_share_directory('aws_robomaker_bookstore_world'), 'launch')
    world_file_name = "bookstore.world"
    world = os.path.join(get_package_share_directory('aws_robomaker_bookstore_world'), 'worlds', world_file_name)
    declare_world=DeclareLaunchArgument(
          'world',
          default_value=[world, ''],
          description='SDF world file')
    declare_gui=DeclareLaunchArgument(
            name='gui',
            default_value='false')
    free_space_file_path = os.path.join(launch_file_dir,'tuples.txt')
    tuples_list = []

    with open(free_space_file_path, 'r') as file:
        for line in file:
            line = line.strip()
            tup = eval(line)
            tuples_list.append(tup)
    # print("Tuples List:", tuples_list)
    pos = random.sample(tuples_list, 3)
    print(pos)

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.00')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    
    urdf_file_name = 'turtlebot3_' + TURTLEBOT3_MODEL + '.urdf'
    urdf_path = os.path.join(
        get_package_share_directory('aws_robomaker_bookstore_world'),
        'urdf',
        urdf_file_name)

    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    model_folder = 'turtlebot3_' + TURTLEBOT3_MODEL
    sdf_path0 = os.path.join(
        get_package_share_directory('aws_robomaker_bookstore_world'),
        'models',
        model_folder,
        'model0.sdf'
    )
    sdf_path1 = os.path.join(
        get_package_share_directory('aws_robomaker_bookstore_world'),
        'models',
        model_folder,
        'model1.sdf'
    )
    sdf_path2 = os.path.join(
        get_package_share_directory('aws_robomaker_bookstore_world'),
        'models',
        model_folder,
        'model2.sdf'
    )

    gazebo_ros = get_package_share_directory('gazebo_ros')
    gazebo_client = IncludeLaunchDescription(
	PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros, 'launch', 'gzclient.launch.py')),
        condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))
     )
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros, 'launch', 'gzserver.launch.py'))
    )

    
    # static_transform_publisher_node = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='link1_broadcaster',
    #     arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'odom'],
    #     output='screen',
    # )
    # Declare arguments
    declare_model_arg = DeclareLaunchArgument(
        'model', default_value=os.environ.get('TURTLEBOT3_MODEL', 'burger'),
        description='Model type [burger, waffle, waffle_pi]'
    )
    

    declare_first_tb3_arg = DeclareLaunchArgument('first_tb3', default_value='tb3_0')
    declare_second_tb3_arg = DeclareLaunchArgument('second_tb3', default_value='tb3_1')
    declare_third_tb3_arg = DeclareLaunchArgument('third_tb3', default_value='tb3_2')

    declare_first_tb3_x_pos_arg = DeclareLaunchArgument('first_tb3_x_pos', default_value=str(pos[0][0]))
    declare_first_tb3_y_pos_arg = DeclareLaunchArgument('first_tb3_y_pos', default_value=str(pos[0][1]))
    declare_first_tb3_z_pos_arg = DeclareLaunchArgument('first_tb3_z_pos', default_value='0.01')
    declare_first_tb3_yaw_arg = DeclareLaunchArgument('first_tb3_yaw', default_value='0.0')

    declare_second_tb3_x_pos_arg = DeclareLaunchArgument('second_tb3_x_pos', default_value=str(pos[1][0]))
    declare_second_tb3_y_pos_arg = DeclareLaunchArgument('second_tb3_y_pos', default_value=str(pos[1][1]))
    declare_second_tb3_z_pos_arg = DeclareLaunchArgument('second_tb3_z_pos', default_value='0.01')
    declare_second_tb3_yaw_arg = DeclareLaunchArgument('second_tb3_yaw', default_value='0.0')

    declare_third_tb3_x_pos_arg = DeclareLaunchArgument('third_tb3_x_pos', default_value=str(pos[2][0]))
    declare_third_tb3_y_pos_arg = DeclareLaunchArgument('third_tb3_y_pos', default_value=str(pos[2][1]))
    declare_third_tb3_z_pos_arg = DeclareLaunchArgument('third_tb3_z_pos', default_value='0.01')
    declare_third_tb3_yaw_arg = DeclareLaunchArgument('third_tb3_yaw', default_value='0.0')
    
    # Include the Gazebo empty_world launch file
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'empty_world.launch.py')),
        launch_arguments={
            'world_name': os.path.join(
                get_package_share_directory('turtlebot3_gazebo'), 'worlds', 'turtlebot3_house.world'),
            'paused': 'false',
            'use_sim_time': 'true',
            'gui': 'true',
            'headless': 'false',
            'debug': 'false'
        }.items()
    )



    # Function to create a group for each turtlebot3
    def create_tb3_group(namespace, x_pos, y_pos, z_pos, yaw,sdf_path,tb):
        return GroupAction([
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name=f'link1_broadcaster_{tb}',
                arguments=['0', '0', '0', '0', '0', '0', '1', 'map', f'{tb}/odom'],
                output='screen',
            ),
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                namespace=namespace,
                output='screen',
                parameters=[{
                    'robot_description': robot_desc,
                    'publish_frequency': 50.0,
                    'use_sim_time': use_sim_time,

                    # 'tf_prefix': 'tb3_0'
                }],
                    remappings=[
                        ('/tf', 'tf'),
                        ('/tf_static','tf_static')
                    
                    ],  
            ),
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-file', sdf_path,
                    '-entity', namespace, 
                    '-x', x_pos, 
                    '-y', y_pos, 
                    '-z', z_pos, 
                    '-Y', yaw, 
                ],
                # remappings=[
                #     ('/odom', '/new_odom'),
                #     ('/imu','/new_imu')
                
                # ],
                output='screen'
            )
        ])
    
    ## spawn Red Cubes
    spawn_red_cubes = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_redcubes.launch.py'))
    )

    ## spawn HUMAN
    spawn_human = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_human.launch.py'))
    )

    # Create the LaunchDescription
    return LaunchDescription([
        # declare_model_arg,
        declare_world,
        gazebo_server,
        gazebo_client,
        # static_transform_publisher_node,
        declare_gui,
        declare_first_tb3_arg,
        declare_second_tb3_arg,
        declare_third_tb3_arg,
        declare_first_tb3_x_pos_arg,
        declare_first_tb3_y_pos_arg,
        declare_first_tb3_z_pos_arg,
        declare_first_tb3_yaw_arg,
        declare_second_tb3_x_pos_arg,
        declare_second_tb3_y_pos_arg,
        declare_second_tb3_z_pos_arg,
        declare_second_tb3_yaw_arg,
        declare_third_tb3_x_pos_arg,
        declare_third_tb3_y_pos_arg,
        declare_third_tb3_z_pos_arg,
        declare_third_tb3_yaw_arg,
        spawn_red_cubes,
        spawn_human,
        # gazebo_launch,
        

        create_tb3_group(LaunchConfiguration('first_tb3'), 
                         LaunchConfiguration('first_tb3_x_pos'), 
                         LaunchConfiguration('first_tb3_y_pos'), 
                         LaunchConfiguration('first_tb3_z_pos'), 
                         LaunchConfiguration('first_tb3_yaw'),sdf_path0,'tb3_0'),
        create_tb3_group(LaunchConfiguration('second_tb3'), 
                         LaunchConfiguration('second_tb3_x_pos'), 
                         LaunchConfiguration('second_tb3_y_pos'), 
                         LaunchConfiguration('second_tb3_z_pos'), 
                         LaunchConfiguration('second_tb3_yaw'),sdf_path1,'tb3_1'),
        create_tb3_group(LaunchConfiguration('third_tb3'), 
                         LaunchConfiguration('third_tb3_x_pos'), 
                         LaunchConfiguration('third_tb3_y_pos'), 
                         LaunchConfiguration('third_tb3_z_pos'), 
                         LaunchConfiguration('third_tb3_yaw'),sdf_path2,'tb3_2')
    ])
