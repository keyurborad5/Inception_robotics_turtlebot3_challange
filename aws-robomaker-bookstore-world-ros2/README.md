# Inception Tutrtlebot3 challange

### Authors:
- Keyur Borad (kborad@umd.edu)- 120426049


### Instructions to Run the Code:
1. Download the package in your workspace and navigate to the workspace directory
    ```
    cd YOUR_DIRECTORY
    ```
2. Setting up in .bashrc
    '''
    Add
    export TURTLEBOT3_MODEL=waffle
    export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:<your absolute path to the models folder inside pacakage>
    Example:
	export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/keyur/incep/inception_challange_ws/src/aws-robomaker-bookstore-world-ros2/models
    '''
2. Build the package:
    ```
    rosdep update
    rosdep install --from-paths . --ignore-src -r -y
    colcon build
    ```
3. For Spawning environment as per the challange
    ```
    source install/setup.sh
    ros2 launch aws_robomaker_bookstore_world multirobot_spawn.launch.py gui:=true
    ```
4. For spawning one Turtlebot3 at origin
    ```
    source install/setup.sh
    ros2 launch aws_robomaker_bookstore_world bookstore.launch.py gui:=true
    ```
### Maps
    '''
    maps by me are stored in
    aws-robomaker-bookstore-world-ros2/maps/turtlebot3_waffle
