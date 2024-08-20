# Inception_turtlebot_challenge


### Authors:
- Keyur Borad (kborad@umd.edu)- 120426049

### Challenge
 This was the Turtlebot3 challenge and the instructions were as follows:
 
   1. Load the aws-robomaker-bookstore-world (found here: https://github.com/aws-robotics/aws-robomaker-bookstore-world) into Gazebo.
   2. Map the environment using a robot of choice. The robot must be equipped with a LiDAR, and RGB or RGB-D camera to perform this project’s tasks.
   3. Spawn ‘n’ robots (n > 1) at various locations in the bookstore environment. ‘n’ can depend on the computational capabilities of your machine.
   4. Spawn a human model at a random free-space location away from any of the ‘n’ robots.
   5. Spawn ‘m’ red cubes at random locations. Have m > n such that the robots will definitely encounter these red cubes. The cubes denote hazards, and the area within a radius around them is considered a hazardous region in the environment. Once a robot detects a red cube, it must store its location w.r.t the map frame, and update the map such that other robots do not plan paths near the hazardous region.
   6. The robots must now explore the environment avoiding hazardous regions, and detect the human. If a robot detects the human, it must communicate this with the other robots and all robots must navigate to the human.

Steps 3-6 must be performed for several iterations. In each iteration, the location of the robots, red cubes, and the human must be random.
Constraints

   1. Use ROS2’s SLAM Toolbox for mapping
   2. Use Nav2’s behavior trees for navigation.
   3. Use a deep learning model such as Yolo for detecting the human.
   4. Use any accurate and efficient method to detect the red cubes. 




### Instructions to Run the Code:
1. Download the package in your workspace and navigate to the workspace directory
    ```
    cd YOUR_workspace_directory/src
    ```
2. Setting up in .bashrc
	Add these exports to your bashrc
   ```
    export TURTLEBOT3_MODEL=waffle
    export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:<your absolute path to the models folder inside pacakage>
	```
   Example:
   ```
	export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/keyur/incep/inception_challange_ws/src/aws-robomaker-bookstore-world-ros2/models
    ```
   
2. Build the package:
    ```
    cd YOUR_workspace_directory
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
    Maps by me are stored in
    aws-robomaker-bookstore-world-ros2/maps/turtlebot3_waffle


### Identifying empty space for random spawning
   1. Identified free space from the my_map.pgm
   2. grayscaled and applied binary thresold
   3. applied Erosion morph to get the feasible space (to avoid stood legs)
   4. appended the pixels in white in a list
   5. Applied coordinate transformation to get pixel value in map coordinate
   6. Scaled the pixel coordinate values to actual map values by multiplying with resolution
   7. made text file of the list and randomly selected coordinate to spawn my various objects