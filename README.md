# Inception_turtlebot_challange
 This was the Turtlebot3 challange and the instructions were as follows:
 
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
