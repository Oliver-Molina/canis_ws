# canis_ws
## Directions
1. Install Ubuntu 20.04 and install ROS Noetic
2. Have both ```source /opt/ros/noetic/setup.bash``` and ```source ~/(your workspace name)/devel/setup.bash``` in your bashrc
3. Open a new terminal to resource your bashrc after having added the previous lines
4. Run ```roslaunch robot_core main.launch``` to run the ROS nodes
5. To test if its working, run ```rostopic echo /actuation/leg/forearm/superior/right``` in a new window to . In another new window run ```rostopic pub /desired_pos/superior/right geometry_msgs/PointStamped "{header: auto, point: {x: 0, y: 0, z: -0.1}}"``` and watch the rostopic echo window and you should see an angle published with the resulting angle position.
6. To set vel run: ```rostopic pub /command/twist geometry_msgs/TwistStamped "{header: auto, twist: {linear: {x: 0.01, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}}}"```
