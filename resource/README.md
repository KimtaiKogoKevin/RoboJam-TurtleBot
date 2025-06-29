sudo apt install ros-galactic-webots-ros2

nano .bashrc
	somehow write a script that can launch simulation or real robot
	

ros2 launch webots_ros2_turtlebot robot_launch.py nav:=true
	
ros2 launch turtlebot4_navigation slam_sync.launch.py
ros2 launch turtlebot4_viz view_robot.launch.py


Starting the TurtleBot
- TurtleBot3 in Webots: /opt/ros/galactic/share/webots_ros2_turtlebot/launch/robot_launch.py
- TurtleBot4 in Gazebo:
- TurtleBot4 in 
