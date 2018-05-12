Multi Agent SLAM+Localization, for Sim/Hardware

The current setup uses two ROS packages to operate: gmapping and amcl. gmapping is a particle filter-based SLAM package,
which takes in the /scan topic and publishes the /map and /map_metadata topics and updates the /odom topic. Some teams also rely on the /dynamic_map service, which immediately returns the latest map. amcl (adaptive Monte Carlo localization) estimates position against a known map, updating the /odom topic.

Custom launch files, an rviz config file, and a client node package must be added to the catkin_ws. Launch files must go into their respective Turtlebot packages.

The overall CONOPS is as follows. Note all commands are for [REMOTE] unless indicated [TURTLEBOT]:
(Optional) SLAM/Localization visualization can be launched with  
	```
	rosrun rviz rviz -d `rospack find turtlebot3_slam`/rviz/turtlebot3_localize.rviz
	```

1. A single Turtlebot is started with ns:=tb3_0
	```
	[TURTLEBOT]
	roslaunch turtlebot3_bringup turtlebot3_robot_multi.launch ns:=tb3_0
	```
2. gmapping is started
	```
	roslaunch turtlebot3_slam turtlebot3_slam_multi.launch ns:=tb3_0
	```
3. A single Turtlebot explores the map using the adaptive sampling team's algorithm to explore
4. When adaptive sampling indicates that sampling is complete, the map is saved as a .yaml to the current directory
	```
	rosrun map_server map_saver -f blargh_map
	```
5. The gmapping algorithm is shut off
	```
	rosnode kill /tb3_0/turtlebot3_slam_gmapping
	```
6. The known map is served. NOTE: change to custom map directory
	```
	roslaunch turtlebot3_slam serve_map.launch map_file:=$HOME/catkin_ws/blargh_map.yaml
	```
7. Localization begins on all the Turtlebots with ns:=tb3_0 ns:=tb3_1 and ns:=tb3_2, respectively. Initial positions can be provided, but currently the configuration assumes no known initial position
	```
	roslaunch turtlebot3_slam turtlebot3_localize_multi.launch ns:=tb3_0 x0:=0.0 y0:=-2.0 th0:=1.57
	roslaunch turtlebot3_slam turtlebot3_localize_multi.launch ns:=tb3_1 x0:=0.0 y0:=-0.5 th0:=1.57
	roslaunch turtlebot3_slam turtlebot3_localize_multi.launch ns:=tb3_2 x0:=0.5 y0:=0.0 th0:=0.0
	```

Because multiple Turtlebots are in use, namespacing divides up their essential topics (/cmd_vel, /imu, /joint_states, /odom, /scan) along with the node /robot_state_publisher which is used for updating tf. Custom launch files allow for this, with command line arguments specifying Turtlebot namespace.

Instructions including sim:
```
[REMOTE]
roscore

#IF Driving around to generate map
	#IF visualization desired
		rosrun rviz rviz -d `rospack find turtlebot3_slam`/rviz/turtlebot3_slam.rviz
	#ENDIF

	#IF Hardware
		[TURTLEBOT]
		roslaunch turtlebot3_bringup turtlebot3_robot_multi.launch ns:=tb3_0

		[REMOTE]
		roslaunch turtlebot3_slam turtlebot3_slam_multi.launch ns:=tb3_0
		# Adaptive sampling drives us around
	#ELSE IF Software
		roslaunch turtlebot3_gazebo turtlebot3_world.launch
		roslaunch turtlebot3_slam turtlebot3_slam.launch
		rosrun turtlebot3_teleop turtlebot3_teleop_key cmd_vel

	#ENDIF

	rosrun map_server map_saver -f blargh_map

#ELSE
	# Localize from given map (requires initial position)
	IF Software
		# Ctrl-C Gazebo
		roslaunch turtlebot3_gazebo multi_turtlebot3.launch
	ELSE
		roslaunch turtlebot3_bringup turtlebot3_robot_multi.launch ns:=tb3_1
		roslaunch turtlebot3_bringup turtlebot3_robot_multi.launch ns:=tb3_2
	ENDIF

	roslaunch turtlebot3_slam serve_map.launch map_file:=$HOME/catkin_ws/blargh_map.yaml
	roslaunch turtlebot3_slam turtlebot3_localize_multi.launch ns:=tb3_0 x0:=0.0 y0:=-2.0 th0:=1.57
	roslaunch turtlebot3_slam turtlebot3_localize_multi.launch ns:=tb3_1 x0:=0.0 y0:=-0.5 th0:=1.57
	roslaunch turtlebot3_slam turtlebot3_localize_multi.launch ns:=tb3_2 x0:=0.5 y0:=0.0 th0:=0.0
	rosrun rviz rviz -d `rospack find turtlebot3_slam`/rviz/turtlebot3_localize.rviz

	# Some sort of movement later
	rosrun turtlebot3_teleop turtlebot3_teleop_key cmd_vel:=tb3_0/cmd_vel

#ENDIF
```
