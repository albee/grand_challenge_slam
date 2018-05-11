Operating Turtlebot for SLAM simulation and hardware

--Single Agent, Hardware--
[REMOTE]
roscore
ssh pi@[ip]
#modify ip in .bashrc
~/.bashrc

[TURTLEBOT]
roslaunch turtlebot3_bringup turtlebot3_robot.launch

[REMOTE]
roslaunch turtlebot3_slam turtlebot3_slam.launch
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
rosrun rviz rviz -d `rospack find turtlebot3_slam`/rviz/turtlebot3_slam.rviz

--Single Agent, Sim--
[REMOTE]
roscore
roslaunch turtlebot3_gazebo turtlebot3_world.launch
roslaunch turtlebot3_slam turtlebot3_slam.launch
rosrun rviz rviz -d `rospack find turtlebot3_slam`/rviz/turtlebot3_slam.rviz
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch


--Multi Agent, Sim--
[REMOTE]
roscore
roslaunch turtlebot3_gazebo multi_turtlebot3.launch
roslaunch turtlebot3_gazebo multi_turtlebot3_slam.launch ns:=tb3_0
roslaunch turtlebot3_gazebo multi_turtlebot3_slam.launch ns:=tb3_1
roslaunch turtlebot3_gazebo multi_turtlebot3_slam.launch ns:=tb3_2
roslaunch turtlebot3_gazebo multi_map_merge.launch 
rosrun rviz rviz -d `rospack find turtlebot3_gazebo`/rviz/multi_turtlebot3_slam.rviz
rosrun turtlebot3_teleop turtlebot3_teleop_key cmd_vel:=tb3_0/cmd_vel


--Multi Agent, Hardware--
[REMOTE]
roscore
ssh pi@[ip1]
#modify ip in .bashrc
~/.bashrc

ssh pi@[ip2]
#modify ip in .bashrc
~/.bashrc

ssh pi@[ip3]
#modify ip in .bashrc
~/.bashrc

[TURTLEBOTx1]
roslaunch turtlebot3_bringup turtlebot3_robot_multi.launch ns:=tb3_0

[TURTLEBOTx2]
roslaunch turtlebot3_bringup turtlebot3_robot_multi.launch ns:=tb3_1

[TURTLEBOTx3]
roslaunch turtlebot3_bringup turtlebot3_robot_multi.launch ns:=tb3_2

[REMOTE]
roslaunch turtlebot3_slam_multi.launch ns:=tb3_0
rosrun turtlebot3_teleop turtlebot3_teleop_key cmd_vel:=tb3_0/cmd_vel
rosrun map_server map_saver -f blargh_map
rosrun map_server map_server map_file:=blargh_map

#Localize from given map (requires initial position)

roslaunch turtlebot3_slam serve_map.launch ns:=blargh_map
roslaunch turtlebot3_slam turtlebot3_localize_multi.launch ns:=tb3_0 x0:=0.0 y0:=-2.0 th0:=1.57
roslaunch turtlebot3_slam turtlebot3_localize_multi.launch ns:=tb3_1 x0:=0.0 y0:=-0.5 th0:=1.57
roslaunch turtlebot3_slam turtlebot3_localize_multi.launch ns:=tb3_2 x0:=0.5 y0:=0.0 th0:=0.0
rosrun rviz rviz -d `rospack find turtlebot3_slam`/rviz/turtlebot3_localize.rviz

