# aruco_melodic_mapping

Get a map using arucos in ROS Melodic

0) Install aruco in ROS with $ sudo apt-get install ros-melodic-aruco-ros (First update and upgrade)
1) Install DroidCam in your mobile and open it
2) Write your WiFi IP (Example: 192.168.86.805:4747) in the launch file aruco_melodic_mapping.launch in line 3 (Example: value="http://192.168.86.805:4747/video)
3) Enter in your terminal and create a workspace with $ mkdir -p ~/catkin_ws/src
4) In the src directory paste aruco_melodic_mapping
5) Go to catkin_ws diretory and use the command $ catkin_make (This tutorial assumes that you have installed catkin)
6) Use the command $ source devel/setup.bash
7) Launch the pkg while DroidCam is open in your mobile $roslaunch aruco_melodic_mapping aruco_melodic_mapping.launch 
8) The program is subscribed to the topic "/aruco/markers" and publish the topic "/aruco/pub_marker", this one is used by rviz to show the tf of the markers and create the map
