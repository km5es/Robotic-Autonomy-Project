# Robotic-Autonomy-Project
MAE-6592 final class project

## Install dependencies:
```
sudo apt update
sudo apt-get install ros-melodic-teleop-twist-keyboard ros-melodic-gazebo-ros-pkgs ros-melodic-gazebo-ros-control
```
### ROS Depth Camera in Gazebo
See http://gazebosim.org/tutorials/?tut=ros_depth_camera for full installation tutorial. Make sure gazebo_ros_pkgs is installed.

## Install ROS nodes
To install the ROS nodes clone this repo into your catkin workspace:
```
cd /home/$USER/catkin_ws/src/
git clone https://github.com/km5es/Robotic-Autonomy-Project.git
cd ..
catkin_make
```
NOTE: In case it is not already so, make the human_detector node executable by:
```
chmod +x src/human_detector/scripts/human_detector.py
```

### Run "human" model in Gazebo:
```
roslaunch kbot_description kbot_base_rviz_gazebo.launch
```

### Run teleop:
```
roslaunch kbot_simple_control kbot_control_teleop.launch
```
Optional: Set Fixed Frame to 'camera_link' and add a PointCloud2 and Image with Topics '/camera/depth/points' and '/camera/color/image_raw' respectively in the open Rviz window.

## Run human_detector node
```
rosrun human_detector human_detector.py
```
This will print the depth information to the console along with EKF data.

![Example screenshot](/images/ros_rviz_example.png)


