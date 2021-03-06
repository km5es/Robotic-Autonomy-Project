# Robotic-Autonomy-Project
MAE-6592 final class project

## Install dependencies:
```
sudo apt update
sudo apt-get install ros-melodic-teleop-twist-keyboard ros-melodic-gazebo-ros-pkgs ros-melodic-gazebo-ros-control
```
### ROS Depth Camera in Gazebo
See http://gazebosim.org/tutorials/?tut=ros_depth_camera for full installation tutorial.

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
chmod +x src/Robotic-Autonomy-Project/human_detector/scripts/human_detector.py
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
The human truth data will be saved in /home/$USER/track_human.csv <br />
The robot truth data will be saved in /home/$USER/track_robot.csv <br />
EKF data will be dumped in /home/$USER/track_EKF.csv <br />
The data folder contains some example data and a video. <br />

### NOTE
If the robot gets stuck in a wall there is an explicit teleop launch to get it out:
```
roslaunch kbot_simple_control follower_control_teleop.launch
```
### Example simulation of robot tracking human
![Example screenshot](/data/ezgif.com-crop(2).gif)


