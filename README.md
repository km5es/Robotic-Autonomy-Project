# Robotic-Autonomy-Project
MAE-6592 final class project

## Install ROS nodes
To install the ROS nodes clone this repo into your catkin workspace:
```
cd /home/$USER/catkin_ws/src/
git clone https://github.com/km5es/Robotic-Autonomy-Project.git
cd ..
catkin_make
```
## Install teleop:
```
sudo apt update
sudo apt install ros-melodic-teleop-twist-keyboard
```
### Run "human" model in Gazebo:
```
roslaunch kbot_description kbot_base_rviz_gazebo.launch
```

### Run teleop:
```
roslaunch kbot_simple_control kbot_control_teleop.launch
```

## ROS Depth Camera in Gazebo
See http://gazebosim.org/tutorials/?tut=ros_depth_camera for full installation tutorial. Make sure gazebo_ros_pkgs is installed.
```
sudo apt-get install ros-melodic-gazebo-ros-pkgs ros-melodic-gazebo-ros-control
```
To view depth camera output in RViz:
```
rosrun rviz rviz
```
Set Fixed Frame to 'camera_link' and add a PointCloud2 and Image with Topics '/camera/depth/points' and '/camera/color/raw_image' respectively.

## Launch file with hall and robot and human_prob_model service node
```
roslaunch src/chapter-03/src/kbot_description/launch/kbot_base_rviz_gazebo.launch
```
In Gazebo, go to the Insert tab in the left panel, click the 'kinect' model and click again in the hall to place it. Make sure the simulation is playing so that the plug-in publishes on the camera topics.

## Run human_detector node
```
rosrun human_detector human_detector.py
```
This will print Hello World and the X and Y pixel coordinates of the centroid of the orange kbot to the terminal. It will also print the state of the prob model.

![Example screenshot](/images/ros_rviz_example.png)
