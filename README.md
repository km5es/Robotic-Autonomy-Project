# Robotic-Autonomy-Project
MAE-6592 final class project

## Install ROS nodes
To install the ROS nodes clone this repo into your catkin workspace:

`cd /home/$USER/catkin_ws/src/`

`git clone https://github.com/km5es/Robotic-Autonomy-Project.git
cd ..
catkin_make`

## Install teleop:
>sudo apt update
>sudo apt install ros-melodic-teleop-twist-keyboard

### Run "human" model in Gazebo:
>roslaunch kbot_description kbot_base_rviz_gazebo.launch

### Run teleop:
>roslaunch kbot_simple_control kbot_control_teleop.launch
