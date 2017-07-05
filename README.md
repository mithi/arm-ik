# Robotic arm - Pick & Place Project
This is my turn-in code as partial fulfillment of Udacity's Robotics Nanodegree. 

# Notes

- **You can find the write up of the thought process in the link below**
- - https://github.com/mithi/arm-ik/blob/master/WRITEUP.pdf

- **You may also be interested in the following Jupyter notebooks:**
- - Exposition of the inverse kinematics thought process
- - - https://github.com/mithi/arm-ik/blob/master/joint_angles_ik_exposition.ipynb
- - Optimization of the inverse kinematics process
- - - https://github.com/mithi/arm-ik/blob/master/joint_angles_ik_optimized.ipynb
- - Derivation of the transforms used in the inverse kinematics process
- - - https://github.com/mithi/arm-ik/blob/master/total_transform.ipynb
- - Exposition of Euler angles and how it to get the last three angles 
- - - https://github.com/mithi/arm-ik/blob/master/playground/euler-angles.ipynb

# Dependencies and Setup

- Install ROS on a computer with Ubuntu 16.04 Xenial
- - http://wiki.ros.org/kinetic/Installation/Ubuntu

- Clone Udacity's original repository
- - https://github.com/udacity/RoboND-Kinematics-Project

- Replace `IK_server.py` in `~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/scripts` with the following script below:
- - https://github.com/mithi/arm-ik/blob/master/IK_server.py

- Follow the instructions as indicated in the README of the [original repository](https://github.com/udacity/RoboND-Kinematics-Project)


# Usage

- Make sure the demo flag is set to "false" in `inverse_kinematics.launch` file under `/RoboND-Kinematics-Project/kuka_arm/launch`

- Launch the project 
```
$ cd ~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/scripts
$ ./safe_spawner.sh  
```

- On a separate terminal, run the inverse kinematics script
```
$ cd ~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/scripts
$ rosrun kuka_arm IK_server.py
```

