# Robotic Arm - Pick & Place Project
This is my turn-in code as partial fulfillment of Udacity's Robotics Nanodegree. 
I wrote code to calculate the joint angles of six degree-of-freedom Kuka Kr210 serial manipulator given the position an orientation of its end effector in 3-D space. This code is used by the robot arm in a simulator to pick cylinders in a shelf 
and drop it in a bin. The simulation is implemented in a ROS environment with Gazebo and Rviz. 

# Notes

## [WRITEUP](./WRITEUP.pd)

## Jupyter notebooks:
- [Exposition of the inverse kinematics thought process](./notebooks/joint_angles_ik_exposition.ipynb)
- [Optimization of the inverse kinematics process](./notebooks/joint_angles_ik_optimized.ipynb)
- [Derivation of the transforms used in the inverse kinematics process](./notebooks//total_transform.ipynb)
- [Exposition of Euler angles and how to get the last three angles](./playground/euler-angles.ipynb)

# Dependencies and Setup
- [Install ROS on a computer with Ubuntu 16.04 Xenial](http://wiki.ros.org/kinetic/Installation/Ubuntu)
- [Clone Udacity's original repository](https://github.com/udacity/RoboND-Kinematics-Project)

- Replace `IK_server.py` in `~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/scripts` with the following script below:
- [IK SERVER PYTHON SCRIPT](./IK_server.py)

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

