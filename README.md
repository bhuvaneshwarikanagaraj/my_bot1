# Differential Drive Robot README

## Overview

This project focuses on building and simulating a differential-drive robot using ROS (Robot Operating System) and URDF (Unified Robot Description Format). A differential-drive robot is characterized by having two driven wheels, one on the left and one on the right, controlling its motion, while other wheels such as caster wheels aid in stability.

## Installation

To run this project, ensure you have ROS Foxy installed. Use the following command to install necessary packages:

To install this package, use `sudo apt install ros-foxy-xacro ros-foxy-joint-state-publisher-gui`.


## Understanding the Whole Picture
Xacro is a macro language that simplifies the creation of URDF files. The robot_state_publisher expects joint values published on the `/joint_states` topic. For initial tests, the `joint_state_publisher_gui` can be used to provide fake values.

![alt text](https://miro.medium.com/v2/resize:fit:1100/format:webp/1*aagM-_T_Pt5Jw16H9TYF-A.png)


### Popular URDF Tags
- `<robot>`: Root element encapsulating the entire robot description.
- `<link>`: Defines rigid bodies or links in the robot.
- `<joint>`: Describes joints connecting links.
- `<joint_type>`: Specifies joint types.
- `<origin>`: Defines positions and orientations.
- `<visual>`: Describes visual representation.
- `<collision>`: Specifies collision properties.
- `<inertial>`: Defines inertial properties.
- `<material>`, `<uri>`, `<geometry>`, `<joint_limit>`: Define material, resources, geometry, and joint limits respectively.

## Creating robot_Core_xacro

```
<material name="white">
   <color rgba="1 1 1 1"/>
</material>

<material name="orange">
   <color rgba="1 0.3 0.1 1"/>
</material>

<material name="blue">
   <color rgba="0.2 0.2 1 1"/>
</material>

<material name="black">
   <color rgba="0 0 0 1"/>
</material>
```

## Creating Chassis joint and link

```
<joint name="chassis_joint" type="fixed">
   <parent link="base_link"/>
   <child link="chassis"/>
   <origin xyz="-0.1 0 0"/>
</joint>

<link name="chassis">
   <visual>
       <origin xyz="0.15 0 0.075" rpy="0 0 0"/>
       <geometry>
           <box size="0.3 0.3 0.15"/>
       </geometry>
       <material name="white"/>
   </visual>
</link>   

```
At this point we can run the launch point and then open rviz, how the robot actual looks like!

# Getting Started Guide

Welcome to our application! If you're a first-time user, here are some steps to get you started:

## Note for First Time Users

- **Change the Fixed Frame to "Base Link"**
  - This ensures that your robot's base frame is correctly set up for visualization and transformation.

- **Click on Show Name, Show Axis, and Show Arrows**
  - These options enhance visualization by displaying names, axes, and arrows for better understanding of your robot's components.

- **Enable TF**
  - TF (Transform) enables the representation of coordinate frames and transformations between them. Make sure this is enabled for accurate visualization.

- **Enable Visual**
  - This option enables the visual representation of your robot, allowing you to see its structure and components.

- **Set Description Topic as Robot Description**
  - Specify the Description Topic as '/robot_description' to ensure that the correct robot model is loaded for visualization.












