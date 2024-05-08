# Differential Drive Robot README

## Overview

This project focuses on building and simulating a differential-drive robot using ROS (Robot Operating System) and URDF (Unified Robot Description Format). A differential-drive robot is characterized by having two driven wheels, one on the left and one on the right, controlling its motion, while other wheels such as caster wheels aid in stability.

## Installation

To run this project, ensure you have ROS Foxy installed. Use the following command to install necessary packages:

To install this package, use `sudo apt install ros-foxy-xacro ros-foxy-joint-state-publisher-gui`.


## Understanding the Whole Picture
Xacro is a macro language that simplifies the creation of URDF files. The robot_state_publisher expects joint values published on the `/joint_states` topic. For initial tests, the `joint_state_publisher_gui` can be used to provide fake values.

![alt text](https://articulatedrobotics.xyz/assets/images/rsp-2afa5bef6c72583be919186c08605d9f.png)


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

## Creating Robot Core

```xml
<material name="white">
   <color rgba="1 1 1 1"/>
</material>
<!-- Other material definitions -->

<joint name="chassis_joint" type="fixed">
   <!-- Joint definition -->
</joint>

<link name="chassis">
   <!-- Chassis link definition -->
</link>

# Wheel Configuration

## Left Wheel

```xml
<joint name="left_wheel_joint" type="continuous">
   <parent link="base_link"/>
   <child link="left_wheel"/>
   <origin xyz="0 0.175 0" rpy="-${pi/2} 0 0"/>
   <axis xyz="0 0 1"/>
</joint>

<link name="left_wheel">
   <visual>
       <geometry>
           <cylinder length="0.04" radius="0.05" />
       </geometry>
       <material name="blue"/>
   </visual>
</link>

## Right Wheel Configuration

```xml
<joint name="right_wheel_joint" type="continuous">
   <parent link="base_link"/>
   <child link="right_wheel"/>
   <origin xyz="0 -0.175 0" rpy="${pi/2} 0 0"/>
   <axis xyz="0 0 -1"/>
</joint>

<link name="right_wheel">
   <visual>
       <geometry>
           <cylinder length="0.04" radius="0.05" />
       </geometry>
       <material name="blue"/>
   </visual>
</link>

