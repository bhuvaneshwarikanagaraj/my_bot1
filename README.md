# Differential Drive Robot README

## Overview

This project focuses on building and simulating a differential-drive robot using ROS (Robot Operating System) and URDF (Unified Robot Description Format). A differential-drive robot is characterized by having two driven wheels, one on the left and one on the right, controlling its motion, while other wheels such as caster wheels aid in stability.

## Installation

To run this project, ensure you have ROS Foxy installed. Use the following command to install necessary packages

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

## Note for First Time Rviz Users

- **Change the Fixed Frame to "Base Link"**
  - This ensures that your robot's base frame is correctly set up for visualization and transformation.

- **Click on Show Name, Show Axis, and Show Arrows**
  - These options enhance visualization by displaying `names`, `axes`, and `arrows` for better understanding of your robot's components.

- **Enable TF**
  - `TF (Transform)` enables the representation of coordinate frames and transformations between them. Make sure this is enabled for accurate visualization.

- **Enable Visual**
  - This option enables the visual representation of your robot, allowing you to see its structure and components.

- **Set Description Topic as Robot Description**
  - Specify the Description Topic as `robot_description`to ensure that the correct robot model is loaded for visualization.

## Creating Left Wheel for our Robot

```
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

```

Moving forward, let’s fabricate the wheels. In ROS, cylinders are oriented along the Z axis by default (up and down). To align with our needs, a “roll” adjustment is necessary, entailing a quarter-turn rotation around the X-axis.



![alt text](https://miro.medium.com/v2/resize:fit:640/format:webp/1*Abg9-DoNrpgD_mxYnfbx4w.gif)

## Creating Right Wheel for our Robot

```
<joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="0 -0.175 0" rpy="${pi/2} 0 0"/>
    <axis xyz="0 0 -1"/>
</joint>

<link name="right_wheel">
    <visual>
        <geometry>
            <cylinder length="0.04" radius="0.05" />/>
        </geometry>
        <material name="blue"/>
    </visual>
</link>
```

Upon running the launch file and initiating a reset, you might encounter an error.


![alt text](https://miro.medium.com/v2/resize:fit:640/format:webp/1*9RDVDW5vrJxkZD4ON82lqQ.png)

This occurs because the joint state publisher for the wheel needs to be activated. Run the joint state publisher to resolve this issue.

This occurs because the joint state publisher for the wheel needs to be activated. Run the joint state publisher to resolve this issue.

Now run `ros2 run joint_state_publisher_gui joint_state_publisher_gui`

![alt text](https://miro.medium.com/v2/resize:fit:640/format:webp/1*5SQ6qC_NguvorWviOtHd5w.png)

## Building Caster Wheel

```
<!-- CASTER WHEEL -->

<joint name="caster_wheel_joint" type="fixed">
    <parent link="chassis"/>
    <child link="caster_wheel"/>
    <origin xyz="0.24 0 0" rpy="0 0 0"/>
</joint>

<link name="caster_wheel">
    <visual>
        <geometry>
            <sphere radius="0.05" />
        </geometry>
        <material name="black"/>
    </visual>
</link>

```
## Adding Collision Tags for the Robot created by us for the chassis

```
<collision>
            <origin xyz="0.15 0 0.075"/>
            <geometry>
                <box size="0.3 0.3 0.15"/>
            </geometry>

```
Easiest way to do this is copy paste from the `<visual> `tag

## Adding Collision and Inertia tags for the drive wheels

```
<collision>
            <geometry>
                <cylinder radius="0.05" length="0.04"/>
            </geometry>
        </collision>
        <xacro:inertial_cylinder mass="0.1" length="0.04" radius="0.05">
            <origin xyz="0 0 0" rpy="0 0 0"/>
        </xacro:inertial_cylinder>

```
## Adding Collision and Inertail tags for teh Caster Wheel

```
<collision>
            <geometry>
                <sphere radius="0.05"/>
            </geometry>
        </collision>
        <xacro:inertial_sphere mass="0.1" radius="0.05">
            <origin xyz="0 0 0" rpy="0 0 0"/>
        </xacro:inertial_sphere>

```

### Now Create file nameed `inertial_macros.xacro`
```
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" >

    <!-- Specify some standard inertial calculations https://en.wikipedia.org/wiki/List_of_moments_of_inertia -->
    <!-- These make use of xacro's mathematical functionality -->

    <xacro:macro name="inertial_sphere" params="mass radius *origin">
        <inertial>
            <xacro:insert_block name="origin"/>
            <mass value="${mass}" />
            <inertia ixx="${(2/5) * mass * (radius*radius)}" ixy="0.0" ixz="0.0"
                    iyy="${(2/5) * mass * (radius*radius)}" iyz="0.0"
                    izz="${(2/5) * mass * (radius*radius)}" />
        </inertial>
    </xacro:macro>  


    <xacro:macro name="inertial_box" params="mass x y z *origin">
        <inertial>
            <xacro:insert_block name="origin"/>
            <mass value="${mass}" />
            <inertia ixx="${(1/12) * mass * (y*y+z*z)}" ixy="0.0" ixz="0.0"
                    iyy="${(1/12) * mass * (x*x+z*z)}" iyz="0.0"
                    izz="${(1/12) * mass * (x*x+y*y)}" />
        </inertial>
    </xacro:macro>


    <xacro:macro name="inertial_cylinder" params="mass length radius *origin">
        <inertial>
            <xacro:insert_block name="origin"/>
            <mass value="${mass}" />
            <inertia ixx="${(1/12) * mass * (3*radius*radius + length*length)}" ixy="0.0" ixz="0.0"
                    iyy="${(1/12) * mass * (3*radius*radius + length*length)}" iyz="0.0"
                    izz="${(1/2) * mass * (radius*radius)}" />
        </inertial>
    </xacro:macro>


</robot>
```


### Here you get the complete robot_core_xacro

```
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" >

    <xacro:include filename="inertial_macros.xacro"/>

    <material name="white">
        <color rgba="1 1 1 1" />
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

    <!-- BASE LINK -->

    <link name="base_link">

    </link>


    <!-- CHASSIS LINK -->

    <joint name="chassis_joint" type="fixed">
        <parent link="base_link"/>
        <child link="chassis"/>
        <origin xyz="-0.1 0 0"/>
    </joint>

    <link name="chassis">
        <visual>
            <origin xyz="0.15 0 0.075"/>
            <geometry>
                <box size="0.3 0.3 0.15"/>
            </geometry>
            <material name="white"/>
        </visual>
        <collision>
            <origin xyz="0.15 0 0.075"/>
            <geometry>
                <box size="0.3 0.3 0.15"/>
            </geometry>
        </collision>
        <xacro:inertial_box mass="0.5" x="0.3" y="0.3" z="0.15">
            <origin xyz="0.15 0 0.075" rpy="0 0 0"/>
        </xacro:inertial_box>
    </link>
    
    <!-- LEFT WHEEL LINK -->

    <joint name="left_wheel_joint" type="continuous">
        <parent link="base_link"/>
        <child link="left_wheel"/>
        <origin xyz="0 0.175 0" rpy="-${pi/2} 0 0" />
        <axis xyz="0 0 1"/>
    </joint>

    <link name="left_wheel">
        <visual>
            <geometry>
                <cylinder radius="0.05" length="0.04"/>
            </geometry>
            <material name="blue"/>
        </visual>
        <collision>
            <geometry>
                <cylinder radius="0.05" length="0.04"/>
            </geometry>
        </collision>
        <xacro:inertial_cylinder mass="0.1" length="0.04" radius="0.05">
            <origin xyz="0 0 0" rpy="0 0 0"/>
        </xacro:inertial_cylinder>
    </link>

    <!-- RIGHT WHEEL LINK -->

    <joint name="right_wheel_joint" type="continuous">
        <parent link="base_link"/>
        <child link="right_wheel"/>
        <origin xyz="0 -0.175 0" rpy="${pi/2} 0 0" />
        <axis xyz="0 0 -1"/>
    </joint>

    <link name="right_wheel">
        <visual>
            <geometry>
                <cylinder radius="0.05" length="0.04"/>
            </geometry>
            <material name="blue"/>
        </visual>
        <collision>
            <geometry>
                <cylinder radius="0.05" length="0.04"/>
            </geometry>
        </collision>
        <xacro:inertial_cylinder mass="0.1" length="0.04" radius="0.05">
            <origin xyz="0 0 0" rpy="0 0 0"/>
        </xacro:inertial_cylinder>
    </link>

    <!-- CASTER WHEEL LINK -->

    <joint name="caster_wheel_joint" type="fixed">
        <parent link="chassis"/>
        <child link="caster_wheel"/>
        <origin xyz="0.24 0 0"/>
    </joint>


    <link name="caster_wheel">
        <visual>
            <geometry>
                <sphere radius="0.05"/>
            </geometry>
            <material name="black"/>
        </visual>
        <collision>
            <geometry>
                <sphere radius="0.05"/>
            </geometry>
        </collision>
        <xacro:inertial_sphere mass="0.1" radius="0.05">
            <origin xyz="0 0 0" rpy="0 0 0"/>
        </xacro:inertial_sphere>
    </link>
    
    
</robot>
```

![alt text](https://miro.medium.com/v2/resize:fit:640/format:webp/1*igiCXIwj01xNSgaXL0ii2g.png)

For a glimpse of the collision geometry, simply uncheck `Visual Enabled` and check `Collision Enabled`.















