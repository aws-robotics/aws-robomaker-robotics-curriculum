---
# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved. // SPDX-License-Identifier: CC-BY-SA-4.0
title: Robot physical properties
permalink: /modules/4/robot-physical-properties.html
---

<!-- http://gazebosim.org/tutorials/?tut=build_robot -->

<!-- http://gazebosim.org/tutorials?tut=ros_gzplugins -->
<!-- http://gazebosim.org/tutorials/?tut=ros_urdf -->

To model any object in simulation, it is necessary to specify the kinematic and dynamics property of the object, the collision model, and the visual representation. 

To simplify the modeling, any object is composed of a number of links and link connectors, called joints. 

There are two types of XML files that can be used: Simulation Description Format (SDF) which is the standard format for Gazebo and Unified Robot Description Format (URDF), used within ROS and converted automatically to SDF by Gazebo.

Building a full realistic object model is relatively complex. Here, we highlight the main components from the ROS perspective that could be changed according to the needs.

Gazebo provides a [complete tutorial on building a full robot in SDF](http://gazebosim.org/tutorials/?tut=build_robot). It is useful also to refer the [SDF documentation](http://sdformat.org/spec).
A full example is provided by this [Gazebo tutorial on building the robot from scratch](http://gazebosim.org/tutorials/?tut=build_robot).

Typically, any robot has a `description` ROS package which includes the URDF. Examples include the [Turtlebot 3](http://wiki.ros.org/turtlebot3_description), [Clearpath Husky](http://wiki.ros.org/husky_description), and the [PR2 robot](http://wiki.ros.org/pr2_description).

For more information, please see the [URDF tutorials](http://wiki.ros.org/urdf/Tutorials) provided by ROS.

For our purpose, we will take a look at the one for the Waveshare Jetbot, in particular at [its URDF file](https://raw.githubusercontent.com/aws-samples/aws-robomaker-jetbot-ros/main/src/jetbot_description/urdf/jetbot.urdf).

```
<?xml version="1.0" ?>
<robot name="JetBot"  xmlns:xacro="http://ros.org/wiki/xacro">
  <xacro:include filename="$(find jetbot_description)/urdf/jetbot.gazebo.xacro"/>

  <property name="M_PI_2" value="1.570796327"/>

  <link name="base_footprint"/>

  <joint name="base_joint" type="fixed">
    <parent link="base_footprint"/>
    <child link="base_link"/>
    <origin xyz="0.0 0.0 0.010" rpy="0 0 0"/>
  </joint>

  <joint name="left_wheel_hinge" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin rpy="0       0       1.57079" xyz="0.03     0.05125  0.03"/>
    <axis xyz="1  0  0"/>
  </joint>

  <transmission name="tran_jetbot_left_wheel_hinge">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="left_wheel_hinge">
      <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
    </joint>
    <actuator name="motor_left_wheel">
      <mechanicalReduction>1</mechanicalReduction>
      <hardwareInterface>VelocityJointInterface</hardwareInterface>
    </actuator>
  </transmission>

  <joint name="right_wheel_hinge" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin rpy="0       0       1.57079" xyz="0.03   -0.0595  0.03"/>
    <axis xyz="1  0  0"/>
  </joint>

  <transmission name="tran_jetbot_right_wheel_hinge">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="right_wheel_hinge">
      <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
    </joint>
    <actuator name="motor_right_wheel">
      <mechanicalReduction>1</mechanicalReduction>
    </actuator>
  </transmission>

  <joint name="camera_joint" type="fixed">
    <parent link="base_link"/>
    <child link="camera_link"/>
    <origin rpy="0    0.25  0" xyz="0.065   0      0.0857"/>
    <axis xyz="0.96891  0       0.2474"/>
  </joint>

  <link name="base_link">
    <inertial>
      <mass value="1.0"/>
      <origin rpy="0  0  0" xyz="0  0  0"/>
      <inertia ixx="1e-3" ixy="0" ixz="0" iyy="1e-3" iyz="0" izz="1e-3"/>
    </inertial>
    <collision name="collision">
      <origin rpy="1.57079  0       1.57079" xyz="0     0     0.043"/>
      <geometry>
        <mesh filename="package://jetbot_description/meshes-jetbot-ros/JetBot-v3-Chassis.dae" scale="0.001 0.001 0.001"/>
      </geometry>
    </collision>
    <visual name="visual">
      <origin rpy="1.57079  0       1.57079" xyz="0     0     0.043"/>
      <geometry>
        <mesh filename="package://jetbot_description/meshes-jetbot-ros/JetBot-v3-Chassis.dae" scale="0.001 0.001 0.001"/>
      </geometry>
    </visual>
  </link>

  <link name="left_wheel">
    <inertial>
      <mass value="0.1"/>
      <origin rpy="0  0  0" xyz="0  0  0"/>
      <inertia ixx="1e-3" ixy="0" ixz="0" iyy="1e-3" iyz="0" izz="1e-3"/>
    </inertial>
    <collision name="collision">
      <origin rpy="0  ${M_PI_2} 0" xyz="0 0 0"/>
      <geometry>
        <cylinder radius="0.030" length="0.008"/>
      </geometry>
    </collision>
    <visual name="visual">
      <origin rpy="0  0  0" xyz="0  0  0"/>
      <geometry>
        <mesh filename="package://jetbot_description/meshes-jetbot-ros/JetBot-v3-Wheel.stl" scale="0.001 0.001 0.001"/>
      </geometry>
    </visual>
  </link>

  <link name="right_wheel">
    <inertial>
      <mass value="0.1"/>
      <origin rpy="0  0  0" xyz="0  0  0"/>
      <inertia ixx="1e-3" ixy="0" ixz="0" iyy="1e-3" iyz="0" izz="1e-3"/>
    </inertial>
    <collision name="collision">
      <origin rpy="0  ${M_PI_2} 0" xyz="0 0 0"/>
      <geometry>
        <cylinder radius="0.030" length="0.008"/>
      </geometry>
    </collision>
    <visual name="visual">
      <origin rpy="0  0  0" xyz="0  0  0"/>
      <geometry>
        <mesh filename="package://jetbot_description/meshes-jetbot-ros/JetBot-v3-Wheel.stl" scale="0.001 0.001 0.001"/>
      </geometry>
    </visual>
  </link>

  <link name="camera_link">
    <inertial>
      <mass value="0.01"/>
      <origin rpy="0  0  0" xyz="0  0  0"/>
      <inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0"/>
    </inertial>
    <visual name="visual">
      <origin rpy="0  0  0" xyz="0  0  0"/>
      <geometry>
        <box size="0.001 0.001 0.001"/>
      </geometry>
    </visual>
  </link>

  <joint name="lidar_joint" type="fixed">
    <parent link="base_link"/>
    <child link="lidar_link"/>
    <origin rpy="0   0   3.14" xyz="0.0 -0.0058  0.127"/>
    <axis xyz="0 0 0"/>
  </joint>

  <link name="lidar_link">
    <inertial>
      <mass value="0.0"/>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0"/>
    </inertial>
    <collision name="collision">
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://jetbot_description/meshes/lidar-base.dae" scale="1.0 1.0 1.0"/>
      </geometry>
    </collision>
    <visual name="visual">
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://jetbot_description/meshes/lidar-base.dae" scale="1.0 1.0 1.0"/>
      </geometry>
    </visual>
  </link>

</robot>

```

The structure is very similar for all wheeled robots:
1. the main body of the robot -- in this case called `chassis`.
2. the effectors and actuators -- in this case the `wheels`.
3. the sensors -- for camera and LiDAR.

There is a connected tree of links, connected by joints. ROS follows some conventions, which are specified in ROS Enhancement Proposals (REPs), in particular [REP 105](https://www.ros.org/reps/rep-0105.html) for Coordinate Frames for Mobile Platforms and [REP 120](https://www.ros.org/reps/rep-0120.html#base-footprint) for coordinate frames of humanoid robots.


Distances between links can be measured by the 3D model. In practice, a calibration is required so that sensors’ extrinsics – i.e., the relative transformation between them – are known. Note that the collision and the visual can be specified with a 3D model (e.g., a `.dae` file) generated with an external CAD software. Typically, the collision is represented by more simplified geometric elements, reducing the workload on the simulator. An example on the Jetbot is in the following figure, which shows in orange the collision box, while maintaining a realistic view.

<img src="/img/collision.png" alt="Robot collision." width="500" height="333">

-----
Now that we have the body of the robot, let's see in the [next unit]({{ site.baseurl }}{% link _modules/mod-4c-sensors.md %}) how to add sensors.
