---
# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved. // SPDX-License-Identifier: CC-BY-SA-4.0
title: Wheel odometry
permalink: /modules/1/odometry.html
---

The odometry gives information about the change in position over time for estimating the motion of a robot. On a wheeled robot, rotary encoders on the motors connected to the wheels provide information about the number of rotations, which, given the size of the wheels and their distance, can be used to estimate the traveled distance of the robot. 

But is that information accurate? In general, it is not. Imagine a case when the robot is stuck in the snow and the wheels continue to spin. In this case, the robot does not move but the estimation would say that the robot actually moved.

Gazebo provides a simulated odometry for a differential drive robot through a Gazebo ROS plugin `ros_diff_drive`. This is, for example, the parameters for the differential drive plugin for the simulated Waveshare Jetbot in `aws-robomaker-jetbot-ros/src/jetbot_description/urdf/robomaker-jetbot.gazebo.xacro`, which includes all plugins and simulated sensors for the robot.

```
  <gazebo>
    <plugin name="jetbot_controller" filename="libgazebo_ros_diff_drive.so">
      <!-- Physical characteristics -->
      <wheelSeparation>0.096</wheelSeparation>
      <wheelDiameter>0.065</wheelDiameter>
      <wheelAcceleration>1</wheelAcceleration>
      <wheelTorque>10</wheelTorque>
      
      <odometrySource>encoder</odometrySource>
      
      <!-- Model of the robot -->
      <robotBaseFrame>base_footprint</robotBaseFrame>
      <leftJoint>left_wheel_hinge</leftJoint>
      <rightJoint>right_wheel_hinge</rightJoint>
      
      <!-- ROS related parameters -->
      <commandTopic>cmd_vel</commandTopic>
      <odometryTopic>odom</odometryTopic>
      <odometryFrame>odom</odometryFrame>
      <publishOdomTF>true</publishOdomTF>
      <publishWheelTF>false</publishWheelTF>
      <publishTf>true</publishTf>
      <publishWheelJointState>true</publishWheelJointState>
      <legacyMode>false</legacyMode>
      <updateRate>30</updateRate>
      <rosDebugLevel>na</rosDebugLevel>
    </plugin>
  </gazebo>

```
There are a number of parameters that we will learn when talking about how to model a robot. For our focus on realism of the sensor,  `odometrySource` is the parameter which  which can be set as `world` or `encoder`. The first one provides updates according to the position in the world of the robot. The second one simulates the motor encoder: if the wheels are spinning, without the robot moving, the odometry will change. In the real world, `encoder` is a source for determining the odometry of the robot. 

In the following two videos, the effect of using `world` vs. `encoder`.

{% include video-file.html url="/img/odometry-world" %}

{% include video-file.html url="/img/odometry-encoder" %}

Using `encoder` is more realistic, at the same time requiring the algorithm to account for such a drift. Note that this simulated odometry does not provide a way to control the noise characteristics. 

Please look at the [code of the differential drive plugin](http://docs.ros.org/en/melodic/api/gazebo_plugins/html/gazebo__ros__diff__drive_8cpp_source.html) for more information.

---
With the basics on the odometry covered, let's learn how the LiDAR sensor is simulated in the [next unit]({{ site.baseurl }}{% link _modules/mod-1c-lidar.md %}).

{% capture ref %}
{% bibliography --cited %}
{% endcapture %}
{% include references.html ref=ref %}
