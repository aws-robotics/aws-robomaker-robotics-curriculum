---
# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved. // SPDX-License-Identifier: CC-BY-SA-4.0
title: Motivation
permalink: /modules/0/motivation.html
menus: header
---

Testing robotics software in the real world requires access to (1) robots that sometimes are very expensive (self-driving cars, underwater robots, etc.) and (2) complex testing facilities and permissions, which sometimes do not allow for covering all possible scenarios that the robot could be in. While testing in the real world is fundamental, simulation plays a crucial role in the advancement of robotics software, as it allows for testing without an actual robot. 

In the following, a short video of a self-driving car simulator used in research, [CARLA](https://carla.org/).
{% include video-youtube.html url="https://www.youtube.com/embed/2c-KlQ8SFcc" %}

There are a number of robotics simulators available and that are integrated with the Robot Operating System (ROS), starting from simpler 2D simulators up to complete 3D simulators with realistic physics engines. To understand which one to use one has to think about what the purpose of the experiment is. If a robot is always in a planar environment and is not equipped with a sensor that captures the 3D information, then a 2D simulator, such as Stage, is enough. Otherwise, a 3D simulator is necessary. Simulators are distinguished by different levels of realism and available simulated robots and sensors.

In this course, we focus on Gazebo, a realistic 3D simulator that is integrated within ROS and out-of-the-box provides a number of robot and sensor models. This is one of the advantages of Gazebo over other simulators which are specialized for specific robots. The following video shows humanoids as well as wheeled robots, however, Gazebo also supports a number of aerial drones, robotic boats, and underwater robots.

{% include video-youtube.html url="https://www.youtube.com/embed/k2wVj0BbtVk" %}

As mentioned in Course 2, the code related to the robot's logic written in ROS is generic and can run in any environment; it does not matter whether the sensor data and actuation commands are within a simulation or an actual device. The ROS messages (laser scan, motor twist, etc.) would be the same. 

Let's discover how the simulator can support the robotics software development and introduce concepts related to the simulation connecting it with the robot software in [this brief introduction to Gazebo]({{ site.baseurl }}{% link _modules/mod-0b-objectives.md %}).

{% capture ref %}
{% bibliography --cited %}
{% endcapture %}
{% include references.html ref=ref %}
