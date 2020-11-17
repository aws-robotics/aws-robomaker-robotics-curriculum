---
# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved. // SPDX-License-Identifier: CC-BY-SA-4.0
title: ROS1 and ROS2 similarities and differences
permalink: /modules/3/ros1-ros2.html
---

The core concepts of ROS1 and ROS2 are similar -- e.g., publish/subscribe paradigm -- thus the concepts learned in ROS1 will be useful for learning ROS2. There are some technical differences though that are useful to highlight, which might affect the workflow.
Here a highlight of some of the similarities and differences that will affect the software development. [Here](http://design.ros2.org/articles/changes.html) the ones currently listed by the ROS2 community. Full details about ROS2 will be provided in a dedicated course for ROS2.

## Distributed
ROS2 does not have the ROS master anymore, making fully distributed systems possible. This means that, differently from what we learned here, `roscore` does not exist anymore in ROS2, making it easier to discover new nodes across networks and more resilient if a network fails.

## Reuse of existing middleware
ROS1 used a custom-made message passing framework, while ROS2 relies on existing data distribution service (DDS) and messages are converted into DDS messages through an interface. This part is transparent to the developer and is handled within the ROS client library provided, making what we learned on publishers and subscribers still applicable. In spite of potentially more limited flexibility, this choice allows for more clearly defined quality of service on the way messages are exchanged between nodes.


## Build system
The build system for ROS1 has been `catkin`, while for ROS2 is `ament` with `colcon`. There are some changes in the files needed to build a package (e.g., within `package.xml` or `CMakeLists.txt`). We have used `colcon` also for ROS1 packages to have a seamless transition to ROS2. [Here](https://index.ros.org/doc/ros2/Contributing/Migration-Guide/) a migration guide from ROS1 to ROS2.

## Supported Operating System
All versions of ROS1 are tested for Ubuntu. Other distributions of Linux, and Mac OS X are tested by the community. Microsoft Windows support was minimal, and only recently Microsoft release a support for ROS1 -- more details [here](https://ms-iot.github.io/ROSOnWindows/).

ROS2 instead is tested on Ubuntu, Mac OS X, and Microsoft Windows 10, making it more broadly usable.

## Programming languages
Both ROS1 and ROS2 support C++ and Python. The main difference will be on the version supported: ROS1 targets C++03 and Python 2, while ROS2 uses C++11 and Python 3.5.

ROS1 does not provide any specific guideline on how to write code, e.g., it could be following Object-Oriented Programming or not.

ROS2 instead follows the OOP paradigm: a new class inherits from the Node object. This means that the code needs changes, however, the key concepts, depite some fundamental changes, are still present: i.e., publisher/subscriber, messages, ROS services, actions. The knowledge gained in ROS1 will enable a smooth transition to ROS2.

## Other tools

The tools that we have seen, including Gazebo, rviz, tf have been received preliminary support and been ported to ROS2; however it is not complete yet. For example, Gazebo keeps track on its [GitHub repository](https://github.com/ros-simulation/gazebo_ros_pkgs/issues/512) of the functionalities currently ported or still requiring work.

## Bridge of ROS1 and ROS2

As we transition from ROS1 to ROS2, there is a way to bridge ROS1 and ROS2 (see [here](https://index.ros.org/p/ros1_bridge/github-ros2-ros1_bridge/)). For this to work, the message or service must be available in both ROS1 and ROS2.

-------
Let's assess our understanding of the main characteristics of ROS1 and ROS2.
