---
# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved. // SPDX-License-Identifier: CC-BY-SA-4.0
title: ROS and ROS 2 similarities and differences
permalink: /modules/3/ros1-ros2.html
---

The core concepts of ROS and ROS 2 are similar -- e.g., publish/subscribe paradigm -- thus the concepts learned in ROS will be useful for learning ROS 2. There are some technical differences though that are useful to highlight and which may affect your workflow.
Here we highlight some of the similarities and differences that will affect the software development. More details can be found at the [current list](http://design.ros2.org/articles/changes.html) identified by the ROS 2 community. Full details about ROS 2 will be provided in a dedicated course for ROS 2.

## Distributed 
ROS 2 does not have the ROS master, making fully distributed systems possible. This means that, differently from what we learned here, `roscore` does not exist anymore in ROS 2, making it easier to discover nodes within a network and more resilient if a network fails.

## Reuse of existing middleware
ROS used a custom-made message passing framework, while ROS 2 relies on existing data distribution service (DDS) and messages are converted into DDS messages through an interface. The default ROS 2 middleware is eProsima Fast RTPS. The ROS 2 API can and has been provided by other middleware implementations, providing greater flexibility for specialized use cases while handling most user’s needs out of the box. For more information, see the [currently supported middleware in ROS 2](https://index.ros.org/doc/ros2/Concepts/DDS-and-ROS-middleware-implementations/).

This part is transparent to the developer and is handled within the ROS client library provided making what we learned on publishers and subscribers still applicable. In spite of potentially more limited flexibility, this choice allows for more clearly defined **quality of service (QoS)** on the way messages are exchanged between nodes.


## Build system
The build system for ROS has been `catkin`, while for ROS 2 is `ament` -- a set of CMake utilities for building C/C++ packages -- with `colcon`, the build invocation tool.
There are some changes in the files needed to build a package (e.g., within `package.xml` or `CMakeLists.txt`). We have used `colcon` also for ROS packages to have a seamless transition to ROS 2. Note that `colcon` can also handle pure Python packages, without requiring the presence of `CMakeLists.txt`. See more details at the [migration guide from ROS to ROS 2](https://index.ros.org/doc/ros2/Contributing/Migration-Guide/). 

## Supported Operating System
All versions of ROS are tested for Ubuntu. Other distributions of Linux, and Mac OS X are tested by the community. Microsoft Windows support was minimal, and only recently Microsoft released support for ROS -- more details at the [page about ROS on Windows](https://ms-iot.github.io/ROSOnWindows/).

ROS 2 is tested on Ubuntu, Mac OS X, and Microsoft Windows 10, making it more broadly usable. The last up-to-date information can be found at the [list of currently supported operating systems for ROS 2](https://ros.org/reps/rep-2000.html).

## Programming languages
Both ROS and ROS 2 support C++ and Python. The main difference will be on the version supported: ROS targets C++03 and Python 2, while ROS 2 uses C++11 and Python 3.5.
In addition, a major difference between ROS and ROS 2 is that ROS implemented the C++ and Python libraries completely duplicated. This made it very hard to add new language support. ROS basically had to be fully reimplemented from scratch. In ROS 2, there is a core C API rcl (ROS Client Library) that provides most of the functionality for the higher level language bindings rclcpp (ROS Client Library C++) and rclpy (ROS Client Library Python) - and because of the ease of binding C to basically every language, there are already community-contributed language clients for other popular languages like Java (rcljava), Rust (rclrs), and NodeJS (rclnodejs) to name a few.

As for coding practices, ROS does not provide any specific guideline on how to write code, e.g., it could be following Object-Oriented Programming or not. 
ROS 2 instead follows the OOP paradigm: a new class inherits from the Node object. This means that the code needs changes, however, the key concepts, despite some fundamental changes, are still present: i.e., publisher/subscriber, messages, ROS services, actions. The knowledge gained in ROS will enable a smooth transition to ROS 2.
A few changes worth to mention are:
- In ROS 2, besides name and type for passing messages as seen in ROS, compatible QoS profiles can be defined. It gives the developer even more control over publisher/subscriber communication. 
- ROS 2 allows many nodes to run in the **same** process just as easily as running one, so that such nodes can exchange messages with zero copy passing. This removes the need for the equivalent ROS concept of "nodelets". The ROS 2 "Components" API allows for loading and unloading nodes from a process even at runtime.


## Other tools

The tools that we have seen, including Gazebo, rviz, tf have been ported to ROS 2; however it is not complete yet. For example, Gazebo keeps track on its [GitHub repository](https://github.com/ros-simulation/gazebo_ros_pkgs/issues/512) of the functionalities currently ported or still requiring work. 

## Bridge of ROS and ROS 2

As we transition from ROS to ROS 2 there is a way to bridge ROS and ROS 2 (see [the ROS/ROS 2 bridge](https://index.ros.org/p/ros1_bridge/github-ros2-ros1_bridge/)). For this to work the message or service must be available in both ROS and ROS 2.

-------
[Let's assess]({{ site.baseurl }}{% link _modules/mod-3c-assessment.md %}) our understanding of the main characteristics of ROS and ROS 2.
