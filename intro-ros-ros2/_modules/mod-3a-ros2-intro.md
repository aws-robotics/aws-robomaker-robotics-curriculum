---
# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved. // SPDX-License-Identifier: CC-BY-SA-4.0
title: ROS2 intro
permalink: /modules/3/ros2-intro.html
---

ROS has been around for over 10 years and has been used on many robotics projects.
Over time, fundamental features missing in ROS have been identified, which make ROS not widely adopted for critical industrial and commercial applications. Examples of these features include real-time, safety, security, and ability to seamlessly manage multiple robots. Introducing such features in ROS would make it not backwards compatible and unstable, therefore, it was decided to develop ROS 2 from scratch. The first release of ROS 2 was in 2017.

Currently, ROS 2 core functionalities are stable, but many of the packages that allow developers to immediately start working with a robot have not been fully ported yet -- e.g., packages for Simultaneous Localization and Mapping are still incomplete.

ROS 1 will be still available until 2025 and there is a specific bridge allowing ROS and ROS 2 nodes to communicate with each other. 

Let's see in the [next unit]({{ site.baseurl }}{% link _modules/mod-3b-ros1-ros2.md %}) how such changes will affect development.
