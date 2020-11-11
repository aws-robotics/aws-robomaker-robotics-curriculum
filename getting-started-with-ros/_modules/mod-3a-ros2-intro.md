---
# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved. // SPDX-License-Identifier: CC-BY-SA-4.0
title: ROS1 and ROS2
permalink: /modules/3/ros2-intro
---

ROS1 has been out there for more than 10 years and has been used on many robotics projects.
Fundamental features missing in ROS1 have been identified, which make ROS1 not widely adopted for critical industrial applications. Examples of these features include real-time, safety, security, and ability to seamlessly manage multiple robots. As introducing such features in ROS1 would make it not backwards compatible and unstable, it was decided to develop ROS2 from scratch. The first release of ROS2 happened in 2017.

Currently, ROS2 core functionalities are stable, but many of the packages that allow developers to immediately start working with a robot have not been fully ported yet -- e.g., packages for Simultaneous Localization and Mapping are still incomplete.

ROS1 will be still available until 2025 and there is a specific bridge allowing ROS1 and ROS2 nodes to communicate with each other.

Let's see in the [next unit]({{ site.baseurl }}{% link _modules/mod-3b-ros1-ros2.md %}) how such changes will affect development, if at all.
