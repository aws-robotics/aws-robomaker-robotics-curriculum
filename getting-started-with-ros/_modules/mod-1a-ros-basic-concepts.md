---
# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved. // SPDX-License-Identifier: CC-BY-SA-4.0
title: ROS basic concepts
permalink: /modules/1/ros-basic-concepts
---

In this module, we will discover the ROS basic concepts, which will allow you to progress and write software in ROS.


## ROS core elements
As the best practice in ROS is to design software as a collection of small programs, a communication mechanism between software is necessary and is provided by ROS. The ROS main communication approach is based on **publish/subscribe** message passing. Publish/subscribe is defined in software engineering as a pattern where publishers send messages without knowing who are the recipients; and subscribers receive messages from publishers based on their subscription to information of interest.

To enable this communication, there are four main ROS core elements:
1. **ROS master**, which is a process to facilitate the communication between robotics programs.
2. **ROS nodes**, a running instance of a ROS program implementing some robotics components.
3. **ROS topics**, named buses over which nodes exchange typed messages.
4. **ROS messages**, containing the information.

In the following a short video graphically showing what happens when a node is publishing velocity commands, and another node is subscribing to the related ROS topic to receive those velocity commands.

{% include video-file.html url="/img/ros-master" %}

Only one ROS master should run in a system. Note that the order of the nodes registration to the ROS master is not guaranteed in general.

ROS defines a number of standardized typed messages which can be found [here](http://wiki.ros.org/common_msgs).

With an understanding of the message passing mechanism, let's see how we can move the robot in the [next unit]({{ site.baseurl }}{% link _modules/mod-1b-master-nodes.md %}).


{% capture ref %}
{% bibliography --cited %}
{% endcapture %}
{% include references.html ref=ref %}
