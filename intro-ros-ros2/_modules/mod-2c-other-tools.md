---
# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved. // SPDX-License-Identifier: CC-BY-SA-4.0
title: ROS tools
permalink: /modules/2/other-tools.html
---

Some useful ROS tools to know are listed below.


## tf2

The `tf2` library provides support for the transformation system, i.e., how reference frames relate to each other. This is important so the robot can be located in a fixed reference frame and its sensor readings can be correctly placed.
The following figure shows a mobile robot that is in an environment. 

{% include image.html url="/img/tf.svg" description="Reference frames in a map and their relation." %} 

For a mobile robot, typically there is a fixed reference frame `map` which is arbitrarily attached to some location in the environment. `odom` is a reference frame that is used for dead-reckoning, e.g., based on wheel odometry, where initially the robot identified with `base_link`, attached to the rotational center of the robot, is at the origin of such a reference frame. Any state estimation package using odometry and other sensors, e.g., LiDAR, will find the correction between `map` and `odom`, so that the robot can be localized in `map`. 
All sensors and effectors on the robot are then described in relation to `base_link`. 
A tree of transformations for a mobile robot with its sensors and effectors  is shown in the following figure.

{% include image.html url="/img/tf-tree.PNG" description="Reference frames of a robot." %} 

Typical conventions are specified [here](http://wiki.ros.org/navigation/Tutorials/RobotSetup/TF). The Python APIs and related tutorials can be found at the`[tf page](http://wiki.ros.org/tf/Tutorials).

## Visualization of messages

`rviz` allows users to visualize some of the messages on a GUI. The following figure shows an example of visualization of simulated camera images and the LiDAR points.

{% include image.html url="/img/rviz.png" description="rviz visualization." %}

There are some rviz panels useful to explore as shown in the following figure.

{% include image.html url="/img/rviz-panels.png" description="rviz panels." %}

The main panels that are useful are:
- the one on the left with "Displays" for selecting which topic to display.
- the one on the right to change view.
- the one on the bottom left, where there is the "Add" button to display new topics.

There are two ways to add a topic to display; (1) by "display type", which will add it regardless of the fact that there is a publisher, and (2) by "topic" which shows the available topic that rviz can plot -- see the following figure.

{% include image.html url="/img/rviz-add.png" description="rviz add window." %}

A **common error** is to display data that should be displayed in a specific reference frame which doesn't exist. 
The following figure shows an error in displaying the laser sensor data as the fixed frame `map` which does not exist. In this specific case, replacing `map` with `base_scan` would address the problem. This shows the measurements from the laser sensor in the reference frame of the sensor.

{% include image.html url="/img/rviz-error.png" description="rviz error frame." %}

More information can be found at the [rviz page](http://wiki.ros.org/rviz). 

## Recording of the messages/logging
`rosbag` allows users to record messages on topics so that they can be played later on. This is quite important for creating robotic datasets and test algorithms, for example SLAM systems. [Here](https://wiki.ros.org/rosbag/Commandline) are the commands that are useful to run to record or play messages over topics.
`rqt_bag` is a GUI tool for displaying data in bag files. The following figure shows the recording of ROS messages on an experiment with a mobile robot.

{% include image.html url="/img/rqt-bag.png" description="rqt_bag." %}

ROS provides a way to record log messages, which are human-readable string messages to report the status of a node. There are different level of severity, from debug to fatal. Log files from the latest execution of `roscore` and ROS nodes are saved by default in `~/.ros/log/latest/`. A tutorial is available [here](http://wiki.ros.org/rospy_tutorials/Tutorials/Logging) to show how to use the API. `rqt_console` is a GUI tool for changing the level of a specific node, so that messages are outputted to the screen.

{% include image.html url="/img/rqt-console.png" description="rqt_console." %}

------
These are some of the main tools that are useful for debugging. There are many others that are available and the ROS website is a great resource. Before moving on to the next module on understanding the differences between ROS1 and ROS2, [let's assess]({{ site.baseurl }}{% link _modules/mod-2d-assessment.md %}) the understanding of what we covered in this module.


{% capture ref %}
{% bibliography --cited %}
{% endcapture %}
{% include references.html ref=ref %}
