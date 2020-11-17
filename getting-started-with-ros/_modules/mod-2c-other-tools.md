---
# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved. // SPDX-License-Identifier: CC-BY-SA-4.0
title: ROS useful tools
permalink: /modules/2/other-tools.html
---

Tools that are useful to know and that are part of the ROS suite are listed below.


## tf2

The `tf2` library provides support for transformation system, i.e., how reference frames relate to each other. This is important to know so that the robot can be located in a fixed reference frame, and its sensor readings can be correctly placed.
The following figure shows a mobile robot that is in an environment.

{% include image.html url="/img/tf.svg" description="Reference frames in a map and their relation." %}

For a mobile robot, typically there are two fixed reference frames: (1) `map` which is arbitrarily attached to some location of the environment, and (2) `odom` which is a reference frame initialized wherever the robot is placed in the environment and is turned on. `base_link` is the one attached to the rotational center of the robot. All sensors are then having a fixed transformation with respect to `base_link`. A tree of transformations is shown in the following figure.

{% include image.html url="/img/tf-tree.PNG" description="Reference frames of a robot." %}

Typical conventions are specified [here](http://wiki.ros.org/navigation/Tutorials/RobotSetup/TF). The Python APIs and related tutorials can be found at the `tf` [page](http://wiki.ros.org/tf/Tutorials).

## Visualization of messages

`rviz` allows users to visualize some of the messages on a GUI. The following figure shows an example of visualization of the simulated camera images and the LiDAR points.

{% include image.html url="/img/rviz.png" description="rviz visualization." %}

There are some panels useful to navigate, shown in the following figure.

{% include image.html url="/img/rviz-panels.png" description="rviz panels." %}

The main panels that are useful are:
- the one on the left with "Displays" for selecting which topic to display.
- the one on the right to change view.
- the one on the bottom left, where there is the "Add" button to display new topics.

There are two ways to add a topic to display: by "display type", which will add it regardless of the fact that there is a publisher, and by "topic" which shows the available topic that rviz can plot -- see the following figure.

{% include image.html url="/img/rviz-add.png" description="rviz add window." %}

Note that a common error is to display some data that should be displayed in a specific reference frame, which however doesn't exist.
The following figure shows an error in displaying the laser sensor data, as the fixed frame `map` does not exist. The fixed frame can be changed through rviz, either by selection or by typing.

{% include image.html url="/img/rviz-error.png" description="rviz error frame." %}

More information can be found [here](http://wiki.ros.org/rviz).


## Recording of the messages/logging
`rosbag` allows users to record messages on topics so that they can be played later on. This is quite important for creating robotic datasets and test algorithms later on, for example SLAM systems. [Here](https://wiki.ros.org/rosbag/Commandline) the commands that are useful to run to record or play messages over topics.
`rqt_bag` is a GUI tool for displaying data in bag files -- see the following figure, which shows the recording of ROS messages on an experiment with a mobile robot.

{% include image.html url="/img/rqt-bag.png" description="rqt_bag." %}

In addition, ROS provides a way to record log messages, which are human-readable string messages to report the status of a node. There are different level of severity, from debug to fatal. Log files from the latest execution of `roscore` and ROS nodes are saved by default in `~/.ros/log/latest/`. A tutorial is available [here](http://wiki.ros.org/rospy_tutorials/Tutorials/Logging) to show how to use the API. `rqt_console` is a GUI tool for changing the level of a specific node, so that messages are outputted to the screen.

{% include image.html url="/img/rqt-console.png" description="rqt_console." %}

------
These are some of the main tools that are useful for debugging. There are many others that are available and the ROS website is the resource to go. Before moving on to the next module on understanding the differences between ROS1 and ROS2, let's assess the understanding of what we covered in this module.


{% capture ref %}
{% bibliography --cited %}
{% endcapture %}
{% include references.html ref=ref %}
