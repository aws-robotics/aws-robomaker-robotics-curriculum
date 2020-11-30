---
title: Simulator with the Waveshare Jetbot of the course
permalink: /modules/5/waveshare.html
---

With the basics on Linux, ROS package installation, and git, we can set up a ROS package containing the model and the tools for the Gazebo simulation with the Waveshare Jetbot, the robot used as reference in this curriculum.

Here a set of commands that can be run to download the ROS package, which can be used in the next courses:

    cd ~/ros_workspace/src/ # going to the `src` directory, containing custom ROS packages.
    git clone https://github.com/cbuscaron/robomaker-jetbot # clone the repository.
    cd ..
    colcon build
    source ~/ros_workspace/install/setup.sh
    roslaunch robomaker-jetbot world.launch

When successful, this is screen that should appear:

![Jetbot in Gazebo](/img/jetbot.png)

This will be the base of what we will learn in the Course 2.



