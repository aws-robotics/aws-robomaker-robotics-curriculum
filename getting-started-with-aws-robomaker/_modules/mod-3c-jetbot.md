---
# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved. // SPDX-License-Identifier: CC-BY-SA-4.0
title: Waveshare Jetbot on AWS RoboMaker
permalink: /modules/3/jetbot.html
---
With the basics on AWS RoboMaker, we can set up a ROS package containing the model and the tools for the Gazebo simulation with the Waveshare Jetbot, the robot used as reference in this curriculum.

## Open Cloud9
To get back to Cloud9, click on

    Development environment

on the left side menu, and then click on

    Open environment
as shown in the following screenshot

![RoboMaker development](/img/robomaker-development.png)

## Download the ROS package through git
Once Cloud9 is loaded, use the terminal to download the ROS package containing the model and the tools for the Gazebo simulation with the Waveshare Jetbot. Finally set up the simulation as seen before.

Here the set of commands:

    cd ~/environment/HelloWorld/simulation_ws/src/
    git clone https://github.com/cbuscaron/robomaker-jetbot  (Links to an external site.)


## Edit simulation configuration
Once downloaded, we need to edit the configuration of the simulation. Click from the top menu:

    Run-> Add or Edit Configurations

Then click on

    Simulation-> HelloWorld

and scroll down all the way down to

    Simulation Application
and modify the Launch package name to robomaker-jetbot and Launch file to world.launch. In this way, that will point to the new ROS package, and save. The results should be as shown in the following screenshot.

![Simulation configuration](/img/jetbot-simulation-configuration.png)

## Build, Bundle, and Run

Now that we have everything set up, let's build and bundle the simulation workspace.

Click from the top menu bar,

    Run->Build->HelloWorld Simulation
Once done, click again from the menu bar

    Run->Bundle->HelloWorld Simulation
Now that the new simulation is bundled, it can be run in the same way as before. Once the simulation is ready, connecting to it and opening GZClient will show the Waveshare Jetbot instead of the Turtlebot3.

![Jetbot in Gazebo](/img/gazebo-jetbot.png)

Now we have the full environment set up to start learning ROS in the next course. [Let’s assess first our understanding of AWS RoboMaker]({{ site.baseurl }}{% link _modules/mod-3d-assessment.md %}).

{% capture ref %}
{% bibliography --cited %}
{% endcapture %}
{% include references.html ref=ref %}
