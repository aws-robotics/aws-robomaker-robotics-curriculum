---
# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved. // SPDX-License-Identifier: CC-BY-SA-4.0
title: Waveshare Jetbot on AWS RoboMaker
permalink: /modules/3/jetbot.html
---
With learning the basics on AWS RoboMaker, we can now set up a ROS package containing the model and the tools for the Gazebo simulation for the Waveshare Jetbot. The Waveshare Jetbot is the robot used as reference in throughout this curriculum.

## Open the IDE
To get back to your previously created IDE click on 

    Development environment

on the left side menu, and then click on

    Open environment

as shown in the following screenshot

![RoboMaker development](/img/robomaker-development.png)

## Download the ROS package through git
Once the IDE is opened, use the terminal to download the ROS package containing the model and the tools for the Gazebo simulation with the Waveshare Jetbot. Finally set up the simulation as seen before.

Here the set of commands:

	cd ~/environment/aws-robomaker-sample-application-helloworld/simulation_ws/src/
	git clone https://github.com/aws-samples/aws-robomaker-jetbot-ros
	rosdep install --from-paths src --ignore-src -r -y # install necessary dependencies.

## Edit simulation configuration
Once downloaded, we can launch the simulation within the IDE using the Virtual Desktop as discussed previously. If not already launched, click from the top menu click on Virtual Desktop -> Launch Virtual Desktop.

As a reminder -- this will open a new browser tab -– if it doesn't, the browser might have blocked the opening of the tab, thus follow the instructions of your browser to allow it.

	# Change directory.
	cd ~/environment/aws-robomaker-sample-application-helloworld/simulation_ws
	# get the correct git repository
	vcs import < .rosinstall
	# install dependencies of such packages
	rosdep install --from-paths src --ignore-src -r -y
	# build all packages in the workspace
	colcon build
	# to connect the terminal with a display where the Gazebo GUI can be shown. **Virtual Desktop MUST be launched** prior to the following command
	export DISPLAY=:0

Now that the new simulation is bundled, it can be run in the same way as before. Once the simulation is ready, connecting to it and opening GZClient will show the Waveshare Jetbot instead of the Turtlebot3.

	# Run Gazebo with the GUI
	roslaunch jetbot_description gazebo.launch gui:=true

The results should be as shown in the following screenshot.

![Jetbot in Gazebo](/img/gazebo-jetbot.png)

----
Now we have the full environment set up to start learning ROS in the next course. [Let’s assess first our understanding of AWS RoboMaker]({{ site.baseurl }}{% link _modules/mod-3d-assessment.md %}).

{% capture ref %}
{% bibliography --cited %}
{% endcapture %}
{% include references.html ref=ref %}
