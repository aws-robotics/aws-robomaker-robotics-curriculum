---
# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved. // SPDX-License-Identifier: CC-BY-SA-4.0
title: AWS RoboMaker Demo - with Virtual Desktop
permalink: /modules/3/robomaker-demo-virtual-desktop.html
---

With the sample application downloaded, we can run the Virtual Desktop which allows for testing of robotics software through a web browser as if the GUI of the computer is available.

Note that this is a new feature that was recently introduced; if you created the development environment before April 30, 2021, you should create a new development environment to have the feature available to you. More information is available at [the related blog post](https://aws.amazon.com/blogs/robotics/aws-announces-a-new-developer-desktop-feature-within-the-aws-robomaker-ide/).

## Exploring the Virtual Desktop

To run the Virtual Desktop, simply click on `Virtual Desktop -> Launch Virtual Desktop` as shown in the following figure

![Virtual Desktop click](/img/virtual-desktop-click.png)
    
This will open a new browser tab -- if it doesn't, the browser might have blocked the opening of the tab, thus follow the instructions of your browser to allow it. The new browser tab will show a full Ubuntu desktop, which was presented in Course 1a.

![Virtual Desktop first screen](/img/virtual-desktop-first-screen.png)

The Virtual Desktop will have access exactly to the same file system as the IDE. Meaning if a file is created from the Virtual Desktop, it is also visible in the IDE.

## Building and running of the robot software

In AWS RoboMaker, the typical structure of the workspaces is one workspace that contains the ROS packages for running the simulation, another one that contains the ROS packages for executing the robot application. The latter can run also on a real robot.

In our Hello World example, it contains these two workspaces, which need to be built, first, by installing the necessary dependencies, second, by compiling the ROS packages.

The usual workflow is to use the IDE for development and executing commands and use the Virtual Desktop for visualization and real-time interaction.

Before we start on building each workspace, let's install `vcstool`, which provides command-line tools for managing repositories in the local workspace. Please see the [vcstool wiki](http://wiki.ros.org/vcstool) for more details. From the terminal tab, type

    sudo apt install python3-vcstool -y
    
Then, for building the simulation workspace (comments above each command line explain the purpose of each command):

    # Change directory.
    cd ~/environment/aws-robomaker-sample-application-helloworld/simulation_ws
    # get the correct git repository
    vcs import < .rosinstall
    # install dependencies of such packages
    rosdep install --from-paths src --ignore-src -r -y
    # build all packages in the workspace
    colcon build

At this point, the simulation can be run:

    # setting the environment variables for letting ROS find the built ROS packages
    source install/setup.bash
    # to connect the terminal with a display where the Gazebo GUI can be shown. Virtual Desktop MUST be launched prior to the following command
    export DISPLAY=:0
    # Run Gazebo with the GUI
    roslaunch hello_world_simulation empty_world.launch gui:=true

Switching to the Virtual Desktop tab, you should be able to see  the Gazebo GUI with a Turtlebot 3 robot.
Note that the command `export DISPLAY=:0` wouldn't be typically necessary on a local machine with Ubuntu. This command is instead necessary on the cloud, as it is not specified which display will be used. At this time, an error might happen preventing Gazebo to run. If that happens, try

    export DISPLAY=:1
    
and run again the `roslaunch` command.

Similarly, in another terminal, for building the robot workspace: 

    cd ~/environment/aws-robomaker-sample-application-helloworld/robot_ws
    vcs import < .rosinstall
    rosdep install --from-paths src --ignore-src -r -y
    colcon build
    
Then, to run the robot application which will make the robot rotate:

    # from the robot_ws folder
    source install/setup.bash
    roslaunch hello_world_robot rotate.launch

You will see in the Virtual Desktop tab the robot rotating. All commands that can be run on a local machine can be run in the IDE terminal or in the terminal in the Virtual Desktop. Please feel free to explore the environment.

![Virtual Desktop Gazebo](/img/virtual-desktop-gazebo.png)

## Stopping the simulation
To stop the simulation, simply press `Ctrl+c` on the terminals where you ran the simulation and the application.

----
Here is a full video showing these steps, together with the creation of the development environment, in case a new one needs to be created.

{% include video-file.html url="/img/robomaker-iam" %}

----
There is another way to run simulation in RoboMaker, which can be useful in some scenarios. This is explained in the [next unit]({{ site.baseurl }}{% link _modules/mod-3b-2-demo.md %}).

{% capture ref %}
{% bibliography --cited %}
{% endcapture %}
{% include references.html ref=ref %}
