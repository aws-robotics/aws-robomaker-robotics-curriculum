---
# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved. // SPDX-License-Identifier: CC-BY-SA-4.0
title: AWS RoboMaker Demo
permalink: /modules/3/robomaker-demo.html
---

With the AWS robot development configured, let's ensure that we understand the main tools and see a sample example which will show the simulation running. In this unit, we will learn what the structure of the code should be and how to run the simulation. We will learn in the next course, as we deepen into ROS, how the code can be modified.

## Simple application "ready-to-go"

We will start from a sample application available in RoboMaker, allowing us to inspect the different elements. To do that, just click on

    "Resources" -> "Download Samples" -> "Hello World".

as shown in the following screenshot:

![Download samples](/img/download-samples.png)

This will initiate the download of the necessary dependencies.

## Directory structure for workspaces

It will take a few minutes for the download. Once downloaded, we will see that there is a new folder `HelloWorld`. This folder contains the ROS workspaces. In particular, the one we are interested in is `simulation_ws` -- the other one is for deployment on the real robot, which we will see in one of the final courses. As shown in the following figure, the structure is identical to the structure of a `colcon` workspace in ROS (and very similar to the `catkin` workspace). In particular, the ROS package that contains code, which will make the robot rotate in place, is called `hello_world_simulation`.

![Structure](/img/structure.png)

## Terminal available

The terminal made available from Cloud9 allows to run all commands that can be run in Ubuntu, including `git` in case a package from a repository needs to be downloaded. Here some example of commands to navigate within the directory structure.

	ubuntu:~ $ ls
	environment  gocode  node_modules  package-lock.json
	ubuntu:~ $ cd environment/HelloWorld/simulation_ws/
	ubuntu:~/environment/HelloWorld/simulation_ws $ ls
	build  bundle  install  log  src
	ubuntu:~/environment/HelloWorld/simulation_ws $ cd src/

## Build and bundle

Once downloaded, start the build and bundle which takes some time. In particular, click on

    Run->Workflow->"Build and Bundle All"->Hello World - Build and Bundle All


which will compile the code. Once the Build is done, click on Run->Bundle->HelloWorld Simulation, which will create a portable environment. Such a portable environment can be moved to a different Linux system and executed as if the contents of the bundle were installed locally.

## Configuring workspaces and simulation options

In the meanwhile, there are a few settings that are important to know so that the robotic application can be built and the simulation run. Clicking on

    "Run"->"Add or Edit Configurations..."

the following pop up window will open

![Configuration](/img/configuration.png)

- In `COLCON BUILD` and `COLCON BUNDLE` the workspaces can be added. In the example, there are two: `HelloWorld Robot` and `HelloWorld Simulation`, referring to `robot_ws` and `simulation_ws`, respectively. Custom workspaces can be added as well.

- `WORKFLOW` specifies all actions that can run with a single click, e.g., build, bundle, and run simulation.

- The one that requires some attention is setting up the simulation job in `SIMULATION`.

The simulation requires connecting to resources created before. In particular, the resources that need to be set are: IAM role, the  S3 destination, security groups and subnets. In the simulation there is also the file that can be used to run the whole ROS simulation. The following video identifies the main steps to execute.

{% include video-file.html url="/img/robomaker-sim-conf" %}

Note that in the video, it is mentioned to choose "zones 1b and 1c" as subnets, referring to the specific zones that are available for an AWS Educate account. Some other zones might be available depending on the subscription.

## Running the simulation
Once the Bundle is done, then clicking on

    Run->Launch Simulation->HelloWorld

will start the simulation. It will take a few minutes to upload the bundle to an S3 bucket and run an EC2 instance for the AWS RoboMaker simulation to start. You will see that the status “Preparing” at the top menu bar, and then it will run.

When "Preparing" becomes "Running", click on

    "Simulation (Running)"->"View Simulation Job Details"

This will open a new  window from which the application tools typically used to inspect the behavior of the code can be launched, by clicking on `"Connect"` of each respective tool. For example `GZClient` for opening the Gazebo graphical user interface.

The simulation can be seen, as shown in the following figure.

![Gazebo](/img/gazebo.png)


## Stopping the simulation
While there is a max time configurable from the configuration of the simulation, it is important to stop the simulation when done to minimize the incurred cost. This can be done clicking from the top right on

    "Actions"->"Cancel"

A message will appear asking for confirmation to "Cancel".

Now that we got started with the environment, let's modify in the [next unit]({{ site.baseurl }}{% link _modules/mod-3c-jetbot.md %}) the simulation to include the Waveshare Jetbot, which is part of this course. This will allow us to be ready to learn how to develop robotics applications in the following course.

{% capture ref %}
{% bibliography --cited %}
{% endcapture %}
{% include references.html ref=ref %}
