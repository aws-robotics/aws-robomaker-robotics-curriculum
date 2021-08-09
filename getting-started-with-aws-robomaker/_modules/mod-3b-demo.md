---
# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved. // SPDX-License-Identifier: CC-BY-SA-4.0
title: AWS RoboMaker Demo
permalink: /modules/3/robomaker-demo.html
---

With the AWS robot development configured, let's ensure that we understand the main tools and see a sample example which will show the simulation running. In this module, we will learn what the structure of the code should be and how to run the simulation. We will learn in the next course, as we deepen into ROS, how the code can be modified.

## Ready to go sample application

We will start from a sample application available in AWS RoboMaker and allow us to inspect the different elements. To download the sample application click on `"Resources" -> "Download Samples" -> "Hello World"` as shown in the following screenshot:

![Download samples](/img/download-samples.png)

This will initiate the download of the necessary dependencies which will take a few minutes.

## Directory structure for workspaces

Once downloaded, we will see that there is a new folder `HelloWorld`. This folder contains the ROS workspaces. In particular, `simulation_ws` contains files for the simulation and `robot_ws` contains the files for the specific robot application and behavior. As shown in the following figure, the structure is identical to the structure of a `colcon` workspace in ROS (and very similar to a `catkin` workspace). In particular, the ROS package that contains code, which will make the robot rotate in place, is called `hello_world_simulation`.

![Structure](/img/structure.png)

## Terminal available

The terminal made available in the IDE allows you to run all commands that can be run in Ubuntu, including `git` in case a package from a repository needs to be downloaded. Here some examples of commands to navigate within the directory structure.

	ubuntu:~ $ ls
	environment  gocode  node_modules  package-lock.json
	ubuntu:~ $ cd environment/HelloWorld/simulation_ws/
	ubuntu:~/environment/HelloWorld/simulation_ws $ ls
	build  bundle  install  log  src
	ubuntu:~/environment/HelloWorld/simulation_ws $ cd src/


---

Now that we got started with the development environment, let's see how to build, run, and test applications in the next two units. We will start with having an experience that is similar to a local machine, in the [next unit]({{ site.baseurl }}{% link _modules/mod-3b-1-demo.md %}).

{% capture ref %}
{% bibliography --cited %}
{% endcapture %}
{% include references.html ref=ref %}
