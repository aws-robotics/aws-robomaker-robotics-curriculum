---
# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved. // SPDX-License-Identifier: CC-BY-SA-4.0
title: AWS RoboMaker architecture
permalink: /modules/1/architecture.html
---

How is AWS RoboMaker structured and what is possible within it?

The following figure shows its architecture.

![aws-robomaker-architecture](/img/2019-05-28_16-44-56.png)

There are two main components from AWS that are important to understand the architecture:
1. Services that run within AWS cloud, e.g., the AWS RoboMaker Development environment.
2. Storage available to store the robot applications and files generated during the execution of the simulation.

The time that is used to running the services and the storage used determines the cost for your development within AWS RoboMaker.

A typical workflow is:

**Build and test your applications in the AWS RoboMaker IDE.** A developer uses the AWS RoboMaker Development environment (IDE), a cloud robotics workspace, to write and test the code that interfaces with ROS. The IDE that is preconfigured with the essential tools (e.g. ROS and Gazebo) to start building and testing robot applications without needing to provision additional hardware. This is especially for beneficial for users who do not want to or cannot install ROS and Gazebo on their local machines. 

The AWS RoboMaker IDE provides a full [Ubuntu](https://ubuntu.com/) desktop through the browser. This provides you the ability to run robot and simulation applications and interact with them via Gazebo, RViz, and other tools directly, as though they were running them on your own desktop. This enables a fast iterative workflow to code-test-refine all within the IDE. 

**Run Simulation Jobs in AWS RoboMaker Simulation.** While you can simulate within the AWS RoboMaker IDE, sometimes you need to run a large batch of simulation jobs or run a simulation as part of a CI/CD flow. AWS RoboMaker Simulation is a fully managed robotics simulation service that can be used to run ROS and ROS2 applications in simulation and automatically scales the underlying infrastructure based on the complexity of the simulation. It provides access to simulations tools for visualizing and interacting with the simulation (e.g. Gazebo, rqt, RViz and terminal access tools), and the output can be integrated with other AWS services, such as those to monitor resources or process video. It also provides developers flexibility to use custom simulation tools in place of, or in addition to the simulation tools provided by default. Log files will be stored in another S3 bucket, later accessible for inspection. 

After the code has been written and tested in the IDE, a robot and simulation application will be created, including all the necessary files to run it and stored in the cloud in an **S3 bucket**. These applications can be loaded in AWS RoboMaker Simulation.

The application can be also deployed directly on a robot using another service, the AWS RoboMaker Fleet Management.

**Deployment to a Physical Robot.** Finally, an application can be deployed directly on a robot using [AWS IoT Greengrass 2.0](https://aws.amazon.com/blogs/robotics/deploying-ros-applications-snaps-aws-iot-greengrass/). We will dive into deployment in a later course.

This an architecture enables the typical process that roboticists go through: development, test, and deploy. 

Now we should have a basic understanding of AWS RoboMaker, its architecture, and its features. We will dive into the next unit on the components from AWS that are necessary to make AWS RoboMaker set up. Before that, [let's see our understanding of AWS RoboMaker]({{ site.baseurl }}{% link _modules/mod-1c-assessment.md %}).


{% capture ref %}
{% bibliography --cited %}
{% endcapture %}
{% include references.html ref=ref %}
