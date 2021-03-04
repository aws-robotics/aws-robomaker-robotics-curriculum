---
# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved. // SPDX-License-Identifier: CC-BY-SA-4.0
title: AWS RoboMaker architecture
permalink: /modules/1/architecture.html
---

How is AWS RoboMaker structured and what is possible within it?

The following figure shows its architecture.

![aws-robomaker-architecture](/img/2019-05-28_16-44-56.png)

There are two main components from AWS that are important to understand the architecture:
1. services that run within AWS cloud, e.g., the AWS RoboMakeer Development environment, the Simulation.
2. storage available to store the robot applications and files generated during the execution of the simulation.

The time that is used to running the services and the storage used determines the cost for your development within AWS RoboMaker.

A typical workflow is: developers first work within the AWS RoboMaker Development environment, an IDE in the cloud, to write code that interfaces with ROS. After the code is written, a robot application will be created, including all the necessary files to run it and stored in the cloud in an **S3 bucket**. Such a robot application can be loaded in AWS RoboMaker Simulation, which is based on Gazebo. AWS RoboMaker provides within the Simulation environment access to applications for visualizing data from the robots, e.g., rviz, and the output can be integrated with other AWS services, such as those to monitor resources or process video. The log files will be stored in another S3 bucket, later accessible for inspection.
The application can be also deployed directly on a robot using another service, the AWS RoboMaker Fleet Management.


Such an architecture enables the typical process that roboticists go through: development, test, and deploy.


Now we should have a basic understanding of AWS RoboMaker, its architecture, and its features. We will dive into the next unit on the components from AWS that are necessary to make AWS RoboMaker set up. Before that, [let's see our understanding of AWS RoboMaker]({{ site.baseurl }}{% link _modules/mod-1c-assessment.md %}).


{% capture ref %}
{% bibliography --cited %}
{% endcapture %}
{% include references.html ref=ref %}
