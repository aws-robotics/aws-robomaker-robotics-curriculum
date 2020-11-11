---
# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved. // SPDX-License-Identifier: CC-BY-SA-4.0
title: Motivation
permalink: /modules/0/motivation
menus: header
---

In robotics, typically software developers write and test their code on local machines that need to be configured, and eventually deploy the developed software on a robot.
This process introduces bottlenecks in the development cycle, including management of local machines and scalability problems.

Cloud services have been growing over the years. For example, cloud application services are expected to grow up to $ 140 billions in 2022 {% cite gartner2020 %}. Cloud services can streamline and enable new robotics applications and is related to concepts of the Internet of Things (IoTs), where devices can talk to each other to achieve tasks more intelligently, they can be updated over the air, and they can be tested and integrated with other services that are available on the cloud, such as speech recognition or computer vision.

Some cloud based robotic systems have appeared over the years and are described in a 2015 survey from researchers at University of California, Berkeley {% cite kehoe2015survey %}.
<!-- Google offers Cloud Robotics Core, an open source platform to build and deploy robotics software  google2020robotics.-->
AWS offers RoboMaker, introduced one year ago, allowing development and simulation of robotic applications in simulation and eventually the deployment on robots. While new, some companies started using it, including iRobot which used it for running tests faster and in parallel {% cite aws2020irobot %}.
In the following a small demo showing fleet simulations with multiple robots on AWS RoboMaker.

{% include video-youtube.html url="https://www.youtube.com/embed/4t9e4L4rVZw" %}

Before we delve into the nuts and bolts of ROS (covered in the following course), let's discover how to set up the development environment through this brief introduction to AWS RoboMaker [course]({{ site.baseurl }}{% link _modules/mod-0b-objectives.md %}).

{% capture ref %}
{% bibliography --cited %}
{% endcapture %}
{% include references.html ref=ref %}
