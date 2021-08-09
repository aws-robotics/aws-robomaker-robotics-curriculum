---
# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved. // SPDX-License-Identifier: CC-BY-SA-4.0
title: Motivation
permalink: /modules/0/motivation.html
menus: header
---

In robotics, software developers typically write and test their code on local machines that need to be configured, and eventually deploy the developed software on a robot. This process can introduce bottlenecks in the development cycle, including management of local machines and scalability problems.

Cloud services have been growing over the years and are expected to grow up to US$140 billions in 2022 {% cite gartner2020 %}. Cloud services can streamline and enable new robotic applications and is related to Internet of Things (IoTs) concepts, where devices can talk to each other to achieve tasks more intelligently. They can be updated over the air and integrated with other cloud services services that are available, such as speech recognition or computer vision.

Some cloud based robotic systems have appeared over the years and are described in a 2015 survey from researchers at University of California, Berkeley {% cite kehoe2015survey %}.
<!-- Google offers Cloud Robotics Core, an open source platform to build and deploy robotics software  google2020robotics.-->
AWS offers AWS RoboMaker, enabling the development and simulation of robotic applications in the cloud. Commercial robotics companies such as iRobot use it for running tests faster and in parallel {% cite aws2020irobot %}.
The following video depicts a small demo showing fleet simulations with multiple robots on AWS RoboMaker.

{% include video-youtube.html url="https://www.youtube.com/embed/4t9e4L4rVZw" %}

Before we delve into the nuts and bolts of ROS (covered in the following course), let’s discover how to set up the development environment through this brief introduction to AWS RoboMaker [course]({{ site.baseurl }}{% link _modules/mod-0b-objectives.md %}).

{% capture ref %}
{% bibliography --cited %}
{% endcapture %}
{% include references.html ref=ref %}
