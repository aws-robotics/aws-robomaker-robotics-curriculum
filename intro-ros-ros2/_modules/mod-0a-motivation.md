---
# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved. // SPDX-License-Identifier: CC-BY-SA-4.0
title: Motivation
permalink: /modules/0/motivation.html
menus: header
---

The best practice in robotics development is to build robotic applications as a collection of small, focused programs that do one thing well and can be reused across different robots. These separate programs are able to talk to each other with the help of a **middleware** which provides the ability to pass **messages** between programs without having those programs be tightly tied to each other. All they need to know is the format of the messages that get passed around.
For example, software for recognizing objects through images is independent of the specific robot and should be re-usable across different robots with a camera.


That vision has been made possible by the Robot Operating System (ROS), which was created back in 2007 by a small team in the Silicon Valley {% cite origin-ros %} and has since then  become a de facto standard in the robotics community. Currently, it supports a large number of robots, including aerial, ground, manipulator, and aquatic robots, as shown [in a list of robots supported by ROS](https://robots.ros.org/).

Can we write a single program that works for example for both a simulated robot and a physical counter-part -- e.g., the Waveshare Jetbot shown in the figure below?

{% include image.html url="/img/jetbot.jpg" description="Waveshare Jetbot in Gazebo and physical robot." %}

Working in simulation and with the physical robot, as well as working with other robots with the same code will allow effective robotics software development.
Let's discover how to write such programs through [this brief introduction to ROS course]({{ site.baseurl }}{% link _modules/mod-0b-objectives.md %}).

{% capture ref %}
{% bibliography --cited %}
{% endcapture %}
{% include references.html ref=ref %}
