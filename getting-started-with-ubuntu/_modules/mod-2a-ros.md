---
title: Getting started with ROS
permalink: /modules/2/ros.html
---

In this module, we will discover the main features provided by the Robot Operating system (ROS), which will allow us to develop robotic software and enable behaviors. 


## What is ROS?
According to its page, ROS is described as follows:

> ROS is an open-source, meta-operating system for your robot. It provides the services you would expect from an operating system, including hardware abstraction, low-level device control, implementation of commonly-used functionality, message-passing between processes, and package management. It also provides tools and libraries for obtaining, building, writing, and running code across multiple computers. (ROS Wiki)

With such a framework, there are a number of benefits provided:
- Software reuse: many of the robotic capabilities described in Course 0 are readily available in ROS, allowing new users to get started with a mobile robot.
- Rapid testing in simulation and with real robots. The hardware abstraction and the standardization of message-passing between processes allow developers to seamlessly work with simulators and with real robots.
- Distributed computation: processes on robots can run on different computers or in the cloud and ROS enables different processes to communicate over the network.

In this module, we will just get started in setting up the development environment. This is also important to know so that new robots are configured properly.

Let's dive into the steps of installing a ROS on Ubuntu in the [next unit]({{ site.baseurl }}{% link _modules/mod-2b-installation.md %}) -- if you already have a machine with ROS you can skip it and jump to the [following unit]({{ site.baseurl }}{% link _modules/mod-2c-development.md %}).


{% capture ref %}
{% bibliography --cited %}
{% endcapture %}
{% include references.html ref=ref %}
