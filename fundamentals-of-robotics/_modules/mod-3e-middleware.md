---
# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved. // SPDX-License-Identifier: CC-BY-SA-4.0
title: Middleware
permalink: /modules/3/middleware.html
---

We have seen different control architectures and how from the high-level perspective they should interact with each other. Robots are complex systems that involve a large number of individual components that need to share information with each other. **Modularity** and **reusability** are key features to enable widespread development and use, similarly to smartphones apps.

A robotics **middleware** provides message passing interface for inter-process communication, so that one software layer is abstracted from another. In this way, developers can focus on their specific module and can interact with other modules with specified communication protocols, and input and output. Sensors of the same type from different vendors, when drivers are available, can be interchanged seamlessly, similar to the way a computer interfaces with different devices.
{% include image.html url="/img/0-middleware.svg" width="400px" description="Middleware diagram." %}

While there are many robot frameworks, such as Player and MOOS, currently, the Robot Operating System (ROS) is a holistic robotics framework, providing a middleware, visualization tools, and libraries. In addition, as it is used by many researchers and companies worldwide, there are many of functionalities we discussed, for example mapping and localization, that are already available.
ROS is integrated with many other libraries, including OpenCV for computer vision and PCL for processing point clouds. In addition, it is integrated with realistic simulators, including Gazebo.
The middleware allows software written for real robots to be tested also on simulators, without any change to the core algorithms. This facilitates rapid testing of robotic modules.

In addition, with the advent of cloud services, ROS has a cloud extension with AWS RoboMaker, which provides a development environment, simulation, and fleet management from the cloud. This will allow to streamline the development, testing, and deployment pipeline.


[Let's revise]({{ site.baseurl }}{% link _modules/mod-3f-assessment.md %}) some of the concepts before ending this course and introducing other resources available to continue your journey in learning robotics software development.
