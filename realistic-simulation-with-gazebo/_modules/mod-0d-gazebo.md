---
# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved. // SPDX-License-Identifier: CC-BY-SA-4.0
title: Gazebo architecture
permalink: /modules/0/gazebo-architecture.html
---

There are many simulators out there and there have been attempts to create interoperable simulations. ROS provides a way for robotics software to interact with different simulators. Simulators also try to define a set of standards so that it is easy to extend them.

The main characteristics of any robotic simulator include:
- physics simulation, 
- world modeling,
- robot modeling,
- visual rendering,
- user interface.

Gazebo is implemented following a distributed architecture, where each of these characteristics are in separate libraries. Gazebo has a communication infrastructure that allows for communication between the different libraries. The communication infrastructure has similarities with ROS, as it follows a publish and subscribe paradigm. The following figure shows Gazebo's architecture. The full details are included in the [architectural overview](http://gazebosim.org/tutorials?tut=architecture&cat=get_started) and [design specification](http://wiki.ros.org/gazebo/Version_1.0_Design_Specification) of Gazebo. These details are based off the first Gazebo version, but the main structure is still valid.

<img src="http://wiki.ros.org/gazebo/Version_1.0_Design_Specification?action=AttachFile&do=get&target=gazebo_dependency_graph.png" alt="AWS WorldForge example" width="500" height="333">

There are a server, which runs the code for simulating the physics and generating the sensor data, and a client that can be run by a user to interact and visualize the simulation.  Different engines can be used to simulate the physics and collisions. Currently, four engines are supported. The Gazebo simulator can be extended through plugins that can be added to the models, including for the physics (e.g., buoyancy plugin), sensors (e.g., a LiDAR).

Following the main characteristics of any simulation, the Gazebo simulation is specified by a world, which contains the specifications about the global properties of the simulation, light, models, and plugins. Each model is a collection of links, joints, and sensors. A high-level architecture is shown in the following diagram from the original research paper published in 2004 {% cite koenig2004design %}. A model can be a static object (e.g., a wall), a robot, or an animated object (actor) that extends a model with scripted animations. An actor is useful to add people in a scene which are visible in the simulated camera images, although they would not be affected by the physics engine (e.g., they would not collide with the robot). 

<img src="https://d3i71xaburhd42.cloudfront.net/2151a214aca6e72ee2980ae8cbf7be47fed0cb7a/2-Figure1-1.png" alt="General structure of Gazebo components (Koenig & Howard, 2004)." width="500" height="333" /> 

For any simulator, including Gazebo, the input is a world file that includes all models that should be simulated. We will learn the main elements of the models in the following modules.

---

Before we move on each of the Gazebo elements that will help us create a realistic simulation, [let's assess]({{ site.baseurl }}{% link _modules/mod-0e-assessment.md %}) our understanding of simulations and Gazebo architecture.


{% capture ref %}
{% bibliography --cited %}
{% endcapture %}
{% include references.html ref=ref %}
