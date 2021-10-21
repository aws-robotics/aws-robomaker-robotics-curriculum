---
# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved. // SPDX-License-Identifier: CC-BY-SA-4.0
title: What are simulations?
permalink: /modules/0/simulations.html
---

Let's first dive into the general concept and purpose of simulations so that we can better understand the structure of Gazebo. 

---

Testing in real world can be expensive and time consuming. **Simulations** allow for the testing of robotics software and systematic data collection within a computer, providing several benefits, including cheaper and safer experiments, experiments that would not be possible in real world (e.g., deep water experiments), and the ability to run experiments faster and in parallel. This has been recognized in a recent paper appearing in the Proceedings of the National Academy of Sciences of the United States of America {% cite choi2021use %}.

A simulation might involve actual people operating the simulated systems or just within the computer programs. A separate type of simulation involves actual use of real equipment but in a mock-up scenario. They are called **virtual**, **constructive**, and **live**, according to the United States Department of Defense in the Modeling and Simulation Glossary. The following short video highlights these different simulation types. 

{% include video-youtube.html url="https://www.youtube.com/embed/LTILL62aaE4" %}

While in this course we use simulations for robots, simulation is a general concept that is applied to many different fields, including economics, health, project management, networking, and weather.

To run simulations, it is necessary to create **models** which represent a system of interest and can be executed over time. A model needs to be as **realistic** as possible so that the simulation is close to the real-world phenomenon; at the same time, it needs to be **simple** so that the models can be run in simulation in real-time.

Simulations can be categorized in different ways according to different dimensions. 
**Time** is one dimension. A simulation can be based on **discrete events** or **continuous time**. A discrete event simulation changes the state when specific events happen, for example to model the *scheduling* of online order deliveries. A continuous time simulation instead changes the state having a simulated time as in the real world. This is the case for most robotics simulators. Note that in continuous time simulations it is possible to modify the **real time factor** that determines whether the simulation runs in real time, slower, or faster. That factor depends also on the computational capabilities of the computer that the simulator is running on: if the computer is not "fast enough", the simulation might run slower. In addition, for such simulations it is possible to define the **simulation time step**, i.e., the frequency at which simulations are advanced.
In discrete events simulations, typically physics modeling is commonly neglected. Instead, robotics simulations are typically based on the **physics models** that represent, e.g., the rigid body dynamics and collisions.
**Randomness** is another dimension. A simulation can be **deterministic** or **stochastic**. A deterministic simulation is one where repeated runs under the same conditions leads to the same results. A stochastic one instead has different results when repeated under the same conditions, because of random variables involved in the process. As we will see in the next module, in Gazebo, randomness can be introduced for sensors readings.

A developer can choose the type of simulation based on their need. For example, in a multirobot coordination scenario, such as allocating robots to handle different orders in a logistic, it is not necessary to have a continuous time simulation. In a different scenario for testing control algorithms, it is necessary to have a continuous time simulation. The choices made for the type of simulation determines the **level of fidelity** of the simulation. From low, where the system responds to inputs and provides outputs, to high, where it is very close to the real system.   The following figure shows an example of a 2D simulator available in ROS, [Stage](), vs. Gazebo, a 3D simulator.

{% include image.html url="/img/stage-gazebo.png" description="Stage 2D simulator vs. Gazebo 3D simulator." %}

In practice, most of the robotic simulators show a relatively high level of fidelity, as they are continuous time, physics based, and stochastic. 

Simulations' physical models are typically validated with some physical reproductions at smaller scale, as shown in the following diagram. There are some studies that included for example the comparison of sensors data from real and simulated sensors {% cite pepper2007robot %}.

{% include image.html url="/img/simulation-real.png" description="Validation process of simulation." %}

When creating a simulation, it is important to think about the goal of the simulation. The goal will guide your decisions on the dimensions of the simulation and the specific world to create. 

A common goal is to test robots as already mentioned. When thinking about this goal it is important to define the **metrics**, which are performance indicators. There are two main robot metrics: **task dependent** -- i.e., specific to the task that the robot is trying to accompligh -- and **task independent**. A task dependent metric in an exploration mission, where the goal is to map the environment, can be the explored area measured in square meters over time; or the traveled distance. In general, such task dependent metrics can be thought as utility vs. cost. Task independent metrics include whether the robot is robust to failure or is able to adapt to changes.  

Having a simulated world, especially if realistic, allows the creation of a synthetic dataset and/or the training of machine learning models for robotics perception and control. We will see an example of reinforcement learning this later in this course.

Simulations are typically performed to test the software -- also called **software-in-the-loop simulations** -- but there are also **hardware-in-the-loop** simulations, where an embedded system is fed with sensor data from the simulator and the embedded system provides outputs to the simulator.

---

Now that we have seen what simulations are, let's see the [high-level structure of Gazebo]({{ site.baseurl }}{% link _modules/mod-0d-gazebo.md %}), which will allow us to dive into each element in the following modules.


{% capture ref %}
{% bibliography --cited %}
{% endcapture %}
{% include references.html ref=ref %}
