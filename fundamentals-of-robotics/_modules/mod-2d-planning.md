---
# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved. // SPDX-License-Identifier: CC-BY-SA-4.0
title: Planning
permalink: /modules/2/planning.html
---

With the sensor information, the robot can then act in the environment.
We will discuss two important components necessary for the robot to move: **control** and **planning**.

## Control
How can the robot achieve and maintain a desired state? The robot needs a **controller** that takes as input the desired state and will find the correspondence to the actuators' commands.

{% capture text_open %}
A simple controller is called **open-loop** or **feedforward** controller. This controller uses a model, e.g., the kinematic models we have seen in the second unit of this module, to predict where it will be and accordingly send commands to the actuators. A practical example from our everyday life is a dryer machine with a timer: the timer does not adjust according to the dryness level of the clothes.
{% endcapture %}
{% capture img_open %}
{% include image.html url="/img/openloop.svg" description="Diagram for open-loop control." %}
{% endcapture %}
{% include two-columns.html description=text_open media=img_open position="right" %}


{% capture text_feedback %}
The robot can use the sensory information to extract the information about its current state and compare it with a desired or goal state, called **set point**. Action commands are sent so that the error between current and desired states is minimized. This type of control is called **feedback** or **closed loop** control. A simple example that we all have in our house is a thermostat, which turn on or off the heating system according to the measured temperature.
{% endcapture %}
{% capture img_feedback %}
{% include image.html url="/img/feedback.svg" description="Diagram for feedback control." %}
{% endcapture %}
{% include two-columns.html description=text_feedback media=img_feedback position="right" %}

A widely used controller in robotics and in other fields is the **PID controller**. **P** stands for proportional controller. The basic idea is that the response command $$u(t)$$ is proportional to the error $$e(t)$$ at time $$t$$: $$u(t)=K_p e(t)$$. This part of the controller accounts for the "present" error. Depending on the parameter that governs the magnitude of the response, also called **proportional gain** $$K_p$$, the robot can overshoot or undershoot the desired state, resulting in an oscillatory behavior. In general, it is difficult to set the proportional gain and remove the oscillatory behavior. One idea is to correct the momentum: when the robot is close to the desired state, it needs to be more finely controlled than when it is far from it. We achieve this with the **D** term, which stands for derivative controller and accounts for the "future": $$u(t)=K_d \frac{de(t)}{dt}$$. Also the derivative controller has a gain.
There might be a fixed error, also called **steady state error**, that would keep the robot far from the desired state of a fixed amount. In that case, we want to use a term that is able to "keep track of the history". This is the goal of the **I**, integral controller: $$u(t)=K_i\int e(t)dt$$, which, as the other controllers, has a gain.
These three controllers can be summed together to get the PID controller. By setting the corresponding gain to 0, then it is possible to achieve different combinations of those terms. For example, to only have the P controller, $$K_i$$ and $$K_d$$ are set to $$0$$.

 Here a visualization of the PID controller on controlling a robot car following a lane.
{% include video-youtube.html url="https://www.youtube.com/embed/4Y7zG48uHRo" %}

In practice, sensor data are received at a specific frequency, e.g., common cameras operate at 30 frames per second, and commands sent to the actuators are updated at those discrete time intervals, thus the discrete time PID controller is: $$u_k = K_p e_k + K_i \Delta t \sum_{j=1}^k e_j + K_d \frac{e_k-e_{k-1}}{\Delta t}$$.

A short exercise on the application of the discrete time PID controller is in the following video.

  {% include video-file.html url="/img/pid-exercise" %}

## Planning
Control allows the robot to execute actions and reach a desired state, without exploiting the full knowledge that comes from state estimation, i.e., state of the robot and the environment. **Planning** is the process of determining a sequence of actions and motions, by looking ahead.

A common problem that is addressed in planning is how to go from a start state A to goal state B, given the geometric description of the robot and the world.
In such a problem, the goal is to find the best path, according to criteria such as traveled distance and number of turns, while never touching any obstacle.
Planning is computationally complex as the planner needs to check many possibilities, otherwise it can potentially miss the best one.

Robots exist in the physical space with an embodiment. This is important to consider when planning so that robots do not collide with obstacles. A map with obstacles can be represented in **configuration space**, which is the space of all possible configurations that the robot can be in. For a mobile robot, the configuration space can be obtained by sliding the robot along the edge of the obstacle regions, and growing each obstacle by the robot's extension. An example for a rectangular translating robot is shown below.

{% include image-import.html content='<a title="Simeon87 / CC BY-SA (https://creativecommons.org/licenses/by-sa/3.0)" href="https://commons.wikimedia.org/wiki/File:Motion_planning_workspace_1_configuration_space_1.svg"><img width="256" alt="Motion planning workspace 1 configuration space 1" src="https://upload.wikimedia.org/wikipedia/commons/thumb/9/90/Motion_planning_workspace_1_configuration_space_1.svg/256px-Motion_planning_workspace_1_configuration_space_1.svg.png"></a>' description='Map with obstacles (grey).'  %}
{% include image-import.html content='<a title="Simeon87 / CC BY-SA (https://creativecommons.org/licenses/by-sa/3.0)" href="https://commons.wikimedia.org/wiki/File:Motion_planning_workspace_1_configuration_space_2.svg"><img width="256" alt="Motion planning workspace 1 configuration space 2" src="https://upload.wikimedia.org/wikipedia/commons/thumb/f/f5/Motion_planning_workspace_1_configuration_space_2.svg/256px-Motion_planning_workspace_1_configuration_space_2.svg.png"></a>' description='Configuration space with enlarged obstacles given the size of the robot (red).'  %}

**Search techniques** reason on some graph representations built on the configuration space to find a path. The idea of any search technique, such as A\*, Dijkstra, Breadth First Search (BFS), and Depth First Search (DFS), is to computationally evaluate different options by maintaining a tree of paths originating at the start node. It extends those paths, one edge at a time, applying a transition function to find the reachable states from a current state. It will continue until the goal is found. In the following short video, an instance of A\* over a grid, where the transition function is returning the neighbor cells of a current cell.
  {% include video-file.html url="/img/search" %}

As the search state space might be large, **sampling-based path planning** are used to create possible paths by randomly adding points to a tree -- Rapidly exploring Random Tree (RRT) -- or a graph -- Probabilistic Road Map (PRM). Any search algorithm can be then applied to such a tree or graph. An example of RRT is shown in the following video.

  {% include video-file.html url="/img/rrt" %}

Another approach is based on **potential fields**. The intuition is that the goal has an attractive force, while obstacles have repulsive force. Combining the two forces would result in the robot going from high to low potential field. While computationally efficient, the method can be subject to local minima, where the robot could get stuck. A random selection can be introduced to recover from such a situation.

  {% include video-file.html url="/img/potential-field" %}


-----
Now, with an idea on the last problem, i.e., planning, let's take a look at the [future of robotics]({{ site.baseurl }}{% link _modules/mod-2e-summary.md %}).

