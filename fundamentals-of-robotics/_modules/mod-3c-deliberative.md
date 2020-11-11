---
# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved. // SPDX-License-Identifier: CC-BY-SA-4.0
title: Deliberative control
permalink: /modules/3/deliberative
---

While reactive control reacts to sensor data, a **deliberative control** senses first, plans, and then executes the plan. The output of each step becomes the input of the following step.

{% include image.html url="/img/0-sense-plan-act.svg" width="400px" description="Sense plan act architecture." %}

In this architecture, it is fundamental that the robot has all the elements we have seen in the previous module: localization, mapping, planning, and control. With the information about its state and the environment, the robot can predict future action outcomes and find a path using any of the techniques already seen, such as A\*. This system can work, in controlled scenarios. In industrial scenarios, for example, a robot manipulator with very precise encoders and motors needs to grasp objects and drop them in a bin, both of them in predefined places.

In general, purely deliberative systems are not used in practice. First, there is a **time-scale** issue: planning in large state spaces can take a very long time. This is typically the case in practice, which suggests that an open loop control, where the robot replans rarely, is preferable. However, we have already seen the issues of an open loop control: sensors and actuators are affected by noise and inaccuracies, and models are an approximation of the real world. This might result in the robot not accurately executing each step of the plan.
Second, while information is assumed to be up-to-date when the planner searches for a plan, it might not be the case. In practice, environments are dynamic -- for example, a robot might plan considering that a door is open, when it is actually locked. The more information, the better, but this is in conflict with the first issue.
Third, representing the state space necessary for planning (localization and mapping) might require large memory storage. RAM is limited on a computer, especially on small embedded systems, and some of the planning algorithms can be very **memory-intensive**.

Let's go ahead and see the architecture of practical systems, using [hybrid architecture]({{ site.baseurl }}{% link _modules/mod-3d-hybrid.md %}).
