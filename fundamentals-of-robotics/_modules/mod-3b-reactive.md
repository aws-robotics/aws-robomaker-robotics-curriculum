---
# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved. // SPDX-License-Identifier: CC-BY-SA-4.0
title: Reactive control
permalink: /modules/3/reactive.html
---

The simplest control architecture is just to react to sensory data, without doing any planning. This is called **reactive system**, as there is a direct mapping between sensors and effectors, and minimal state information. It does not plan.
{% include image.html url="/img/0-sense-act.svg" width="280px" description="Sense act architecture." %}

The diagram shows a single sense-action pair, but in practice, there can be multiple concurrent sensory input data feeding into different reactive rules, which will send commands to the actuators. The simplest way is to have **mutually exclusive** rules. In some situations, this is possible and a table can be used to map observations with actions. For example, consider a robot with a left and right bumpers. If the left bumper is hit, then the robot rotates right, if the right bumper is hit, then the robot rotates left. Otherwise, the robot drives forward. Sensor with continuous values can still be represented in a table, if discrete range of values are defined.

An actual practical example where this sytem worked is a vacuum cleaner. The figure shows a time lapse of the path followed. One of the behaviors it has is called **random walk**, which is similar to the behavior described above: rotate randomly when encountering an obstacle. As it is a random rotation, eventually the robot will have the whole floor covered and cleaned.

<a title="Chris Bartle / CC BY (https://creativecommons.org/licenses/by/2.0)" href="https://commons.wikimedia.org/wiki/File:Roomba_time-lapse.jpg"><img width="512" alt="Roomba time-lapse" src="https://upload.wikimedia.org/wikipedia/commons/thumb/7/77/Roomba_time-lapse.jpg/512px-Roomba_time-lapse.jpg"></a>

In some cases rules are not triggered by mutually exclusive conditions, resulting in more than one action commands being sent to the effectors. Two strategies can be adopted to select an action:
1. **command arbitration**, where only one action is selected, for example with some preassigned or dynamic priorities, especially for discrete actions (e.g., stop);
2. **command fusion** when multiple actions are combined into a single output, especially for outputs that have lower-level representations (e.g., velocity).


A continuous system that we have seen in the previous module is the feedback controller, which corrects according to the sensor data. The controller is keeping some history with the integral term to address the steady-state error.

Only reacting is not enough to achieve complex tasks that require planning and prediction. Let's see the other extreme, where the robot, instead, reasons to find the best plan, with a [deliberative architecture]({{ site.baseurl }}{% link _modules/mod-3c-deliberative.md %}).
