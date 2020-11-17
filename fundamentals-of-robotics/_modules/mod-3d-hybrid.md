---
# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved. // SPDX-License-Identifier: CC-BY-SA-4.0
title: Hybrid control
permalink: /modules/3/hybrid.html
---

The hybrid control architecture brings together the advantages of both the reactive and deliberative control architectures.
In particular, it consists of three layers: a reactive layer where sensor readings are mapped to actuator commands, a planner that uses the information from the sensors to build the representation, and send actions, and a layer that connects the two.

{% include image.html url="/img/0-hybrid.svg" width="300px" description="Hybrid architecture." %}

The challenge is to design it so that the best trade-off between deliberative and reactive behaviors is achieved. For example, should the robot move, following a previous plan, while replanning, or should it stop? If a corridor seems to be blocked, should the robot wait or find an alternative path?
The first step, regardless on how the system deals with such situations, is to update the representation of the world. In this way, the deliberative layer can plan with more updated information -- pre-requisite discussed in the previous unit. To address replanning, one of the key approaches is not to start from scratch in replanning. The previous plan and the intermediate representations can be stored so that they can be udpated as appropriate and reused to more efficiently find new plans. This allows the system to learn changes in the environment.

In practice, this is the architecture being used for robots. How can we write software that can work for different systems? Let's find out in the next module on [middleware]({{ site.baseurl }}{% link _modules/mod-3e-middleware.md %}).
