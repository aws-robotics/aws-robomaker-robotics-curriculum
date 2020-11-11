---
# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved. // SPDX-License-Identifier: CC-BY-SA-4.0
title: Robot Definition
permalink: /modules/1/robot-definition
---

What are the robots that we have seen in the exciting applications made of? In this module we will discover the main hardware that defines a robot.

## What is a robot?
The word **robot** was first used to represent a fictional humanoid by the Czech playwright Karel &#268;apek in a 1920 play called Rossum's Universal Robots.

Over the years, the term was defined in different ways depending on the specific progress.

For example, in the seventies, given the prominence of industrial manipulators, a robot was defined as follows:
> A reprogrammable, multifunctional manipulator designed to move material, parts, tools, or specialized devices through various programmed motions for the performance of a variety of tasks -- Robot Institute of America, 1979

A more general definition is provided by the International Organization for Standardization (ISO) in 2012:
> ... actuated mechanism programmable in two or more axes (4.3) with a degree of autonomy (2.2), moving within its environment, to perform intended tasks -- {% cite iso8373 %}

Currently, the ISO is revising that standard, covering the new technological progresses.

While definitions change over time, we can consider a robot as
an **autonomous system** {% cite mataric2007robotics %}, which:
- **exists** in the physical world,
- can **sense** its environment,
- can **act** on it,
- and has a **goal**.

{% include image.html url="/img/robot.svg" description="Depiction of an autonomous robot." %}

Such a definition covers all current characteristics that a robot needs to have to operate reliably and safely in different environments, covering the examples shown in [Motivation]({{ site.baseurl }}{% link _modules/mod-0a-motivation.md %}).

**Robotics** is a research area that encompasses multiple disciplines, including mechanical, electrical engineering and computer science, aimed at studying robots to achieve the autonomy level necessary to complete tasks in the real world.

## What is in a robot?

A robot is composed of a body that has:
1. **physical** (hardware) and,
2. **computational** (software) components.

Additionally, a robot might need **communication** mechanisms to communicate with a human operator or other robots.

Let's dive into the physical components of a robot, starting from devices that allow [Actions]({{ site.baseurl }}{% link _modules/mod-1b-actions.md %}).


{% capture ref %}
{% bibliography --cited %}
{% endcapture %}
{% include references.html ref=ref %}
