---
# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved. // SPDX-License-Identifier: CC-BY-SA-4.0
title: Physical models
permalink: /modules/2/models
---

With a mobile robot in hand with actuators and effectors, It is important to study how a robotics system moves so that we can understand how to create control software for a specific robot.

**Kinematics** allows us to study the motion of the robot without considering forces, considering geometric models that describe the correspondence between actuator motion and the resulting effector motion.

For defining a kinematic model, we need to define its related elements:
- **State**: a representation for encoding the complete description of the robot's situation in the environment.
- **Action**: an input or control that can change the state.
- **Forward kinematics**: a function that determines a new state given the current state and an action.
- **Inverse kinematics**: given the goal state for the effector, determine the actuator motions.

Let's take an example of a single rolling wheel with radius $$r$$ on a line. What is its kinematic model?

{% include image.html url="/img/single-wheel.svg" description="Single wheel model." %}

The state can be represented as any location on the line, let's call it $$x$$ and assuming an infinite line, it can be any real number.

The action can be represented as a rotation angle $$u$$, measured in radians. Again, it can be any real number.

The forward kinematics then is as follows, given the current state $$x_t$$ and action $$u_t$$, using the formula to calculate the length of an arc: $$f(x_t,u_t)=x_t+u_t*r=x_{t+1}$$.

Simply solving the forward kinematics equation for $$u_t$$, we can find the inverse kinematics: $$g(x_t,x_{t+1})=\frac{x_t-x_{t+1}}{r}=u_t$$.

In general, the inverse kinematics is a difficult problem as there could be multiple actions to reach the same goal state.

{% capture text_frames %}
To define the state, it is also important to define reference frames or coordinate systems. Any robot is composed of multiple reference frames, attached to each sensor, and knowing the relationship between them allows to properly model motions and sensor readings.
{% endcapture %}

{% capture img_frames %}
{% include image.html url='/img/reference-frames.png' description='Reference frames of Husarion ROSbot 2.0.'  %}
{% endcapture %}
{% include two-columns.html description=text_frames media=img_frames position='right' %}

As any model, this is an approximation on how the system works. The example model for the single wheel doesn't account for any potential real-world problems, including slipping, sliding, or lateral motion. In addition, we are not considering any force, which is studied with **dynamic models**.

There are models already defined for many of the robots, including differential drive robots and car-like robots (Ackermann steering).

{% include image.html url='/img/diffdrive.svg' description='Differential drive robot.'  %}

{% include image.html url='/img/car.svg' description='Car-like robot.'  %}

To further deepen into the math, please refer to the {% cite lavalle2006planning %}[Chapter 13].
For the purpose of this course, just knowing the terminology and the intuition behind each of them is sufficient to then understand the next subproblems. Next, we will take a look at how to use sensor data to do [sensing]({{ site.baseurl }}{% link _modules/mod-2c-sensing.md %}).

{% capture ref %}
{% bibliography --cited %}
{% endcapture %}
{% include references.html ref=ref %}
