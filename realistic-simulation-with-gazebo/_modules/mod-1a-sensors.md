---
# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved. // SPDX-License-Identifier: CC-BY-SA-4.0
title: Simulated sensors
permalink: /modules/1/sensors.html
---

Before we start diving into Gazebo, let's understand the characteristics of the simulated sensors. In this way, our expectations can be adjusted when deploying on the real robot. 
As mentioned in Course 0, there are a number of parameters that affect the reliability of the sensors used on the robot -- for example, the noise affecting the measurements. A realistic simulator will simulate that noise to test the robustness of the robotic software.

## Sensors on a wheeled robot

 A wheeled ground robot has typically a number of sensors:

- encoder for odometry,
- LiDAR,
- camera.

The Waveshare Jetbot, used as reference within this course, is shown in the following figure. Note that by default, this robot does not come with the wheel encoders.
{% include image.html url="/img/jetbot.svg" description="Waveshare Jetbot." %}

While we will learn about the main elements in subsequent courses, the referenced code in the following can be found in the [github repository for the model of the robot](https://github.com/aws-samples/aws-robomaker-jetbot-ros). 

---

In the next units we will walk through each of these simulated sensors in Gazebo. Let's start with the [odometry]({{ site.baseurl }}{% link _modules/mod-1b-odometry.md %}).


{% capture ref %}
{% bibliography --cited %}
{% endcapture %}
{% include references.html ref=ref %}
