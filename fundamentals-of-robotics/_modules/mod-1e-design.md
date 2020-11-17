---
# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved. // SPDX-License-Identifier: CC-BY-SA-4.0
title: Robot Design
permalink: /modules/1/robot-design.html
---

{% capture text_computational %}
What is important to consider when designing a robot? Compared to other fields, e.g., civil engineering, robotics is not mature yet and there are not widespread commercial tools to design a robot. Some initial attempts are being made by the research community to achieve **robot design through computation**, where according to some specifications, e.g., in terms of motion, a combination of actuators and effectors are determined algorithmically. An example of such a work is shown in the video on the right.
{% endcapture %}
{% capture video_computational %}
  {% include video-youtube.html url="https://www.youtube.com/embed/tyzUR_vilDw" %}
{% endcapture %}
{% include two-columns.html description=text_computational media=video_computational position="right" %}

-------
Here an overview of the criteria to consider that will guide the choices on the components we have seen so far in the previous units.

**Application/domain** specifications will have a significant impact on the actions and sensing devices that need to be on the robot. A rough terrain most likely will require a legged robot, while a mostly flat terrain can work with a car-like robot. If daylight with clear view, a camera can work well. During night, or if foggy, a RADAR might be necessary to detect obstacles. The selected sensors and their related uncertainty will determine the data that need to be processed, which will affect the requirements on the computational power and whether GPU computation is necessary or not to run deep learning models.

**Size and energy source** of the robot is a design choice that will affect the sensors that can be mounted on the robot. Current 3D LiDARs are relatively bulky that cannot be mounted on a small drone. Additional weight will reduce the flight time given the same battery. A small drone indeed typically only carries a camera.

**Budget** is typically a hard constraint, which determines what components can be bought. Some of the devices can range from a few dollars to hundreds of thousands of dollars, e.g., an IMU. Other devices, are only available starting with a much higher cost. For example, a 3D LiDAR currently costs more than a few thousands US dollars making it not affordable for a low-budget project.

-------
Let's see a couple of examples of robots torn down with all the different components exposed showing all components and implicitly highlighting the criteria listed above.

{% include video-youtube.html url="https://www.youtube.com/embed/Uv0oQWvdpGk" %}

{% include video-youtube.html url="https://www.youtube.com/embed/9Uy9QNidi1U" %}

Clearly in both the vacuum cleaner and the drone there is a strong emphasis on a compact and a reasonably priced product that consumers can find appealing. The specific domain of operation of the vacuum cleaner --indoor environments -- allows a the choice of a two wheel robot with a caster wheel, and with infrared sensor to avoid obstacles. The drone shown uses vision sensors to enable intelligent obstacle avoidance, so that it can operate in outdoor environments. Overall, these two examples show different design choices that balance the criteria above and heavily rely on assumptions of the specific application domain so that they can reliably operate.

-------
We have covered the hardware part of the robot. Many roboticists work with robot kits and commercial robots readily available. Knowing the characteristics will allow you to develop software that accounts for the strengths and weaknesses of the specific robot.

Before we move on, go ahead with the assessment for this module.
