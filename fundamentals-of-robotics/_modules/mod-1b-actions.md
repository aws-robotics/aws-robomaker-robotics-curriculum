---
# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved. // SPDX-License-Identifier: CC-BY-SA-4.0
title: Robot Actuators and Effectors
permalink: /modules/1/robot-action.html
---

Robots perform two main actions in the environment: **locomotion** and **manipulation**.
Two main components are necessary:
- **Effectors**: a device that is controlled by a robot and that affects the environment. A number of them are available, depending on the specific task, as shown in the four examples below.

{% capture fig_wheel %}
<a title="NASA/JPL-Caltech / Public domain" href="https://commons.wikimedia.org/wiki/File:Mars_rovers_wheels_isometric.jpg"><img width="128" alt="Mars rovers wheels isometric" src="https://upload.wikimedia.org/wikipedia/commons/thumb/6/62/Mars_rovers_wheels_isometric.jpg/512px-Mars_rovers_wheels_isometric.jpg"></a>
{% endcapture %}
{% include image-import.html content=fig_wheel description="Wheels"  %}

{% capture fig_track %}
<a title="JMiall / Public domain" href="https://commons.wikimedia.org/wiki/File:Caterpillar_track_shingle.JPG"><img width="112" alt="Caterpillar track shingle" src="https://upload.wikimedia.org/wikipedia/commons/thumb/f/f5/Caterpillar_track_shingle.JPG/128px-Caterpillar_track_shingle.JPG"></a>
{% endcapture %}
{% include image-import.html content=fig_track description="Continuous track"  %}

{% capture fig_foot %}
<div style="position: relative; width: 130px; height: 85px; overflow: hidden ">
<a title="David Buckley / CC BY-SA (http://creativecommons.org/licenses/by-sa/3.0/)" href="https://commons.wikimedia.org/wiki/File:2005-11-14_ShadowLeg_Finished_medium.jpg"><img width="256" alt="2005-11-14 ShadowLeg Finished medium" src="https://upload.wikimedia.org/wikipedia/commons/thumb/0/07/2005-11-14_ShadowLeg_Finished_medium.jpg/256px-2005-11-14_ShadowLeg_Finished_medium.jpg" style="position: absolute; top: -165px; left: 0px"></a>
</div>
{% endcapture %}
{% include image-import.html content=fig_foot description="Foot effector"  %}

{% capture fig_finger %}
<div style="position: relative; width: 130px; height: 85px; overflow: hidden ">
 <a title="Richard Greenhill and Hugo Elias (myself) of the Shadow Robot Company / CC BY-SA (http://creativecommons.org/licenses/by-sa/3.0/)" href="https://commons.wikimedia.org/wiki/File:Shadow_Hand_Bulb_large.jpg"><img width="128" alt="Shadow Hand Bulb large" src="https://upload.wikimedia.org/wikipedia/commons/thumb/c/c5/Shadow_Hand_Bulb_large.jpg/256px-Shadow_Hand_Bulb_large.jpg" style="position: absolute; top: -35px; left: 0px"></a>
</div>
{% endcapture %}
{% include image-import.html content=fig_finger description="Hand"  %}

Wheels, continuous tracks, and feet are what make locomotion possible. Typically, they are arranged in such a way that the robot is statically stable, i.e., can stand still without falling. A robot is **statically stable** when the center of gravity (COG) is within the polygon determined by the contact points of the robot on the ground, also called **polygon of support**. The depiction of the contact points and the center of gravity for a robot is depicted on the right figure.
{% include image.html url="/img/stability.png" description="Contact points (blue) and COG (black/white) on a ROSbot 2.0." %}
A hand allows the robot to manipulate objects in the environment.

- **Actuators**: the mechanism that allows the effectors to execute an action. In particular, such devices can cause **linear** or **rotary** motion. They convert a control signal to either a speed or a force/torque.

{% capture text_passive %}
  Some mechanisms are passive, i.e., they do not draw any energy from supplies such as battery and fuel, but they exploit the interaction with the environment. A notable example is the passive walker that relied on gravity to move over a slope. The natural swings of the legs without any motor makes such a mechanism very efficient. An intrinsic disadvantage is that such a robot can only work on downhill slopes {% cite collins2001three %}, as shown in the video on the right.
{% endcapture %}
{% capture video_passive %}
  {% include video-youtube.html url="https://www.youtube.com/embed/wMlDT17C_Vs" %}
{% endcapture %}
{% include two-columns.html description=text_passive media=video_passive position="right" %}


Other mechanisms are **active** actuators, which, differently from passive actuators, require an energy source. There are many types of actuators, including photo-reactive, chemically reactive materials, piezoelectric materials. Actuators based on electric current (e.g., DC motor) and pressure changes (pneumatic) are two examples that can be commonly found in robots.
{% capture fig_dc %}
<a title="Abnormaal / CC BY-SA (http://creativecommons.org/licenses/by-sa/3.0/)" href="https://commons.wikimedia.org/wiki/File:Electric_motor.gif"><img width="128" alt="Electric motor" src="https://upload.wikimedia.org/wikipedia/commons/8/89/Electric_motor.gif"></a>
<a title="Dcaldero8983 / CC BY-SA (https://creativecommons.org/licenses/by-sa/3.0)" href="https://commons.wikimedia.org/wiki/File:DC_Motor.jpg"><img width="128" alt="DC Motor" src="https://upload.wikimedia.org/wikipedia/commons/thumb/f/f4/DC_Motor.jpg/512px-DC_Motor.jpg"></a>
{% endcapture %}
{% include image-import.html content=fig_dc description="DC motor"  %}
{% capture fig_pneumatic %}
<div style="position: relative; width: 630px; height: 128px; overflow: hidden ">
<a title="Kamarton / CC BY (https://creativecommons.org/licenses/by/3.0)" href="https://commons.wikimedia.org/wiki/File:Pneumatic_cylinder_(animation).gif"><img width="128" alt="Pneumatic cylinder (animation)" src="https://upload.wikimedia.org/wikipedia/commons/0/0d/Pneumatic_cylinder_%28animation%29.gif" style="transform: scale(0.35, 0.35); position: absolute; top: -180px; left: -30px;"></a>
<a title="JakeUM / CC BY-SA (https://creativecommons.org/licenses/by-sa/3.0)" href="https://commons.wikimedia.org/wiki/File:Underwater_Linear_Actuator.png"><img width="512" alt="Underwater Linear Actuator" src="https://upload.wikimedia.org/wikipedia/commons/thumb/f/f3/Underwater_Linear_Actuator.png/512px-Underwater_Linear_Actuator.png" style="transform: scale(0.4, 0.4); position: absolute; left: -80px;"></a>
</div>
{% endcapture %}
{% include image-import.html content=fig_pneumatic description="Pneumatic actuator"  %}


{% capture text_pwm %}
As many of the robots available to consumers use DC motors, it is useful to know how to control them through a computer which can send digital signals. A common way is to use a Pulse Width Modulation (PWM), a digital signal that can only be high (e.g., 5 Volts) or low (ground). The main idea is to change the time when the signal is high and low so that the average voltage is at the desired target. The graph on the right shows the actual signal sent in blue, and in red the corresponding average voltage.
{% endcapture %}
{% capture fig_pwm %}
<a title="Zureks / CC BY-SA (http://creativecommons.org/licenses/by-sa/3.0/)" href="https://commons.wikimedia.org/wiki/File:PWM,_3-level.svg"><img width="192" alt="PWM, 3-level" src="https://upload.wikimedia.org/wikipedia/commons/thumb/8/8e/PWM%2C_3-level.svg/512px-PWM%2C_3-level.svg.png"></a>
{% endcapture %}
{% capture fig_pwm_include %}
{% include image-import.html content=fig_pwm description="PWM signal"  %}
{% endcapture %}
{% include two-columns.html description=text_pwm media=fig_pwm_include position="right" %}

{% capture text_dof %}
Most simple actuators control a single **degree of freedom**. A degree of freedom represents a way in which an object can move. Two fundamental types of movement are:
- translation, which can be broken down in 3 kind of movements -- forward/backward (red), left/right (yellow), up/down (blue);
- rotation, again split in 3 motions -- roll (green), pitch (cyan), and yaw (orange).
{% endcapture %}
{% capture fig_dof %}
<a title="GregorDS / CC BY-SA (https://creativecommons.org/licenses/by-sa/4.0)" href="https://commons.wikimedia.org/wiki/File:6DOF.svg"><img width="128" alt="6DOF" src="https://upload.wikimedia.org/wikipedia/commons/thumb/2/2a/6DOF.svg/512px-6DOF.svg.png"></a>
{% endcapture %}
{% capture fig_dof_include %}
{% include image-import.html content=fig_dof description="Six Degrees of Freedom"  %}
{% endcapture %}
{% include two-columns.html description=text_dof media=fig_dof_include position="right" %}

{% capture text_joints %}
The mechanical design of the robot and the actuators will determine the degrees of freedom of a robot. Rigid or flexible bodies -- **links** --  may be connected together by **joints**. An example on how to represent a 2D manipulator with a fixed base to a flat surface with three joints and 4 links + the end effector is displayed on the right.  
Having this knowledge is important then to study the possible motion of the robot without considering forces -- **kinematics** -- and explicitly modeling forces -- **dynamics** {% cite spong2006robot %}.
{% endcapture %}
{% capture fig_joints %}
<a title="Jan Boddez / Public domain" href="https://commons.wikimedia.org/wiki/File:Inverse-kinematics-multiple-solutions.svg"><img width="128" alt="Inverse-kinematics-multiple-solutions" src="https://upload.wikimedia.org/wikipedia/commons/thumb/e/e3/Inverse-kinematics-multiple-solutions.svg/128px-Inverse-kinematics-multiple-solutions.svg.png"></a>
{% endcapture %}
{% capture fig_joints_include %}
{% include image-import.html content=fig_joints description="Manipulator example"  %}
{% endcapture %}
{% include two-columns.html description=text_joints media=fig_joints_include position="right" %}

With the capability to act, the robot needs devices to sense the environment -- topic we cover with the next unit on [Sensors]({{ site.baseurl }}{% link _modules/mod-1c-sensors.md %}).

{% capture ref %}
{% bibliography --cited %}
{% endcapture %}
{% include references.html ref=ref %}
