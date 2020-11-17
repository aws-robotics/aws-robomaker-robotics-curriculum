---
# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved. // SPDX-License-Identifier: CC-BY-SA-4.0
title: Robot Computation and Communication
permalink: /modules/1/robot-computer.html
---

Several choices are available for the computation devices processing the data from the sensors and sending commands to the actuators. Laptops can be a first choice for a powerful platform easy to use. In practice, other considerations are important, including **cost**, **power usage**, and **size**. In self-contained robots, roboticists dedicate effort to configure embedded systems or small form factor computers, allowing for lower cost and power usage. Examples include those below. A microcontroller, like an Arduino Uno, can run one program at a time, over and over again, useful a single purpose, e.g., used for controlling motors. Others, like Raspberry Pi 4 and Intel NUC are full computers that can run as normal laptops. The difference is with CPU, memory, and other interfaces. Some of those embedded systems have also GPU capabilities, such as the NVIDIA Jetson Nano.

{% include image-import.html content='<a title="SparkFun Electronics from Boulder, USA / CC BY (https://creativecommons.org/licenses/by/2.0)" href="https://commons.wikimedia.org/wiki/File:Arduino_Uno_-_R3.jpg"><img height="128px" alt="Arduino Uno - R3" src="https://upload.wikimedia.org/wikipedia/commons/3/38/Arduino_Uno_-_R3.jpg"></a>' description="Arduino Uno" %}

{% include image-import.html content='<a title="Miiicihiaieil  Hieinizilieir / Wikimedia Commons" href="https://commons.wikimedia.org/wiki/File:Raspberry_Pi_4_Model_B_-_Side.jpg"><img height="128px" alt="Raspberry Pi 4 Model B - Side" src="https://upload.wikimedia.org/wikipedia/commons/thumb/f/f1/Raspberry_Pi_4_Model_B_-_Side.jpg/512px-Raspberry_Pi_4_Model_B_-_Side.jpg"></a>' description="Raspberry Pi 4"  %}

{% include image-import.html content='<a title="Ubahnverleih / CC0" href="https://commons.wikimedia.org/wiki/File:Nvidia_Jetson_Nano_2_Development_Kit_15_14_39_352000.jpeg"><img height="128px" alt="Nvidia Jetson Nano 2 Development Kit 15 14 39 352000" src="https://upload.wikimedia.org/wikipedia/commons/thumb/7/76/Nvidia_Jetson_Nano_2_Development_Kit_15_14_39_352000.jpeg/512px-Nvidia_Jetson_Nano_2_Development_Kit_15_14_39_352000.jpeg"></a>' description="NVIDIA Jetson Nano" %}

{% include image-import.html content='<a title="Laserlicht / CC BY-SA (https://creativecommons.org/licenses/by-sa/4.0)" href="https://commons.wikimedia.org/wiki/File:Intel_NUC6_inside.jpg"><img height="128px" alt="Intel NUC6 inside" src="https://upload.wikimedia.org/wikipedia/commons/thumb/6/6c/Intel_NUC6_inside.jpg/512px-Intel_NUC6_inside.jpg"></a>' description='Intel NUC' %}

Typically more than one computational device is present on the robot. In this way, various parts of the robot can be processed in **parallel**. **Differentiation** of work is one reason for multiple computational devices. For example, it is typical to have a micro-controller for the motors, to safely operate the robot in the environment, and another computer for processing sensor data. **Redundancy** can be another reason to have more than one computer. Computational power requirements depend on the sensor data to be processed and the software run on-board. Deep learning techniques require GPU capabilities. Clearly, the more computational power, the more heat and power consumption in the system.

Communication systems might be necessary if there is a requirement that the robot communicates with a human operator for tele-operation or high-level supervision, or other robots. Different strategies are available and are drawn from the way devices are interconnected. The most straightforward way is to **tether** a robot for reliable tele-operation. However, that comes with the challenges of cable management. If the robot is not aware of that, it can easily become tangled, as shown in the example picture below of an underwater robot.
Other solutions involve wireless solutions. A common solution is to use devices based on radio waves. Operating frequencies include 27 MHz or 49 MHz (RC), 900 MHz (radio), 2.4 GHz (IEEE 802.11 -- common band for WiFi and Bluetooth). An example of WiFi interface is on the figure below on the right.).

The choice depends on the amount of data to share and the distance to cover: the lower the frequency, the longer the line-of-sight distance between two communicating devices, but the lower the bandwidth. Some other media have been used. For example, light, e.g., infrared or green light. Acoustic modems are the typical communication for underwater environments.

{% include image-import.html content='<img height="128" alt="Tethered Dartmouth BlueROV." src="/img/20200405_145405-dartmouth-bluerov-tether.jpg">' description='Tether (in yellow) for an underwater robot.' %}

{% include image-import.html content='<a title="AutolycusQ / CC BY-SA (https://creativecommons.org/licenses/by-sa/3.0)" href="https://commons.wikimedia.org/wiki/File:ETRX357_ZigBee_module_with_size_ref.JPG"><img height="128" alt="ETRX357 ZigBee module with size ref" src="https://upload.wikimedia.org/wikipedia/commons/thumb/2/29/ETRX357_ZigBee_module_with_size_ref.JPG/512px-ETRX357_ZigBee_module_with_size_ref.JPG"></a>' description='2.4 GHz radio communication module.' %}

Now that we have seen all the hardware components of a robot, how do we design a robot? Let's see the main characteristics that need to be considered -- [Robot design]({{ site.baseurl }}{% link _modules/mod-1e-design.md %}).
