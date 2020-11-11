---
[//]: # Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved. // SPDX-License-Identifier: CC-BY-SA-4.0
title: Robot Sensors
permalink: /modules/1/robot-sensors
---

Sensors are devices to measure physical properties.
They can be classified in two categories:
1. **Proprioceptive**: measure properties of the robot itself.
2. **Exteroceptive**: measure properties of the surrounding.

Another categorization is determined by whether they emit energy into the environment -- **active**  sensors -- or not -- **passive** sensors.

Here a number of passive proprioceptive sensors that are common in mobile robots.

{% include two-columns.html description='A rotary **encoder** provides **rotation** information, which can be used to determine displacement, velocity, or angle. Such a sensor, for example, is present on cars to measure the distance traveled.' media='<a title="Sidehack at English Wikibooks / Public domain" href="https://commons.wikimedia.org/wiki/File:Incremental_directional_encoder.gif"><img width="128" alt="Incremental directional encoder" src="https://upload.wikimedia.org/wikipedia/commons/1/1e/Incremental_directional_encoder.gif"></a> <a title="Joao Paulo Chagas / CC BY (https://creativecommons.org/licenses/by/4.0)" href="https://commons.wikimedia.org/wiki/File:Encoder_incremental_Dynapar_B58N.jpg"><img width="96" alt="Encoder incremental Dynapar B58N" src="https://upload.wikimedia.org/wikipedia/commons/a/a1/Encoder_incremental_Dynapar_B58N.jpg"></a>' position="right" %}

----------
{% capture text_imu %}
**Inertial Measurement Unit (IMU)** is a proprioceptive sensor which provides **angular velocity and linear acceleration** measurements, which can be used to estimate the position and orientation of a robot. Micro Electro-Mechanical Systems (MEMS) IMUs have become very popular and widely used in electronics systems (e.g. mobile phone) due to their low-cost and light-weight. However, the measurements are corrupted by noises and biases.
{% endcapture %}
{% capture fig_imu %}
<a title="SparkFun / CC BY (https://creativecommons.org/licenses/by/2.0)" href="https://commons.wikimedia.org/wiki/File:SparkFun_6DoF-IMU-Digital-Combo-Board_ITG3200%2BADXL345_10121-01d.jpg"><img width="128" alt="SparkFun 6DoF-IMU-Digital-Combo-Board ITG3200+ADXL345 10121-01d" src="https://upload.wikimedia.org/wikipedia/commons/5/5d/SparkFun_6DoF-IMU-Digital-Combo-Board_ITG3200%2BADXL345_10121-01d.jpg"></a>
{% endcapture %}
{% include two-columns.html description=text_imu media=fig_imu position="right" %}

----------
{% include two-columns.html description='**Global Positioning System (GPS) sensor** uses triangulation from satellites to determine **geolocation and time**. Usual errors for consumer GPS, deriving from, e.g., clock drift, are in the order of tens of meters. Poor GPS signal reception can happen in cities with tall buildings, because of shadowing and multipath effects.' media='<a title="Éric Chassaing / CC BY-SA (https://creativecommons.org/licenses/by-sa/3.0)" href="https://commons.wikimedia.org/wiki/File:Geolocation.png"><img width="160" alt="Geolocation" src="https://upload.wikimedia.org/wikipedia/commons/thumb/4/4f/Geolocation.png/512px-Geolocation.png"></a><a title="oomlout / CC BY-SA (https://creativecommons.org/licenses/by-sa/2.0)" href="https://commons.wikimedia.org/wiki/File:Adafruit_GPS_Module_Breakout.jpg"><img width="128" alt="Adafruit GPS Module Breakout" src="https://upload.wikimedia.org/wikipedia/commons/thumb/8/8f/Adafruit_GPS_Module_Breakout.jpg/512px-Adafruit_GPS_Module_Breakout.jpg"></a>' position="right" %}


Exteroceptive active sensors that measure distance to obstacles include:

{% include two-columns.html description='**LiDAR (Light Detection and Ranging)** sensor emits pulses of laser beams into the surroundings and measures the time for each pulse to bounce back to calculate the distance travelled. LiDAR creates real-time 3D representation of the environment both at day and night-time, possibly better at night without the interference of the sun.' media='<a title="sleepisfortheweak / Public domain" href="https://commons.wikimedia.org/wiki/File:LIDARanim.gif"><img width="128" alt="LIDARanim" src="https://upload.wikimedia.org/wikipedia/commons/1/1b/LIDARanim.gif"></a><a title="APJarvis / CC BY-SA (https://creativecommons.org/licenses/by-sa/4.0)" href="https://commons.wikimedia.org/wiki/File:Velodyne_ProductFamily_BlueLens_32GreenLens.png"><img width="128" alt="Velodyne ProductFamily BlueLens 32GreenLens" src="https://upload.wikimedia.org/wikipedia/commons/thumb/3/3d/Velodyne_ProductFamily_BlueLens_32GreenLens.png/512px-Velodyne_ProductFamily_BlueLens_32GreenLens.png"></a>' position="left" %}

----------
{% include two-columns.html description='**SONAR (Sound Navigation and Ranging)** uses acoustic beams to create a map of the surrounding environment using the time-of-flight technique. Sonar is specifically useful for underwater exploration as sound waves travel farther than radar and light waves in water. There are different types of sonars -- imaging sonar, mechanical scanning profiling sonar, echo-sounder etc.' media='<a title="SparkFun / CC BY (https://creativecommons.org/licenses/by/2.0)" href="https://commons.wikimedia.org/wiki/File:Maxbotix_Ultrasonic-Range-Finder_LV-MaxSonar-EZ0.jpg"><img width="64" alt="Maxbotix Ultrasonic-Range-Finder LV-MaxSonar-EZ0" src="https://upload.wikimedia.org/wikipedia/commons/2/27/Maxbotix_Ultrasonic-Range-Finder_LV-MaxSonar-EZ0.jpg"></a><a title="Georg Wiora (Dr. Schorsch) / CC BY-SA (http://creativecommons.org/licenses/by-sa/3.0/)" href="https://commons.wikimedia.org/wiki/File:Sonar_Principle_EN.svg"><img width="192" alt="Sonar Principle EN" src="https://upload.wikimedia.org/wikipedia/commons/thumb/0/07/Sonar_Principle_EN.svg/1000px-Sonar_Principle_EN.svg.png"></a>' position="left" %}

----------
{% include two-columns.html description='**RADAR (RAdio Detection And Ranging)** system emits radio waves which reflect back to the radar when they come into contact with any surrounding objects. Radar can measure the range from time-of-flight (similar to LiDAR) and speed of an object using Doppler shift.' media='<a title="Hihiman / CC0" href="https://commons.wikimedia.org/wiki/File:Distance_Radar_DR-1DHP_61_GHz_Symeo.jpg"><img width="96px" alt="Distance Radar DR-1DHP 61 GHz Symeo" src="https://upload.wikimedia.org/wikipedia/commons/thumb/5/5c/Distance_Radar_DR-1DHP_61_GHz_Symeo.jpg/512px-Distance_Radar_DR-1DHP_61_GHz_Symeo.jpg"></a>' position="left" %}


Exteroceptive passive sensors are energy efficient because they measure natural emissions from the environment:

{% include two-columns.html description='**RGB Camera** is one of the cheapest, small, light-weight, and energy-efficient sensors. If it is configured as a pair, also called stereo camera, it can create a 3D map from a single stereo frame. The underlying assumption is that the environment has sufficient illumination with enough texture.' media='<div style="position: relative; width: 256px; height: 85px; overflow: hidden; float: left; "><a title="NASA/JPL-Caltech/MSSS/ASU" href="https://www.jpl.nasa.gov/spaceimages/details.php?id=PIA16799"><img width="256" alt="Typical commercial color cameras." src="https://www.jpl.nasa.gov/spaceimages/images/largesize/PIA16799_hires.jpg" style="position: absolute; bottom: -90px; left: 0px"></a></div><a title="the Raspberry Pi Foundation / CC BY-SA (https://creativecommons.org/licenses/by-sa/4.0)" href="https://commons.wikimedia.org/wiki/File:Raspberry_Pi_Camera_Module_v2_with_ribbon.jpg"><img width="96" alt="Raspberry Pi Camera Module v2 with ribbon" src="https://upload.wikimedia.org/wikipedia/commons/thumb/7/70/Raspberry_Pi_Camera_Module_v2_with_ribbon.jpg/512px-Raspberry_Pi_Camera_Module_v2_with_ribbon.jpg"></a>' position="left" %}

----------
{% include two-columns.html description='**RGB-D Camera** provides per-pixel depth or distance from the camera along with the RGB image. RGB-D camera projects infrared light pattern onto the scene and calculates the depth information using the known projected pattern. However, it can only work indoors cause sunlight will washout the speckle pattern.' media='<a title="Kinect_Sensor_at_E3_2010_(front).jpg: James Pfaff (litheon)
derivative work: AlphathonTM (talk)
Captions by Dancter (talk) (added to this version by AlphathonTM (talk)) / CC BY-SA (https://creativecommons.org/licenses/by-sa/3.0)" href="https://commons.wikimedia.org/wiki/File:KinectSensor.png"><img width="192" alt="KinectSensor" src="https://upload.wikimedia.org/wikipedia/commons/thumb/f/fe/KinectSensor.png/256px-KinectSensor.png"></a>' position="left" %}


Sensors are **not perfect**: measurements are **noisy** -- e.g., the distance measurement of a SONAR place in the same location can be different -- and do not describe the whole environment. In addition, sensors cannot be modeled completely. As such, understanding the properties of the sensors is fundamental to explicitly model the uncertainty and not to let a robot fail in a real environment.

How to process the data from the sensors and send commands to the actuators? Let's see the [computation devices]({{ site.baseurl }}{% link _modules/mod-1d-computation.md %}).
