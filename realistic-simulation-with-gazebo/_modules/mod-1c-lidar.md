---
# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved. // SPDX-License-Identifier: CC-BY-SA-4.0
title: LiDAR
permalink: /modules/1/lidar.html
---

Let's see the simulated LiDAR sensor which we already used in Course 2.
The simulated LiDAR in Gazebo introduces  a Gaussian noise which is typically present in actual LiDAR sensors. Gaussian noise is caused by things such as the interaction of the signal with specific materials.

<!-- http://gazebosim.org/tutorials?tut=guided_i3 --> 

In the following the parameters for the LiDAR sensor in `aws-robomaker-jetbot-ros/src/jetbot_description/urdf/robomaker-jetbot.gazebo.xacro`
```
  <gazebo reference="lidar_link">
    <sensor type="ray" name="lds_lfcd_sensor">
      <pose>0 0 0 0 0 0</pose>
      <visualize>$(arg laser_visualize)</visualize>
      <update_rate>5</update_rate>
      <ray>
        <!-- properties of the LiDAR -->
        <scan>
          <horizontal>
            <samples>428</samples>
            <resolution>0.84</resolution>
            <min_angle>0</min_angle>
            <max_angle>6.28319</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>0.1</min>
          <max>8</max>
          <resolution>0.01</resolution>
        </range>
        <noise>
          <type>gaussian</type>
          <mean>0.0</mean>
          <stddev>0.01</stddev>
        </noise>
      </ray>
      <!-- ROS related pluugin for publishing -->
      <plugin name="gazebo_ros_lds_lfcd_controller" filename="libgazebo_ros_laser.so">
        <topicName>scan</topicName>
        <frameName>lidar_link</frameName>
      </plugin>
    </sensor>
  </gazebo>
```

There are a number of parameters to control the LiDAR parameters, in particular all of those included in `<ray>`. Some of these parameters were seen in the previous course, such as the resolution, minimum and maximum angle for the field of view. The other parameters that controls the noise characteristics of the measurements are under `noise`: `mean` and `stddev` are the mean and standard deviation of the Gaussian distribution. The noise, sampled from the Gaussian distribution, is added to the measurement. Increasing `stdev` for the noise results in more noisy measurements. In the following figure and video, an example where the noise is 0 and the noise is very high (standard deviation of 1.0). 

{% include image.html url="/img/laser-no-noise.png" description="LiDAR without noise." %}

{% include video-file.html url="/img/laser-noise" %}

To set the proper values, it is necessary to collect data with the LiDAR sensor and observe the distribution of the measurements. Characterizing properly that noise is important to ensure that any behavior relying on LiDAR data accounts for the related noise. 
See the [sensor noise model](http://gazebosim.org/tutorials?tut=sensor_noise#Ray(laser)noise) in Gazebo for more details.


---
Now that we have seen the LiDAR, let's see the other sensor present on the Waveshare Jetbot, the camera, in the [next unit]({{ site.baseurl }}{% link _modules/mod-1d-camera.md %}).

{% capture ref %}
{% bibliography --cited %}
{% endcapture %}
{% include references.html ref=ref %}
