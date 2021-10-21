---
# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved. // SPDX-License-Identifier: CC-BY-SA-4.0
title: Camera
permalink: /modules/1/camera.html
---

<!-- http://gazebosim.org/tutorials?tut=sensor_noise#Cameranoise -->

A camera is composed of light-sensing elements arranged in a 2D array. Each of these elements is a pixel. The images taken from a camera are typically affected by noise, which is a random variation of brightness (pixel values). There can be multiple causes, including electronic noise.

In Gazebo, a Gaussian noise can be added to each pixel of each color channel independently.
In the following the parameters for the camera sensor in `aws-robomaker-jetbot-ros/src/jetbot_description/urdf/robomaker-jetbot.gazebo.xacro`
```
  <gazebo reference="camera_link">
    <sensor type="camera" name="camera">
      <always_on>true</always_on>
      <visualize>$(arg camera_visualize)</visualize>
      <!-- Camera characteristics -->
      <camera>
          <horizontal_fov>2.79253</horizontal_fov>
          <image>
              <width>1280</width>
              <height>720</height>
              <format>R8G8B8</format>
          </image>
          <clip>
              <near>0.01</near>
              <far>100</far>
          </clip>
          <noise>
            <type>gaussian</type>
            <mean>0.0</mean>
            <stddev>0.07</stddev>
          </noise>
      </camera>
      <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
        <!-- Camera characteristics -->
        <hackBaseline>0.07</hackBaseline>
        <distortionK1>0.0</distortionK1>
        <distortionK2>0.0</distortionK2>
        <distortionK3>0.0</distortionK3>
        <distortionT1>0.0</distortionT1>
        <distortionT2>0.0</distortionT2>
        
        <!-- ROS related parameters -->
        <alwaysOn>true</alwaysOn>
        <updateRate>30.0</updateRate>
        <cameraName>camera</cameraName>
        <frameName>camera_link</frameName>
        <imageTopicName>rgb/image_raw</imageTopicName>
        <cameraInfoTopicName>rgb/camera_info</cameraInfoTopicName>
      </plugin>
    </sensor>
  </gazebo>
```

There are a number of parameters that determine the sensor properties, including the field of view, the resolution, how objects are rendered in the image (`near` and `far`). Similarly to the LiDAR, the noise is Gaussian with `mean` and `stddev`.  The short video shows such a noise, typically called salt and pepper (`stddev` was set to 0.2).
{% include video-file.html url="/img/camera-noise" %}

There are also a number of other parameter that model the lens of a camera, which introduces a distortion on the image.
See the [camera sensor noise model](http://gazebosim.org/tutorials?tut=sensor_noise#Cameranoise) in Gazebo for more information.

Note that a number of other disturbances can be present but are not modeled in the main Gazebo repository, for example, haze. 

-------
 [Let's assess the understanding about sensor characteristics and noise.]({{ site.baseurl }}{% link _modules/mod-1e-assessment.md %}).

{% capture ref %}
{% bibliography --cited %}
{% endcapture %}
{% include references.html ref=ref %}
