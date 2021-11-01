---
# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved. // SPDX-License-Identifier: CC-BY-SA-4.0
title: Sensors
permalink: /modules/4/sensors.html
---

<!-- http://gazebosim.org/tutorials/?tut=build_robot -->

<!-- http://gazebosim.org/tutorials?tut=ros_gzplugins -->

There are a number of simulated sensors that can be added to a robot. In the first module, we have seen camera and LiDAR. Please look at the [full list of Gazebo plugins that are interfaced with ROS](http://gazebosim.org/tutorials?tut=ros_gzplugins).

Typically, a second URDF file is used to specify the sensors, separate from the URDF describing the physical properties of the robot. For the Waveshare Jetbot, please see the corresponding [URDF file](https://github.com/aws-samples/aws-robomaker-jetbot-ros/blob/main/src/jetbot_description/urdf/jetbot.gazebo.xacro).
The URDF can be modified to add a new sensor inside the `<robot>` element. The main elements that are common for each sensor are:
- the link that the sensor will be attached to, specified in `reference`.
- the specification of the sensor type.
- the specific parameters for each sensor.

For example, to add an IMU:

```
  <gazebo reference="imu_link">
    <gravity>true</gravity>
    <sensor name="imu_sensor" type="imu">
      <always_on>true</always_on>
      <update_rate>100</update_rate>
      <visualize>true</visualize>
      <topic>__default_topic__</topic>
      <plugin filename="libgazebo_ros_imu_sensor.so" name="imu_plugin">
        <topicName>imu</topicName>
        <bodyName>imu_link</bodyName>
        <updateRateHZ>10.0</updateRateHZ>
        <gaussianNoise>0.0</gaussianNoise>
        <xyzOffset>0 0 0</xyzOffset>
        <rpyOffset>0 0 0</rpyOffset>
        <frameName>imu_link</frameName>
        <initialOrientationAsReference>false</initialOrientationAsReference>
      </plugin>
      <pose>0 0 0 0 0 0</pose>
    </sensor>
  </gazebo>
```

In that case, the robot model requires the addition of the `imu_link` and a joint connecting to the `base_link`.

In general, all plugins are open source and the implementation can be found in the repository; for example, [the implementation of the IMU plugin](http://docs.ros.org/en/melodic/api/gazebo_plugins/html/gazebo__ros__imu__sensor_8cpp_source.html).

---
Let's see in the [next unit]({{ site.baseurl }}{% link _modules/mod-4d-assessment.md %}) our understanding on robot modeling in Gazebo.
