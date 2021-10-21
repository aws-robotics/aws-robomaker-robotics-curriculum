---
# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved. // SPDX-License-Identifier: CC-BY-SA-4.0
title: Gazebo world
permalink: /modules/3/gazebo-world.html
---

A Gazebo world is specified in Simulation Description Format (SDF) -- an XML format describing environments and objects in robotic simulators. For more information about the format, please see the [SDF webpage](http://sdformat.org/), which includes the specifications.

Any element of the world is specified within the `world` tag. There are a number of elements that specify the graphical user interface, the physics engine parameters, and the appearance of the scene. Then, the typical structure of an SDF file for a simulated world includes objects in the environment. There are a number of models that are included by default. The Open Source Robotics Foundation provides a number of [models for Gazebo](https://github.com/osrf/gazebo_models).

Here an example of file with an empty world ([`empty.world`](https://github.com/cbuscaron/robomaker-jetbot/blob/main/world/empty_world.world)), and just the robot. Note that, while the robot is added in the SDF, typically that is not necessary when working with ROS, as the robot is spawned in the running simulator.

```
<?xml version='1.0'?>
<sdf version="1.4">
    <world name="default">
      <!-- Graphical user interface configuration. -->
      <gui>
        <camera name="default_camera">
          <pose>0.8 -0.75 0.35 0 0.25 2.35</pose>
        </camera>
      </gui>

      <!-- Physics engine -->
      <physics type="ode">
        <real_time_update_rate>1000.0</real_time_update_rate>
        <max_step_size>0.001</max_step_size>
        <real_time_factor>1</real_time_factor>
        <ode>
        <solver>
          <type>quick</type>
          <iters>150</iters>
          <precon_iters>0</precon_iters>
          <sor>1.400000</sor>
          <use_dynamic_moi_rescaling>1</use_dynamic_moi_rescaling>
        </solver>
        <constraints>
          <cfm>0.00001</cfm>
          <erp>0.2</erp>
          <contact_max_correcting_vel>2000.000000</contact_max_correcting_vel>
          <contact_surface_layer>0.01000</contact_surface_layer>
        </constraints>
        </ode>
      </physics>

      <!-- Scene appearance -->
      <scene>
        <ambient>0.4 0.4 0.4 1</ambient>
        <background>0.7 0.7 0.7 1</background>
        <shadows>true</shadows>
      </scene>

        <!-- A global light source -->
      <include>
        <uri>model://sun</uri>
      </include>

      <!-- A ground plane -->
      <include>
        <uri>model://ground_plane</uri>
      </include>

      <!-- custom waveshare robomaker jetbot model -->
      <include>
        <uri>model://robomaker-jetbot</uri>
      </include>
    </world>
</sdf>
```

The included models must be existing locally. For custom models, it is important to set the environment variable `GAZEBO_MODEL_PATH` so that it includes the path to that model. Any model is typically in its folder with the same name with a `.config` file  and a `.sdf` file. The first one includes metadata, such as the author name and the description. Please see the [ground plane example](https://github.com/osrf/gazebo_models/tree/master/ground_plane). The latter specifies the physical properties of the object, such as the collision and the visual properties. Simple geometrical shapes are directly supported. Another way to build complex objects is by using an external Computer-Aided Design (CAD) software for 3D modeling.

-------
With a first general understanding of the Gazebo world and its models, let's see some ways to modify the world in the [next unit]({{ site.baseurl }}{% link _modules/mod-3c-building-editor.md %}).



<!-- http://gazebosim.org/tutorials?cat=build_world&tut=building_editor -->


<!-- https://aws.amazon.com/robomaker/faqs/#Simulation_WorldForge -->

<!-- https://aws.amazon.com/robomaker/resources/ -->
<!-- The first modification of the world is by directly using the Gazebo graphical user interface and the available models that can be added in the world and saving such modifications to a world file. Models that are loaded beforehand can be added in the world. -->
