---
# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved. // SPDX-License-Identifier: CC-BY-SA-4.0
title: Actor
permalink: /modules/3/actor.html
---

Besides creating a world with walls and objects, it is possible to have **actors** in the world, i.e., animated models that are not affected by the physics engine but can be detected by cameras and depth sensor. One of the reasons why actors are not affected by the physics engine (e.g., collision or gravity) is to simplify the simulation. 

Here we highlight the main components to include an actor in the world and refer to the [complete tutorial provided in Gazebo](http://gazebosim.org/tutorials?tut=actor&cat=build_robot).

An actor can be added in the world SDF file as in the following example that contains just a ground plane, a light source, and an actor.

```
<?xml version="1.0" ?>
<sdf version="1.6">
   <world name="default">
      <!-- A ground plane -->
      <include>
         <uri>model://ground_plane</uri>
      </include>
      <!-- A global light source -->
      <include>
         <uri>model://sun</uri>
      </include>
      <actor name="actor">
        <skin>
          <filename>walk.dae</filename>
        </skin>
        <animation name="animation">
          <filename>walk.dae</filename>
          <interpolate_x>true</interpolate_x>
        </animation>
        <script>
          <trajectory id="0" type="walking">
            <waypoint>
              <time>0</time>
              <pose>0 2 0 0 0 -1.57</pose>
            </waypoint>
            <waypoint>
              <time>2</time>
              <pose>0 -2 0 0 0 -1.57</pose>
            </waypoint>
            <waypoint>
              <time>2.5</time>
              <pose>0 -2 0 0 0 1.57</pose>
            </waypoint>
            <waypoint>
              <time>7</time>
              <pose>0 2 0 0 0 1.57</pose>
            </waypoint>
            <waypoint>
              <time>7.5</time>
              <pose>0 2 0 0 0 -1.57</pose>
            </waypoint>
          </trajectory>
        </script>
      </actor>
    </world>
</sdf>
```

The key elements within `<actor>` are (please see [the full SDF specification of `<actor>`](http://sdformat.org/spec?ver=1.6&elem=actor)):
- `<skin>` to specify the visual appearance of the actor and the skeleton (links and joints) that moves the actor, typically in `.dae` file format.
- `<animation>` to specify an animation for the skin, compatible with the skeleton specified in `<skin>`, typically in `.dae` file format.
- `<script>` where a trajectory can be specified as a series of waypoints.

The example will move the robot the person on a line back and forth. 

<img src="https://github.com/osrf/gazebo_tutorials/raw/master/actor/files/full_animation.gif" alt="Person moving back and forth through actor [from Gazebo tutorial]." width="500" height="333">

Multiple actors can be added, changing the `name` of the actor and the trajectory.

Note that a plugin could be implemented in C++ and included so that the trajectory can be achieved programmatically, without hardcoding waypoints. More details in [the tutorial](http://gazebosim.org/tutorials?tut=actor&cat=build_robot).

-------
[Let's assess]({{ site.baseurl }}{% link _modules/mod-3f-assessment.md %}) our understanding of Gazebo world modeling.

