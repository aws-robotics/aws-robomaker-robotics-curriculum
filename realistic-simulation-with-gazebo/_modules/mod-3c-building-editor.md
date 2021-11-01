---
# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved. // SPDX-License-Identifier: CC-BY-SA-4.0
title: Modifying the world
permalink: /modules/3/modifying-world.html
---

Gazebo graphical user interface provides access to the available models, which can be selected, and added to the world. After adding the models, the world can be saved in the appropriate directory and the ROS launch file modified to load that world file.

{% include image.html url="/img/gazebo-gui.png" description="Gazebo Graphical User Interface." %}

To navigate the scene, keep the mouse left click pressed in the part of the screen with the simulated world and move the mouse around. To change the orientation, press also shift. To zoom in and out, use the mouse scrolling wheel.

To add a new object of an existing model, click on the "Insert" tab. There are a number of models listed. Click on the one to add to the scene. Now, when moving the mouse cursor in the scene, the object appears in the simulated world. To confirm the location click the mouse left button. To cancel, instead, the insertion, press esc. There are a number of other tools helping to move or rotate those objects: on the top left, after the mouse cursor icon, there is the translation, rotation, and resizing (when possible).

----
For 2D environments, Gazebo provides also a floorplan editor, where walls, windows, open doors, can be added, and textures applied to the wall. Please see the [Gazebo tutorial on the building editor](http://gazebosim.org/tutorials?cat=build_world&tut=building_editor). To access that, click on "Edit->Building Editor".

{% include image.html url="/img/gazebo-building.png" description="Gazebo building editor." %}


----
Another modification that is useful to know is by using images. For example, the track in the previous module was generated by an image. 

Here the structure of the model

```
models/
└── track
    ├── materials
    │   ├── scripts
    │   │   └── track.material
    │   └── textures
    │       └── track.png
    ├── model.config
    └── model.sdf
```

`track.png` is the image file that specifies the track. That can be modified with any image manipulation software.
Its `model.sdf` specify the physical properties of it, as well as the corresponding size in meters.

```
<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="track">
    <static>true</static>
    <link name="track_link">
      <collision name="track_collision">
        <geometry>
          <plane>
            <normal>0 0 1</normal>
            <size>4 4</size>
          </plane>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>100</mu>
              <mu2>50</mu2>
            </ode>
          </friction>
        </surface>
      </collision>
      <visual name="track_visual">
        <cast_shadows>false</cast_shadows>
        <geometry>
                <plane>
            <normal>0 0 1</normal>
            <size>4 4</size>
          </plane>
        </geometry>
              <material>
          <script>
            <uri>model://track/materials/scripts</uri>
            <uri>model://track/materials/textures</uri>
            <name>track</name>
          </script>
        </material>
      </visual>
    </link>
  </model>
</sdf>
```

To add such a model to the world, in the `.world` file, the model can be included.

```
     <include>
      <uri>model://track</uri>
      <pose> 0 0 0 0 0 -1.54</pose>
    </include>
```

Please see the [repository](https://github.com/aws-samples/aws-robomaker-jetbot-ros) which contains all the files. Once the image is modified, the new track will be loaded, when running again the simulation with the corresponding world file.

-------
We have seen some custom-made way to modify the world. There are many others. In the [next unit]({{ site.baseurl }}{% link _modules/mod-3d-worldforge.md %}), we will see a tool provided by AWS to automatically generate worlds.