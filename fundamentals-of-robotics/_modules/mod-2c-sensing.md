---
# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved. // SPDX-License-Identifier: CC-BY-SA-4.0
title: Sensing
permalink: /modules/2/sensing.html
---

We took a look at the [models]({{ site.baseurl }}{% link _modules/mod-2b-models.md %}). In practice, the robot needs to sense the surrounding to purposefully act in the environment.

In general, achieving robust sensing is a difficult problem, because:
1. The sensing process might be indirect: it might not be possible to directly measure the quantity of interest. For estimating the traveled distance -- process called odometry -- for example, wheel encoders on wheeled robots are used to measure rotations, and then infer the location. We cannot directly measure location of the robot.
2. Sensor measurements are noisy.
3. Measurements may be intermittent.

Let's take a look at how sensor data can be used.

## Sensor data processing

Different sensors have different level of computation required. Readings from contact sensors and wheel odometry for example require simple conditional checks and math integration. Data from 3D LiDAR and cameras, instead, require high computation to extract the information about obstacles and to recognize objects, necessary for the robot to accomplish its task.
We will briefly see what this processing entails for each reading from those sensors.

{% capture text_lidar %}
### 3D LiDAR
At each time step, a 3D LiDAR returns a 3D point cloud of a portion of the environment. The covered part of the environment depends on the horizontal and vertical field of view. The angular resolution determines the density of the point cloud.
Raw data can be used to find the closest points. This information could be used to stop the robot for example.
The 3D point cloud has much more information that can be extracted. Objects, like moving and parked cars, are visible to our eyes, as can be seen from the image on the right.
Clustering techniques can be used to identify points that are part of the same object. Object classification can be performed by matching 3D models from a database.
{% endcapture %}
{% capture img_lidar %}
{% include image.html url="/img/lidar.gif" description="Example of 3D point cloud from LiDAR." %}
{% endcapture %}
{% include two-columns.html description=text_lidar media=img_lidar position="right" %}


{% capture text_camera %}
### Cameras
Camera images are data-heavy. A 1080p RGB camera has 1920x1080 pixels per each channel. Considering 30 frames per second, any software that process such images receives about 186 millions of values per second.
What a robot needs though is not the full interpretation of the full images.
From each image frame, computer vision techniques can be used to recognize objects. Many of the current methods rely on deep learning, thanks to its success in many domains, including recognizing pedestrians, cars, and traffic signs.
Image frames can be used also to extract depth or egomotion with multiple views and/or stereo vision.

{% endcapture %}
{% capture img_camera %}
{% include image-import.html content='<a title="Comunidad de Software Libre Hackem [Research Group] / CC BY-SA (https://creativecommons.org/licenses/by-sa/3.0)" href="https://commons.wikimedia.org/wiki/File:Computer_vision_sample_in_Sim%C3%B3n_Bolivar_Avenue,_Quito.jpg"><img width="512" alt="Computer vision sample in Simón Bolivar Avenue, Quito" src="https://upload.wikimedia.org/wikipedia/commons/thumb/3/3b/Computer_vision_sample_in_Sim%C3%B3n_Bolivar_Avenue%2C_Quito.jpg/512px-Computer_vision_sample_in_Sim%C3%B3n_Bolivar_Avenue%2C_Quito.jpg"></a>' description="Example of object recognition." %}
{% endcapture %}
{% include two-columns.html description=text_camera media=img_camera position="right" %}


## State estimation

The information coming and processed from the sensors can be integrated over time to estimate the robot' **state** and that of the surrounding. The state, as seen previously, is a set of quantities (e.g., position, orientation) that describes the robot's motion over time and the obstacles in the environment.

To achieve better sensing, information from multiple sensors can be combined together so that the robot has better information about the world. This process is called **sensor fusion**.

The main techniques used to solve state estimation are based on **probabilistic robotics**.
Depending on the available information and sensors, different subproblems can be identified.

### Localization

The **localization** problem looks at answering the question "where is the robot?", given the sensor measurements and a map of the environment.
Intuitively, given the state space, i.e., any location where the robot can be, the location on where the robot is can be represented as a probability distribution, also called **belief**.

If the robot doesn't know where it is initially, the problem is called **global localization**.
The following short video depicts a global localization problem for a wheeled robot equipped with encoders, providing **odometry** information, and a sensor capable of detecting doors.
{% include video-file.html url="/img/localization" %}

In general, even if localized, the robot needs to perform the same steps for continuously localizing. This problem is called **tracking**. The reason the same process needs to run is that sensors that can provide an estimate on the position, such as wheel odometry, are subject to **drift**. Wheels can have unequal wheel diameter, different contact points with the floor and variable friction which can lead to slipping. An example of drift for a wheeled robot is shown in the following video.

  {% include video-file.html url="/img/odometry" %}

Exteroceptive sensors can correct such a drift. For example, with a prior knowledge of the map of the environment and with a LiDAR, corrections can be applied to the pose of the robot so that the current LiDAR reading matches with the map.
Note to properly apply the correction to the pose of the robot, it is important to have the coordinate systems of the sensors and the robot body and their relation, as discussed in the previous unit. Encountering the same place and correcting the pose means to perform what is called **loop closure**. An example is shown in the figure below, where initially the robot has an angle offset and then aligning laser sensor measurements with the map allows the robot to correct its pose.

{% include image.html url="/img/laser-before-after.png" description="Drifted pose of the robot (left) corrected with the LiDAR (right)." %}


### Mapping

How can the robot determine how the world looks like? This is the **mapping** problem. Given its pose, the robot needs to use the sensor measurements to create a map of the environment.

Consider for example a robot that can has location information, for example with GPS, but doesn't have the map of the surrounding, e.g., the positions of the landmarks. The intuition behind the mapping process can be observed in the following video.
  {%include video-file.html url="/img/mapping" %}

The map of the environment can be represented in different ways. Three basic types are the following:

{% capture text_landmark %}
**Landmark or feature maps**, which we have seen in the simple examples in the videos, are a vector of landmarks, typically represented with their pose in the environment. They are very useful as they have a compact representation. This allows an efficient update of the map. The chosen landmarks should be unique in the environment so that it is easy for the robot to recognize.
{% endcapture %}
{% capture img_landmark %}
{% include image.html url="/img/map-landmark.svg" description="Landmark map." %}
{% endcapture %}
{% include two-columns.html description=text_landmark media=img_landmark position="right" %}

{% capture text_grid %}
**Grid maps** represent the environment with fixed-size shapes (e.g., cells), which can be either occupied, freespace, or unknown. The size of the cell is determined by the selected **resolution** of the grid. The higher the resolution, the more cells for representing an area. In general, the grid map requires more memory and might be not feasible to use for very large environments.
{% endcapture %}
{% capture img_grid %}
{% include image.html url="/img/map-grid.png" description="Grid map." %}
{% endcapture %}
{% include two-columns.html description=text_grid media=img_grid position="right" %}

{% capture text_topology %}
A **topology map** connects objects and locations through a graph. An example is the interconnection between rooms. This can be useful to reduce the computation needed to reason on such a representation.
{% endcapture %}
{% capture img_topology %}
{% include image.html url="/img/map-semantic.svg" description="Topology map." %}
{% endcapture %}
{% include two-columns.html description=text_topology media=img_topology position="right" %}

### Simultaneous Localization and Mapping

If the robot doesn't know where it is and how the world looks like, then the **Simultaneous Localization and Mapping (SLAM)** problem needs to be solved.
For example, a robotic vacuum cleaner doesn't have the map of the house when turned on for the first time. As initial location, a global reference frame is arbitrary set and its origin is where the robot started. As the robot moves it can map of the environment given the pose with respect to that global reference frame. An example for a robot with LiDAR is shown in the following video.

  {% include video-file.html url="/img/slam" %}

This is a "chicken and egg" problem, as for localizing a map is necessary, but for creating a map a pose is needed.

### Current trends in SLAM - current trends

A large part of the research community is working on this technology to make it more robust for robots to operate in any environment. In particular, there are some sensors/systems that are particularly of interest for many applications.

{% capture text_vio %}
**Visual-Inertial Odometry/SLAM (VIO/VI-SLAM)** systems have shown enormous success in a wide range of robotic applications including drone-based navigation, virtual and augmented reality, and autonomous driving. Visual and inertial sensors are low-cost, yet have accurate estimates due to their complementary sensing capabilities. Filtering and optimization based approaches are the two main categories to solve a visual-inertial navigation problem with open problems like efficient management of computational resources and failure recovery mechanisms. Recently, learning-based methods have also shown promising results making it an active research area. Though visual and inertial sensors are ubiquitous, there are certain requirements (some of them coming from sensory limitations), -- for example, having sufficient illumination and preferably constant brightness, enough texture in the scene --  for VI-SLAM to work successfully.
{% endcapture %}
{% capture video_vio %}
  {% include video-youtube.html url="https://www.youtube.com/embed/Gh5pAT1o2V8" %}
{% endcapture %}
{% include two-columns.html description=text_vio media=video_vio position="right" %}

{% capture text_lidar %}
Currently, **LiDAR** is used as the main sensor in Autonomous car navigation due to its capability of providing real-time accurate dense 3D model of the environment. Often times, IMU measurements are used to correctly align pointcloud generated by the high frequency LiDAR range measurements. However, LiDAR could be expensive, costing up to US$80,000 and yet the ability to detect objects could be affected  in the presence of fog or snow.
{% endcapture %}
{% capture video_lidar %}
  {% include video-youtube.html url="https://www.youtube.com/embed/jcKnb65wpWA" %}
{% endcapture %}
{% include two-columns.html description=text_lidar media=video_lidar position="right" %}

With the robot capable of localizing in the environment and having a sense of the surrounding, it can plan to achieve a task. Let's take a look at how the robot can achieve [planning]({{ site.baseurl }}{% link _modules/mod-2d-planning.md %}).
