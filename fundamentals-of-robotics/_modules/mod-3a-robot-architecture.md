---
[//]: # Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved. // SPDX-License-Identifier: CC-BY-SA-4.0
title: Robot Architecture
permalink: /modules/3/robot-architecture
---

In the first two modules, we have seen the robot hardware and the fundamental computational tasks that the robots need to solve together with some of the main algorithms.

How do we combine all these different elements together to have an intelligent autonomous robot?
How do we write software that can work for different robots, as shown in these figures?

<a title="The original uploader was Guzugi at English Wikipedia. / Public domain" href="https://commons.wikimedia.org/wiki/File:Roomba3g.jpg"><img height="256" alt="Roomba3g" src="https://upload.wikimedia.org/wikipedia/commons/thumb/6/6b/Roomba3g.jpg/512px-Roomba3g.jpg"></a>
<a title="Niccolò Caranti / CC BY-SA (https://creativecommons.org/licenses/by-sa/4.0)" href="https://commons.wikimedia.org/wiki/File:ICub_-_Festival_Economia_2018_1.jpg"><img height="256" alt="ICub - Festival Economia 2018 1" src="https://upload.wikimedia.org/wikipedia/commons/thumb/3/39/ICub_-_Festival_Economia_2018_1.jpg/256px-ICub_-_Festival_Economia_2018_1.jpg"></a>
<a title="Dllu / CC BY-SA (https://creativecommons.org/licenses/by-sa/4.0)" href="https://commons.wikimedia.org/wiki/File:Waymo_Chrysler_Pacifica_in_Los_Altos,_2017.jpg"><img height="256" alt="Waymo Chrysler Pacifica in Los Altos, 2017" src="https://upload.wikimedia.org/wikipedia/commons/thumb/d/d3/Waymo_Chrysler_Pacifica_in_Los_Altos%2C_2017.jpg/512px-Waymo_Chrysler_Pacifica_in_Los_Altos%2C_2017.jpg"></a>
<a title="DARPA / Public domain" href="https://commons.wikimedia.org/wiki/File:Bio-inspired_Big_Dog_quadruped_robot_is_being_developed_as_a_mule_that_can_traverse_difficult_terrain.tiff"><img height="256" alt="Bio-inspired Big Dog quadruped robot is being developed as a mule that can traverse difficult terrain" src="https://upload.wikimedia.org/wikipedia/commons/thumb/2/20/Bio-inspired_Big_Dog_quadruped_robot_is_being_developed_as_a_mule_that_can_traverse_difficult_terrain.tiff/lossy-page1-462px-Bio-inspired_Big_Dog_quadruped_robot_is_being_developed_as_a_mule_that_can_traverse_difficult_terrain.tiff.jpg"></a>


In this module, we will delve into the robot **control architecture**, which provides the principles and constraints for organizing the robot' sensor data, decisions, and commands, so that we can get the insights to answer these questions.

Let's start with the [reactive control]({{ site.baseurl }}{% link _modules/mod-3b-reactive.md %}).
