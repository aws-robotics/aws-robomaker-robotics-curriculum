---
# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved. // SPDX-License-Identifier: CC-BY-SA-4.0
title: AWS RoboMaker WorldForge
permalink: /modules/3/aws-worldforge.html
---

Creating a new world that is realistic from scratch can be very time-consuming. AWS provides WorldForge, a tool that can automatically create randomized realistic simulation worlds. Currently, it supports indoor home environments with furniture. 

There are a number of parameters that can be set, including the aspect ratio, the rooms, their connections, flooring, wall paintings, and furniture. Once those parameters are set, a number of worlds will be generated with some variations in arrangement. For example, this is living/dining room with some variations.

<img src="https://d2908q01vomqb2.cloudfront.net/da4b9237bacccdf19c0760cab7aec4a8359010b0/2020/08/14/Screen-Shot-2020-08-13-at-10.29.29-PM.png" alt="AWS WorldForge example" width="500" height="333">



This will allow testing in more randomized realistic environments, as well as training. To get started, access your AWS account and search for WorldForge. There is an intuitive graphical user interface allowing the creation of the world. Please see the [full guide describing the process of creating a new world with AWS WorldForge](https://aws.amazon.com/blogs/aws/aws-announces-worldforge-in-aws-robomaker/). 

Once generated, from the AWS RoboMaker sidebar, select "Worlds". The generated world should appear in that page. That world can be selected and an export job can be created. After selecting the S3 bucket where to store the export job, access your S3 bucket. You will find an archive containing the exported world file, which includes the .world file as well as the models used for that world. Here an example of the exported archive:

```
.
├── aws_robomaker_worldforge_shared_models
│   ├── CMakeLists.txt
│   ├── LICENSE
│   ├── README.md
│   ├── models
│   └── package.xml
└── aws_robomaker_worldforge_worlds
    ├── CMakeLists.txt
    ├── LICENSE
    ├── README.md
    ├── launch
    ├── package.xml
    └── worlds
```

That world can then be used in Gazebo in the same way as other worlds.


-------
Let's see a new element that can be added to the world to make the simulation more interesting, that is animated models, such as humans, in [the next unit]({{ site.baseurl }}{% link _modules/mod-3e-actor.md %}).


<!-- http://gazebosim.org/tutorials?cat=build_world&tut=building_editor -->


<!-- https://aws.amazon.com/robomaker/faqs/#Simulation_WorldForge -->

<!-- https://aws.amazon.com/robomaker/resources/ -->
