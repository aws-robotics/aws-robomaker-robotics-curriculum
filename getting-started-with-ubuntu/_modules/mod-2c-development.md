---
title: ROS development
permalink: /modules/2/ros-development.html
---

ROS software is organized in **packages** {% cite ros-packages %}, each containing collection of files, including configuration files, executables, source code, etc.
Each package is typically logically organized to provide specific functionalities to robots. For example, there is a package for Simultaneous Localization and Mapping, called 'gmapping'. 

A specific command to check what packages are currently installed is

    class@ubuntu:~$ rospack list

    actionlib /opt/ros/melodic/share/actionlib
    actionlib_msgs /opt/ros/melodic/share/actionlib_msgs
    actionlib_tutorials /opt/ros/melodic/share/actionlib_tutorials
    angles /opt/ros/melodic/share/angles
    bond /opt/ros/melodic/share/bond
    bondcpp /opt/ros/melodic/share/bondcpp
    bondpy /opt/ros/melodic/share/bondpy
    camera_calibration /opt/ros/melodic/share/camera_calibration
    camera_calibration_parsers /opt/ros/melodic/share/camera_calibration_parsers
    camera_info_manager /opt/ros/melodic/share/camera_info_manager
    catkin /opt/ros/melodic/share/catkin
    class_loader /opt/ros/melodic/share/class_loader
    cmake_modules /opt/ros/melodic/share/cmake_modules
    compressed_depth_image_transport /opt/ros/melodic/share/compressed_depth_image_transport
    compressed_image_transport /opt/ros/melodic/share/compressed_image_transport
    control_msgs /opt/ros/melodic/share/control_msgs
    control_toolbox /opt/ros/melodic/share/control_toolbox
    controller_interface /opt/ros/melodic/share/controller_interface
    controller_manager /opt/ros/melodic/share/controller_manager
    controller_manager_msgs /opt/ros/melodic/share/controller_manager_msgs
    cpp_common /opt/ros/melodic/share/cpp_common
    cv_bridge /opt/ros/melodic/share/cv_bridge
    
    ... and many more ...

All the packages installed with the `apt` command are in `/opt/ros/melodic/share` by default.

Let's install some packages to test the ROS installation and see a simulation of a robot in Gazebo, a 3D robotic simulator. In particular, we will get the packages containing the 3D models of a robot called Turtlebot 3 {% cite ros-turtlebot %}.

    sudo apt install ros-melodic-turtlebot3-gazebo ros-melodic-turtlebot3-description

Note that `sudo` is necessary as system folders are modified with the installation through `apt`.

We can run it by typing the following commands

    export SVGA_VGPU10=0 # Only necessary within the Virtual Machine to reduce the GPU instructions and save resources.
    export TURTLEBOT3_MODEL=burger # Specify the model of the robot.
    roslaunch turtlebot3_gazebo turtlebot3_world.launch # Run the simulation.
    
After the last command is run, the terminal will display text and the 3D simulator will appear, as in the following.

![Turtlebot 3 robot in Gazebo](/img/gazebo-turtlebot3.png)

In some cases, when trying to run specific executables an error can appear about package which is not found. For example, 

    Resource not found: turtlebot3_description
    ROS path [0]=/opt/ros/melodic/share/ros
    ROS path [1]=/opt/ros/melodic/share
    The traceback for the exception was written to the log file

"Resource not found" indicates that a package is not found in the paths listed in the environment variable `$ROS_PACKAGE_PATH`. In this specific example `turtlebot3_description` is missing. 
To solve such an issue, the easiest solution is to install the missing package, when it is available. Given the specified missing resource, it is easy to use `apt` and install the missing package: replace '_' with '-' and add as prefix `ros-<version>`. In the case of Ubuntu 18.04, it is `ros-melodic`.

    sudo apt install ros-melodic-turtlebot3-description
    

As of now, we have seen packages that are already existing. What if we want to create a new package? Typically, a workspace is created to contain such custom packages:

    mkdir -p ~/ros_workspace/src

(`-p` creates both directories `ros_workspace` and `src`).

We will discover the exciting world of ROS in the next course. For now, we are continuing in our journey on how to set up the development environment. Before moving forward, [let's assess]({{ site.baseurl }}{% link _modules/mod-2d-assessment.md %}) your understanding of the ROS configuration.


{% capture ref %}
{% bibliography --cited %}
{% endcapture %}
{% include references.html ref=ref %}
