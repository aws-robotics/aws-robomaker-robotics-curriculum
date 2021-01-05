---
title: Custom ROS packages fix
permalink: /modules/4/fix.html
---

We have discovered how to build a ROS package. 

However, if we try to run immediately the new simulation 

    roslaunch turtlebot_gazebo turtlebot_world.launch

We will find the following error:

    Resource not found: The following package was not found in <arg default="$(find xacro)/xacro '$(find turtlebot_description)/robots/$(arg base)_$(arg stacks)_$(arg 3d_sensor).urdf.xacro'" name="urdf_file"/>: turtlebot_description
    ROS path [0]=/opt/ros/melodic/share/ros
    ROS path [1]=/home/class/ros_workspace/install/turtlebot_simulator/share
    ROS path [2]=/home/class/ros_workspace/install/turtlebot_stdr/share
    ROS path [3]=/home/class/ros_workspace/install/turtlebot_stage/share
    ROS path [4]=/home/class/ros_workspace/install/turtlebot_gazebo/share
    ROS path [5]=/opt/ros/melodic/share
    The traceback for the exception was written to the log file

In this unit we will see a couple of common errors you might encounter in setting up a custom package so that you can identify them and try a couple of options.

Remembering from the previous module on ROS, the first step is to try to see if the missing resource, `turtlebot_description` is available through `apt`. We find that is not. Thus, we look at it on GitHub, and we find it in this repository [https://github.com/turtlebot/turtlebot](https://github.com/turtlebot/turtlebot). Thus, we need to clone it in the same way we did in the previous unit.

    cd ~/ros_workspace/src/
    git clone https://github.com/turtlebot/turtlebot
    cd ..
    colcon build

Here, after a very long prints, we will observe another error after calling `colcon`:

    -- Could NOT find joy (missing: joy_DIR)
    -- Could not find the required component 'joy'. The following CMake error indicates that you either need to install the package with the same name or change your environment so that it can be found.
    CMake Error at /opt/ros/melodic/share/catkin/cmake/catkinConfig.cmake:83 (find_package):
      Could not find a package configuration file provided by "joy" with any of
      the following names:
    
        joyConfig.cmake
        joy-config.cmake
    
      Add the installation prefix of "joy" to CMAKE_PREFIX_PATH or set "joy_DIR"
      to a directory containing one of the above files.  If "joy" provides a
      separate development package or SDK, be sure it has been installed.
    Call Stack (most recent call first):
      CMakeLists.txt:5 (find_package)
    
    
    -- Configuring incomplete, errors occurred!
    

This identifies that a library -- `joy` -- that is needed for a package is not present. Luckily this time, that library is available through `apt`. Thus,

    sudo apt install ros-melodic-joy

This exercise was for you to see the errors and identify low-level how to fix them. ROS provided a powerful tool which is able to fix this type of errors. In particular, just running:

    rosdep install --from-paths src --ignore-src -r -y

finds the necessary dependencies from `apt` repositories and install them automatically through `apt`.

In summary, the following commands need to be run to have the Turtlebot 2 simulation running:

    cd ~/ros_workspace/src
    git clone https://github.com/turtlebot/turtlebot
    git clone https://github.com/yujinrobot/kobuki
    rosdep install --from-paths src --ignore-src -r -y
    colcon build
    source ~/ros_workspace/install/setup.sh # to run again if on a new terminal and not in .bashrc

Finally

    roslaunch turtlebot_gazebo turtlebot_world.launch

will run the simulation

![Turtlebot 2 in Gazebo](/img/turtlebot-gazebo.png)

While this is one source of error, there might be other errors that might be trickier to identify, especially those related to the code. Typically, if a package is widely used, you might find an answer in the [ROS Q&Aforum](https://answers.ros.org/questions/). To fix the error, otherwise, it requires ability to navigate the code and identify the error related to the language, which is out of the scope for this course. Most of the time, for common packages, this process of fixing the errors should work.


We got the main tools up and running to continue our journey in the next courses for learning more about ROS and robot software development. [Let's assess]({{ site.baseurl }}{% link _modules/mod-4d-assessment.md %}) first the understanding of these tools. 

{% capture ref %}
{% bibliography --cited %}
{% endcapture %}
{% include references.html ref=ref %}
