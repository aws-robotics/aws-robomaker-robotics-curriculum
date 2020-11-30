---
title: ROS installation
permalink: /modules/2/ros-installation.html
---

There are multiple versions of ROS depending on the Ubuntu version.
There are different ways to install ROS, including compiling the source code or using Docker -- a lightweight virtual machine. 
In general, unless there are specific features necessary that are included only in short term support version of ROS and Ubuntu, it is good to stick to the long-term support ones. In this way, libraries will be more stable.

For our purpose, we will install ROS melodic for Ubuntu 18.04 and configure the virtual machine set up in the previous module. This will allow us to get familiar with the installation process and the steps to configure the environment. The installation process is relatively straightforward {% cite rosinstall %} -- here the main highlights.

## Adding the ROS repository

We have discussed the package manager present in Ubuntu. We rely to that for our installation. In particular:
1. the repository where `apt` needs to know where to find the packages needs to be setup. 
```
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```
2. to authenticate the packages a key needs to be added:
```
    sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```
## Installation and configuration

As we added a new repository, it is important to update the list of packages with the command

    sudo apt update

Finally, the installation can proceed using a meta package that will include the core part of ROS and the most common ROS packages:

    sudo apt install ros-melodic-desktop-full python-rosdep
    sudo rosdep init
    rosdep update

Once the installation is done -- it will take some time -- the environment can be configured so that all ROS commands are available on the Terminal. In particular:

    echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
    source ~/.bashrc

This command, which needs to be run only once, will set the environment variables automatically every time a new Terminal is open. The new environment variables added can be checked with the command in the Terminal

    class@ubuntu:~$ env | grep ROS

    ROS_ETC_DIR=/opt/ros/melodic/etc/ros
    ROS_ROOT=/opt/ros/melodic/share/ros
    ROS_MASTER_URI=http://localhost:11311
    ROS_VERSION=1
    ROS_PYTHON_VERSION=2
    ROS_PACKAGE_PATH=/opt/ros/melodic/share
    ROSLISP_PACKAGE_DIRECTORIES=
    ROS_DISTRO=melodic


Now that we have a working ROS system, we can dive into setting up the development environment, covered in the [next unit]({{ site.baseurl }}{% link _modules/mod-2c-development.md %}).


{% capture ref %}
{% bibliography --cited %}
{% endcapture %}
{% include references.html ref=ref %}
