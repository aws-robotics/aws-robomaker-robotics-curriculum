---
title: Build Tool
permalink: /modules/4/build-tool.html
---

We have configured ROS and we have seen how to use `git` to download ROS packages directly from the source. What composes a package? How to build them so that executables and other files are generated? 

To simplify this process, ROS has adopted specific build tools for enabling wide distribution of packages, packages that can be compiled across different environments, and portability. In practice, this means that with a single invocation the build tool will build a set of packages.


There have been different build systems since the inception of ROS. The current official one is `catkin` for ROS. However, there is a transition to another build tool `colcon` which is adopted by ROS2. Because the ROS packages that can be built with `catkin_make` can also be built with `colcon build`, to be ready for the new tool, we will focus on the latter in the [next unit]({{ site.baseurl }}{% link _modules/mod-4b-colcon.md %}). 

{% capture ref %}
{% bibliography --cited %}
{% endcapture %}
{% include references.html ref=ref %}
