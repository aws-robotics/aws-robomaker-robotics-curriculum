---
title: Colcon
permalink: /modules/4/colcon
---

After having cloned the package, as seen in the [previous module]({{ site.baseurl }}{% link _modules/mod-3b-git.md %}), we are ready to get into the building process with `colcon`. We will highlight the main steps to have the ROS packages usable in the environment. Note that `colcon` can provide additional tasks, including unit tests. For full details, please see the [documentation](https://colcon.readthedocs.io/en/released/user/quick-start.html).

With a single command, `colcon` can build all packages that are within the workspace.

    cd ~/ros_workspace # go to the workspace
    colcon build # compile and create the files necessary 

Such a command creates the following directories:
- `build` directory, where intermediate built files are stored in a subfolder created for each package.

- `install` directory, where each package will be installed to. By default each package will be installed into separate subdirectories.

- `log` directory, containing logging information about each `colcon` invocation.

To ensure that the new packages can be found, the environment variables need to be set:

    source ~/ros_workspace/install/setup.sh # setup the environment variables so that the ROS packages are found


Note that, if a ROS package is already installed with the same name, after using the command `source` as shown above, the package in the workspace will be prioritized. This feature is called **overlay workspace**, where any packages in the "sourced workspace" overlay any preexisting packages in "lower workspaces". This can be a powerful tool for different development workflows. 

Every time a new terminal is opened the `source` command must be repeated. When working with a single workspace, similarly to the source command for the general ROS installation, modify `.bashrc` with the following one-time command:

    echo "source ~/ros_workspace/install/setup.sh" >> ~/.bashrc
    source ~/.bashrc

In this way, the `source` command does not need to be repeated every time a new terminal is opened. However, when working with multiple workspaces, where packages might be overlaid, thus this is not recommended.

To clean the build, the created directories can be removed with the command `rm -r` (`-r` for recursive).

Analyzing the structure of each package, we notice that two files that appear consistently are `package.xml` and `CMakeLists.txt`. The first file is an XML file containing information about the package, e.g., package name, description, version, authors, maintainers, websites and dependencies. The second file is for the CMake build system for building software packages. There is a specific format that needs to be followed. High-level, it specifies dependencies, executables to build, and install rules. There are a few additional instructions for adding Python support and creating messages. In practice, it is fine to start from an existing `CMakeLists.txt` and modify it according to the specific needs. More information [here](http://wiki.ros.org/Packages).

Now that we are able to use `colcon build` let's see in the [next unit]({{ site.baseurl }}{% link _modules/mod-4c-colcon-fix.md %}) a couple of common errors that are more straightforward to identify and fix.



{% capture ref %}
{% bibliography --cited %}
{% endcapture %}
{% include references.html ref=ref %}
