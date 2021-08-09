---
# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved. // SPDX-License-Identifier: CC-BY-SA-4.0
title: ROS package details
permalink: /modules/1/package-details.html
---

Now that we ran our first node with some code, let's review the main elements of a Python ROS package.

## Structure 

`simple_motion` is a ROS package which follows the typical structure of a ROS package.

	simple_motion
	 |- launch/
	    |- launch files # XML files to run multiple nodes with a single command.
	 |- src/
		|- simple_motion/
		  |- __init__.py
		  |- python_modules.py # Exported Python modules, but not used in this case.
	 |- nodes/
		|- ROS nodes # Scripts to run the nodes.
     |- CMakeLists.txt # CMake build file.
	 |- package.xml # XML file for package manifest for catkin.
	 |- setup.py # python file to tell what module are installed.

`package.xml` and `CMakeLists.txt` are the ones that need to be modified according to the dependencies.

`setup.py` remains pretty much the same just the package name needs to be modified.

Any new executable as a node should be included in `nodes`, while `src/simple_motion` contains Python files for exporting them as Python modules. In that way, `import simple_motion` can be used, and any module there can be used within any other ROS Python script.

`launch` contains XML files allowing to specify multiple nodes to run in a single command as seen previously.

## ROS launch
A ROS launch file is an XML file where the parameters and nodes to run can be specified. Take a look at the one available:

	<launch>
		<node pkg="simple_motion" type="simple_motion" name="simple_motion" output="screen"/>
	</launch>    


- `pkg` specifies the package where the node is found.
- `type` specifies the executable -- the ROS node -- to run.
- `name` is an arbitrary name assigned to the node for the ROS master registration.
- `output` specifies that the output of the node will be printed on the standard output.

More information can be found [roslaunch XML documentation](http://wiki.ros.org/roslaunch/XML).

Common errors related to the XML file not being properly formatted include forgetting to close the tag either in-line with `<node ... />` or `<node ...> </node>`.

## Other parts

There are other parts that are not in the example but are useful to know, especially when thinking about expanding what ROS provides. In particular, we have used existing messages. Sometimes a new custom-message must be defined. Such declarations need to be included in the `CMakeLists.txt` and in a new folder called `msg/`. See the [creating a msg tutorial](https://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv#Creating_a_msg) for full details.

-------
What we learned so far allows us to achieve robotic behaviors. Before moving on to the next module that will look at some other ROS components and tools available, [let's assess the understanding about ROS publish/subscribe]({{ site.baseurl }}{% link _modules/mod-1e-assessment.md %}).

{% capture ref %}
{% bibliography --cited %}
{% endcapture %}
{% include references.html ref=ref %}
