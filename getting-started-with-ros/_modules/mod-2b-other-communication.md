---
# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved. // SPDX-License-Identifier: CC-BY-SA-4.0
title: Other interprocess communication means
permalink: /modules/2/srv-act-param.html
---

Publishers and subscribers is one of the main communication methods between different processes. There are a few others that are used. Given that there will be a transition towards ROS2, here we will give the main conceptual understanding and refer to other resources for examples.

## ROS services

Publish/subscribe model provides a many-to-many communication. In some other cases, a **bi-directional one-to-one** communication is preferable. Simplifying, you can think of a function call, which instead of happening on your code, happens on another program. This is also called remote procedure call (RPC).

There are two entities that communicate with each other: (1) a client making a request, and (2) a server which executes some action based on the request and returns a response to the client. The service is synchronous, i.e., the client waits until it gets the response back from a server. Typically it is used for short executions, e.g., short calculations or sending signals to turn on or off the robot.

Similarly to topics, ROS services also should have a unique name. Command line tools are available to inspect the available services and send a request, with `rosservice`. The usage is similar to `rostopic`. For example, to see the available services:

    rosservice list

For example, with the Gazebo simulator running, the following service can be called, which takes an empty request, to reset the world.

    rosservice call /gazebo/reset_world "{}"

Similarly to defined messages, services need to be defined, with the specification on what the request and the response contain. Only a few standard ROS services are available (see [here](http://wiki.ros.org/std_srvs)). Most of them are custom for specific packages and functionalities provided by the nodes.

To write the code, it is very similar to the way to include publishers and subscribers:
1. import the related service type.
2. instantiate a ROS service server, which will include the name and the type of the service, and the callback.
3. instantiate a ROS service client on a different node, specifying the service name and type.

A tutorial can be found [here](http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29).

## ROS actions

Similarly to ROS services, ROS actions involve a client and a server. The main difference is that actions are asynchronous and provide mechanisms for getting feedback on its execution, as well ways to cancel its execution. Usage mainly includes durative tasks. For example, planning and driving the robot from A to B. As they were not one of the core functionalities, ROS actions do not have explicit command line tools as services and topics.

[Here](http://wiki.ros.org/actionlib_tutorials/Tutorials) the related tutorials for ROS actions.

## ROS parameters

How is it possible to set parameters of a node, e.g., the safety distance to obstacles or the maximum velocity?

There are two ways directly supported in ROS to handle parameters for nodes.

First, there is a parameter server handled by the ROS master, which will store such parameters globally. Nodes can retrieve the values of those parameters through specific API calls. While the parameters can be changed at run-time, a node need to explicitly query the parameter server to check for changes. This is mainly used for setting parameters that won't be changing during the execution of the node.
[Here](http://wiki.ros.org/rospy_tutorials/Tutorials/Parameters) an example of getting parameters in a Python script. Commandline tool to see the available parameters is `rosparam`. To see the available parameters:

    rosparam list

Second, to address dynamic reconfiguration of parameters in a seamless way, another tool is provided, called `dynamic_reconfigure`. In this way, any change to a dynamic parameter will trigger the execution of a callback in the node. This is preferable when changes in parameters should change the behavior provided by a node, e.g., a controller with the maximum velocity parameter.
Tutorials on how to configure them can be found [here](http://wiki.ros.org/dynamic_reconfigure/Tutorials). Similarly to `rosparam`, there is a command-line tool:

    rosrun dynamic_reconfigure dynparam COMMAND

where COMMAND can be for example `list`. A nice GUI is from `rqt_reconfigure`, which can be run with the following command

    rosrun rqt_reconfigure rqt_reconfigure

{% include image.html url="/img/rqt-reconfigure.png" description="Dynamic reconfiguration of the parameters through rqt_reconfigure." %}

These additional tools for enabling communication with robotic software will allow for a cleaner design of the code. Let's see in the next [unit]({{ site.baseurl }}{% link _modules/mod-2c-other-tools.md %}) other tools that are useful for debugging.

{% capture ref %}
{% bibliography --cited %}
{% endcapture %}
{% include references.html ref=ref %}
