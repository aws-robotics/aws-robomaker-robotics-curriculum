---
# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved. // SPDX-License-Identifier: CC-BY-SA-4.0
title: ROS master and nodes example
permalink: /modules/1/ros-demo.html
---

Here, we will run a simulator to better understand the concepts of publishers, subscribers, messages, and nodes, and specific commands to inspect nodes and topics.

## Starting the simulator

In Ubuntu, open a terminal and run the ROS master with the command:

```
roscore
```

Open another terminal and run 

```
roslaunch robomaker-jetbot world.launch
```

This command will run a 3D simulator, Gazebo, as described in the previous Courses 1a and 1b. If not ready, please look at the specific units in the previous courses. 

## Teleoperating the robot

Let's control the robot with keyboard commands, using a program called `teleop_twist_keyboard.py` in the ROS package `teleop_twist_keyboard`. We will use the command `rosrun`. Open a new terminal and run the following command:

```
rosrun turtlebot3_teleop turtlebot3_teleop_key
```

This ROS node will allow you to move the robot pressing the keys indicated in the screen.

## Hands-on understanding of ROS basic concepts

Running the previous commands will allow us to observe the concepts introduced in the previous unit.

There is a GUI tool, `rqt_graph`, which can be run from the command line. The command will open a window showing the nodes running and the topics, as displayed in the following figure.
{% include image.html url="/img/rqt-graph.png" description="Nodes and topics currently running." %}

The ovals represent nodes and the squares represent topics.

------

Let's see the information seen from `rqt_graph` directly on the terminal through command-line tools -- such tools are important when a screen is not available, as on most of the robots.

## Inspecting nodes

Typing

```
rosnode list
```

shows the nodes that are currently running. The output in particular is:

```
/gazebo
/gazebo_gui
/rosout
/teleop_twist_keyboard
```


## Inspecting topics

The topics can be seen via the command

```
rostopic list
```

The result with that is

```
/camera/parameter_descriptions
/camera/parameter_updates
/camera/rgb/camera_info
/camera/rgb/image_raw
/camera/rgb/image_raw/compressed
/camera/rgb/image_raw/compressed/parameter_descriptions
/camera/rgb/image_raw/compressed/parameter_updates
/camera/rgb/image_raw/compressedDepth
/camera/rgb/image_raw/compressedDepth/parameter_descriptions
/camera/rgb/image_raw/compressedDepth/parameter_updates
/camera/rgb/image_raw/theora
/camera/rgb/image_raw/theora/parameter_descriptions
/camera/rgb/image_raw/theora/parameter_updates
/clock
/cmd_vel
/gazebo/link_states
/gazebo/model_states
/gazebo/parameter_descriptions
/gazebo/parameter_updates
/gazebo/set_link_state
/gazebo/set_model_state
/joint_states
/odom
/rosout
/rosout_agg
/scan
/tf

```

There are a number of topics that are typically available on wheeled robots -- their names became a standard de facto:
- `cmd_vel`, as conceptually discussed in the example for the ROS master, nodes, and topics, is for sending control commands to the robot. 
- `scan` is for messages from the laser sensor.
- `/camera/rgb/image_raw` is for messages from the camera. There are a number of other topics related to the camera, and more information is available at the [image_transport page](http://wiki.ros.org/image_transport#Known_Transport_Packages).
- `tf` for transformations, i.e., the relations between different coordinate frames.

Details about who is/are publisher/s and subscriber/s for `cmd_vel`, and the message type, can be visualized with the command

```
rostopic info cmd_vel
```

which outputs

```
Type: geometry_msgs/Twist

Publishers:
 * /teleop_twist_keyboard (http://ubuntu:45357/)


Subscribers: 
 * /gazebo (http://ubuntu:35049/)
```

Currently, there is only one subscriber, `gazebo` node, which is the 3D simulator with a simulated Waveshare Jetbot robot, and a publisher, from `teleop_twist_keyboard`.

The type of message is `geometry_msgs/Twist`. The information can be found at the [page for Twist message](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/Twist.html).
From the command, it is also possible to visualize that information with the following command

    rosmsg show geometry_msgs/Twist

This will output the information about the message

    geometry_msgs/Vector3 linear
      float64 x
      float64 y
      float64 z
    geometry_msgs/Vector3 angular
      float64 x
      float64 y
      float64 z

Let's see two specific topics that we will use in the next unit to write a program.

### Publish Twist messages to control the robot

We are creating now a publisher from command line, using `rostopic pub`. In particular, by typing `rostopic pub cmd_vel` and pressing the Tab key two times, will autocomplete the command with the type of message -- `geometry_msgs/Twist` -- and the content of that message. Initially they have `0.0` values. Changing the value of `z`, e.g., to `0.5` would make the robot rotate counterclockwise at 0.5 radians per second. Here the complete command, which will send a single message:

```
rostopic pub cmd_vel geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.5"
```

`cmd_vel`, as said, is a typical name for the topic to control the robot via velocity commands. For a wheeled robot such as the Jetbot -- a differential drive robot --, the controllable velocities are `linear.x` and `angular.z`.

Now, in another terminal, typing again `rostopic info cmd_vel`, will show another publisher:

```
Type: geometry_msgs/Twist

Publishers: 
 * /teleop_twist_keyboard (http://ubuntu:45357/)
 * /rostopic_14997_1603685769938 (http://ubuntu:38565/)

Subscribers: 
 * /gazebo (http://ubuntu:35049/)
```

This will result in the rotation of the robot. To stop the robot, press `Ctrl+c` and send a Twist message with zeros velocities.

```
rostopic pub cmd_vel geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
```

Robot motor drivers typically implement two behaviors: (1) the driver might execute the last command sent, even if no new message is being sent, or (2) the driver will stop the robot if no new message is received within a given timeframe. For the Jetbot, behavior (1) is implemented, thus it is important to send a Twist message with all zeros.


### Laser scan messages

A LiDAR sensor might be available for avoiding obstacles. The related topic is typically named `scan` and the message type is `sensor_msgs/LaserScan`. More information at the [LaserScan page](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/LaserScan.html).

`rostopic` provides also a program for subscribing to topics and printing them out in the terminal. For example, typing

```
rostopic echo scan
```

will print in the terminal

```
header: 
  seq: 10
  stamp: 
    secs: 685
    nsecs: 878000000
  frame_id: "base_scan"
angle_min: 0.0
angle_max: 6.28318977356
angle_increment: 0.0175019223243
time_increment: 0.0
scan_time: 0.0
range_min: 0.119999997318
range_max: 3.5
ranges: [1.100996732711792, ..., 1.096024990081787]
intensities: [0.0, ..., 0.0]
```

The topic named `scan` typically is associated to messages from a laser sensor, where `ranges` is an array containing distance measurements within the field of view determined by the `angle_min` and `angle_max`, spaced of `angle_increment`.

The following figure shows the `ranges` with the indices and the corresponding angle for a LiDAR sensor. The reference frame is called `laser_link` and the `x` and `y` axes are colored in red and green, respectively. 

{% include image.html url="/img/lidar-explained.svg" description="Depiction of a LiDAR." %}

## Common errors 

-----
Note that `roslaunch` will run `roscore` in case it is not running. When `roscore` is not running and a new node is run, a warning message will appear:

```
Unable to register with master node [http://localhost:11311]: master may not be running yet. Will keep trying.
```

To solve that error, `roscore` needs to be run. Only one `roscore` should be run on the same computer (more advanced configurations could be possible), as otherwise an error will appear

```
RLException: roscore cannot run as another roscore/master is already running. 
Please kill other roscore/master processes before relaunching.
The ROS_MASTER_URI is http://ubuntu:11311/
The traceback for the exception was written to the log file
```

However, if that `roslaunch` that started `roscore` is terminated, also the `roscore` will be terminated. Thus, while there are cases where the following might not be necessary, it is a good practice to include the option `--wait` when using `roslaunch`, which will not run `roscore`.  

------

Note that the topic name should match between nodes that need to communicate with each other through the publish/subscribe paradigm. If it doesn't match, then a new topic will be registered, and the nodes will not exchange information between each other. A simple test to verify this is to run `rostopic pub` run above with, e.g.,  `command_vel` instead `cmd_vel`. In such a case the robot would not move.
This is one of the first checks to do when two nodes are not communicating with each other.
------

## More than roslaunch and rosrun

While the workflows shown here involves the use of `roslaunch` and `rosrun`, note that they are just tools to invoke executables/scripts which are programs that call ROS APIs. This means that a developer can run ROS commands interactively, e.g., through IPython, Jupyter notebooks, or a plain Python Interpreter. 

------


With the basics on nodes, topics, and messages, let's learn in the [next unit]({{ site.baseurl }}{% link _modules/mod-1c-simple-program.md %}) how to write a small Python program that has a publisher and a subscriber.

{% capture ref %}
{% bibliography --cited %}
{% endcapture %}
{% include references.html ref=ref %}
