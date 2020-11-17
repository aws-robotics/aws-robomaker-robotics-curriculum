---
# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved. // SPDX-License-Identifier: CC-BY-SA-4.0
title: ROS node with publisher and subscriber
permalink: /modules/1/ros-simple-node.html
---

Now that we know the concept of publisher and subscriber, let's write a simple python ROS node to make the simulated Jetbot robot move for a specified distance or rotation, and stop if there is anything in front.

## Structure of the code

There are three parts that are necessary in any ROS node:
1. initialization of the node, i.e., registering with the ROS master.
2. instantiating publishers and subscribers.
3. running of the actual node.


## Actual code

Below a sample program that follows the identified three parts. It allows the robot to achieve simple motions by publishing messages on the topic `cmd_vel`, which type message is `geometry_msgs/Twist`:

```
#!/usr/bin/env python
#The line above is important so that this file is interpreted with Python when running it.

# Import of python modules.
import math # use of pi.

# import of relevant libraries.
import rospy # module for ROS APIs
from geometry_msgs.msg import Twist # message type for cmd_vel
from sensor_msgs.msg import LaserScan # message type for scan

# Constants.
FREQUENCY = 10 #Hz.
LINEAR_VELOCITY = 0.2 # m/s
ANGULAR_VELOCITY = math.pi/4 # rad/s
LASER_ANGLE_FRONT = 0 # radians
MIN_THRESHOLD_DISTANCE = 0.5 # m, threshold distance.
DEFAULT_CMD_VEL_TOPIC = 'cmd_vel'
DEFAULT_SCAN_TOPIC = 'scan'

class SimpleMotion():
    def __init__(self, linear_velocity=LINEAR_VELOCITY, angular_velocity=ANGULAR_VELOCITY):
        """Constructor."""

        # Setting up publishers/subscribers.
        self._cmd_pub = rospy.Publisher(DEFAULT_CMD_VEL_TOPIC, Twist, queue_size=1)

        # Setting up subscriber.
        self._laser_sub = rospy.Subscriber(DEFAULT_SCAN_TOPIC, LaserScan, self._laser_callback, queue_size=1)

        # Other variables.
        self.linear_velocity = linear_velocity # Constant linear velocity set.
        self.angular_velocity = angular_velocity # Constant angular velocity set.
        self._close_obstacle = False # Flag variable that is true if there is a close obstacle.

    def move_forward(self, distance):
        """Function to move_forward for a given distance."""
        # Rate at which to operate the while loop.
        rate = rospy.Rate(FREQUENCY)

        # Setting velocities.
        twist_msg = Twist()
        twist_msg.linear.x = self.linear_velocity
        start_time = rospy.get_rostime()
        duration = rospy.Duration(distance/twist_msg.linear.x)

        # Loop.
        while not rospy.is_shutdown():
            # Check if traveled of given distance based on time.
            if rospy.get_rostime() - start_time >= duration:
                break

            # Publish message.
            if self._close_obstacle:
                self.stop()
            else:
                self._cmd_pub.publish(twist_msg)

            # Sleep to keep the set publishing frequency.
            rate.sleep()

        # Traveled the required distance, stop.
        self.stop()

    def rotate_in_place(self, rotation_angle):
        """
        Rotate in place the robot of rotation_angle (rad) based on fixed velocity.
        Assumption: Counterclockwise rotation.
        """
        twist_msg = Twist()
        twist_msg.angular.z = self.angular_velocity

        duration = rotation_angle / twist_msg.angular.z
        start_time = rospy.get_rostime()
        rate = rospy.Rate(FREQUENCY)
        while not rospy.is_shutdown():
            # Check if done
            if rospy.get_rostime() - start_time >= rospy.Duration(duration):
                break

            # Publish message.
            self._cmd_pub.publish(twist_msg)

            # Sleep to keep the set frequency.
            rate.sleep()

        # Rotated the required angle, stop.
        self.stop()

    def stop(self):
        """Stop the robot."""
        twist_msg = Twist()
        self._cmd_pub.publish(twist_msg)

    def _laser_callback(self, msg):
        """Processing of laser message."""
        # Access to the index of the measurement in front of the robot.
        # NOTE: assumption: the one at angle 0 corresponds to the front.
        i = int((LASER_ANGLE_FRONT - msg.angle_min) / msg.angle_increment)
        if msg.ranges[i] <= MIN_THRESHOLD_DISTANCE:
            self._close_obstacle = True
        else:
            self._close_obstacle = False


def main():
    """Main function."""

    # 1st. initialization of node.
    rospy.init_node("simple_motion")

    # Sleep for a few seconds to wait for the registration.
    rospy.sleep(2)

    # Initialization of the class for the simple motion.
    simple_motion = SimpleMotion()

    # If interrupted, send a stop command.
    rospy.on_shutdown(simple_motion.stop)

    # Robot moves 1 m forward, rotate of 180 degrees, and moves 1 m forward.
    try:
        simple_motion.move_forward(1)
        simple_motion.rotate_in_place(math.pi)
        simple_motion.move_forward(1)
    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interrupted.")


if __name__ == "__main__":
    """Run the main function."""
    main()
```


## Break-down

Here the main highlights of the parts of the code that are ROS specific.

    import rospy

is the the ROS Python library for interfacing with ROS topics.

------

    rospy.init_node('simple_motion')
    rospy.sleep(2)

These are for initializing the node with name `simple_motion`. The call to the `sleep()` function -- 2 seconds -- is to ensure that the node is properly registered as in practice some unintended behavior are experienced, such as not being able to get the ROS time.

------

    self._cmd_pub = rospy.Publisher(DEFAULT_CMD_VEL_TOPIC, Twist, queue_size=1)

This line creates a `Publisher`: the first parameter is the topic name and should match the one used by the other node with a subscriber; `Twist` is the type of message and can be found with the `rosmsg` command presented before; `queue_size` is the number of messages that will be stored in a queue for publishing to nodes that might subscribe later. A size of 1 means the the latest will be published. [Here](http://wiki.ros.org/rospy/Overview/Publishers%20and%20Subscribers#Choosing_a_good_queue_size) more details about `queue_size`.

------

    twist_msg = Twist() # ROS message
    twist_msg.linear.x = LINEAR_VELOCITY

This is the initialization of the message and setting the specific member value. The easiest to find what the members are for a message and to know how to set them is to look at the documentation. For example for `Twist`, [here](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/Twist.html) the specific documentation.

------

    publisher.publish(twist_msg)

This is for the actual publishing of the message, which will be received by a node subscribing to the topic. In this case, it will sent a velocity command.

------

    rate = rospy.Rate(FREQUENCY)

and the call to its function, usually in a loop

    rate.sleep()

allows for code to sleep so that the frequency is maintained. For example, 10 Hz means that the publishing of the `Twist` message is every 100 ms. The reason why this is necessary is that anyway the driver reading such messages and sending commands to the motors might not be able to operate at higher frequencies. Note that if the code in the loop takes longer, the execution is not at that frequency.

------
Similarly, for the Subscriber, there is the topic and the message type to specify. In addition, there is callback function, executed every time a message is received.

    self._laser_sub = rospy.Subscriber(DEFAULT_SCAN_TOPIC, LaserScan, self._laser_callback, queue_size=1)

The callback function has one parameter, `msg`, which allows the code to access the received message. The details for `LaserScan` can be found [here](http://docs.ros.org/en/api/sensor_msgs/html/msg/LaserScan.html).

    def _laser_callback(self, msg):
        """Processing of laser message."""
        # Access to the index of the measurement in front of the robot.
        # NOTE: assumption: the one at angle 0 corresponds to the front.
        i = int((LASER_ANGLE_FRONT - msg.angle_min) / msg.angle_increment)
        if msg.ranges[i] <= MIN_THRESHOLD_DISTANCE:
            self._close_obstacle = True
        else:
            self._close_obstacle = False

The main highlights of the message are:
- `ranges` which is an array containing range measurements.
- `angle_min` which corresponds to the start angle of the scan, i.e., the element at index 0 of `ranges`.
- `angle_max` which corresponds to the last angle of the scan, i.e., the element at the last index of `ranges`.
- `angle_increment`, i.e., the difference between the angles of the range at index `i+1` and `i`.


------

The other parts of the code implements the logic for the robot to move for a given distance (or rotation), given a velocity and a target distance (or angle), based on the calculated necessary time.


------

## Getting the code into a ROS package

As seen in the previous course on setting up the environment (Linux or AWS RoboMaker), ROS software is structured around packages.

For a standalone system, going to the created workspace (e.g., `ros_workspace`), and downloading the corresponding ROS package from GitHub, the package can be built:

    cd ~/ros_workspace/src/
    git clone https://github.com/quattrinili/simple_motion.git
    cd ..
    colcon build

At this point, in another terminal, the Gazebo simulator can be run. For example, using the Jetbot:

    roslaunch robomaker-jetbot world.launch

and in a second terminal:

    roslaunch simple_motion simple_motion.launch

### Standalone machine
Here a short video showing the steps in a standalone machine.

{% include video-file.html url="/img/simple-motion-standalone" %}

### AWS RoboMaker

For AWS RoboMaker, the main steps are:
1. downloading the package in the correct workspace.
2. changing the configuration of the simulation so that the new launch file will be run.
3. build and bundle.
4. run the simulation.

A video for the steps in AWS RoboMaker is in the following.

{% include video-file.html url="/img/simple-motion-robomaker" %}

Note that as the standalone machine, it is possible to run again the launch file.

Please remember to stop the simulation once done to minimize credit use.

Now that we have seen how to write a ROS node, let's understand the structure of the ROS package in the [next unit]({{ site.baseurl }}{% link _modules/mod-1d-details-package.md %}).

{% capture ref %}
{% bibliography --cited %}
{% endcapture %}
{% include references.html ref=ref %}
