---
# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved. // SPDX-License-Identifier: CC-BY-SA-4.0
title: Implementation
permalink: /modules/2/implementation.html
---

The implementation relies on a popular library for image processing and computer vision, [OpenCV](https://opencv.org/). The code can be found at the following git repository:
[https://github.com/aws-samples/aws-robomaker-jetbot-ros/blob/main/src/aws_example_apps/line_following/nodes/line_following](https://github.com/aws-samples/aws-robomaker-jetbot-ros/blob/main/src/aws_example_apps/line_following/nodes/line_following)

Here the code that will be discussed in this unit.


```
#!/usr/bin/env python
#The line above is important so that this file is interpreted with Python when 
#running it.

# Import of python modules.
import Queue
import math # use of pi.
import cv2 # Computer vision library.
import numpy

# import of relevant libraries.
import rospy # module for ROS APIs
import cv_bridge # convert ROS message to OpenCV
from geometry_msgs.msg import Twist # message type for cmd_vel
from sensor_msgs.msg import Image # message type for camera

# Constants related to ROS.
FREQUENCY = 30 #Hz.
DEFAULT_CMD_VEL_TOPIC = 'cmd_vel'
DEFAULT_CAMERA_TOPIC = 'camera/rgb/image_raw'
# Constant to identify color.
DEFAULT_COLOR = numpy.array([[40, 10, 10], [80, 255, 250]]) # Green in HSV space
# Constants to set a mask for the image. 
TOP_WINDOW = 3.0 / 5 # part of the image from row 0 to TOP_WINDOW will be set to 0.
BOTTOM_WINDOW = 4.0 / 5 # part of the image from row BOTTOM_WINDOW to height will be set to 0.
CENTROID_LOCATION_RATIO = 0.5 # ratio for width to determine where the centroid should be.
# Constants for velocity.
LINEAR_VELOCITY = 0.2 # m/s
P_GAIN = 0.01 # Proportional gain for the P controller.
DEBUG = False

class LineFollowing():
    def __init__(self, linear_velocity=LINEAR_VELOCITY, 
        color_range=DEFAULT_COLOR, debug=DEBUG):
        """Constructor."""

        # Setting up publishers/subscribers.
        self._cmd_pub = rospy.Publisher(DEFAULT_CMD_VEL_TOPIC, Twist, 
        queue_size=1)

        # Setting up subscriber.
        self._image_sub = rospy.Subscriber(DEFAULT_CAMERA_TOPIC, Image,
            self._image_callback, queue_size=1)

        # For converting ROS image messages to OpenCV.
        self.bridge = cv_bridge.CvBridge()

        self.debug = debug
        if self.debug:
            self.image_pub = rospy.Publisher('camera/rgb/image_raw/marker', 
                Image, queue_size=1)

        # Other variables.
        self.linear_velocity = linear_velocity # Constant linear velocity set.
        self.color_range = color_range # HSV color range.
        self.centroid = Queue.Queue(maxsize=1) # queue containing only last.

    def _image_callback(self, msg):
        """Image callback"""
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.find_line_centroid(image)

    def find_line_centroid(self, image):
        """Find centroid of line within a part of the image."""
        # Conversion to HSV space.
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # find only pixels those withing the HSV range.
        mask = cv2.inRange(hsv, self.color_range[0], self.color_range[1])

        # Consider only a part of the image that is at the bottom.
        height, self.width, channels = image.shape
        top_window = int(height * TOP_WINDOW)
        bottom_window = int(height * BOTTOM_WINDOW)
        mask[0:top_window, :] = 0
        mask[bottom_window:height, :] = 0

        # Find the centroid based on moment.
        M = cv2.moments(mask)
        if M['m00'] > 0:
            centroid_x = int(M['m10']/M['m00'])
            centroid_y = int(M['m01']/M['m00'])
            try:
                self.centroid.put_nowait([centroid_x, centroid_y])
            except Queue.Full:
                pass

        if self.debug:
            # If debug, publish image with marker corresponding to the centroid.
            cv2.circle(image, (cx, cy), 20, (0, 0, 255), -1)
            image_w_marker_msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
            self.image_pub.publish(image_w_marker_msg)

    def spin(self):
        """Control the robot based on the centroid."""
        r = rospy.Rate(FREQUENCY)
        twist = Twist()
        twist.linear.x = self.linear_velocity
        while not rospy.is_shutdown():
            try:
                centroid = self.centroid.get_nowait()
            except Queue.Empty:
                continue

            # Assumes that the centroid should be at the center of the image.
            error = self.width * CENTROID_LOCATION_RATIO - centroid[0]
            # Depending on the error, rotates left or right
            twist.angular.z = P_GAIN * float(error)
            # Publish velocity message.
            self._cmd_pub.publish(twist)
            r.sleep()

    def stop(self):
        """Stop the robot."""
        twist_msg = Twist()
        self._cmd_pub.publish(twist_msg)

def main():
    """Main function."""

    # 1st. initialization of node.
    rospy.init_node("line_following")

    # Initialization of the class for the simple motion.
    line_following = LineFollowing()

    # If interrupted, send a stop command.
    rospy.on_shutdown(line_following.stop)

    # Robot follows a line.
    try:
        line_following.spin()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interrupted.")

if __name__ == "__main__":
    """Run the main function."""
    main()
```

We will highlight the lines of code that are new compared to those seen in Course 2.

-----
```
image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
```
allows for converting a [ROS image message](https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Image.html) to a 2D array (in `numpy`).

-----
```
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
# find only pixels those withing the HSV range.
mask = cv2.inRange(hsv, self.color_range[0], self.color_range[1])
```
This is the main part of the image processing algorithm, i.e., the conversion to HSV and masking the image considering only those that are in the "green range".

-----
```
height, self.width, channels = image.shape
top_window = int(height * TOP_WINDOW)
bottom_window = int(height * BOTTOM_WINDOW)
mask[0:top_window, :] = 0
mask[bottom_window:height, :] = 0

# Find the centroid based on moment.
M = cv2.moments(mask)
if M['m00'] > 0:
    centroid_x = int(M['m10']/M['m00'])
    centroid_y = int(M['m01']/M['m00'])
```
Then, only the bottom part of the image is considered and the centroid is found, through the image moments. This information is used by the controller to send angular velocities, depending on the error. The error is multiplied by `P_GAIN`, which clamps the possible error values within the maximum angular velocity. A value of 0.01 was chosen and worked well experimentally.

While the command could be sent within the image callback, the command is sent within the main loop, so that a maximum rate to send control commands is satisfied. This will allow also for a combination of behaviors – e.g., stop the robot if there is an obstacle in front, as a finite state machine.

------

Here the commands to set up the necessary packages (assuming that `ros_workspace` was existing already):

```
cd ~/ros_workspace/src/
git clone https://github.com/aws-samples/aws-robomaker-jetbot-ros
cd aws-robomaker-jetbot-ros
chmod +x ./setup.sh
./setup.sh

```

**Reminder:** if using the AWS RoboMaker development environment, open the Virtual Desktop by clicking the virtual desktop button in the top navigation tool bar. This will open a pop-up window, if you are prompted, allow your browser to open the pop-up window. In the terminal of the IDE, set the display to the virtual desktop:

```
export DISPLAY=:0
```

Next, we will build the ROS application to run in the simulation. Note: every time you make a code change to the ROS packages in this sample repository, you will need to run this command

```
cd ~/environment/aws-robomaker-jetbot-ros
colcon build
```

To run the simulation first ensure you are in the base workspace directory and the application is sourced.

```
cd ~/environment/aws-robomaker-jetbot-ros
source install/setup.sh

#launch simulation
roslaunch line_following line_following_sim.launch
```

This will result in the following behavior.

{% include video-file.html url="/img/line-following-behavior" %}

------

Let's see in the [next unit]({{ site.baseurl }}{% link _modules/mod-2d-extensions.md %}) some possible extensions to this code.


{% capture ref %}
{% bibliography --cited %}
{% endcapture %}
{% include references.html ref=ref %}
