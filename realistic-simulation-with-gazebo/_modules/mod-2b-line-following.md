---
# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved. // SPDX-License-Identifier: CC-BY-SA-4.0
title: Pipeline for line following
permalink: /modules/2/line-following.html
---

The task that the robot needs to accomplish is to follow a line that is on the floor (e.g., tape). The sensor used is a camera. Note that this experiment will be eventually performed with a real robot in Course 5. The following figure shows the robot on the track with the green line and its camera view.

{% include image.html url="/img/line-robot-view.png" description="Robot on the track with camera view." %}

The pipeline will be as follows. 
First, the image is transformed from RGB to Hue Saturation Value (HSV) space, which allows for more effective color detection. The main reason is that HSV separates image intensity from the color information. Only the hue is necessary to capture for example green like color.

Here a visualization of how the colors appear in the different axes for RGB and HSV, respectively.

<a title="SharkD, CC BY-SA 4.0 &lt;https://creativecommons.org/licenses/by-sa/4.0&gt;, via Wikimedia Commons" href="https://commons.wikimedia.org/wiki/File:RGB_color_solid_cube.png"><img width="256" alt="RGB color solid cube" src="https://upload.wikimedia.org/wikipedia/commons/thumb/a/af/RGB_color_solid_cube.png/512px-RGB_color_solid_cube.png"></a>
<a title="HSV_color_solid_cylinder.png: SharkD
derivative work: SharkD  Talk, CC BY-SA 3.0 &lt;https://creativecommons.org/licenses/by-sa/3.0&gt;, via Wikimedia Commons" href="https://commons.wikimedia.org/wiki/File:HSV_color_solid_cylinder_saturation_gray.png"><img width="256" alt="HSV color solid cylinder saturation gray" src="https://upload.wikimedia.org/wikipedia/commons/thumb/3/33/HSV_color_solid_cylinder_saturation_gray.png/512px-HSV_color_solid_cylinder_saturation_gray.png"></a>

Second, thresholding and masking are applied for identifying only the line in the image, by selecting the pixels that are within an HSV range, corresponding to the selected color. An assumption is made: the line is already visible and is close to the robot. This allows us to focus just on a slice of the image, instead of considering the full image.

Finally the centroid of the pixels corresponding to the line identified in the second step is determined to identify the direction the robot should move to. If that centroid is at the center of the image over the width, then the robot should just move forward. If it is to the right, the robot should steer right (negative angular velocity). Conversely, if it is to the left, the robot should steer left (positive angular velocity).

-----
Let's see in the [next unit]({{ site.baseurl }}{% link _modules/mod-2c-code.md %}) the code that implements this pipeline.

{% capture ref %}
{% bibliography --cited %}
{% endcapture %}
{% include references.html ref=ref %}
