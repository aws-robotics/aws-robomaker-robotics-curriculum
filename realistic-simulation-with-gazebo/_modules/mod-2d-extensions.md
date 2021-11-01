---
# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved. // SPDX-License-Identifier: CC-BY-SA-4.0
title: Possible extensions
permalink: /modules/2/extensions.html
---

We implemented a very simple line following algorithm. There are a number of extensions to improve the implementation, including, but not limited to:
- Currently a proportional controller based on the offset from the center is used. If there were systematic errors, the integral term could be added or if there was oscillation at higher velocity, the derivative term could be included.
- The assumption is that the line is visible from the beginning and won't be lost with any control sent to the robot. A recovery mechanism could be added in case the line is not visible anymore.
- The implementation assumes that the line can be well recognized. However, we have seen that noise can affect the image, potentially impacting the line detection. There are strategies to deal with such a noise; for example, applying a Gaussian filter to the image.

In general, image processing and computer vision is a rich research area that goes beyond robotics. A robotics book that covers vision and control is from Peter Corke, professor at Queensland University of Technology {% cite corke2017robotics %}. 

------
[Let's assess]({{ site.baseurl }}{% link _modules/mod-2e-assessment.md %}) the understanding of what we covered in this module.


{% capture ref %}
{% bibliography --cited %}
{% endcapture %}
{% include references.html ref=ref %}
