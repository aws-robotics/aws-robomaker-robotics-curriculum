---
title: Motivation
permalink: /modules/0/motivation
menus: header
---

Software running on any computing device requires an operating system (OS), which manages software and hardware on a computer. The one that has the biggest share is Microsoft Windows, followed by Mac OS -- for example, for desktop computers, an estimated share in July 2020 is 77.74% vs. 17.07% for Windows and Mac OS, respectively {% cite statcounter2020 %}.

In robotics, in recent years, Linux-based OSes (e.g., Ubuntu) have been the main operating system used, in conjunction with the Robot Operating System (ROS), a middleware which is projected to be on 55% of the commercial robots shipped by 2024 {% cite abiresearch2019 %}. One of the main reasons why Linux-based OS has had success in this space, in particular Ubuntu, is the fact that being opensource is necessary to avoid any limitation imposed by proprietary operating systems. Ubuntu specifically has put a specific team on robotics and embedded systems in general {% cite canonical2018 %}. 

Not only research uses such a system, but also robotics companies have adopted ROS. Some examples are included in the following video showing use cases of ROS.

{% include video-youtube.html url="https://player.vimeo.com/video/146183080" %}

Before we delve into the nuts and bolts of ROS (covered in the following course), let's discover how to set up the development environment through this brief introduction to Linux, ROS, and development tools [course]({{ site.baseurl }}{% link _modules/mod-0b-objectives.md %}).

{% capture ref %}
{% bibliography --cited %}
{% endcapture %}
{% include references.html ref=ref %}
