---
title: Installation process
permalink: /modules/1/installation
---

There are multiple ways to install a new operating system. All of them have three steps that are in common:
1. Download of the OS image.
2. Preparation of the device used for installation.
3. Boot from that device and follow the installation process.
    
For practice, we will use a virtual machine -- you are welcome though to install it as a dual-boot on your machine (follow {% cite canonical2020install %}) or use directly Ubuntu if already installed. There are multiple options for VMs out there. For performance, here in this course we will use VMware player, which is free for personal use.
1. Download the application for [Linux/Windows](https://www.vmware.com/products/workstation-player/workstation-player-evaluation.html) or [Mac OS](https://www.vmware.com/products/fusion.html) and install it following the instructions.
2. Download [Ubuntu 18.04](https://releases.ubuntu.com/18.04/ubuntu-18.04.5-desktop-amd64.iso).
3. Open VMware player and create a new machine, selecting the downloaded Ubuntu image.
4. Follow the prompts to install Ubuntu.

A short video showing these steps is in the following. 

{% include video-file.html url="/img/ubuntu-installation" %}

Note that steps 2., 3., and 4., listed above would not be that different from an actual machine. The only difference is that for the installation on an actual machine, a USB drive needs to be prepared with the image, so that the USB drive can be selected as boot option for installation.

A full discussion on Ubuntu is in its official book {% cite hill2016official %}.

Now that we have a functioning machine, let's dive into the main commands from the command line in the [next unit]({{ site.baseurl }}{% link _modules/mod-1c-commandline.md %}).


{% capture ref %}
{% bibliography --cited %}
{% endcapture %}
{% include references.html ref=ref %}
