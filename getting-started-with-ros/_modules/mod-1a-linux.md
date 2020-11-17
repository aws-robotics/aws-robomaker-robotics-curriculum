---
title: Linux
permalink: /modules/1/linux
---

In this module, we will discover the main features of such an operating system and put in practice some commands that will allow us to work with a command-line. This skill is necessary to work with real robots and servers on the cloud.


## What is Linux?
Linux is an operating system. While there is not a universally accepted definition, generally an OS is a "program" acting as intermediary between the computer hardware and a user. 

<a title="mlibre / CC0" href="https://commons.wikimedia.org/wiki/File:Linux_kernel_and_Computer_layers.png"><img width="256" alt="Linux kernel and Computer layers" src="https://upload.wikimedia.org/wikipedia/commons/thumb/9/9f/Linux_kernel_and_Computer_layers.png/512px-Linux_kernel_and_Computer_layers.png"></a>


Why is it important to consider an OS within a robot? The OS allocates resources and controls the execution of a program. As such, it is important to consider whether the OS will be able to allocate fairly resources over software that controls, for example, the engine of a robot. For those interested in deepening in OS, {% cite silberschatz2014operating %}.

For our purpose, we will take a look at the command line, i.e., a user interface that based on commands typed with the keyboard, and useful commands that we will encounter throughout the curriculum. This is an important skill to know as it is the base for configuring robots, as typically they do not have any screen on-board, as well as to work on servers in the cloud.

Let's dive into the steps of installing a new instance of Ubuntu on a Virtual Machine in the [next unit]({{ site.baseurl }}{% link _modules/mod-1b-installation.md %}) -- if you already have a machine with Ubuntu you can skip the following unit.


{% capture ref %}
{% bibliography --cited %}
{% endcapture %}
{% include references.html ref=ref %}
