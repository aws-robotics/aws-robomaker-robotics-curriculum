---
title: Installation with Docker
permalink: /modules/1/installation-docker.html
---

Another way to work with Linux and software, regardless of the machine used, is through Docker.  From a very high-level view, Docker is similar to a virtual machine in that it is possible to run another operating system in any machine. One of the main differences is that Docker's objective is to provide software in packages called **containers**, rather than a full virtual machine. The necessary software and specific versions are specified in a "recipe" file, called `Dockerfile`. This choice allows Docker containers to be lightweight compared to a regular virtual machine.  
More details can be found on the [Docker website](https://www.docker.com/resources/what-container).

For the purpose of this robotics curriculum, we will use a readily available container that has installed ROS  melodic. As the Docker container is lightweight, it does not provide a direct visualization system as in other virtual machines. We will have the graphical user interface through a graphical desktop-sharing system through VNC, Virtual Network Computing.

To get the environment ready, please follow the [Docker installation instructions](https://docs.docker.com/get-docker/) for your Operating System.

Once installed, please run Docker. The following window will appear.

![Docker first image](/img/docker-first.png)

Install also git on your computer, following [these instructions](https://git-scm.com/book/en/v2/Getting-Started-Installing-Git) for your system.

Once installed, you can open your terminal and run the following command

    git clone https://github.com/quattrinili/vnc-ros.git

which will create a folder called `vnc-ros` in the current directory. The current directory could be changed using the `cd` command, e.g., in Linux-based system, to have `vnc-ros` in the `Desktop` folder of the current user, type `cd ~/Desktop`, before the `git` command.

Then, enter the created folder

    cd vnc-ros
    
At this point you can run the following command to "configure and turn on the container" -- you can think about this step as installing the operating system and turning on the computer:

    docker-compose up --build
    
Open a new terminal, go to the folder `vnc-ros` and run the following command

    docker-compose exec ros bash
    
which will allow you to have a terminal connected to that container, where you can run the commands we will see in this module. This command can be run only after the `docker-compose up --build` as otherwise it will fail, as the container wouldn't be up and running. If it runs correctly, you would see in the terminal, something like:

    root@e75b5d66b0b9:~/catkin_ws

which will demonstrate that now you are in the terminal of the docker container running. At this point any commands we will see later should run.

There is a folder that is shared between the container and your system, called `catkin_ws` in the Docker and `vnc-ros/workspace` in your system. Any file added/modified/removed in the Docker will be visible to your system and vice versa.  

Note that modification to the system, such as installation of new software packages, might not be permanent outside of that shared folder. The best practice is to include the necessary software in `Dockerfile`.

You can run multiple terminals by repeating the steps for `docker-compose exec ros bash`.

To visualize software that shows a graphical user interface, you can:

1) Open your favorite browser,

2) go to the following address `localhost:8080/vnc.html`

3) click "Connect"

The screen will show the "desktop" of the container.

![Docker vnc](/img/docker-vnc.png)

This desktop should be only used as a way to display, rather than running commands. The commands should be run through the terminal.

A full discussion on Docker can be found [here](https://docker-curriculum.com/).

-----
Now that we have a functioning machine, let's dive into the main commands from the command line in the [next unit]({{ site.baseurl }}{% link _modules/mod-1c-commandline.md %}).


{% capture ref %}
{% bibliography --cited %}
{% endcapture %}
{% include references.html ref=ref %}
