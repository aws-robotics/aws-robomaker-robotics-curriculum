---
title: Git
permalink: /modules/3/git.html
---

In this unit, our objective is to set up a ROS package from git. If `git` is not installed in your Ubuntu system, run in the terminal

    sudo apt install git


Git is a **free and open source** version control system. Many companies provide servers running such a version control system. GitHub is of the many companies that provide git management services and has shown a high popularity among developers -- a 2020 survey from StackOverflow shows that 82.8% of respondents (in total 52,883 responses) uses GitHub {% cite stackoverflow_survey %}. 

## Get a package from an existing repository

We have seen in the previous module how to find existing packages and install them. That is the preferred option. Sometimes it is necessary to modify the code of a package or the package is not available in the repository, as for example `turtlebot_simulator` as shown in the screenshot of the related page below.

![turtlebot_simulator](/img/turtlebot-simulator.png)


To get a ROS package on GitHub, the typical steps are (figures refer to `turtlebot_simulator`):
- First, check the corresponding existence of the package on GitHub (or other repositories) and checkout.

![turtlebot_simulator on GitHub](/img/turtlebot-simulator-git.png)

- Second, change the branch as necessary. Different branches might be available, typically marking versions of the code that work with different versions of ROS.

![turtlebot_simulator on GitHub](/img/turtlebot-simulator-branches.png)

- Third, compile.

In the following, an example for the package `turtlebot_simulator`, assuming the folders `~/ros_workspace/src/` are existing. 

    cd ~/ros_workspace/src/ # going to the `src` directory, containing custom ROS packages.
    git clone https://github.com/ros/turtlebot_simulator # clone a repository.
    cd turtlebot_simulator # access the downloaded ROS package.
    git checkout melodic # change branch as necessary, e.g., to point to the correct version of ROS

If we wanted to apply changes, then it makes sense to fork the public repository so that changes can be applied and tracked. If improvements are provided, then such changes can be contributing back to the project through a pull request. More information [here](https://www.atlassian.com/git/tutorials/comparing-workflows/forking-workflow).

## Create a new repository

It is possible also to create a new GitHub repository, to keep track of the code. The set of instructions to set it up online, is at the [following link](https://docs.github.com/en/free-pro-team@latest/github/getting-started-with-github/create-a-repo).

Once created, follow the same commands above for cloning the repository. A usual workflow for software development includes the following steps (run within the package):
1. `git pull`: fetch the current copy from the remote repository to ensure up-to-date local copy.
2. Modify the local copy, e.g., adding, modifying, or removing files.
3. `git add <directory or files>`: stage all changes of specified `<directory or files>`.
4. `git commit -m <meaningful message>`: to commit the staged snapshot and include a message to identify the modification done.
5. `git push`: push the changes to the remote repository.

There are cheatsheets available, for example [directly from GitHub](https://education.github.com/git-cheat-sheet-education.pdf).

Before discovering what the main files are in a package (next module), [let's assess]({{ site.baseurl }}{% link _modules/mod-3c-assessment.md %}) the understanding of `git`.

{% capture ref %}
{% bibliography --cited %}
{% endcapture %}
{% include references.html ref=ref %}
