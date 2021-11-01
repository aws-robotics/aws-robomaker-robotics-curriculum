---
# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved. // SPDX-License-Identifier: CC-BY-SA-4.0
title: How to run on AWS RoboMaker Simulation Job
permalink: /modules/5/maze-robomaker-virtual-desktop.html
---

The code seen before could be run in any machine. Given the need for a GPU and long training, it makes sense to exploit the cloud services and RoboMaker.

Let's see first how to run the simulation on the Virtual Desktop.

High-level, the steps include, as usual, the cloning of the [repository](https://github.com/aws-samples/aws-robomaker-sample-application-meirorunner), the installation of the dependencies, the compilation of the simulation and robot workspaces, and the execution of the simulation. The Virtual Desktop can be used to visualize the simulation.

Once in the development environment, the main commands to run in a terminal for the simulation application are:

```
# Cloning
cd ~/environment
git clone https://github.com/aws-samples/aws-robomaker-sample-application-meirorunner.git  
# Installation of dependencies and build
cd aws-robomaker-sample-application-meirorunner/simulation_ws/
rosdep install --from-paths src --ignore-src -r -y
colcon build

# Run of simulation.
source install/setup.bash
export DISPLAY=:0
# Run first the Virtual Desktop from the IDE, then
roslaunch meiro_runner_simulation create_maze_world.launch gui:=true
```

In a new terminal, the main commands to run for the robot application are:

```
# With the environment already cloned, install dependencies and build
cd aws-robomaker-sample-application-meirorunner/robot_ws/
rosdep install --from-paths src --ignore-src -r -y
colcon build
# Note installation of dependencies of non-ROS python package
cd src/rl_agent
python setup.py egg_info
pip3 install -r rl_agent.egg-info/requires.txt

# Run of the robot application for training
cd ~/environment/aws-robomaker-sample-application-meirorunner/robot_ws/
source install/setup.bash
# Export of environment variables needed by the launch file.
export MODEL_S3_BUCKET=meirorunnersimjoboutput
export MODEL_S3_PREFIX=model-store
export ROS_AWS_REGION=us-east-1
export MARKOV_PRESET_FILE=meiro_runner.py

roslaunch meiro_runner_robot local_training.launch
```

---
Note that it is assumed that an S3 bucket called `meirorunnersimjoboutput` is already existing. If not, it can be created with the following command as seen in Course 1b.

```
aws s3 mb s3://meirorunnersimjoboutput
```
---

The software for training the model should run and in Virtual Desktop, the simulation will show the robot moving and resetting its location when hitting an obstacle.

----
To stop the simulation, press `Ctrl+c` on the terminals where you ran the simulation and the application. The script for the training runs independently and should be stopped using `htop` by sending a kill signal: select the  entry corresponding to that script, i.e., the one with `markov.single_machine_training_worker`, press `k`, then press `9`, and press enter. 

---

Here a full video demonstrating the steps to run the example using the Virtual Desktop.

{% include video-file.html url="/img/rl-aws-wd" %}

---
To fully exploit the cloud services, let's see how to create a simulation job to train the model in the [next unit]({{ site.baseurl }}{% link _modules/mod-5d-2-run-simulation-job.md %}).
