---
# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved. // SPDX-License-Identifier: CC-BY-SA-4.0
title: How to run on AWS RoboMaker Simulation Job
permalink: /modules/5/maze-robomaker-simulation-job.html
---

The code seen before could be run in any machine. Given the need for a GPU and long training, it makes sense to exploit the cloud services and RoboMaker.

High-level, the steps include, as usual, the cloning of the [repository](https://github.com/aws-samples/aws-robomaker-sample-application-meirorunner), the compilation and creation of the bundle, and the execution of the simulation. Metrics from the training and evaluation can be observed through AWS CloudWatch.

Once in the development environment, the main commands to run are:

```
cd ~/environment
git clone https://github.com/aws-samples/aws-robomaker-sample-application-meirorunner.git  
cd aws-robomaker-sample-application-meirorunner
./ws_setup.sh
```

The last script configures the environment with the permissions and the simulation necessary.

As done in Course 1b, we need to create the S3 buckets (or use existing ones) that will contain the simulation and robot application to create. Here are the commands to run:

```
# Creation of S3 bucket containing sources
aws s3 mb s3://meirorunnersource
# Creation of S3 bucket for output
aws s3 mb s3://meirorunnersimjoboutput

cd ~/environment/aws-robomaker-sample-application-meirorunner/

# Copy robot application bundle on S3 bucket and create related application
aws s3 cp robot_ws/bundle/output.tar s3://meirorunnersource/meirorunner-robot-app.tar.gz
aws robomaker create-robot-application --name MeiroRunnerRobotApplication --sources s3Bucket=meirorunnersource,s3Key=meirorunner-robot-app.tar.gz,architecture=X86_64 --robot-software-suite name=ROS,version=Melodic

# Copy simulation application bundle on S3 bucket and create related application
aws s3 cp simulation_ws/bundle/output.tar s3://meirorunnersource/meirorunner-sim-app.tar.gz
aws robomaker create-simulation-application --name MeiroRunnerSimApplication --sources s3Bucket=meirorunnersource,s3Key=meirorunner-sim-app.tar.gz,architecture=X86_64 --robot-software-suite name=ROS,version=Melodic --simulation-software-suite name=Gazebo,version=9 --rendering-engine name=OGRE,version=1.x
```

Then, by going through the related menu in AWS, select "Simulation jobs" and "Create Simulation job". Follow through the prompts (please see Course 1b for more details):

1. On the Simulation configuration page:

    1. Simulation job duration: 1 hour

    2. Failure behavior. Choose fail to terminate the if the simulation job fails.

    3. ROS Distribution: Melodic

    4. IAM Role, select your previously create role or select Create new role to create one.
    
    5. Select the Default "VPC", Security Groups, and choose all subnets available. This step is necessary to enable the simulation and CloudWatch to work together. 

    6. Scroll down and choose Next.
    
2. On the Specify robot application page:

    1. Select Choose existing application: MeiroRunnerRobotApplication

    2. Launch package name: meiro_runner_robot

    3. Launch file: local_training.launch

    4. Add the following environment variables (left side is the name, right side is the value):
    
     ```
     MARKOV_PRESET_FILE meiro_runner.py
     MODEL_S3_BUCKET    meirorunnersimjoboutput
     MODEL_S3_PREFIX    model-store
     ROS_AWS_REGION     us-east-1
    ```

    5. Expand Robot application tools and select "Use default tools" to use preconfigured tools.

    6. Scroll down and choose Next.

3. Similarly, on the Specify simulation application page,

    1. Select Choose existing application: MeiroRunnerSimApplication

    2. Launch package name: meiro_runner_simulation

    3. Launch file: create_maze_world.launch

    4. Expand Simulation application tools and select "Use default tools"  to use preconfigured tools.

    5. Scroll down and choose Next.

4. Scroll down and select Create to create the simulation job.

Afterwards, the simulation job is created.  It will take a few minutes for the AWS RoboMaker simulation to start. You will see that the status “Preparing”. When "Preparing" becomes "Running", you can use the application tools typically used to inspect the behavior of the code can be launched, by clicking on `Connect`. You can use `GZClient` to see the Gazebo graphical user interface.

This simulation uses CloudWatch to plot the reward. The reward can be visualized by using the search bar in AWS, typing "CloudWatch". After clicking on it, the webpage will change. Click on "Metrics->All metrics". Then click on "AWSRoboMakerSimulation" and "Metrics with no dimensions". Select "MeiroRunnerEpisode". A plot like the following will appear.

{% include image.html url="/img/cloudwatch.png" description="Reward plot on CloudWatch." %}

---
Here a short video demonstrating the configuration of AWS RoboMaker.

{% include video-file.html url="/img/rl-aws" %}

After the training, the evaluation with the learned model can be run to see how well the robot achieves the task.

---
Before moving forward, let's assess our understanding of the material on reinforcement learning in the [next unit]({{ site.baseurl }}{% link _modules/mod-5e-assessment.md %}).
