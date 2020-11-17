---
# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved. // SPDX-License-Identifier: CC-BY-SA-4.0
title: Setting up AWS RoboMaker
permalink: /modules/3/robomaker.html
---

Now that we know the different pieces that are needed for AWS RoboMaker, let's learn the main steps on setting up an AWS RoboMaker environment. As you run through these steps, if you have a question you might find an answer in the [RoboMaker Q&A forum](https://forums.aws.amazon.com/forum.jspa?forumID=313).

The first step is to log in the AWS account -- if not created, please create a new account. As we have seen in the previous module, this involves setting up the different core elements needed for AWS RoboMaker.


## Setting up S3 Bucket
First, we will set the S3 Bucket, as shown in the following video.

{% include video-file.html url="/img/robomaker-s3" %}

This step will create the storage for containing the robot application.

## Setting up IAM permissions
Second, we set the IAM permissions so that the relevant services can be accessed as part of the RoboMaker. The video shows the steps.

{% include video-file.html url="/img/robomaker-iam" %}

EC2 is to allow access to computing resources, while S3 is to allow access to the storage. CloudWatch is for monitoring the simulation when running.

## Setting up EC2 and Cloud9

Finally, we can set up the RoboMaker development environment and create a new environment that will contain the code.

{% include video-file.html url="/img/robomaker-ide" %}

The environment from the example is set with ROS Melodic. The instance type refers to the hardware allocated for the development environment and simulation. [Here](https://aws.amazon.com/ec2/instance-types/) the specific types, and RoboMaker supports all of them but T2, which does not have enough resources to run ROS.

Now that we configured the environment, let's move to the [next unit]({{ site.baseurl }}{% link _modules/mod-3b-demo.md %}) to start a simple example and get comfortable with the environment.

{% capture ref %}
{% bibliography --cited %}
{% endcapture %}
{% include references.html ref=ref %}
