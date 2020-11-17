---
# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved. // SPDX-License-Identifier: CC-BY-SA-4.0
title: AWS core services
permalink: /modules/2/aws.html
---

As we discovered what AWS RoboMaker is and the features it provides, let's discover in this module what are the AWS services that are needed to configure AWS RoboMaker. We will find out in the following module how to set up an AWS RoboMaker environment.


## What is needed for AWS RoboMaker?

AWS provides on-demand cloud computing platforms. Such cloud computing platforms are composed of distributed computer nodes and storage units that are allocated on demand to different users. From the user perspective, cloud services allow developers to ignore how the underlying infrastructure or service is maintained, and focus on the specific service being offered by the developers.

Many of the concepts that you might be familiar with for a local machine still apply in this context.

To get started, an AWS account is needed to access available AWS infrastructure and services. [Here](https://aws.amazon.com/premiumsupport/knowledge-center/create-and-activate-aws-account/) the instructions to create an account. Similarly to a user on a computer, this is needed to properly allocate the resources, and in the case of the cloud service, appropriately bill for usage of cloud services. If you are an educator or student access to AWS can also be achieved through [AWS Educate](https://aws.amazon.com/education/awseducate/).

### Identity and Access Management (IAM)
As any machine, users need to be granted permissions to different resources. AWS manages this with the Identity and Access Management (IAM). This allows for example other developers part of the same team to use the same resources, computing or storage. Permissions need to be granted to AWS RoboMaker. More details about IAM are included here {% cite aws2020iam %}.

### Amazon S3

Because now machines are distributed, another service necessary is the Amazon Simple Storage Service (Amazon S3), which provides the infrastructure to store data in the cloud. In particular, for AWS RoboMaker, we need to store our application and the log files. More details are included here {% cite aws2020s3 %}.

### Amazon EC2

Finally, we need computing resources, to run the services. Amazon offers Elastic Computer Cloud (Amazon EC2). This will allow us to run services for robotics software development, such as the development environment and the simulation. More general details are here {% cite aws2020ec2 %}.


All the three components just mentioned are needed to run any cloud service. For AWS RoboMaker, the other fundamental component is AWS Cloud9, which is a cloud-based integrated development environment that works directly on the browser and can run the simulation. In terms of interface, it offers pretty much similar capabilities as any other popular IDEs, such as Eclipse, Visual Studio, and Netbeans. More details on how to get started can be found here {% cite aws2020cloud9 %}.

Note that the cloud platform relies on a global infrastructure, with  centers in different regions -- [here](https://aws.amazon.com/about-aws/global-infrastructure/regions_az/?p=ngi&loc=2) the map with the different regions available. Some of the services might not be available in all regions. AWS RoboMaker for example is available in 7 regions ([here](https://docs.aws.amazon.com/general/latest/gr/robomaker.html) the details).

As these resources will be available for specific accounts, there is an important concept that is worth to introduce: the Amazon Resource Names (ARNs) which **uniquely** identify AWS resources.  This is necessary to define for example IAM policies of specific Application Programming Interface (API) calls. A typical format is `arn:partition:service:region:account-id:resource-id`, where `partition` identifies a group of AWS regions; `service` identifies the specific service -- e.g., for S3, it will be `s3`; `region`, for example `us-east-1` for US East (N. Virginia); `account-id` is the ID of the AWS account; and `resource-id` is the resource identifier.

These tools are enough to start the development. Let's see in the [next unit]({{ site.baseurl }}{% link _modules/mod-2b-services.md %}) other services that are useful at runtime.


{% capture ref %}
{% bibliography --cited %}
{% endcapture %}
{% include references.html ref=ref %}
