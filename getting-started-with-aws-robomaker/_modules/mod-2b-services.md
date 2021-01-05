---
# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved. // SPDX-License-Identifier: CC-BY-SA-4.0
title: AWS services at run-time
permalink: /modules/2/services.html
---

After setting up the core services, there are a number of services that are useful to know to fully exploit the potential from RoboMaker and AWS.

## Amazon CloudWatch
Amazon CloudWatch is a service that allows for complete observability of the system, e.g., by providing tools for monitoring application and for resource optimization {% cite aws2020cloudwatch %}. This is particularly useful for example to observe whether a robotic application will use too much resources making it not deployable on an embedded system.



## AWS IoT Greengrass and AWS Lambda
AWS IoT Greengrass allows local devices, such as robots, to work locally with the embedded system, at the same time connect to the cloud for management, analytics, and storage {% cite aws2020greengrass %}. The local device needs greengrass software to be installed to enable communication with the cloud.

Another service, AWS Lambda are functions that can respond to data produced by the devices. In addition, AWS Lambda can ensure a safe communication with other devices even when not connected to the Internet {% cite aws2020lambda %}.

These two services are useful for deployment of robotic applications on the physical robots.

## Other services for data processing

The previous two services are useful at run-time, for any type of robotics applications, especially thinking to connect to real robots.

There are a number of other services that can boost the intelligence of the robots and that are integrated within AWS RoboMaker:
- Amazon Polly for text-to-speech capabilities.
- Amazon Lex for dialogue capabilities {% cite aws2020polly %}.
- AWS Kinesis for  processing data -- e.g., audio, video, telemetry data -- in real-time that can be processed by other services {% cite aws2020kinesis %}.
- AWS Rekognition for image and video analysis {% cite aws2020reko %}, e.g., detecting objects and people for autonomous robots.



Now we should have a bigger picture about AWS RoboMaker and how that interfaces with the whole ecosystem. Before moving forward on setting up an AWS RoboMaker environment, [let's assess our knowledge on AWS]({{ site.baseurl }}{% link _modules/mod-2c-assessment.md %}).


{% capture ref %}
{% bibliography --cited %}
{% endcapture %}
{% include references.html ref=ref %}
