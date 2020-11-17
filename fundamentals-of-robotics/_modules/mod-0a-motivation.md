---
# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved. // SPDX-License-Identifier: CC-BY-SA-4.0
title: Motivation
permalink: /modules/0/motivation.html
menus: header
---
In recent years, autonomous mobile robotics is increasingly becoming an effective way to carry out tasks that are difficult, dangerous, or simply boring for humans. Here a few examples, representative of some current trends.


{% capture text_exploration %}
## Exploration
Space and ocean have been fascinating the human kind and multiple expeditions have been carried out over the years to learn more about these environments. Nevertheless, such missions are very dangerous. Hence, it is natural to send autonomous robots. NASA's Jet Propulsion Laboratory has participated in more than 100 mission to explore such unreachable environments {% cite jpl %}. Many inventions were created as part of these research projects, including camera phones and CAT scanners  {% cite jpl-inventions %}.
{% endcapture %}
{% capture video_ocean %}
  {% include video-youtube.html url="https://www.youtube.com/embed/nBzBeYsdNsM" %}
{% endcapture %}
{% include two-columns.html description=text_exploration media=video_ocean position="right" %}

{% capture video_logistics %}
  {% include video-youtube.html url="https://www.youtube.com/embed/4MH7LSLK8Dk" %}
{% endcapture %}
{% capture text_logistics %}
## Logistics
In the last decade, the retail industry has been undergoing a revolution through e-commerce. A recent research showed that worldwide retail e-commerce sales amounted to 3.53 trillion US dollars and they are projected to grow to 6.53 trillion US dollars in 2022 {% cite emarketer2019 %}. Such a revolution resulted in a high need of logistics infrastructure and workforce to handle the large volumes of shipments. However, the decline in population levels in the Western world leads to a shortage in the available workforce, with potential significant impacts in the next decade {% cite robotics2016dpdhl %}. Robots have been successfully used in some controlled scenarios, as in warehouses -- e.g., the Kiva robots at Amazon -- where robots collaborate with humans to pack items.
{% endcapture %}
{% include two-columns.html description=text_logistics media=video_logistics position="right" %}

{% capture text_delivery %}
## Delivery
Last-mile delivery has been receiving more attention and significant investment to meet the future demand -- from 390 million US$ in 2014 to 3.9 billion US$ in 2018 {% cite deloitte2019 %}. Aerial drones are under research and development by many of the big companies for the last-mile delivery, especially to cover rural areas.
{% endcapture %}
{% capture video_drones %}
  {%include video-file.html url="https://m.media-amazon.com/images/G/01/acs/test/jr/121216/PrimeAirVideo._CB509077587_" %}
{% endcapture %}
{% include two-columns.html description=text_delivery media=video_drones position="right" %}

{% capture text_transportation %}
## Transportation
According to the National Highway Traffic Safety Administration (NHTSA), 94% of the serious crashes are due to human error {% cite nhtsa %}. Self-driving cars are expected to revolutionize the transportation sector and to contribute to reducing fuel usage, carbon emissions, and  traffic congestion. While it is believed unlikely that fully autonomous cars will reach wide acceptance  a report from Business Wire {% cite businesswire2019 %}, many companies are significantly invested in research and development of autonomous cars -- almost 2.9 million miles driven by autonomous cars in California {% cite dmvca %}.
{% endcapture %}
{% capture video_cars %}
  {% include video-youtube.html url="https://www.youtube.com/embed/aaOB-ErYq6Y" %}
{% endcapture %}
{% include two-columns.html description=text_transportation media=video_cars position="right" %}


How do the robots seen in some of these applications work? Let's discover it through this brief introduction to robotics [course]({{ site.baseurl }}{% link _modules/mod-0b-objectives.md %}).

{% capture ref %}
{% bibliography --cited %}
{% endcapture %}
{% include references.html ref=ref %}
