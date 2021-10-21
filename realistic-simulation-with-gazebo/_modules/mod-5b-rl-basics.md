---
# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved. // SPDX-License-Identifier: CC-BY-SA-4.0
title: Reinforcement learning basics
permalink: /modules/5/rl-basics.html
---

The purpose of reinforcement learning is to learn to make good sequences of decisions. As depicted in the following figure, the robot (or agent in general) has repeated interactions with the world, gets a reward for each decision, without necessarily having any prior knowledge about the world.

<a title="Megajuice, CC0, via Wikimedia Commons" href="https://commons.wikimedia.org/wiki/File:Reinforcement_learning_diagram.svg"><img width="256" alt="Reinforcement learning diagram" src="https://upload.wikimedia.org/wikipedia/commons/thumb/1/1b/Reinforcement_learning_diagram.svg/256px-Reinforcement_learning_diagram.svg.png"></a>

The goal is then to optimize decisions that yield to the best outcome. This is a difficult problem as the consequences are delayed. In order to learn, the robot needs to explore the possible decisions and observe the rewards. At the same time, the robot might want to exploit the known information to maximize the reward. 

To formulate a reinforcement learning problem, the following main elements need to be defined:
- the set of states the robot can be in, called state space. For example, the position of the robot.
- the set of observations the robot can have access to about the state of the environment. For example, a robot in a maze can only see the portion of the maze where the robot is.
- the set of actions that the robot can take, called action space. For example, the robot motion.
- a reward function that returns an immediate reward after transitioning from one state to another after the robot took an action.
- a transition function that, given the current state and the action, will lead to a new state.

The robot can be trained in several episodes. In each episode, the robot takes a sequence of actions from the initial state until the terminal state. For example, in a maze, a terminal state can be the robot reaching the exit or the robot colliding with an obstacle.

The details about the methods that are available can be found in the following reinforcement learning book {% cite sutton2018reinforcement %}

---
Let's see in the [next unit]({{ site.baseurl }}{% link _modules/mod-5c-maze.md %}) the example of a robot escaping from a maze.


{% capture ref %}
{% bibliography --cited %}
{% endcapture %}
{% include references.html ref=ref %}
