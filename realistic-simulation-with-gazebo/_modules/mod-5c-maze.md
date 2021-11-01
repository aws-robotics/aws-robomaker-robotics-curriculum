---
# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved. // SPDX-License-Identifier: CC-BY-SA-4.0
title: Maze example
permalink: /modules/5/maze.html
---

Let's define the different elements for a robot that needs to escape a maze:
- environment: 2D environment with obstacles and goal location.
- actions: 5 actions corresponding to rotation in place, left and right, move straight, and steering left and right.
- reward: the robot receives high positive and negative rewards when it gets to the goal or crashes, respectively. Otherwise, the rewards depends on how far the robot is from the walls.
- observations: determined by LiDAR data.

<img src="https://github.com/aws-samples/aws-robomaker-sample-application-meirorunner/raw/master/docs/images/image1.png" alt="AWS reinforcement learning sample application.">


---

This example implementation is based on a reinforcement learning framework from Intel Labs, Coach, providing learning algorithms, and an environment allowing for repeated simulation, OpenAI Gym, which integrates with Gazebo.

The main code to look at is how to define all the elements listed above. Let's look at the [corresponding code](https://github.com/aws-samples/aws-robomaker-sample-application-meirorunner/blob/master/robot_ws/src/rl_agent/markov/environments/meiro_runner_env.py).

Here, we highlight the main elements corresponding to the reinforcement learning problem.

The action space, i.e., the control of the robot, is discretized: the first value represents the steering angle (-1 for left, 0 straight, 1 right), and the second value for throttle (0 no throttle, 1 maximum forward velocity),
```
# actions -> steering angle, throttle
self.action_space = spaces.Box(low=np.array([-1, 0]), high=np.array([+1, +1]), dtype=np.float32)
```

The observation space consider the LiDAR which has 360 values, covering 360 degree field of view. The lowest value of the measurement is 0, while the highest is the maximum range.
```
high = np.array([LIDAR_SCAN_MAX_DISTANCE] * TRAINING_IMAGE_SIZE)
low = np.array([0.0] * TRAINING_IMAGE_SIZE)
self.observation_space = spaces.Box(low, high, dtype=np.float32)
```

The reward considers then different cases the robot can be in:
- the robot reached the goal results in a positive reward.
- the robot crashed results in a negative reward.
- the robot is close to a wall, but hasn't crashed yet results in a slightly negative reward.
- the robot is far from the wall results in a positive reward favoring throttle.
- the times the robot revisited the same area, which can result either in the same reward as crash, when revisiting an area more than twice, or a discount.

```
reward = 0
if self.last_position_x >= FINISH_LINE:
    print("Congratulations! You passed the finish line!")
    if self.steps == 0:
        reward = 0.0
        done = False
    else:
        reward = FINISHED / self.steps
        done = True
        
elif min_distance < CRASH_DISTANCE:
    # Robot likely hit the wall
    reward = CRASHED
    done = True
    
else:
    # When robot is close to the wall, give score based by how far from the wall. 
    # doubled the score when robot is leaving from the wall.
    if min_distance < 0.19:
        
        if min_distance < 0.15:
            reward = 0.05                
        elif min_distance < 0.17:
            reward = 0.15                
        else:
            reward = 0.35
            
        if min_distance - self.last_min_distance - 0.01 > 0:
            reward *= 2
            
        done = False
    else:
        # While the robot is away enough from the wall, give throttle value as score. 
        # (Max throttle 0.1 => 1.0 for score)
        reward = throttle * 10.0 
        done = False

# leave footstep marker to the place robot has passed through.
footstep_marker = self.calc_footsteps_mark_position(self.x, self.y)
if not self.last_footsteps_mark_position == footstep_marker:
    # if the robot had been already walked through that area more than twice, treat it as crashing.
    if self.footsteps_marker[footstep_marker[0]][footstep_marker[1]] > 1:
        reward = CRASHED
        done = True
    # if the robot had been already walked through that area, reduce the reward.
    elif self.footsteps_marker[footstep_marker[0]][footstep_marker[1]] > 0:
        reward = reward * 0.01
        
    self.footsteps_marker[footstep_marker[0]][footstep_marker[1]] += 1
    self.last_footsteps_mark_position = footstep_marker
    
self.reward_in_episode += reward
```

Note that there are a number of other functions in that file that allows for training, in particular, `reset()`, so that the simulation can be restarted, `step()` to send an action to the robot and receive the observation and reward. All other functions are helpers to interact with Gazebo and ROS. The structure of the code would be very similar for any other task: the main parts to change are the elements defined above and the access to such information.

---
Let's see in the [next unit]({{ site.baseurl }}{% link _modules/mod-5d-1-run-virtual-desktop.md %}) how to set it up on AWS RoboMaker.
