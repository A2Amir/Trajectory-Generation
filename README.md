# 1. Introduction
Now, I am  going to where we actually want to find a specific trajectory that the car can take. A trajectory is not just a curve that the car can follow but also a time sequence in which we say how fast the car should go. In finding trajectory, which is smooth and elegant as possible, there are many important things to watch out for:

* The most important one is don't crash
* Don't collide with something else 
* And also important is things like passenger comfort


In this lesson, I'm going to teach you about continuous trajectory planning and more specifically, how to generate drivable trajectories. A quick overview of the lesson:

* First, we will define what the motion planning problem is and discuss a few important concepts and priorities regarding motion planning algorithms.

* Then we'll do a quick review of A* to get you ready for the first new algorithm we will cover which is called Hybrid A* (As name suggests, Hybrid A* isn't purely discrete or continues).

* Next, we explore sampling based method called polynomial trajectory generation, which is really useful for highway driving.

# 2. The Motion Planning Problem

In this [repository](https://github.com/A2Amir/Search-Algorithms-A-Star-and-Dynamic-Prgramming) ,  you saw some best planning algorithms (Dynamic Programming, Optimal Policy, A* ), which solve the motion planning problem but we never formally defined that problem. I'd like to do that now.

Now, there is a word that you might encounter quite a bit if you start reading material regarding motion planning algorithm. That word is "configuration space," which defines all the possible configurations of a robot in the given world.

Consider the maze below where these worlds were all 2D grids, the robot configuration was sometimes two dimensional when we presented it as a x and y point and sometimes three dimensional when also including the robot's heading (x, y, heading). In fact, the configuration space for vehicle that can become even larger depending on what motion planning algorithms we decide to use. 

<p align="right"> <img src="./img/1.png" style="right;" alt=" the maze " width="600" height="400"> </p> 

With this idea of configuration space in mind, we can define a motion planning problem as follows. We're given three things:

* An initial configuration (q <sub> start</start>)
* A goal configuration(q <sub> goal</start>)
* Some constraints describing how the vehicle was allowed to move, its dynamics and the description of the environment.

Usually, the start configuration is the current configuration of the car given to us by the localization value and the sensors that give us information about car location, speed, acceleration they can go.

The behavior layer gives us a desired end configuration(q <sub> goal</start>) and maybe some constraints regarding where to go and at which speed.

Finally, the prediction completes this problem by giving us information about how the obstacle region will evolve in time.


**The motion planning problem can then be defined as final sequence of feasible movements in
the configuration space that's moved the robot from a start configuration to an end configuration without hitting any obstacles**.


