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

<p align="right"> <img src="./img/1.png" style="right;" alt=" the maze " width="200" height="150"> </p> 

With this idea of configuration space in mind, we can define a motion planning problem as follows. We're given three things:

* An initial configuration (q <sub> start</sub>)
* A goal configuration(q <sub> goal</sub>)
* Some constraints describing how the vehicle was allowed to move, its dynamics and the description of the environment.

Usually, the start configuration is the current configuration of the car given to us by [the localization](https://github.com/A2Amir/Markov-Localization-and-the-Kidnapped-Vehicle-) value and the sensors that give us information about car location, speed, acceleration they can go.

The behavior layer gives us a desired end configuration(q <sub> goal</sub>) and maybe some constraints regarding where to go and at which speed.

Finally, the prediction completes this problem by giving us information about how the obstacle region will evolve in time.


**The motion planning problem can then be defined as final sequence of feasible movements in
the configuration space that's moved the robot from a start configuration to an end configuration without hitting any obstacles**.


## 2.1 Properties of Motion Planning Algorithms
When discussing planning algorithms, there are two important properties that we like to talk about.

The first one is called **completeness**. This means that if a solution exists through the multiplying problem, the planner will find it and if a solution does not exist, the planner will terminate and report that no solution exists.

Consider the following two situations. In the first one, the complete algorithm might correctly identify this as a solution and the second one, the algorithm would terminates and tell us there is no solution.


<p align="right"> <img src="./img/2.png" style="right;" alt=" the maze " width="500" height="300"> </p> 

Now, there may be something bothering you about the solution identified in the first example, which brings us to the second property, **optimality**. A planning algorithm is optimal if it always return the sequence, which minimizes some cost function. If we're using total distance traveled as a cost function for example, then the following situation would be an optimal solution for the first one.

<p align="right"> <img src="./img/3.png" style="right;" alt=" the maze " width="200" height="150"> </p> 


## 2.2 Types of Motion Planning Algorithms
There are many classes of motion planning algorithms and today we'll focus on one of these classes, but it's worth mentioning the others.

**Combinatorial methods** usually consists in dividing the free space into small pieces and first solving
the motion planning problem by connecting the atomic elements. They are very intuitive ways to find initial approximate solution, but they usually do not scale well for large environments.

<p align="right"> <img src="./img/4.png" style="right;" alt=" Combinatorial methods " width="300" height="300"> </p> 

Next, **potential fields** are reacting methods. Each obstacle is going to create a sort of anti-gravity field, which makes it harder for the vehicle to come close to it. For example, you could imagine using this idea around pedestrians or bikes to encourage your planning algorithm to find trajectories that stay away from them. The main problem with most potential field methods is that they sometimes push us into local minima, which can prevent us from finding a solution.

<p align="right"> <img src="./img/5.png" style="right;" alt=" potential field methods " width="300" height="200"> </p> 


**Optimal control** consists in trying to solve the motion planning problem and the controlling input generation in one algorithm using a dynamic model of a vehicle or start configuration and end configuration. We want to generate a sequence of inputs, for example, steering angle and throttle inputs, that would lead us from start to end configuration while optimizing a cost function relative to the control inputs such as minimizing gas consumption and relative to the configuration of the car, such as staying at a distance from other vehicles.

There are a lot of very nice ways to do that. Most of them based on numerical optimization methods.However, it is hard to incorporate all of the constraints related to the other vehicles in a good enough way in order for these algorithms to work fast.

Finally, there are **sampling based methods**,which are what we will focus on today.These algorithms are very popular because they require a somewhat easier to compute definition of the free space. Sampling based methods use a collision detection module that probes the free space to see if a configuration is in collision or not.

Unlike combinatorial or optimal control methods, which analyzes the whole environment, not all parts of the free space need to be explored in order to find a solution. Explored parts are stored in a graph structure that can be searched with a graph search algorithm like Dijkstra or A star.

<p align="right"> <img src="./img/6.png" style="right;" alt=" sampling based methods " width="500" height="300"> </p> 

Two main classes of methods can be identified as sampling based:  

* Discrete methods, which rely on a finite set of configurations and/or inputs, like a grid superposed on top of our configuration space.

* Probabilistic methods, which rely on the probabilistic sample of a continuous configuration space. The set of possible configurations or states that will be explored is potentially infinite, which gives some of these methods the nice property that they are probabilistically complete and sometimes probabilistically optimal meaning that they will always find a solution if you allow them enough computation time.

We have scratched the surface on all the different sorts of planning algorithms that exist.


Next, I'm going to present the Hybrid A* algorithm but before that, I suggest you re-read this repository that you saw earlier on A*. 



