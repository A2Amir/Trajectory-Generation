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

1.**Combinatorial methods** usually consists in dividing the free space into small pieces and first solving
the motion planning problem by connecting the atomic elements. They are very intuitive ways to find initial approximate solution, but they usually do not scale well for large environments.

<p align="right"> <img src="./img/4.png" style="right;" alt=" Combinatorial methods " width="300" height="300"> </p> 

2.Next, **potential fields** are reacting methods. Each obstacle is going to create a sort of anti-gravity field, which makes it harder for the vehicle to come close to it. For example, you could imagine using this idea around pedestrians or bikes to encourage your planning algorithm to find trajectories that stay away from them. The main problem with most potential field methods is that they sometimes push us into local minima, which can prevent us from finding a solution.

<p align="right"> <img src="./img/5.png" style="right;" alt=" potential field methods " width="300" height="200"> </p> 


3.**Optimal control** consists in trying to solve the motion planning problem and the controlling input generation in one algorithm using a dynamic model of a vehicle or start configuration and end configuration. We want to generate a sequence of inputs, for example, steering angle and throttle inputs, that would lead us from start to end configuration while optimizing a cost function relative to the control inputs such as minimizing gas consumption and relative to the configuration of the car, such as staying at a distance from other vehicles.

There are a lot of very nice ways to do that. Most of them based on numerical optimization methods.However, it is hard to incorporate all of the constraints related to the other vehicles in a good enough way in order for these algorithms to work fast.

4.Finally, there are **sampling based methods**,which are what we will focus on today.These algorithms are very popular because they require a somewhat easier to compute definition of the free space. Sampling based methods use a collision detection module that probes the free space to see if a configuration is in collision or not.

Unlike combinatorial or optimal control methods, which analyzes the whole environment, not all parts of the free space need to be explored in order to find a solution. Explored parts are stored in a graph structure that can be searched with a graph search algorithm like Dijkstra or A star.

<p align="right"> <img src="./img/6.png" style="right;" alt=" sampling based methods " width="500" height="300"> </p> 

Two main classes of methods can be identified as sampling based:  

* Discrete methods, which rely on a finite set of configurations and/or inputs, like a grid superposed on top of our configuration space.

* Probabilistic methods, which rely on the probabilistic sample of a continuous configuration space. The set of possible configurations or states that will be explored is potentially infinite, which gives some of these methods the nice property that they are probabilistically complete and sometimes probabilistically optimal meaning that they will always find a solution if you allow them enough computation time.

We have scratched the surface on all the different sorts of planning algorithms that exist.


Next, I'm going to present the Hybrid A* algorithm but before that, I suggest you re-read this [repository](https://github.com/A2Amir/Search-Algorithms-A-Star-and-Dynamic-Prgramming) that you learned earlier on A*. 

# 3. The Hybrid A* algorithm

All true statements about [A*](https://github.com/A2Amir/Search-Algorithms-A-Star-and-Dynamic-Prgramming) are:
* It uses a discrete search space
* It uses an optimistic heuristic function to guide grid cell expansion.
* It always finds a solution if one exists (Completeness).
* Solutions it finds are not drivable (it depends on the search space that is being used and what it represents. If the search space represent only the position x,y and we're trying to move a car, there is no guarantees that the sequence of position returned by A* will be drivable but We can define a more complex search space including heading).
* Solutions it finds are always optimal assuming an admissible heuristic.

The fundamental problem here is A* is discrete whereas the robotic world is continuous so the question arises, is there version of A* that can deal with the continuous nature and give us probably executable paths? 

The key to solving this with A* has to do with a state transition function.Suppose we are in a cell like below and you play a sequence of very small steps simulations using our continuous math from before, then a state at the point A might find itself right in the corner (B) of the next discrete state. 

Instead of assigning just the next discrete state to the grid cell, an algorithm called Hybrid A* memorizes the exact X prime Y prime and theta and associate it with the next discrete state, the first time the grid cell (the next discrete state cell) is expanded.  Then when expanding from the next discrete state it uses a specific starting point (B) to figure out what the next cell might be.

<p align="right"> <img src="./img/7.png" style="right;" alt=" Hybrid A*  " width="500" height="200"> </p> 

Like below, it might happen that the same cell used to get in A* maybe from C going into a different continuous polymerization of X, Y and theta (D), because in A* we tend to expand cells along the shortest path before we look the longer paths, we not just cut this(C to D) off and never consider again. This leads to a lack of completeness, which means there might be solutions to the navigation problem that this algorithm doesn't capture. It does give us correctness so as long as our motion equations are correct, the resulting paths can be executed. 

<p align="right"> <img src="./img/8.png" style="right;" alt=" Hybrid A*  " width="500" height="200"> </p> 

Now, here is a caveat, every time we expand the grid cell, we memorize explicitly the continuous values of X prime, Y prime and theta with the grid cell. 

All true statements about Hybrid A* are:

* It uses a continuous search space
* It uses an optimistic heuristic function to guide grid cell expansion.
* It does not always find a solution if one exists.
* Solutions it finds are drivable.
* Solutions it finds are not always optimal.

## 3.1 Hybrid A* in Practice

In the image above about Hybrid A*, the update equations that we used were somewhat generic to most X Y field configuration spaces. In particular, we don't specify what Omega (w) is. Omega is the heading rate of change (that would mean that the robot can turn around its Z axis without constraints). Now for some robots, it might be that we can specify any omega independently of its state. For car though, this is not very realistic and we should probably use the bicycle model.

<p align="right"> <img src="./img/9.png" style="right;" alt="  the update equation  " width="300" height="150"> </p> 

The bicycle model gives us an equation for Omega like below. Here, 'L' is the distance between the front and rear axle , 'δ' is the steering angle and ⱴ is the constant positive velocity. 
<p align="right"> <img src="./img/10.png" style="right;" alt="   bicycle model gives us an equation for Omega " width="300" height="250"> </p> 

Now, for our Hybrid A* algorithm, we need to decide which configuration can be reached from the current node considered, in order to add them to the open set. In practice, we would probably use some number of steering angles between maximum left steer and maximum right steer angle but the more different Deltas you add the longer it takes for your algorithm to complete.

Therefore, in this example, we will only choose three angles to pick delta from:

* The maximum left steering angle.
* Zero steering angle.
* The maximum right steering angle.

In the case of a car that can turn its wheels 35 degrees at most, that would be -35, 0 and 35.This allows us to expand the search tree with three new motion primitives
* go straight
* steer left 
* steer right.

Before seeing below how that works in a robot maze i should mention:

* I will use gray color to indicate adding a cell to open list.
* We would generate three trajectories, the one for going straight, the one for turning right, the one for turning left. In this case, we can discard the trajectories that go off the map or collide.
* Closed cell is indicated with the green check-mark.
* In all these steps, I'm keeping track of not just what cell I am in but also where in that cell I am, including orientation.

<p align="right"> <img src="./img/9.gif" style="right;" alt="  in a robot maze " width="500" height="350"> </p> 


Sometimes, the algorithm fails to find the solution when one actually exists and Hybrid A* will not find any. We can reduce the likelihood of this problem by increasing the resolution of our grid or adding a third dimension to the search space for the heading. Indeed, this would allow us not to close the position in the map for all headings but only for a specific range, which may allow us to find the solution.

## 3.2 Hybrid A* Heuristics


Hybrid A* is guided by two heuristics, called the non-holonomic-without-obstacles heuristic and the holonomic-with-obstacles heuristic. As the name suggests, the first heuristic ignores obstacles but takes into account the non-holonomic nature of the car. This heuristic, which can be completely pre-computed for the entire 4D space (vehicle location, and orientation, and direction of motion), helps in the end-game by approaching the goal with the desired heading. The second heuristic is a dual of the first in that it ignores the non-holonomic nature of the car, but computes the shortest distance to the goal. It is calculated online by performing dynamic programming in 2-D (ignoring vehicle orientation and motion direction). 

Both heuristics (h1, h2) are admissible and the following combinations are also valid admissible combinations:
* (h1+h2)/2
* max(h1,h2)
* min(h1,h2)

The paper [Junior: The Stanford Entry in the Urban Challenge](https://d17h27t6h515a5.cloudfront.net/topher/2017/July/595fe838_junior-the-stanford-entry-in-the-urban-challenge/junior-the-stanford-entry-in-the-urban-challenge.pdf) is a good read overall, but Section 6.3 - Free Form Navigation is especially good and goes into detail about how the Stanford team thought about heuristics for their Hybrid A* algorithm (which they tended to use in parking lots).

## 3.3 Hybrid A* Pseudocode
The pseudocode below outlines an implementation of the A* search algorithm using the bicycle model. The following variables and objects are used in the code but not defined there:

* State(x, y, theta, g, f): An object which stores x, y coordinates, direction theta, and current g and f values.
* grid: A 2D array of 0s and 1s indicating the area to be searched. 1s correspond to obstacles, and 0s correspond to free space.
* SPEED: The speed of the vehicle used in the bicycle model.
* LENGTH: The length of the vehicle used in the bicycle model.
* NUM_THETA_CELLS: The number of cells a circle is divided into. This is used in keeping track of which States we have visited already.

The bulk of the hybrid A* algorithm is contained within the search function. The expand function takes a state and goal as inputs and returns a list of possible next states for a range of steering angles. This function contains the implementation of the bicycle model and the call to the A* heuristic function.
~~~c++

def expand(state, goal):
    next_states = []
    for delta in range(-35, 40, 5): 
        # Create a trajectory with delta as the steering angle using 
        # the bicycle model:

        # ---Begin bicycle model---
        delta_rad = deg_to_rad(delta)
        omega = SPEED/LENGTH * tan(delta_rad)
        next_x = state.x + SPEED * cos(theta)
        next_y = state.y + SPEED * sin(theta)
        next_theta = normalize(state.theta + omega)
        # ---End bicycle model-----

        next_g = state.g + 1
        next_f = next_g + heuristic(next_x, next_y, goal)

        # Create a new State object with all of the "next" values.
        state = State(next_x, next_y, next_theta, next_g, next_f)
        next_states.append(state)

    return next_states

def search(grid, start, goal):
    # The opened array keeps track of the stack of States objects we are 
    # searching through.
    opened = []
    # 3D array of zeros with dimensions:
    # (NUM_THETA_CELLS, grid x size, grid y size).
    closed = [[[0 for x in range(grid[0])] for y in range(len(grid))] 
        for cell in range(NUM_THETA_CELLS)]
    # 3D array with same dimensions. Will be filled with State() objects 
    # to keep track of the path through the grid. 
    came_from = [[[0 for x in range(grid[0])] for y in range(len(grid))] 
        for cell in range(NUM_THETA_CELLS)]

    # Create new state object to start the search with.
    x = start.x
    y = start.y
    theta = start.theta
    g = 0
    f = heuristic(start.x, start.y, goal)
    state = State(x, y, theta, 0, f)
    opened.append(state)

    # The range from 0 to 2pi has been discretized into NUM_THETA_CELLS cells. 
    # Here, theta_to_stack_number returns the cell that theta belongs to. 
    # Smaller thetas (close to 0 when normalized  into the range from 0 to 
    # 2pi) have lower stack numbers, and larger thetas (close to 2pi when 
    # normalized) have larger stack numbers.
    stack_num = theta_to_stack_number(state.theta)
    closed[stack_num][index(state.x)][index(state.y)] = 1

    # Store our starting state. For other states, we will store the previous 
    # state in the path, but the starting state has no previous.
    came_from[stack_num][index(state.x)][index(state.y)] = state

    # While there are still states to explore:
    while opened:
        # Sort the states by f-value and start search using the state with the 
        # lowest f-value. This is crucial to the A* algorithm; the f-value 
        # improves search efficiency by indicating where to look first.
        opened.sort(key=lambda state:state.f)
        current = opened.pop(0)

        # Check if the x and y coordinates are in the same grid cell 
        # as the goal. (Note: The idx function returns the grid index for 
        # a given coordinate.)
        if (idx(current.x) == goal[0]) and (idx(current.y) == goal.y):
            # If so, the trajectory has reached the goal.
            return path

        # Otherwise, expand the current state to get a list of possible next states.
        next_states = expand(current, goal)
        for next_s in next_states:
            # If we have expanded outside the grid, skip this next_s.
            if next_s is not in the grid:
                continue
            # Otherwise, check that we haven't already visited this cell and that there is not an obstacle in the grid there.
            stack_num = theta_to_stack_number(next_s.theta)
            
            if closed[stack_num][idx(next_s.x)][idx(next_s.y)] == 0 and grid[idx(next_s.x)][idx(next_s.y)] == 0:
                    # The state can be added to the opened stack.
                    opened.append(next_s)
                    # The stack_number, idx(next_s.x), idx(next_s.y) tuple has now been visited, so it can be closed.
                    closed[stack_num][idx(next_s.x)][idx(next_s.y)] = 1
                    # The next_s came from the current state, and is recorded.
                    came_from[stack_num][idx(next_s.x)][idx(next_s.y)] = current
~~~


## 3.4 [Implement Hybrid A*](https://github.com/A2Amir/Trajectory-Generation/blob/master/C%2B%2B/HybirdAStar.cpp) in C++ 

In this section, is provided a working implementation of a breadth first search algorithm which does not use any heuristics to improve its efficiency.

One possible implementation for Hybrid A* is to try to make the appropriate modifications to the breadth first search algorithm so that it takes advantage of heuristic functions (possibly the ones mentioned in the previous section using the "distance to goal" heuristic function) to reduce the number of grid cell expansions required. In this implementation, we have added an f value to the maze_s struct, which is set in the expand function. Additionally, we've added two new functions: heuristic and compare_maze_s. The compare_maze_s function is used for comparison of maze_s objects when sorting the opened stack.

To get an even lower number of expansions, try reducing NUM_THETA_CELLS in hybrid_breadth_first.h to reduce the total number of cells that can be searched in the closed array. Be careful though! Making NUM_THETA_CELLS too small might result in the algorithm being unable to find a path through the maze. 

Another possibility for improvement is to use the regular A* algorithm to assign a cost value to each grid cell. This grid of costs can then be used as the heuristic function, which will lead to an extremely efficient search. If you are looking for additional practice, give this a try! 


## 3.5 Environment Classification


We have just implemented Hybrid A* which is one of the best algorithm for finding trajectories in unstructured environments. An example of such an environment is a parking lot or a maze and these environments tend to have less specific rules than highway or streets and lower driving speeds. They also do not have an obvious reference path that corresponds to what we should be driving 90% of the time because they change so much.

On the other hand, highway or streets are very structured environments and in these environments, all motion is more constrained by predefined rules regarding how we can move on the road. In the image below, you can see below differences between unstructured and structured environments.

<p align="right"> <img src="./img/11.png" style="right;" alt="  Environment Classification " width="500" height="250"> </p> 


All these rules impose constraints, which have to be satisfied but also for guidance regarding what the trajectory should look like. While a Hybird A* is great at finding solutions everywhere, it does not take advantage of this information (For example, the road structure itself can be used as a reference path). Let's see how we can use it.


## 3.6 The Need for Time


Instead of Cartesian coordinates we will be relying heavily on Frenet coordinates, which help us simplify the problem and as a reminder of how Frenet coordinates  work, you can check this [repository](https://github.com/A2Amir/Prediction-Phase-in-the-trajectory-generation-of-cars).

Now I'm going to emphasize why it is important to not just generate a sequence of configurations for vehicle using algorithms like A* and Hybrid A*, which refers as path planning, but also to decide when we're going to be in each configuration. let's see the gif below using discrete time increments.

<p align="right"> <img src="./img/12.gif" style="right;" alt="discrete time increments" width="500" height="250"> </p> 

As see in the gif, there is traffic on highways, which means that the traversal space is constantly changing over time. To avoid collision in dynamic environment, we need to think about time as well and that means that driving in traffic is fundamentally a three dimensional(s and d from FreNet Coordinate and t from time) problem.

The A* and Hybird A*algorithms play an important role in autonomous driving but the path they generate don't really take predictions into account. For example, let's project back to when the blue vehicle first wanted to change lane in the gif above. Here we will include the time as a third dimension.

<p align="right"> <img src="./img/13.gif" style="right;" alt="discrete time increments" width="500" height="250"> </p> 

It is not sufficient to solve the planning problem when there are dynamic objects around us which in autonomous driving is all the time. Therefore, we have to plan not only a sequence of configuration,S d and theta but also have these configurations all set up in time.The trajectory is a function which for any time t associates a configuration of a vehicle. 

<p align="right"> <img src="./img/12.png" style="right;" alt="configurations all set up in time" width="300" height="90"> </p> 

Now, we need a way to generate such a trajectory and the way we're going to do this, is by separating the planning we do in the S dimension from the planning we do in a d dimension. Instead of thinking about one picture in three dimension, we usually decide to reason about two pictures. Each of which is in two dimensions. I'm going to walk through the same vehicle tracking example you see in the previous gif.

<p align="right"> <img src="./img/15.gif" style="right;" alt="separating the planning we do in the S dimension from the planning we do in a d dimension" width="500" height="300"> </p> 

As seen above it's a trajectory and we can fully describe what the vehicle did within these 15 seconds.

# 4. Structured Trajectory Generation
In the next few segments, I'm going to present a method for trajectory planning that is useful in structured environments like highway driving. To do that, I'll introduce the idea of Jerk minimization to generate a nice trajectory from a start configuration to a goal configuration.

I will show you how to generate Jerk minimizing trajectories using polynomials and then present a brief derivation on how to compute the coefficients of the polynomial that solves our problem.

Then I will show you an example of what one such trajectory looks like on the highway and how we can evaluate its drive-ability and compare it to other trajectories.

Finally, I will show you how by generating many similar jerk minimizing trajectories we can compare them and select the best one for the situation we want to drive through.

## 4.1 Trajectories with Boundary Conditions

By assuming, a situation like below s, t and d, t trajectories turns out that these aren't physically possible. The kink in the slope would translate in an instantaneous jumping speed, which would require infinite acceleration.

<p align="right"> <img src="./img/16.png" style="right;" alt="Trajectories with Boundary Conditions" width="500" height="320"> </p> 

Now, if we were to send them (s, t and d, t trajectories) to our motion control module, maybe they can track this trajectory, but probably not and even if they tried to track it, it would result in a very strong acceleration of our car, which is both uncomfortable and dangerous.


This is why we need both continuity and smoothness in our trajectories. But how much continuity and smoothness?

We know we want continuity in position, since our car cannot teleport and it would not make sense to compute a trajectory starting 10 metres ahead of us and we also know that our car cannot instantaneously change speed, therefore the speed should also be continuous. Following this logic you could also say that the acceleration should be continuous and maybe the acceleration's derivity as well.

**Position becomes velocity, velocity because acceleration, acceleration becomes jerk, and going even further you get snap, crackle, and pop**. It turns out that one of these quantities is directly related to human perception of comfort. Jerk is the quantity that humans perceive as uncomfortable. We can tolerate high acceleration but we don't like it when our acceleration changes too quickly which is high jerk. When we design trajectories for self-driving car, we would like it, if they were jerk minimizing.

## 4.2 Jerk Minimizing Trajectories

What is a jerk minimizing trajectory? An interesting and useful fact to us is that it is relatively easy to compute a jerk optimal trajectory in one dimension. In fact, consider a function s of t different from time zero to time tf.

<p align="right"> <img src="./img/17.png" style="right;" alt="Jerk Minimizing Trajectories" width="200" height="60"> </p> 

recall that jerk is a third derivative of position. The total jerk accumulated over the duration of the trajectory and here, we want to analyze both positive and negative jerk therefore, we're going to look at total square jerk.

Our problem is then to find a function s of t that minimizes the integral in the image below. If we go through the math required to find the minimum, you'll find that all the time derivatives of s of order six and more have to be zero in order for S to be jerk minimal.

<p align="right"> <img src="./img/18.png" style="right;" alt="Jerk Minimizing Trajectories" width="200" height="150"> </p> 

Now, we can also use a helpful fact about functions, which is that any function can be written like below.  

<p align="right"> <img src="./img/19.png" style="right;" alt="Jerk Minimizing Trajectories" width="350" height="120"> </p> 

When you take this form of s and add in the information that we have about it's time derivative, you find, All of the coefficient bigger than five are zero, which means that all minimum jerk trajectories can be represented as a fifth order polynomial like below.

<p align="right"> <img src="./img/20.png" style="right;" alt="Jerk Minimizing Trajectories" width="350" height="200"> </p> 

You can see the equation has six coefficients and six coefficients means six tuneable parameters that we can choose from in order to define the shape of a 1D trajectory and in practice, we use them to specify the boundary conditions of our trajectory.

The things we'd like to constrain are the initial position, velocity, and acceleration as well as the final position, velocity and acceleration. Now, this is just the 1D representing the longitude of displacement but, the same for the lateral displacement applies. This gives us 12 variables to pick in order to fully define the motion of our vehicle in S&D over time.

<p align="right"> <img src="./img/21.png" style="right;" alt="Jerk Minimizing Trajectories" width="350" height="150"> </p> 

## 4.3 Derivation Overview

In the previous section, I showed you that the position of a jerk minimizing trajectory is given by a quintic polynomial. In this section, I'm going to present a quick and non-rigorous walk through of how we find the six coefficients that describe these polynomial. 

First, we differentiate the equations of the position in order to get the equations for the velocity and then we differentiate it again in order to get the equation for the acceleration. Now, we could just plug in our six boundary conditions to get the six equations. 

<p align="right"> <img src="./img/22.png" style="right;" alt="Derivation Overview" width="350" height="150"> </p> 


But first we're going to do something that will make our life a little bit easier. We're going to set the initial time as zero and when we do that, we find 3 equations which means that three or four unknowns don't need to be identified anymore, which reduces the program from six unknowns to three. 

<p align="right"> <img src="./img/23.png" style="right;" alt="Derivation Overview" width="350" height="190"> </p> 

As seen below It simplifies those three trajectories by removing three of the unknowns here. 

<p align="right"> <img src="./img/24.png" style="right;" alt="Derivation Overview" width="700" height="150"> </p> 

We can gather the known terms into functions of the star boundary conditions to make it look like a bit cleaner. 

<p align="right"> <img src="./img/25.png" style="right;" alt="Derivation Overview" width="350" height="140"> </p> 

The C1, C2, and C3 terms are functions of the initial conditions what remains is that we only have to identify three parameters(α<sub>3</sub>, α<sub>4</sub>, α<sub>5</sub>) that depends only on our N boundary condition. Those equations look like below and we know the final position, velocity and acceleration (s<sub>f</sub>,s˙<sub>f</sub>,s¨<sub>f</sub>) at time t<sub>f</sub>, since those are all the quantities that we beat.

<p align="right"> <img src="./img/26.png" style="right;" alt="Derivation Overview" width="350" height="190"> </p> 

Since we know them, a more useful way to think of these equations is like b, where I have deliberately separated the known quantities from the unknown variables that we wish to determine. 

<p align="right"> <img src="./img/27.png" style="right;" alt="Derivation Overview" width="500" height="150"> </p> 


This is starting to look like a very solvable system of three equations and the best way to solve this is with a matrix that looks like below.
<p align="right"> <img src="./img/28.png" style="right;" alt="Derivation Overview" width="500" height="190"> </p> 

The problem can be solved by inverting this matrix using any matrix mathematical library, which we will do later. 

# 5. Polynomial Trajectory Generation

I am  going to to implement a polynomial solver to generate Jerk minimizing trajectories.

First, I want to show you how it's being used. Let's consider the S coordinates first. As an input, it takes the current vehicle state, the goal state, and the duration and as an output, it will generate six coefficients, which uniquely define the polynomial that describes the longitudinal trajectory.Similarly, we feed input parameters to our solver regarding lateral configurations and compute the lateral trajectory.

<p align="right"> <img src="./img/29.png" style="right;" alt="Polynomial Trajectory Generation" width="500" height="300"> </p> 


## 5.1 Implement [Quintic Polynomial Solver](https://github.com/A2Amir/Trajectory-Generation/blob/master/C%2B%2B/PolynomialSolver.cpp) in C++

In this section you will implement a quintic polynomial solver. This will let you take boundary conditions as input and generate a polynomial trajectory, which matches those conditions with minimal jerk.

* Inputs: the solver will take three inputs.
1. start - [si,si˙,si¨]
2. end - [sf,sf˙,sf¨]
3. T - the duration of maneuver in seconds.
* Tips: Remember, you are solving a system of equations: matrices will be helpful! The equations for position, velocity, and acceleration are given by:

<p align="right"> <img src="./img/30.png" style="right;" alt="Polynomial Trajectory Generation" width="500" height="120"> </p> 

All these quantities are known except for α3,α4,α5.




