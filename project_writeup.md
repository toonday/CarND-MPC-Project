
---

**Model Predictive Control (MPC) Controller Project**

## [Rubric](https://review.udacity.com/#!/rubrics/896/view) Points
###Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
###Writeup / README

###Reflection

####1. The Model.

In this project, I used a Model Predictive Controller (MPC) to manipulate the controls of the vehicle so that it follows a desired path/trajectory.
In order to make my MPC model work I needed to get the current state of the vehicle at time t (lines 87 - 141 in the `main.cpp` file).
The state of the vehicle is a group of variables that model the current vehicle state. They are:
- The position of the vehicle (this is the xy coordinate position of the vehicle in the world space).
- The heading of the vehicle. (This is given by the variable psi which is a measure of the current directional heading of the vehicle)
- The velocity of the vehicle
- The cross track error of the vehicle (The difference between the current position of the vehicle and the expection position from the reference trajectory)
- The heading error (This is the epsi variable that measures the error difference between the current heading and the expected heading)

I pass the state into a solver function which updates future state values (lines 126 - 156 in the `MPC.cpp` file) of the vehicle with the goal of minimizing the cost as described by a cost function (lines 90 - 114 in the `MPC.cpp` file).
The output of the solver is a set of actuator values (lines 293 - 294 in the `MPC.cpp` file) that will minimize the cost as described by the cost function.
This actuator values are then sent as commands to the vehicle to control its rotational actuator (steering) and linear actuator (positive or negative acceleration).


####2. Timestep Length and Elapsed Duration (N & dt).

The timestep (N) and elapsed duration (dt) values were chosen by experimentation.
I chose n N value that was not large or not too small.
Through experementation, I noticed that using a large N value would be looking to far into the "future" and skew the controls incorrectly, while using a small N value would make the vehicle follow the path for changes in short intervals but not react adequately to big changes to the path coming in the future (future anticipation would be off).
Choosing a value for N that could predict sufficient into the future was what I observed to be most desirable.

For the dt values, I chose a value that will reduce the amount of computation done by the solver. The smaller the dt value (given a constant N) the higher the number of computations that will be done by the solver.
Although this may lead to more accurate values, this will also increase the computation time which will affect latency (not good).
Also, the higher the dt value (given a constant N) the less computation will be done by the solver which may lead to inaccurate actuator outputs (not good).

In general my goal was to choose N and dt values that optimized model computation time, responsiveness to large changes in the reference path and accuracy of actuator values.


####3. Polynomial Fitting and MPC Preprocessing.

The way points (ptsx, ptsy) are provided in the world's space, but I needed to transform them (lines 112 - 118 in the `main.cpp` file) to the vehicle's world space.
A 3rd order polynomial (in line 123 of the `main.cpp` file) is then used to fit the transformed way points.
This polynomial then becomes the reference trajectory for the model.


####4. Model Predictive Control with Latency.

I addressed latency  in lines 129 - 136 of the `main.cpp` file.
What I did was after transforming the state of the vehicle from the world's space to the vehicle's space, I then predict the vehicle's state after a time delay equal to the latency.
I then pass the predicted state as an input to the MPC solver instead of the the current vehicle's state.

