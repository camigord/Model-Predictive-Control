# Model Predictive Control
Self-Driving Car Engineer Nanodegree Program

---
<img src="./assets/video.gif?raw=true" width="500">

## Overview

Final project for the second term of the UDACITY's Self-Driving Car Nanodegree. Implementing a _Model Predictive Controller_ capable of driving a car in a simulated environment by controlling both the steering wheel and the acceleration pedal.

## Dependencies

* cmake >= 3.5
* make >= 4.1(mac, linux), 3.81(Windows)
* gcc/g++ >= 5.4
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download it from [here](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. `./clean.sh`
3. `./build.sh`
4. `./run.sh`


## Rubric Criteria

### Vehicle Model

The state of the vehicle is defined as:

> state = [x, y, &psi;, v, cte, e&psi;]

where:

  - _x_, _y_, _&psi;_ represent the position and orientation of the car respectively
  - _v_ represents the current velocity of the vehicle
  - _cte_ and _e&psi;_ represent the _Cross Track Error_ and _Orientation Error_ respectively.

The controller will drive the vehicle using two actuators: the steering angle &delta; and the acceleration _a_.

> actuators = [&delta; _a_]

Given the control inputs [&delta; _a_] and an estimate of the elapsed time between actuations _dt_, we can update the state of the vehicle using the following equations:


* x<sub>t+1</sub> = x<sub>t</sub> + v<sub>t</sub> * cos(&psi;<sub>t</sub>) * _dt_
* y<sub>t+1</sub> = y<sub>t</sub> + v<sub>t</sub> * sin(&psi;[t]) * _dt_
* &psi;<sub>t+1</sub> = &psi;<sub>t</sub> + v<sub>t</sub> / _Lf_ * &delta;<sub>t</sub> * _dt_
* v<sub>t+1</sub> = v<sub>t</sub> + a<sub>t</sub> * _dt_
* cte<sub>t+1</sub> = _f_(x<sub>t</sub>) - y<sub>t</sub> + v<sub>t</sub> * sin(e&psi;<sub>t</sub>) * _dt_
* e&psi;<sub>t+1</sub> = &psi;<sub>t</sub> - &psi;_desired<sub>t</sub> + v<sub>t</sub> * &delta;<sub>t</sub> / _Lf_ * _dt_


where:

 * _Lf_ measures the distance between the front of the vehicle and its center of gravity. This value needs to be found for each vehicle configuration and it can be done by driving the vehicle with a constant (and larger than 0) velocity and steering angle. Just adjust the value of _Lf_ until the radius of the circular trajectory generated from driving the car is similar to what the model predicted.

 * _f(x<sub>t</sub>)_ is our reference trajectory. This trajectory is defined as a third order polynomial.

 * &psi;_desired represents our desired orientation and can be calculated as the tangential angle of the polynomial _f_ evaluated at _x<sub>t</sub>_: &psi;_desired<sub>t</sub> = _arctan(f'(x<sub>t</sub>))_.


### Timestep Length and Elapsed Duration (N & dt)

The final values for _N_ and _dt_ are 10 and _0.1 seconds_ respectively. I tried several permutations for _N_ and _dt_ trying always to keep a time horizon (_T = N*dt_) lower than _1 second_. The reason for this is that as _T_ gets larger, the estimates computed by the model become more and more inaccurate and the vehicle behaves erratically. 

Latency had also an important effect on the selection of _N_ and _dt_. As mentioned below, _dt_ was selected in order to easily account for computation delays.

Some of the values which were tried are: 10/0.2, 10/0.05, 5/0.1, 20/0.05 among others.


### Polynomial Fitting

[This](./DATA.md) file describes the details about the data which is sent back from the simulator. As it can be seen, the simulator sends two arrays _ptsx_ and _ptsy_ containing the global _x_ and _y_ coordinates of the reference trajectory or _waypoints_. These _waypoints_ are preprocessed by transforming them to the vehicle's reference frame (_main.cpp_ lines 100 to 108) and then used to fit a third order polynomial describing the desired trajectory.

### Dealing with Latency

To solve the latency problem I took inspiration from the approaches provided by [Jeremy Shannon](https://github.com/jeremy-shannon/CarND-MPC-Project) and [Andrew Stromme](https://github.com/astromme/CarND-MPC-Project), among some other comments from the Udacity Forums. The main idea is to account the effect of latency on the state of the vehicle and/or the model equations.

The original kinematic equations presented above depend upon the actuations from the previous timestep. These actuations, however, are delayed by latency (_~100ms_). To simplify the problem, we can define our timestep interval _dt_ to be the same as the expected latency; doing this allows us to easily account for the latency effect by considering the actuations from two timesteps ago whenever we update the model state (as shown in _MPC.cpp_ lines 105-108).
