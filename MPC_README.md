# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Requirements

* The Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases).  
* Set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems.  
* Ipopt and CppAD: refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`. 

## Components

### main.cpp  
* read the measurements with the simulator
* convert the view points from the global coordinate system to the car-center local coordinate system
* do the polynomial fitting to get the polynomial coefficients 
* call MPC controller to obtain the steering and throttle values
* send the steering and throttle to the simulator to control the car

### mpc.cpp 
* A FG_eval class used to build MPC cost and constrains
* A MPC class used to implement MPC 

## MPC Design and Implementation

### System Model

* All computation is performed at the car-centered local coordinate system.  The map global view points are transformed to that local coordinate system.
* A third-order polynomial is used to model the road curves.  The simulator sends six road view points, which could used to compute the polynomial coefficients.
* The cost function is built to   
 1. reduce the road center error, the angle error and the velocity error  
 2. minimize the use of actuators
 3. minimize the value gap between sequential actuation
* The different weights are applied for the above errors
* The constrains follow the car kinetic model
* Adding the limitations for steering angle and acceleration

### Parameter Tuning

Timestep Length (N) and Elapsed Duration (dt) are two key parameters in MPC.  Ideally, increasing the production of N and dt while decreasing dt could give the better performance.  However, this could significantly increase the computation cost.  
Here, we are going to show two examples.  In these two examples, the production N and dt are the same, but different N and dt.

* N = 20 and dt = 0.1
The resulting [video](N10.mp4) shows that the car could complete the whole path.

* N = 10 and dt = 0.2
The resulting [video](N20.mp4) shows that the car leave the road.

This is clear that dt = 0.1 definitely gives the better performance.

### Additional System Latency

Another 100ms latency is added to simulate the system command delay which could happen at the real vehicle.  However, above N=20, and dt=0.1 MPC does not give good results as shown in this [video](latency.mp4). The car has more swings, and finally leaves the road.

This is clear that adding addition system latency cause the system more difficult to control.  The previous MPC method fails.  In order to make the car easy to control, we reduce the reference velocity from 25 mph to 20 mph.  Also, we reduce N from 20 to 10.  Making N smaller forces MPC to more focus on the near side, which could compensate the system latency.  After that, the car could successfully complete the whole loop as shown in this [video](final.mp4).


