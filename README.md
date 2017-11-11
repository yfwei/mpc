# Model Predictive Control
The final project of the Udacity Self-Driving Car Nanodegree term 2.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc` 

## Rubric Points
### Here I will consider the [rubric points](https://review.udacity.com/#!/rubrics/896/view) individually and describe how I addressed each point in my implementation.  

---

#### 1. The Model
The vehicle model used for this project is as follows:

| Source        | Destination   | 
|:-------------:|:-------------:| 
| 555, 475      | 200, 0        | 
| 730, 475      | 1080, 720      |
| 1060, 680     | 1080, 720      |
| 240, 680      | 200, 0        |

#### 2. Timestep Length and Elapsed Duration (N & dt).
The N(timestep length) and dt(elapsed duration between timesteps) were chosen based on the maximum speed the model can handle. 

#### 3. Polynomial Fitting and MPC Preprocessing
Before starting the MPC procedure, I transformed the waypoints from the map's coordinate system to the vehicle's coordinate system, compensated the actuation latency and adjusted the reference velocity based on the curvature of the road.

#### 4. Model Predictive Control with Latency
I predicted the initial state with the vehicle model for the duration of the latency, which is 100 milliseconds in this project. The resulting state beomes the new initial state for MPC.

## Result
My car is able to reach 102 MPH on my computer and runs the lap as long as I become tired watching it.
