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
The vehicle model used for this project is based on a Kinematic model. It comprises the following components:
- State
`[x, y, psi, v, cte, epsi]` 

  where x and y is the position of the vehicle, psi is the vehicle orientation, v is the vehicle velocity, cte (cross track error) is the distance of vehicle from the trajectory and epsi is the difference of vehicle orientation and trajectory orientation.

- Control Inputs
`[delta, a]`

  where delta is the steering angle and a is the acceleration. Both inputs are used to move the vehicle.

- Constraints
  - -25 < steering angle < 25
  - -1 < acceleration < 1

- Vehicle Model
  - x[t+1] = x[t] + v[t] * cos(psi[t]) * dt
  - y[t+1] = y[t] + v[t] * sin(psi[t]) * dt
  - psi[t+1] = psi[t] + (v[t] / Lf) * delta[t] * dt
  - v[t+1] = v[t] + a[t] * dt
  - cte[t+1] = cte[t] + v[t] * sin(epsi[t]) * dt
  - epsi[t+1] = epsi[t] + (v[t] / Lf) * delta[t] * dt

The above equations will be treated as constrains for MPC.

- Cost Functions
  - Cross Track Error
  - The difference of vehicle orientation and trajectory orientation
  - The reference velocity
  - Steering angle change rate
  - Acceleration change rate
  - The change between sequential actuations
 
My MPC controller ouptuts the future trajectory by optimizing the cost functions subject to the constraints.

#### 2. Timestep Length and Elapsed Duration (N & dt).
The *N*(timestep length) and *dt*(elapsed duration between timesteps) were chosen based on the maximum speed the model wants to reach. The first thing I did was to determine how long the duration of the future predictions (*T*), which is the product of *N* and *dt*, should be for the speed I wanted to achieve. After a lot trial and error, I found 2 seoncds seems good enough to drive the car at 100 MPH. 

Once *T* was selected, I tried to use a large *N*(e.g. 50) and a small *dt*(e.g. 0.04) and thought it was going to drive my car like a pro racing driver, but the reality hit me hard. The model drove the car like a drunk driver. The prediction time become too long to make any useful trajectory predictions on my computer due to a large N. In order to preserve *T*, the only thing I could do is to lower *N* and increase *dt*. The final values are *15* and *0.13*, respectively.    

#### 3. Polynomial Fitting and MPC Preprocessing
Before starting the MPC procedure, I transformed the waypoints from the map's coordinate system to the vehicle's coordinate system, compensated the actuation latency and adjusted the reference velocity based on the curvature of the road.

#### 4. Model Predictive Control with Latency
I predicted the initial state with the vehicle model for the duration of the latency, which is 100 milliseconds in this project. The resulting state beomes the new initial state for MPC.

## Result
My car is able to reach 102 MPH on my computer and drives around the track endlessly.
