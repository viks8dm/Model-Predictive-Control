# Model Predictive Control
Self-Driving Car Engineer Nanodegree Program

---

[image1]: ./results/sample_image.jpg "Sample MPC simulation image"

--

![alt text][image1]
--

In this final project of term-2 of the self driving car nanodegree program by Udacity, a non-linear Model Predictive Control has been successfully implemented to drive the car around the track on Udacity's [simulator](https://github.com/udacity/self-driving-car-sim/releases/). The simulator provides a feed of values containing the position of the car, its speed, heading direction, coordinates of waypoints along a reference trajectory.

The project has been created using Udacity's [starter Code](https://github.com/udacity/CarND-MPC-Project)


## Implementation details

The requirement for successful implementation is that no tire should leave the drivable portion of the track surface, i.e.,  car may not pop up onto ledges or roll over any surfaces that would otherwise be considered unsafe. In addition, there is a 100milisecond latency in actuation that needs to be taken care of.

#### Vehicle Model

The implementation is based on bicycle model outlined in the lectures, where the state-variables are vehicle's x & y coordinates, heading-angle and vehicle velocity, `[x, y, psi, v]` and the actuators are steering-angle and vehicle acceleration (or throttle) `[delta, a]`. The bicycle model updates the states as:

* `x[t+1] = x[t] + v[t]*cos(psi[t])*dt`
* `y[t+1] = y[t] + v[t]*sin(psi[t])*dt`
* `psi[t+1] = psi[t] + (v[t]/Lf)*delta*dt`
* `v[t+1] = v[t] + a[t]*dt`

Here `Lf` is the distance between the center of mass of the vehicle and the front wheels and affects the maneuverability.

In addition, at every time step, the model also updates the cross-track error (`cte`) and orientation-error (`epsi`), as follows:

* `cte[t+1] = f(x[t]) - y[t] + v[t]*sin(epsi[t])*dt`
* `epsi[t+1] = psi[t+1] - psides[t]`

where, `f(x[t])` is 3rd order estimate for vehicle's next position and `psides[t]` is slope (or orientation) estimate.

The implementation of this vehicle model is very similar to that done in `MPC_on_line` quiz in class and can be found under `class FG_eval` in `MPC.cpp` for this project.


#### Polynomial Fitting

The waypoints provided are used for a 3rd order polynomial fitting, in order to estimate the next vehicle position. The original waypoint is given in global coordinate system, which are transformed to vehicle coordinate system using the 2D transformation equations shwon in one of the lectures. This transformation is done in the `main.cpp` script.

#### Cost function

The state estimation for next time-step is done by using the current state variable values from simulator and optimizing the trajectory over next N time steps, by minimizing a cost function, similar to that defined in one of the examples presented in a lecture-quiz. The fundamental difference from the class-quiz is in use of different weights for different contributors to the cost function, unlike the quiz problem where weights are kept same and equal to 1.

The cost function mentioned above, is basically summation of quadratic expressions in cross-trak-error, vehicle-orientation error (or heading-direction error), velocity error with respect to a set reference, actuator values and actuator value difference between adjacent time-steps. The implementation can be found under `class FG_eval` in `MPC.cpp`.

#### Parameter Tuning

The final parameters used for the project implementation are as listed below, with some comments on why they were given these specific values:

* time related parameters:
	* `dt = 0.05`
		* this value was chosen to account for 100ms latency in the system
	* `N = 15`
		* I started with `N=25` but that appeared to give very long curve which used to bounce about and sometimes trace a path that was going out of the road-boundaries, hence I reduced it.
* reference values
	* `REF_CTE = 0` (desired value)
	* `REF_EPSI = 0` (desired value) 
	* `REF_V = 65` (reference speed)
		* I set initial speed to 40mph, however, once a smooth driving profile was obtained, I tried values like 50, 60, 65 and 80. For each of these cases I had to adjust the `WT_V` (weight for velocity error) parameter. The intention behind this tuning was that the car should reach max speed specified by this reference value, however I could not reach a combination of parameters that helps achieve that, so the actual speed during driving always remains less than that specified by `REF_V`.

		
* cost function weights: For all the following values I checked by changing the numbers between very high (~ 800) and default (1.0) to assess the impact of a particular parameter. In the end, it seemed that variation in `WT_DELTA_DIFF` has biggest impact followed by `WT_EPSI`. The weights that did not lead to significant change in tranjectory were left unchanged and set to default value of `1.0`, as listed below:
	* `WT_CTE = 5.0`
	* `WT_EPSI = 10.0`
	* `WT_V = 0.5`
	* `WT_DELTA = 60`
	* `WT_A = 15`
	* `WT_DELTA_DIFF = 700`
	* `WT_A_DIFF = 1.0`


A video of the simulated car driving around the track with this final set of parameters, can be found [here](./results/MPC.mov).


#### Latency

To account for the 100ms latency in the actuation, the car position estimation is propagated forward by 100ms when the actuation is expected to start (2-time steps here, since dt = 0.05). This is highlighted in the line shown below from `MPC::Solve` function:

`vector<double> result = {solution.x[delta_start+2],   solution.x[a_start+2]};`


## To run the code

Clone this repository and enter following commands in a terminal

`mkdir build && cd build`

`cmake .. && make`

`./mpc`

After execution of `./mpc`, simulator should be opened and it should be started by selecting the appropriate project for this implementation.



# Model-Predictive-Control
