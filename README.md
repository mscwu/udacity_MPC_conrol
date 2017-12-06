# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Reflection
### Model Description  
In this project, a kinematic vehicle model is used.  

The states are:  
* x: longitudinal position of the vehicle
* y: lateral position of the vehicle
* psi: heading of the vehicle
* v: longitudinal speed of the vehicle
* cte: cross-track error
* epsi: heading error  

The actuators are:  
* delta: steer angle
* a: throttle    

Update equations:  
* x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
* y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
* psi_[t+1] = psi[t] - v[t] / Lf * delta[t] * dt
* v_[t+1] = v[t] + a[t] * dt
* cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
* epsi[t+1] = psi[t] - psides[t] - v[t] * delta[t] / Lf * dt  
Note that due to sign convention, the sign of delta is flipped.
### Choosing N and dt  
Choosing suitable N and dt is very important for the MPC control problem. I chose N = 10 and dt = 0.08.  
I started with N = 10 and dt = 0.25. However, it turns out that dt is too large. The problem with too large a dt is that it neglects the fact that our model is an approximation and our optimization routine assumes that the model is perfectly executed. In reality, for example, due to latency with accuator at each time step, unless the latency is considered for *every* step in the optimization loop, the model tends to lose the ability to adapt to changes in the enviroment.  
Another issue worth mentioning is that a longer dt expands the solution space for an "Optimal" solution. This is generally a good thing. However, during my tuning, I found that a large dt will sometimes cause the optimizer to converge to a path and actuator solution that doesn't make sense. By having a small dt resolved the problem.  
Increasing the number of N will further discretize the problem, assuming the same predictio horiziont, corresponding to a small dt. However, increasing N will greatly increase computation time and an optimal solution is not guaranteed to be found for a limited planning window.  

### Polynomial fitting and MPC preprocessing    
For polynomial fitting, I implemented a function called `global2vehicle`. This function transforms waypoint from global coordinate system to vehicle coordinate system. This is good both for visualization and simplifying the problem.  
```cpp
std::vector<double> global2vehicle(double global_x, double global_y, double px, 
                                   double py, double psi) {
  double vehicle_x = (global_x - px) * cos(psi) + (global_y - py) * sin(psi);
  double vehicle_y = -(global_x - px) * sin(psi) + (global_y - py) * cos(psi);

  return {vehicle_x, vehicle_y};  
}
```
The beauty of this transformaion is that when we are creating polynomial with in vehicle coordinate system, the initla state of the vehicle will always be px = 0, py = 0 and psi=0.  

To make a more robust polynomial coefficient vector, a sanity check routine is implemented. This routine will make sure that the newly fitted coefficient is not too drastically different from the previous one. If discrepency occurs, previous fitting will be taken into account to create a more smooth curve.  
```cpp
// Calculated coefficients
auto cal_coeffs = polyfit(vehicle_ptsx_fit, vehicle_ptsy_fit, 3);
Eigen::VectorXd coeffs = cal_coeffs;

// Coefficients sanity check, starting from the second step
if (mpc.is_initial_run_) {
  previous_coeffs = coeffs;
  mpc.is_initial_run_ = false;
}
else {
  if (abs(cal_coeffs[0])>1000*abs(previous_coeffs[0])
      || abs(cal_coeffs[1])>1000*abs(previous_coeffs[1]) 
      || abs(cal_coeffs[2])>25000*abs(previous_coeffs[2])
    ) {
    for(int i = 0; i < coeffs.size(); i++) {
      coeffs[i] = 0.4 * previous_coeffs[i] + 0.6 * cal_coeffs[i];
    }
    std::cout << "Update coefficients failed. Using previous coefficients." << std::endl;
  }
  else {
    previous_coeffs = coeffs;
    std::cout << "Updated coefficients." << std::endl;
  }
}
```
Cost function is slightly modified. There is an additional cost added.  
```cpp
fg[0] += 300 * CppAD::pow(vars[a_start + t] * vars[delta_start + t], 2);
```
It penalizes the product of steer and throttle. The reason is that the model does not take into account the dynamics of the system and the vehicle's ability to turn under acceleration. Although I am not sure about the actual vehicle model in Unity, I think this is a very good protection to limitations of the simplified model. As for the tuning of the cost weights, I put heavy weights on the change in steer and throttle and also heavy weight on the cost of using steer. My goal is to have a silky ride and maintain a constant speed.  
As for the tuning of the parameters, it is important to get an understanding of the magnitude of all the states. This enables a quick convergence to a good solution. For example, CTE is in the range of (-5, 5), steer is in the range of (-0.5, 0.5). If I want to have actuation cost heavier than CTE, I should use a weight on steer that is at least 100 times that of CTE.  

### Model Predictive Control with Latency  
To consider the latency, the states that are used are modified before being sent to the optimizer. A state update is run with vehicle state transition equations with a timestep equal to the latency, 100ms in this case. This way, when the optimized actuation is excuted (first step), the execution corresponds to the state at 100ms later.
```cpp
// state with delay
const double delay = 0.1; // 100ms delay
double x = state[0] + state[3] * cos(state[2]) * delay;
double y = state[1] + state[3] * sin(state[2]) * delay;
double psi = state[2] - state[3] * current_steer / Lf * delay;
double v = state[3] + current_throttle * delay;
double f = coeffs[0] + coeffs[1] * state[0] + coeffs[2] * pow(state[0], 2) + coeffs[3] * pow(state[0], 3);
double psides = atan(coeffs[1] + 2 * coeffs[2] * state[0] + 3 * coeffs[3] * pow(state[0], 2));
double cte = f - state[1] + state[3] * sin(state[5]) * delay;
double epsi = state[2] - psides - state[3] * current_steer / Lf * delay;
```

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
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
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment.

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).
