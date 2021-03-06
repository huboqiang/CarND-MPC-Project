# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program


# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

Please run 

```
./mpc
```

after compilation


---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
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
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Compilation

Your code should compile.

```
git clone https://github.com/huboqiang/CarND-MPC-Project
mkdir build
cd build
cmake ..
make
./mpc
```

## Implementation

### The Model

Student describes their model in detail. This includes the state, actuators and update equations.

There are eight scalars that were used to describe  this model:

Scalar   |   Type  | Meaning
---------|---------|---------
x        | Const| car's position(font/back)
y        | Const| car's position(left/right)
psi      | Const| direction
v        | Const| speed
cte      | Const| Cross-Tracking Error
epsi     | Const| Direction Error 
delta    | Var  |  Turning Rate (throttle_value)
a        | Var  | Acceleration Rate (steer_value)

Where [x, y, psi, v, cte, epsi] were constant value for describing the current state of the car, and [a, delta] were the returning values of this model and would be send to the simulator to control the car.

#### 1. States


States were defined in `main.cpp`, line 120.

As described above, six scalars were used to describe the current state of the car. The **expected position of the car** were firstly defined as the center of the road(`y=0`) with the same direction of the road(`psi=0`) with the same `x` position as the real position of the car. However, the car may not just at the center of the road, for example, in the next paragraph, the car is 1m left to the expected position, so it's coords was `x=0,y=1`. 



![png](./coord.png)


If we simply defined the expected path as the x-axis on this figure, then the cte value would be `cte=-1` and epsi value would be `epsi=-π/4`. However, the actually `cte` and `epsi` were defined using the coeffs of the 3rd-polynomial using the input ptsx and ptsy. With this information, we can get the differenial value between the expected position of the car and its real positions:

```cpp
// eptsx for std::vector => eigen::VectorXd conversion
auto coeffs = polyfit(eptsx, eptsy, 3);  

// the estimated position of y minus the expected y(0)
double cte  = polyeval(coeffs, 0.0);

// the the expected psi(0) minus the estimated state of psi
double epsi = 0.0 - atan(coeffs[1]);

Eigen::VectorXd states(6);
// Expected x-Pos, Expected y-Pos, Expected psi-value, cte, epsi
states << 0.0, 0.0, 0.0, v, cte, epsi;  
```

#### 2. Actuators

The aim of the MPC controller is to optimize a cost function with inputs of `states` which were described in the above section:

![gif](eq1.gif)

While we could minimize the loss function with constant state values([x, y, psi, v, cte, epsi]), what we need is to get the `delta`(Turning Rate) and `a` (Acceleration Rate) values in condition of this loss function with restriction of:

![gif](eq2.gif)

and than return this value to the simulator as `steer_value` and `throttle_value` to control the car.

The cost value of the loss were defined in `MPC.cpp` line 46-62, class `FG_eval`, as `fg[0]`, :

```cpp
fg[0] = 0.0;
for (int i=0; i<N; i++){
  fg[0] += 6000*CppAD::pow(vars[cte_start+i], 2);
  fg[0] += 3000*CppAD::pow(vars[epsi_start+i], 2);
  fg[0] += 16*CppAD::pow(vars[v_start+i]-ref_v, 2);
}
for (int i=0; i<N-1; i++){
  fg[0] += 50*CppAD::pow(vars[delta_start+i], 2);
  fg[0] += 5*CppAD::pow(vars[a_start+i], 2);
  fg[0] += 150*CppAD::pow(vars[epsi_start+i]*ref_v, 2);
}
for (int i=0; i<N-2; i++){
  fg[0] += 10000*CppAD::pow(vars[delta_start+i+1]-vars[delta_start+i], 2);
  fg[0] += 10*CppAD::pow(vars[a_start+i+1]-vars[a_start+i], 2);
}
```

The restrictions for `delta` and `a` were defined as:

```cpp
for (int i = delta_start; i < a_start; i++) {
  vars_lowerbound[i] = -0.436332;
  vars_upperbound[i] = 0.436332;
}
for (int i = a_start; i < n_vars; i++) {
  vars_lowerbound[i] = -1.0;
  vars_upperbound[i] = 1.0;
}
```

The range of other vars were defined as any real values:

```cpp
for (int t = 0; t < delta_start; t++) {
  vars_lowerbound[t] = -1.0e19;
  vars_upperbound[t] = 1.0e19;
}
```

and the range of `state` values were all constant and should be defined as zero.

```cpp
constraints_lowerbound[x_start] = x;
constraints_upperbound[x_start] = x;
//...
```

Finally, the `ipopt` solver were applied to minimize the loss function with `CppAD::ipopt::solve` :

```cpp
CppAD::ipopt::solve_result<Dvector> solution;
CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

// Check some of the solution values
ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

// Cost
auto cost = solution.obj_value;
  
vector<double> result;
result.push_back(solution.x[delta_start]);
result.push_back(solution.x[a_start]);

for (int i = 0; i < N-1; i++) {
  result.push_back(solution.x[x_start + i + 1]);
  result.push_back(solution.x[y_start + i + 1]);
}
```

Where the `fg_eval` were applied to update the values of the `vars`. This process would be introduced in detail in the next section.

#### 3. Update Equations

The update function were described in Lesson19, section9 of the class:

![gif](eq3.gif)

Code:

```cpp
AD<double>delta0 = vars[delta_start+i-1];
AD<double>a0     = vars[a_start+i-1];
AD<double>f0     = coeffs[0] + coeffs[1] * x0 + coeffs[2] * CppAD::pow(x0, 2) + coeffs[3] * CppAD::pow(x0, 3);
AD<double>psides0= CppAD::atan(coeffs[1] + 2 * coeffs[2] * x0 + 3 * coeffs[3] * CppAD::pow(x0, 2));

fg[1+x_start+i]    = x1    - (x0+v0*CppAD::cos(psi0)*dt);
fg[1+y_start+i]    = y1    - (y0+v0*CppAD::sin(psi0)*dt);
fg[1+psi_start+i]  = psi1  - (psi0-v0*delta0*dt/Lf);
fg[1+v_start+i]    = v1    - (v0+a0*dt);
fg[1+cte_start+i]  = cte1  - ((f0-y0)+(v0*CppAD::sin(psi0)*dt));
fg[1+epsi_start+i] = epsi1 - ((psi0-psides0) - (v0*delta0*dt/Lf));
```

### Timestep Length and Elapsed Duration (N & dt)

They were defined in `MPC.cpp`. A big `N` value would be good because it could provide more parameters to the solver, which could help to get a better estimation.  However, if `N` were too too big, the computational consumption would be too large, which would result in a long time-interval between the returning of controling commands and the car might be out-of-control. For example, the `ipopt::solve` would run about 70 times with `N=10` while it would be just 20 times with `N=50` in the same time period in my code. As the car ran very fast, 20 controlling command is not enough to control the car well and it just ran out of the way.

Next, the value of `N * dt` could just represent the length of road that it would take into consideration. I found that `N = 10, dt = 0.1` could generate a green line with just the same length of the yellow-line reference, which could cover the road ahead well. However, if I used `N = 10, dt = 0.01` in my code, the green line would be too short to see. In other words, it is a blind-car, which could hardly see the road ahead comparing with `N = 10, dt = 0.1`, and would be hard make a right decision for the MPC controller.

So I just choose:

```cpp
size_t N  = 10;
double dt = 0.1;
```

### Polynomial Fitting and MPC Preprocessing

Befort polynomial fitting, the input values for MPC were preprocessed for the waypoint transformation. This step is just like the first step of the `updateWeights` function of [particle filter](https://github.com/huboqiang/CarND-Kidnapped-Vehicle-Project/blob/master/src/particle_filter.cpp), line 147-150. 
 
```cpp
for(int i=0; i<ptsx.size(); i++){
  double shift_x = ptsx[i] - px;
  double shift_y = ptsy[i] - py;
  ptsx[i] = (shift_x*cos(0-psi) - shift_y*sin(0-psi));
  ptsy[i] = (shift_x*sin(0-psi) + shift_y*cos(0-psi));
}
```

Then a polynomial is fitted to waypoints, which were plotted as a yellow line in the simulator.

```cpp
//.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
// the points in the simulator are connected by a Yellow line
int num_points = 10;
double poly_inc = 2.5;
vector<double> next_x_vals;
vector<double> next_y_vals;

// eptsx for std::vector => eigen::VectorXd conversion
auto coeffs = polyfit(eptsx, eptsy, 3);  
for (int i=1; i<num_points; i++){
  next_x_vals.push_back(poly_inc*i);
  next_y_vals.push_back(polyeval(coeffs, poly_inc*i));
}
```

The waypoints information were not pretrained. 

### Model Predictive Control with Latency

I found this system were delayed 100ms in the main function before sending the operation for car controling:

```cpp
this_thread::sleep_for(chrono::milliseconds(100));
ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
```

So while update the equations, a `latencyIdx` were used to describe this delay:

```cpp
int latencyIdx = 0;
if(i > 1){
  latencyIdx = 1;
}
AD<double>delta0 = vars[delta_start+i-1-latencyIdx];
AD<double>a0     = vars[a_start+i-1-latencyIdx];
```

## Simulation

### The vehicle must successfully drive a lap around the track.

Yes I did it. And this is the demo Video:

![gif](./running.gif)

Youtube:

[https://youtu.be/Zr5CT8hBqpU](https://youtu.be/Zr5CT8hBqpU)
