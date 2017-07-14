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

#### 1. States

States were defined in `main.cpp`, line 120.

To define the states, the location of the car were first transformed from the map's coords to the expected position of the vehicles's coords. The **expected position of the car** were defined as the center of the road(`y=0`) with the same direction of the road(`psi=0`) with the same `x` position as the real position of the car. However, the car may not just at the center of the road, for example, in the next paragraph, the car is 1m left to the expected position, so it's coords was `x=0,y=1`. 

```
=============================
Y

Car(x=0,y=1,psi=0)
            road direction
Expected    ------------->   X
(x=0,
 y=0,
 psi=0)

=============================
```


This step is just like the first step of the `updateWeights` function of [particle filter](https://github.com/huboqiang/CarND-Kidnapped-Vehicle-Project/blob/master/src/particle_filter.cpp), line 147-150. 
 
```cpp
for(int i=0; i<ptsx.size(); i++){
  double shift_x = ptsx[i] - px;
  double shift_y = ptsy[i] - py;
  ptsx[i] = (shift_x*cos(0-psi) - shift_y*sin(0-psi));
  ptsy[i] = (shift_x*sin(0-psi) + shift_y*cos(0-psi));
}
```

Next, the ptsx and ptsy were fit with a 3rd polynomial. With this information, we can get the differenial value between the expected position of the car and its real positions:

```cpp
// eptsx for std::vector => eigen::VectorXd conversion
auto coeffs = polyfit(eptsx, eptsy, 3);  

// the estimated position of y minus the expected y(0)
double cte  = polyeval(coeffs, 0.0);

// the the expected psi(0) minus the estimated state of psi
double epsi = 0.0 - atan(coeffs[1]);

Eigen::VectorXd states(6);
states << 0.0, 0.0, 0.0, v, cte, epsi;
```

#### 2. Actuators

The aim of the MPC controller is to optimize a cost function with inputs of `states` which were described in the above section:

$$ Loss = \sum_N^{i}{ [6000* cte_i^{2} + 3000*a_i^{2} + 16*(v_i-v_{ref})^2 } + $$

$$  \sum_{N-1}^{i}{ [50 * \delta_i^2 + 5 * a_i^2 + 150 * (e\psi_i*v_{ref})^2]} + $$

$$  \sum_{N-2}^{i}{ [10000 * (\delta_{i+1}) - \delta_{i}))^2 + 10 * ( a_{i+1} - a_{i})^2]} $$

Where:

$$ -\frac{25}{180}\pi \le \delta \le \frac{25}{180}\pi $$

$$ -1.0 \le a \le 1.0 $$

$$ v_{ref}=100 $$

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

and the range of constant values were defined as zero.

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

$$ x_{t+1} = x_{t} + v_t * cos(\psi_t) * dt$$

$$ y_{t+1} = y_{t} + v_t * sin(\psi_t) * dt$$

$$ \psi_{t+1} = \psi_{t} + \frac{v_t}{L_f} * \delta * dt $$

$$ v_{t+1} = v_{t} + a_t * dt$$

$$ cte_{t+1} =  f(x_t) - y_t + v_t * sin(e\psi_t) * dt$$

$$ e\psi_{t+1} = \psi_t - \psi des_t + (\frac{v_t}{L_f} * \delta_t * dt) $$

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

They were defined in `MPC.cpp`. As described in the classroom, `T` should be as large as possible and `dt` should be as small as possible. However, if `T` were too too big or `dt` too small, the computational consumption would be too large. So I just chosen the following parameters:

```cpp
size_t N  = 10;
double dt = 0.1;
```

### Polynomial Fitting and MPC Preprocessing

A polynomial is fitted to waypoints, which were plotted as a yellow line in the simulator.

```cpp
//.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
// the points in the simulator are connected by a Yellow line
int num_points = 10;
double poly_inc = 2.5;
vector<double> next_x_vals;
vector<double> next_y_vals;
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

Youtube:


