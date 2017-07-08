# CarND-Controls-PID

The goal of this project is to drive around a simulated racetrack in a car whose throttle and steering are controlled by a Proportional, Integral and Derivative Controller. The appropriate P,I,D coefficients were computed using a local hill climbing algorithm affectionately named "twiddle".

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
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 


## Reflection

#### Describe the effect each of the P, I, D components had in your implementation.

The P component computes the steering angle as a direct multiple of the observer cross track error of the car. In the PID controller since this is the only parameter that is directly proportional to the CTE, this has the greatest impact on the car's steering. High values cause very quick corrections i.e., counter steers. Also, this component suffers from the problem of overshooting the desired ideal car position on an ongoing basis. This is because it doesn't adaptively start straightening the steering as we get closer and closer to the desired ideal position. This is where the differential component 'D' comes in. This component adds to the total steering angle as a multiplier of the temporal derivative of the CTE. In effect, this components causes steering angle to be less aggressive as we get closer to the desired ideal position as the derivative will approach zero in that case. Finally, the integral component 'I' is used for correcting systematic bias in the car's handling that can be caused by things like misaligned tires that gives the car a tendency to always turn left, for example. In this case, despite the steering input, the car's CTE never quite approaches zero i.e., the car never hits the ideal position. Therefore, the I component is computed as the sum of all observed CTEs over time. If this sum becomes large, we know that we need to over steer a little bit to offset the systematic bias i.e., larger steering angle than normally required will be computed.

#### Describe how the final hyperparameters were chosen.

The final hyperparameter selection turned out to be quite a challenge in my case due to a poor simulation environment. I ran the simulator on a low power linux machine with software-only LIBGL rendering. As as a result, I got very low frame-rates during my runs. So in addition to tuning my P, I, D coefficients, I also had to use a very low throttle speed of 3 MPH! This roughly matched my simulator's frame rate.

For identifying the PID coefficients, I implemented the twiddle algorithm discussed by Sebastian by maintaining the loop and algorithm state as the OnMessage function gets called repeatedly in main.cpp. I ran this overnight with a tolerance of 0.2 and for 200 steps per simulated run. Every time I hit the total number of steps required per simulation, I sent a reset command to the simulator to start parameter exploration anew. The final numbers seen in the codee are lowest the algorithm got to within the total run limit I set (controlled by the iteration parameter in the twiddle() function). I then manually plugged in this set of parameters onto the drive() function that actually drives the car without terminating and verified that the car indeed stayed on track throughout.
