# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

[img1]: ./imgs/simulator.png
[img2]: ./imgs/formula.png
[img3]: ./imgs/process.png
[img4]: ./imgs/controllers.png
[img5]: ./imgs/tuning.png

output:
![img1]
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
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

Fellow students have put together a guide to Windows set-up for the project [here](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/files/Kidnapped_Vehicle_Windows_Setup.pdf) if the environment you have set up for the Sensor Fusion projects does not work for this project. There's also an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3).

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`.

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/e8235395-22dd-4b87-88e0-d108c5e5bbf4/concepts/6a4d8d42-6a04-4aa6-b284-1697c0fd6562)
for instructions and the project rubric.

## Project overview

### About PID controller

PID control is a technique to correct vehicle steering Angle, throttle threshold and other behaviors according to loss components of P(proportion), I(integral) and D(differential) given vehicle crossing trajectory error.

In the Udacity automotive simulator, the CTE value is read from the data message sent by the simulator, and the PID controller updates the error value and predicts the steering angle based on the total error.

*PID formula and process (image from Wikipedia):*

![img2]
![img3]

P : Proportional — This item applies to correcting steering wheel scale errors. If we get too far from the target, we turn the wheel in the other direction.

D : Derivative — The purpose of item D is to suppress this oscillation effect by adding the change of error into the formula. The PD controller find that the error slightly reduces to the expection from a smooth path.

I : Integral — The term of I is used to correct a constant error that brought by mechanical bias. So it's necessary to add a term to penalize the sum of cumulative errors.

The Twiddle PID curve corresponds to the use of an algorithm to find the coefficients more rapidly and thus to converge more quickly towards the reference trajectory.

![img4]

The PID controller is the simplest and most common in use. It has the advantage of being implemented quickly and operating in simple situations. But it is impossible to model the physics of the vehicle. When we drive, we naturally adjust our maneuvers according to the size, mass and dynamics of the vehicle. A PID controller can not do it.

### Implementation of the PID controller

**1. Update PID**

Work in PID.cpp:

    PID::PID() {}

    PID::~PID() {}

    void PID::Init(double Kp_, double Ki_, double Kd_) {
        /**
        * TODO: Initialize PID coefficients (and errors, if needed)
        */
        Kp = Kp_;
        Ki = Ki_;
        Kd = Kd_;
        p_error = 0.0;
        i_error = 0.0;
        d_error = 0.0;
    }

    void PID::UpdateError(double cte) {
        /**
        * TODO: Update PID errors based on cte.
        */
        d_error = cte - p_error;
        p_error = cte;
        i_error += cte;
    }

    double PID::TotalError() {
        /**
        * TODO: Calculate and return the total error
        */
        return -Kp*p_error - Ki*i_error - Kd*d_error;  // TODO: Add your total error calc here!
    }

**2. manual turnning**

The most important part of the project is to tune the hyperparameters. This can be done by different methods such as manual tuning, Zieglor-Nichols tuning, SGD, Twiddle. A manual tuning before using twiddle to narrow the search range, and the following table summerizes the effect of each parameter on the system.

![img5]

The result of manual turning is {0.10000, 0.00050, 1.50000}

**3. Twiddle**

The key part is to adjust each parameter by increasing and decreasing its value, and to see how the error changes. If either direction is conducive to error minimization, the change rate in that direction should be amplified; otherwise, the change rate should be reduced.

Pseudocode for implementing the Twiddle algorithm has been shown by Sebastian:

    function(tol=0.2) {
      p = [0, 0, 0]
      dp = [1, 1, 1]
      best_error = move_robot()
      while sum(dp) < tol:
        for i in range(len(p)):
          p[i] += dp[i]
          error = move_robot()
          
          if err < best_err
              best_err = err
              dp[i] *= 1.1
          else
            p[i] -= 2 * dp[i]
            error = move_robot()

            if err < best_err
              best_err = err
              dp[i] *= 1.1
            else
              p[i] += dp[i]
              dp[i] *= 0.9
      return p
    }

After each run of the 700 points loop, the PID error is updated to make the twiddle algorithm better finetuned.

The result of twiddle turning is {0.155, 0.0011, 1.691}

### Discussion

1. The less tolerance is set, the more simulator loops are needed and the more time is required. Is there any way to save time?

2. Speed and throttle can also be added to PID control.
