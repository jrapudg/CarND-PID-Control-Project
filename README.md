# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---
## Reflection

[here.](https://github.com/mvirgo/PID-Control-Project/blob/master/PID_control_vid.mov)

### PID algorithm
The PID controller is an algothm used to follow a reference through a closed loop system that calcultes the error from a setpoint and the actual state of the system and set the control input as a proportion of this cross-track error or CTE. The setpoint is the x and y coordinates at the center of the road of the desired trajectory of the car.

In the PID algorithm, the **P** stands for proportional, the **I** for integral and the **D** for differential. Each part of the algorithm plays an important role to control the postion of the car and how it reaches the goal position. The P and D modifies the transient response of the system and the I eliminates the steady state error. These gains do this job through the error of the system. The proportional gain means that the steering angle of the car will be set in proprtion of the CTE.


The architecture of the controller is:


``steering_angle = Kp*cte + Ki*cte_integral + Kd*cte_differential``

Tunning the gains is not an easy task. A high **I** gain turns into sharp oscilations and the car will roll out of the road. Similarly, finding the right **P** and **D** should be done also carfully to improve the transient response. If the P gain is to high, the car will overcorrect the CTE and improve amplitud of oscilations or if it is to low, it will have a really slow reaction, which is not a good scenario when referring to curves.

The strategy that I followed was, first by hand, I chose gains and saw how the system behaved. The values selected for Kp, Ki and Kd were:  .

A video of the simulated car driving around the track can be found here.

ard, but making the controller actually perform well is the tough part. Having knowledge of each part of "PID" is import
The "I" for integral sums up all CTEs up to that point, such that too many negative CTEs (in this case, meaning the car has been to the left of the middle of the lane for awhile) will drive up this value, causing the car to turn back toward the middle, preventing the car from driving on one side of the lane the whole time. If the coefficient is too high for I, the car tends to have quicker oscillations, and does not tend to get up to a quick speed. A low coefficent for I will cause the car to tend to drift to one side of the lane or the other for longer periods of time.
The "D" for derivate is the change in CTE from one value to the next. This means that 1) if the derivative is quickly changing, the car will correct itself (i.e. higher steering angle) faster, such as in the case of a curve, and 2) if the car is moving outward from the middle, this will cause the steering to get larger (as the derivative sign will match the proportional sign), but if the car is moving toward the center (meaning the derivative value will be negative), the car's steering angle will get smoothed out, leading to a more smoother driving experience. Too high of a coefficient leads to almost constant steering angle changes of large degrees, where although the car will be well-centered it can hardly move. Too low of a D coefficient will lead to the oscillations being too high with more overshooting.
These behaved fairly in line with what I was expecting.

Finding the right coefficients
Although it is removed from the final code, I had used Twiddle a little bit to try out different parameters, but found the values found in the original project lessons to be sufficient under my current implementation. I had also used an approach with SGD (stochastic gradient descent) before, but that did not seem to perform as well as Twiddle. I ended up deciding against Twiddle as the results tended to vary greatly - one build of my PID Controller, in a test of 10 runs around the track, made it around six times, half of which were very slow, with the others (using a throttle fully based on steering angle instead of the measured approach I use now with a percentage of steering + a set value) managing to race around the track in excess of 70 mph. Unfortunately, there were also four runs ending with crashes, two of which even happened before the first curve. Part of this was due to noise/variance in the simulator itself, from the looks of it.

In the end, the final values were determined by manual tuning. The ratio of the coefficients to each other that I chose (0.2, 0.004, 3.0) seemed to work well, and I also tried lowering & raising them in conjunction with each other as well as tuning each individually. I typically found that I was creating too high of steering angles (causing crashes if the speed had gotten too high on a straight) by raising them in conjunction with each other, while lowering all of them together meant the car struggled on the larger curves, sometimes not turning enough and shooting off the track.




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
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/e8235395-22dd-4b87-88e0-d108c5e5bbf4/concepts/6a4d8d42-6a04-4aa6-b284-1697c0fd6562)
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

