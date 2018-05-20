# FCND-Term1-P3-3D-Quadrotor-Controller
Udacity Flying Car Nanodegree - Term 1 - Project 3 - 3D Quadrotor Controller

# Project description

In this project, we implement and tune a [cascade PID controller](https://controlstation.com/cascade-control-cascade-control-configured/) in a simulator for quadcopter. 
The controller design uses feed-forward strategy as explained in this paper, [Feed-Forward Parameter Identification for Precise Periodic
Quadrocopter Motions](http://www.dynsyslab.org/wp-content/papercite-data/pdf/schoellig-acc12.pdf), by
[Angela P. Schoellig](http://www.dynsyslab.org/prof-angela-schoellig/).
 The following diagram could be found on that paper describing the cascaded control loops of the trajectory-following controller:

![Cascade control](./images/cascade_control_from_article.png)

## Implementation

The simulator is realistic in the physics that it models. So we need to put some constraints on the 
outputs of different part of the controller otherwise things can go wrong when some of those 
limits are not implemented correctly. Instructions on how to use the simulator can be found in [How to use simulator](How_to_use_simulator.md). 
There are in total five scenarios that cover all the aspects of the controller.

- [/config/QuadControlParams.txt](./config/QuadControlParams.txt): This file contains all control gains and other desired tuning parameters.
- [/src/QuadControl.cpp](./src/QuadControl.cpp): This file contains all of the code for the controller.

### Build and Run the code

### Linux ###

For Linux, the recommended IDE is QtCreator.

1. Download and install QtCreator.
2. Open the `.pro` file from the `<simulator>/project` directory.
3. Compile and run the project (using the tab `Build` select the `qmake` option.  You should see a single quadcopter, falling down.

**NOTE:** You may need to install the GLUT libs using `sudo apt-get install freeglut3-dev`

#### CMake on Linux ####

For doing manual builds using `cmake`, we have provided a `CMakeLists.txt` file with the necessary configuration.

**NOTE: This has only been tested on Ubuntu 16.04, however, these instructions should work for most linux versions.  Also note that these instructions assume knowledge of `cmake` and the required `cmake` dependencies are installed.**

1. Create a new directory for the build files:

```sh
cd FCND-Controls-CPP
mkdir build
```

2. Navigate to the build directory and run `cmake` and then compile and build the code:

```sh
cd build
cmake ..
make
```

3. You should now be able to run the simulator with `./CPPSim` and you should see a single quadcopter, falling down.

#### Scenario 1: Intro
If the mass doesn't match the actual mass of the quad, it'll fall down. 
So we tune the Mass parameter in [/config/QuadControlParams.txt](./config/QuadControlParams.txt) to make the vehicle more or less stay in the same spot.

![C++ Scenario 1](./images/1.gif)

The standard output when the scenario 1 passes:

```
PASS: ABS(Quad.PosFollowErr) was less than 0.500000 for at least 0.800000 seconds
```

#### Scenario 2: Body rate and roll/pitch control

To accomplish this we implement:

* [GenerateMotorCommands method](./src/QuadControl.cpp#L56-L90) - It implement these equations:

![Moment force equations](./images/force_formula.png)

`F_0` to `F_3` are the individual motor's thrust, `tau(x,y,z)` are the moments along xyz axes.
`F_t` is the total thrust, `kappa` is the drag/thrust ratio and `l` is the drone arm length over square root of 2.\
(The calculation implementation for the motor commands is in [/src/QuadControl::GenerateMotorCommands method ](./src/QuadControl.cpp#L56-L90) from line 56 to 90)

**NOTE:** We are using NED coordinates ie, the `z` axis is inverted. This means the 
yaw is defined positive when drones yaw clockwise looking from above.

* [BodyRateControl method](./src/QuadControl.cpp#L92-L125) - A [P controller](https://en.wikipedia.org/wiki/Proportional_control) that controls body rate ie roll rate,
 pitch rate and yaw rate.
 
At this point, the `kpPQR` parameter has to be tuned to stop the drone from flipping.\
(The body rate control is implemented in [/src/QuadControl::BodyRateControl method ](./src/QuadControl.cpp#L92-L125) from line 92 to 125)

* [RollPitchControl method](./src/QuadControl.cpp#L128-L165) - A P controller for the roll, pitch and yaw. 

![Roll and pitch P controller](./images/dynamics.png)


This controller receives the commanded accelerations in x and y directions. And to achieve these accelerations
 we need to apply a P controller to the elements `R13` and `R23` of the [rotation matrix](https://en.wikipedia.org/wiki/Rotation_matrix) from world frame to body frame:
(The roll pith control is implemented in [/src/QuadControl::::RollPitchControl method ](./src/QuadControl.cpp#L128-L165) from line 128 to 165)

![Roll and pitch P controller](./images/roll_pitch_p_controller.png)

But to output roll and pitch rates we another equation to apply:

![From b to pq](./images/roll_pitch_from_b_to_pq.png)

**NOTE:** It is important to note that the thrust received is positive but while working with NED coordinates
we need it to be inverted and converted to acceleration before applying the equations. 
After the implementation is done, start tuning `kpBank` and `kpPQR`.

![C++ Scenario 2](./images/2.gif)

The standard output when the scenario 2 passes:

```
PASS: ABS(Quad.Roll) was less than 0.025000 for at least 0.750000 seconds
PASS: ABS(Quad.Omega.X) was less than 2.500000 for at least 0.750000 seconds
```

#### Scenario 3: Position/velocity and yaw angle control

There are three methods to implement here:

* [AltitudeControl](./src/QuadControl.cpp#L167-L208): This is a [PD controller](https://en.wikipedia.org/wiki/PID_controller) to control the acceleration in z direction and outputs the thrust needed to control the altitude.

![Altitude controller equations](./images/altitude_control.png)

Here, c is the magnitude of acceleration produced by the motors thrust. We already included the direction of c 
while subtracting the matrix.\
(The altitude control is implemented in [/src/QuadControl::AltitudeControl method ](./src/QuadControl.cpp#L167-L208) from line 167 to 208)

* [LateralPositionControl](./src/QuadControl.cpp#L211-L254) Here we use a cascaded proportional controller: an inner one for velocity, 
with gain Kv and an outer one with gain Kp to control accelerations in 
`x` and `y` direction.\
(The lateral position control is implemented in [/src/QuadControl::LateralPositionControl method ](./src/QuadControl.cpp#L211-L254) from line 211 to 254)

![Cascaded controller equation](./images/cascaded_controller.png)

* [YawControl](./src/QuadControl.cpp#L257-L292): This is a simple P controller which outputs the yaw rate (in body frame). We wrap the yaw error in range `[-pi, pi]`.

When looking at a controller linearized for small motions, r is approximately equal to the yaw rate.  If we used a linearized controller for roll and pitch, the equations would also be much simpler, but we would have much larger control errors, so we need to take into account the attitude in those cases.  For yaw, with this type of control, we can get away with keeping the linearized assumption on the controller, which is why we pass yaw rate back directly without taking into account the attitude.\
(The yaw control is implemented in [/src/QuadControl::YawControl method ](./src/QuadControl.cpp#L257-L292) from line 257 to 292)

We start tuning for altitude controller using `kpPosZ` and `kpVelZ` and then move to tuning 
the lateral controller by using `kpPosXY`, `kpVelXY`. In last we tune yaw controller with `kpYaw` 
. Here is a video of the scenario when it passes:

![C++ Scenario 3](./images/3.gif)

The standard output when the scenario 3 passes:

```
PASS: ABS(Quad1.Pos.X) was less than 0.100000 for at least 1.250000 seconds
PASS: ABS(Quad2.Pos.X) was less than 0.100000 for at least 1.250000 seconds
PASS: ABS(Quad2.Yaw) was less than 0.100000 for at least 1.000000 seconds
```

#### Scenario 4: Non-idealities and robustness

By now each controller is implemented and tuned. 
Ok, now we need to add an integral part to the altitude controller to move it from PD to PID controller.

![Altitude controller equation with integral term](./images/integral_term.png)

Now we need to tune the integral control, and other control parameters until all the quads successfully move properly.

![C++ Scenario 4](./images/4.gif)

The standard output when the scenario 4 passes::

```
PASS: ABS(Quad1.PosFollowErr) was less than 0.100000 for at least 1.500000 seconds
PASS: ABS(Quad2.PosFollowErr) was less than 0.100000 for at least 1.500000 seconds
PASS: ABS(Quad3.PosFollowErr) was less than 0.100000 for at least 1.500000 seconds
```

#### Scenario 5: Tracking trajectories

Now that we have all the working parts of a controller, we will put it all together and test it's performance once again on a trajectory.
The drone needs to follow a 8 shaped trajectory. In order to pass this we need to put a constraint on `bx` and 
`by` so that the drone does not try to tilt more than a limit. For example, it could tilt 90 degrees and try to 
produce infinite thrust to hover.

![C++ Scenario 5](./images/5.gif)

The standard output when the scenario 5 passes::

```
PASS: ABS(Quad2.PosFollowErr) was less than 0.250000 for at least 3.000000 seconds
```
