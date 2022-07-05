# Orient_Tbot_Polaris
This project aims to autonomously orient the Turtlebot3 to Polaris (the North Star) from any given location on Earth.

## Problem statement

The Polaris star is aligned with True North from anywhere on Earth. However, Magnetic North and True North are not the same almost anywhere on Earth. This is caused by the fact that the Earth's magnetic poles are not aligned with its axis of rotation, and drift on a yearly basis. Therefore, to compute True North heading, knowing the Magnetic Declination is necessary in addition to magnetic heading. By adding the Magnetic Declination to the Magnetic North heading, we can obtain the True North heading. The Magnetic Declination can be computed using the World Magnetic Model 2020-2025, and is a function of latitude, longitude and altitude above sea level.

The Turtlebot3 can execute velocity commands in x and y and can rotate on itself around the z axis. It is equipped with an IMU incorporating a magnetometer, which allows it to compute its absolute heading with respect to magnetic North, in the ENU frame of reference. To achieve its objective, this ROS implementation performs the following tasks:
- Given a latitude, longitude and altitude, calculate the True North heading, corresponding to the North Pole.
- Orient the Turtlebot to the True North heading, observe data from the `/odom` topic and publish angular velocity commands to the `/cmd_vel` topic.
- Simulate the Turtlebot in the Gazebo simulator in order to validate the implementation.

## Setup

### Dependencies

This package requires ROS Melodic and Gazebo 9 and was tested on Ubuntu 18.04. It also requires the turtlebot3 packages, which can be installed with

```sudo apt install ros-melodic-turtlebot3-*```

### Installation

First clone the repository into your catkin workspace `src` folder. From your catkin workspace, build and source the package using

```catkin build turtlebot_polaris```

```source devel/setup.bash```

## Execution

The package is designed to run directly on the Turtlebot3 or in parallel with a simulated model in Gazebo.

To run on the Turtlebot3, launch:
```roslaunch turtle_polaris turtlebot3_controller.launch```

To launch the simulation environment, launch:
```roslaunch turtle_polaris turtlebot3_sim.launch```

Alternatively, you can launch both at the same time using:
```roslaunch turtle_polaris turtle_to_polaris_main.launch```

`turtlebot3_controller.launch` and `turtle_to_polaris_main.launch` accept 3 launch arguments: `lat`, `lon`, and `alt`. They represent latitude (deg_dec), longitude (deg_dec) and altitude above sea level (m). They are set to 0.0 by default.

`turtlebot3_sim.launch` accepts 1 launch argument: `yaw` (rad). It represents the initial yaw of the robot and is set to 0.0 by default (due East).
