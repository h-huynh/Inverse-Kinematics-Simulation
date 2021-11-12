## Inverse Kinematics
A project by Henry Huynh (huynh407@umn.edu) / (henryh1404@gmail.com)

## About

A short project for _CSCI 5611: Animation and Planning in Games_. The directive was to use inverse kinematic techniques to simulate a multi-arm skeleton.

Currently, it has two arms and a total of 8 joints. The joints are connected to a single root, but offset slightly along the x-axis.

There are a number of adjustable parameters that can change how the simulation runs.
- The LERP variable, t, which controls how fast the linear interpolation between new and previous angles occurs at every frame update.
- The location of the root.
- The default lengths and widths of the links.
- The rotational limits of the joints.

The goal is currently controlled by the position of the mouse cursor. If it out of reach, the arms will still attempt to reach as closely to the goal as possible.

## Code

The source code is available for download [here](https://github.com/h-huynh/Inverse-Kinematics-Simulation).

## Media

### Demonstration

A short demonstration of the project.

[youtube link](https://youtu.be/pReWYlVceWI)

Notable timestamps:
- 0:04 - when goal is out of reach
- 0:10 - demonstrating the rotational limits
- 0:19 - demonstrating how linear interpolation of the angles prevents the arm from instantly jumping to a new best configuration


## Credit

The Vec2 library provided by _CSCI 5611: Animation and Planning in Games_ was utilized.
