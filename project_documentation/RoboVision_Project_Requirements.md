# RoboVision Project Requirements

[TOC]

## Intro

This document lists out all the requirements for the SOFA project RoboVision. It's a project that's being hosted by Fontys HS, Venlo for the second semester of year 2020. The purpose of the project is primarily to serve as a detailed insight into using ROS for extremely practical and complex robotics programming. The project is supposed to use a depth camera, robot arm and conveyor system all in unison to perform sorting work.

### Terms

ROS - Robot Operating System

### Scope

Scope goes as follows - object recognition software for depth camera, actuation for robot arm, actuation of the conveyor belt. As this is more so a proof of concept project than anything else, efficient documentation and at least one fully functioning component are the core of the end-goal. Outside of working with ROS to test and develop the hardware that we will be working with, everything else is outside of the scope of the project. The camera is supposed to recognize a set of specific objects, as such:

![](./documentation_images/Requirements_1.png)

The robot arm is going to have to account for these shapes specifically, with the potential addition of bags, but that remains optional.

## Requirements

### General Project Requirements

- Accurate and Detailed documentation of the code, enough that whoever reads would be fully able to continue the work where the SOFA team left off.
  - Diagrams for general overview workflow, as well as workflow for the conveyor, robot arm and camera.
  - Diagrams depicting the full ROS structure of the code in the program.
  - Documentation of all functions, nodes, message types, etc.

- Proper project plan to explain steps to completing this project, the order of the steps and how far the team eventually gets and stops.
- A definitive yes or no proof of whether or not ROS is a worth-while time, effort and money investment to use over other expensive black-box solutions.

### Object Recognition

- Has to be able to recognize all shapes listed in above image.

- if possible, should work with D-435 camera rather than the 23,000 Euro custom black-box camera.

- Must detect the orientation of the object it's looking at.

- Must detect size of object.

- Must accurately detect 3-D position of object.

- Has to operate on machine-learning logic, not manual programming.

- Must communicate with robot arm and conveyor for coordination.

- Must work with maximum efficient conveyor and robot arm speed.

  - This must be calculated through taking the maximum efficient work-speed of the robot arm and the maximum efficient work-frame-rate of the camera. Of course, also the conveyor speed should be considered.
  
  

### Robot Arm

- Must move at safe speeds.
- Must be able to utilize all of its degrees of freedom efficiently.
  - It shouldn't do a 340 rather than a 20 degree turn.
  - It shouldn't not use any of its degrees of freedom.
- Must be able to pick-up all listed geometries from the above image.
- Must operate from machine learning logic, not manually programmed logic.
- Must coordinate with data from the depth camera.
  - If object is unknown, must not pick up.
  - if object is known, must pick up.
  - Must use camera data to pick up object.
- Must actuate/perform tasks at it's highest stable speed.
  - Currently this speed is unknown, it must be researched and added onto this sub-point at a later date.
- Must grab items correctly.
  - Must grab item in a way that would ensure a strong grip onto it.
  - Must grab item in a way that would not cause it to be damaged under the weight of its own inertia being focused within a structural weak spot.
- Must deposit items correctly.
  - Must not accelerate carried objects to projectile speeds when depositing.
  - Must not release from height that could threaten the structural stability of the carried object.
  - Must deposit items within a designated area and not outside of it.

- Must hold on to items during moving without incident of dropping.(no butter fingers)

### Conveyors

- Must work in tandem with the camera and arm.
  - Must move and stop when prompted to by the camera/external sensors.
  - Must not cause objects on conveyor to fly off due to inertia and incorrect speeding up and speeding down.
- Must be able to dynamically slowdown if errors or item overload(for instance, way too many objects are put on the conveyor at once, we want it to slow down and account for this.)
- Must actuate to transport items.





________________________________________________________________________________________________________________________________________________________END