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
- Proper project plan to explain steps to completing this project, the order of the steps and how far the team eventually gets and stops.
- A definitive yes or no proof of whether or not ROS is a worth-while time, effort and money investment to use over other expensive black-box solutions.

### Object Recognition

- Has to be able to recognize all shapes listed in above image.

- if possible, should work with D-435 camera rather than the 23,000 Euro custom black-box camera.

- Must detect the orientation of the object it's looking at.

- Must detect size of object.

- Must accurately detect 3-D position of object.

- Has to operate on machine-learning logic, not manual programming.

- ??? - Below this point are things we have to ask the customer that we missed - ???

- What conveyor speed should it work with?

  

### Robot Arm

- Must move at safe speeds.
- Must be able to utilize all of its degrees of freedom efficiently
- Must be able to pick-up all listed geometries from the above image.
- Must operate from machine learning logic, not manually programmed logic.
- Must work with data from the depth camera.
- Must actuate.
- ??? - Below this point are things we have to ask the customer that we missed - ???
- At what speeds SHOULD the robot arm operate?
- Is there a requirement for how softly it should grasp all objects?
- How delicately should it move all given objects? Is inertia a consideration?

### Conveyors

- Must work in tandem with the camera and arm.
- Must be able to dynamically slowdown if errors or item overload(for instance, way too many objects are put on the conveyor at once, we want it to slow down and account for this.)
- Must actuate to transport items.