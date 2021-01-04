# Robovision Documentation

[TOC]

## Project Description

This document covers all of the documentation for the code-side of the Robovision project. This is a project, funded, supervised and assisted by GTL(GreenTech Labs) and Fontys HS in Venlo. The purpose of the project is to use ROS(Robot Operating System) to develop a system that uses conveyors, a robot arm and a depth camera to establish a working robot-object-sorting automated system. All of the documentation below goes into some detail about the nodes of the project, the general structure and the hardware components. The most of the actual documentation, however, can be found within the code of the respective nodes.

## Nodes

### CameraSubscriber

This node connects the ROS environment with the running D-435 camera, processes image data via inference and posts processed data to other ROS topics. It has an inner class called image_converter which is to do with the actual data processing and connection setup.

#### Methods

| Name      | Parameters                    | Description                                                  |
| --------- | ----------------------------- | ------------------------------------------------------------ |
| subscribe | self                          | Subscribes to the D-435                                      |
| callback  | self<br />data - any          | Processes incoming data via inference(NOT FULLY IMPLEMENTED) |
| publish   | self<br />processedData - any | Publishes processed data to a designated ROS topic           |

