# Project Plan

[TOC]

## Context

This document serves as the project plan for the Robovision project. It details the overall goals of this project, the different phases that will comprise the project and the order in which those phases will be executed. Each phase will be comprised of a description and a listing of the goals of the given phase. This, alongside some time organizing, serves the purpose of giving way of measuring the length of progress, as well as the quality/completeness of progress.

## Goals

The point of the Robovision project, succinctly speaking, is to get a robot arm to pickup and put down objects effectively from box A into box B using a depth camera that is cheaper than it's industrial-standard counterpart. It's a sort of experiment for both cheaper solutions in depth cameras, as well as a testing grounds for the ROS(Robot Operating System) development environment in more practical fields. To put this to a list, it would go like this:

- Make the robot arm work with ROS code.
- Make the cheaper depth camera work with ROS code.
- Pick up and put down items from one box to another, using a robot arm.
- Document the entirety of this project(code, workflow diagrams, etc.) for later continuation.
- Learn a bunch about ROS

## Phases

Each of the phases listed below is reflected in a list of requirements and diagrams that describe the planned system. So, in the event that everything in the planning, documentation and diagrams is implemented successfully, all of the phases below can be considered finished.

### Phase I

This project phase consists of the startup. Here the team primarily gets settled in, meets the project customer, discuss requirements, make documentation and prepare their work environment for ROS programming. Due to the nature of this project and the students working on it, this phase also includes a brief introduction into working with ROS. This phase's goals can be summarised as such:

- Collect all specifications and requirements about the project from the customer. Understand the status of the project and plan out work accordingly.
- Create all necessary standard project documentation.
  - Requirements list.
  - Project Plan.
  - Structure Diagrams.
  - Workflow Diagrams.
- Set up Ubuntu and ROS so that everyone can work with it.
- Learn about the basics of working with ROS. Everyone should understand how to create nodes in ROS and make them talk to each other via topics.
- Create project repository to have the skeleton of the project ready and a place to put all documentation.
- Organise a workflow for the team. Standardise work distribution, work planning, etc.

### Phase II

This phase will consist of the actual digging into the project. Documentation will continue to be written and updated, code will also start to be written here. At this point, experimentation with the camera hardware should be done and a machine learning algorithm should at the very least be selected and developed. The primary goal of Phase II is to get the cheaper D-435 depth camera to work in the way it needs to for the arm system or otherwise thoroughly prove that this is simply not possible(this is done since the robot arm already comes bundled with a very expensive camera, we just need to see if we can do without it). On-site experimentation is also expected for this phase. The goals can be summarised as follows:

- Set up all the required software and hardware in regards to the depth camera portion of the Robovision project. Follow the diagrams that were developed during Phase I or update them accordingly in Phase II.
  - Experiment and research whether or not the D-435 depth camera can fulfil the requirements of the project in practice.
  - Develop the ROS code necessary to link the camera to the main system.
  - Develop and implement machine learning algorithm to teach the camera everything it needs for correct object recognition.
- Create and maintain code documentation for the project as a whole. Additionally, document results on the camera - why it does or does not work and why it is or is not preferable to the alternative.
- Continue making sprints and keeping work organised

This phase can take between 1 and 3 months. The complexity of this can vary wildly and the experience gain of the team can change things a lot, but a reasonable expectation is for it to be done within a month, maybe month and a half.

### Phase III

This phase will be very similar to Phase II in structure, except it will be about the robot arm. Goals go as follows:

- Implement all code and structure/workflows for the robot arm.
  - Connect the robot arm to ROS(the main system).
  - Using machine learning, teach the arm to fully actuate and work.
  - Coordinate the arm with information from the camera system.
  - Have it move items from one box to another in the appropriate manner.
- Continue documenting everything.

Frankly, this phase can be so obscenely difficult and complex that it might not even get started to be worked on, or if it does - it won't have any hope to be finished by the inexperienced team that is working on this project for the duration of the given semester. It can take anywhere between a month to half a year or even more. 

### Phase IV

This phase will act as the closing phase of the project. In the event that Phase III is simply beyond the doing of the team, given their time and experience limitations, it will be paused and/or skipped altogether so that the final phase can be seen through. This phase will mostly consist of showcasing final demos, wrapping up all necessary documentation, be it personal or group, and final tune-ups to the thus-far implemented system. Primarily should be document work however, not programming. Goals of the phase go as follows:

- Reflect on all complete and incomplete work. Write down the results for future reference when it comes to avoiding common mistakes/potholes.
- Finish the final documentation of the code and the project as per Fontys requirements.
- Finish all personal documentation goals.
- Save all progress on the project and hand it off to GTL.
- Save demo material.

This is the ending phase so it shouldn't take more than 2 weeks realistically.

## Conclusion

Given the correct following of all the listed phases and sticking within the allotted times, this should allow all requirements of the Robovision project to be completed. Given errors and setbacks, it should at least show a detailed documentation of all the development processes and problems for future continuation of the project. If nothing else, the conclusion of this project should succinctly prove whether or not the cheaper camera alternative is viable and whether or not ROS is a desirable development environment.