# Nodes

### ROS Environment

- Depth Camera Node - responsible for receiving input messages from the depth camera system and processing the data into a usable format for everything else in the system.
- Robot Arm Node - responsible for taking object position information from the Depth Camera Node and applying it to the machine learned algorithm which calculates how and where to the robot arm needs to manoeuvre in order for the grabber to be within grabbing distance.
- Robot Grabber Node - responsible for taking information from the ROS Environment Robot Arm Node and relaying it to the Micro-Controller Environment.

### Micro-Controller Environment

- Robot Arm Node - responsible for actuating the hardware of the robot arm after it gets the calculated directions from the ROS Environment node.
- Robot Grabber Node - Responsible for actuating the grabber bits.

# 3rd Party Packages

- Camera Package - 3rd Party software that will read the raw input from whatever depth camera we use into a useable format.

# Topics

- camera_msgs_raw - the raw data from the cameras. Used by the Depth Camera Node.
- object_position_msgs - translated information from the Depth Camera Node that makes useable data from the raw camera data. This is used by the robot arm node.
- arm_actuation_msgs - actuation instructions for the micro-controller environment.
- grabber_msgs - used by both the arm and the grabber to talk back and forth when the grabber needs to grab or stop grabbing something. The grabber communicates when it is done with either of the functions so that the robot arm knows it's safe to move on.
- grabber_actuation_msgs - actuation instructions for the micro_controller environment.