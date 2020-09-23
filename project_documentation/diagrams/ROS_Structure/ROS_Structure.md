# Nodes

- Depth Camera Node - responsible for receiving input messages from the depth camera system and processing the data into a useable format for everything else in the system.
- Feeding Conveyer Node - responsible for actuating the conveyor. Specifically for stopping it when the box of goods to be moved arrives at the appropriate location.
- Robot Arm Node(ROS Environment) - responsible for taking object position information from the Depth Camera Node and applying it to the machine learned algorithm which calculates how and where to robot arm needs to manoeuvre in order for the grabber to be within grabbing distance.
- Robot Arm Node(Micro-Controller Environment) - responsible for actuating the hardware of the robot arm after it gets the calculated directions from the ROS Environment node.
- Robot Grabber Node(ROS Environment) - responsible for taking information from the ROS Environment Robot Arm Node and relaying it to the Micro-Controller Environment.
- Robot Grabber Node(Micro-Controller Environment) - Responsible for actuating the grabber bits.

# 3rd Party Packages

- Camera Package - 3rd Party software that will read the raw input from whatever depth camera we use into a useable format.

# Topics

- camera_msgs_raw - The raw data from the cameras. Used by the Depth Camera Node.
- conveyor_stop_msgs - Tells the conveyor when to stop moving. It will only really be used for stopping it for our purposes, can be changed to fit stop and go functions later down the line.
- object_position_msgs - translated information from the Depth Camera Node that makes useable data from the raw camera data. This is used by the robot arm node.
- arm_actuation_msgs - actuation instructions for the micro-controller environment.
- grabber_msgs - used by both the arm and the grabber to talk back and forth when the grabber needs to grab or stop grabbing something. The grabber communicates when it is done with either of the functions so that the robot arm knows it's safe to move on.
- grabber_actuation_msgs - actuation instructions for the micro_controller environment.