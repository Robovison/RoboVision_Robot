# User stories

[TOC]

## Depth Camera

| Name                | Detect Object                                                |
| :------------------ | ------------------------------------------------------------ |
| Description         | The camera needs to detect all objects in front of itself(that it has been trained to detect) accurately and correctly. |
| Pre-requisite state | Activated                                                    |
| Steps               | 1. Object(s) is/are placed in front of the camera.<br />2. Camera uses machine learned algorithm to analyse incoming stream of images.<br />3. Camera determines the amount of recognized objects. |
| End-state(Success)  | Camera now has a list of crudely recognized objects in front of itself. |
| End-state(Failure)  | If the camera fails to recognize any objects, it results in one of two things:<br />   1. A few missed objects during sorting.<br />   2. Catastrophic failure that causes damage to undetected objects. |

| Name                | Analyse Object                                               |
| :------------------ | ------------------------------------------------------------ |
| Description         | The camera analyses detected objects, using machine-learned algorithms, to detect their depth and orientation. |
| Pre-requisite state | Detect Object(Successful)                                    |
| Steps               | 1. Camera analyses detected objects.<br />2. Camera determines the depth and orientation of recognized objects. |
| End-state(Success)  | Camera now has a list of fully recognized, depth-gaged and orientation-established objects in front of itself. |
| End-state(Failure)  | Camera is unable to gage the given objects correctly, resulting in an error-state that could prohibit the work of the arm. |

| Name                | Send Coordinate Commands to Robot Arm                        |
| ------------------- | ------------------------------------------------------------ |
| Description         | The camera communicates coordinate information to the robot arm. |
| Pre-requisite state | Analyse Object(Successful)                                   |
| Steps               | 1. Camera sends out messages over the topic "object_position_msgs". |
| End-state(Success)  | Camera successfully publishes it's information.              |
| End-state(Failure)  | Camera fails to publish it's data, which halts the entirety of the sorting process. |

## Robot Arm

| Name                | Move To Idle Stance                                          |
| :------------------ | ------------------------------------------------------------ |
| Description         | The robot arm moves to a default idle position where it won't get in the way of anything else. |
| Pre-requisite state | Arm is in a non-idle stance                                  |
| Steps               | 1. Arm actuates to a default idle stance                     |
| End-state(Success)  | Arm is back to a default idle stance                         |
| End-state(Failure)  | Arm throws an exception and is not in its default idle stance. Could cause the system to stop entirely since it might be reaching into a box when it freezes. |

| Name                | Move To Designated Coordinates For Pickup                    |
| :------------------ | ------------------------------------------------------------ |
| Description         | The robot arm moves itself to designated coordinates in a way that allows it's grabber to do a little grabby-grabby action. |
| Pre-requisite state | Activated and not in an error-state.                         |
| Steps               | 1. Arm receives coordinates information. <br />2. Arm actuates to a given coordinate. |
| End-state(Success)  | Arm is in correct position for grabby-grabby procedure.      |
| End-state(Failure)  | Arm throws an exception and is not in its correct position. In the event of such a malfunction, the arm absolutely freezes and shuts down, if unable to revert back to safe idle stance. |

| Name                | Deposit Item In An Organized Stacking Way                    |
| :------------------ | ------------------------------------------------------------ |
| Description         | The robot arm moves to stack item within the deposit-box.    |
| Pre-requisite state | Holding an object that needs to be deposited.                |
| Steps               | 1. Arm moves to calculated and automated position for drop.  |
| End-state(Success)  | If deposit is successful, this can go 2 ways - the arm does not receive anymore items for pickup and goes to idle state. Alternatively, the arm goes back to pickup the next object, pointed to by the camera. |
| End-state(Failure)  | Something goes wrong within the arm and it throws an exception and freezes. Since it has no external input of whether or not it's completely missed the mark or just gone haywire, this state can only be reached via code exception. |

## Robot Grabber

| Name                | Grab                                                         |
| :------------------ | ------------------------------------------------------------ |
| Description         | The grabber grabs onto an object.                            |
| Pre-requisite state | Moved into position to grab a given object via the robot arm. |
| Steps               | 1. The grabber closes, clasping onto the object it's supposed to grab. |
| End-state(Success)  | A grabbed state is entered, in which the grabber is holding an object. |
| End-state(Failure)  | An exception is thrown, in the event the grabber failed to hold onto an object. |

| Name                | Release                                                      |
| :------------------ | ------------------------------------------------------------ |
| Description         | The grabber releases an object that it was holding when prompted by the arm. |
| Pre-requisite state | Holding an object that needs to be deposited.                |
| Steps               | 1. The grabber releases it's held object.                    |
| End-state(Success)  | Grabber is now in a idle or "not-holding-anything" state.    |
| End-state(Failure)  | Grabber fails to release the object. In this event, the grabber throws an exception and the system retries several times before freezing(for safety reason). |