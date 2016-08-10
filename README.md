# leap_motion_controller

ROS driver for Leap Motion (TM) Controller (https://www.leapmotion.com). It reads data from Leap Motion Controller and publishes it as ROS messages.
 
### Instructions
**Install:**

Since leap_motion_controller uses Leap Motion SDK, first download the original drivers and the SDK from https://developer.leapmotion.com. Install using instructions in their README and copy LeapSDK to any desired location.

Add `export LEAP_SDK=[PATH TO YOUR LeapSDK FOLDER]` to your .bashrc file.

Clone this repository to your catkin workspace and compile using catkin_make.

**Run:**

`rosrun leap_motion_controller leap_motion`

