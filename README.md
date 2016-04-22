# leap_motion_controller

ROS driver for Leap Motion (TM) Controller (https://www.leapmotion.com). It reads data from Leap Motion Controller and publishes it as ROS messages.
 
#### Instructions
Install:

leap_motion_controller uses Leap Motion SDK. Download original drivers and the SDK from https://developer.leapmotion.com and install it in any desired location.

Add `export LEAP_SDK=[PATH TO YOUR LEAP SDK INSTALLATION]` to your .profile or .bashrc file.

Clone and compile.

Run:

`rosrun leap_motion_controller leap_motion`

