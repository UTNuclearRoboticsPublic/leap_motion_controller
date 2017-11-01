# leap_motion_controller

ROS driver for Leap Motion (TM) Controller (https://www.leapmotion.com). It reads data from Leap Motion Controller and publishes it as ROS messages.
 
### Instructions
**Install:**

1. Download the Leap Motion V2 Desktop SDK. Use the files given by 'jdonald' here:
https://community.leapmotion.com/t/error-in-leapd-malloc/4271/5.

2. The downloaded archive also contains a folder named LeapSDK. Copy this LeapSDK folder to any desired location, e.g. your home folder (in examples I will refer to it as ~/LeapSDK)

3. CMakeLists.txt expects to find LeapSDK at $LEAP_SDK, so add, for example, the following line to your .bashrc file:

`export LEAP_SDK=~/LeapSDK`

4. Clone this repository to your catkin workspace and compile using catkin_make or catkin build.

**Run:**

`rosrun leap_motion_controller leap_motion`

