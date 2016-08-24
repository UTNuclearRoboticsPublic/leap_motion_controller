// Copyright (c) 2016, The University of Texas at Austin
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
// 
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
// 
//     * Neither the name of the copyright holder nor the names of its
//       contributors may be used to endorse or promote products derived from
//       this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "Leap.h"	// comes from Leap Motion SDK

#include <iostream>
#include <sstream>
#include <cstring>

#include "ros/ros.h"
#include "tf/transform_datatypes.h"
#include "leap_motion_controller/LeapMotionOutput.h"

/** Implementation of Leap::Listener class. */
class LeapListener : public Leap::Listener
{
  public:
    virtual void onInit(const Leap::Controller&);
    virtual void onConnect(const Leap::Controller&);
    virtual void onDisconnect(const Leap::Controller&);
    virtual void onExit(const Leap::Controller&);
    virtual void onFrame(const Leap::Controller&);

    // ROS publisher for Leap Motion Controller data
    ros::Publisher ros_publisher_;
  private:
};

void LeapListener::onInit(const Leap::Controller& controller)
{
  std::cout << "Leap Motion Controller Initialized" << std::endl;
}

void LeapListener::onConnect(const Leap::Controller& controller)
{
  std::cout << "Leap Motion Controller Connected" << std::endl;
  
  // Enable recognition of the key tap gesture
  controller.enableGesture(Leap::Gesture::TYPE_KEY_TAP);

  // Configure KeyTapGesture parameters
  controller.config().setFloat("Gesture.KeyTap.MinDownVelocity", 50.0);
  controller.config().setFloat("Gesture.KeyTap.HistorySeconds", 0.1);
  controller.config().setFloat("Gesture.KeyTap.MinDistance", 3.0);
  controller.config().save();
}

void LeapListener::onDisconnect(const Leap::Controller& controller)
{
    std::cout << "Leap Motion Controller Disconnected" << std::endl;
}

void LeapListener::onExit(const Leap::Controller& controller)
{
    std::cout << "Leap Motion Controller Exited" << std::endl;
}

/** This method is executed when new Leap motion frame is available.
 *  It extracts relevant data from Leap::Frame and publishes a corresponding ROS message. 
 */
void LeapListener::onFrame(const Leap::Controller& controller)
{
  // Get the most recent frame
  const Leap::Frame work_frame = controller.frame();

  // Create a local instance of LeapMotionOutput message
  leap_motion_controller::LeapMotionOutput ros_msg;
  
  // Add ROS timestamp to ros_msg.header
  ros_msg.header.stamp = ros::Time::now();

  // Set ROS frame_id in ros_msg.header
  ros_msg.header.frame_id = "leap_motion";

  // FYI
  std::cout << "Frame id: " << work_frame.id()
            << ", timestamp: " << work_frame.timestamp()
            << ", hands: " << work_frame.hands().count()
            << ", extended fingers: " << work_frame.fingers().extended().count()
            << ", gestures: " << work_frame.gestures().count() << std::endl;
  
  Leap::HandList hands_in_frame = work_frame.hands();			// get HandList from frame
  ros_msg.left_hand = false;						// by default set both hands false in msg;
  ros_msg.right_hand = false;
  if (hands_in_frame.count() > 2)					// if, for some reason, there are more than 2 hands, that's NOT OK, imho
    ROS_INFO("WHAT? There are more than 2 hands in the frame - that's way too many hands! Going to pretend that there are no hands until next frame.");
  else if (hands_in_frame.count() > 0)					// if there are more than 0 hands
  {
    Leap::Hand left_hand, right_hand;					// create empty objects for left_hand and right_hand
    for(int i = 0; i < hands_in_frame.count(); i++)			// go thru all the elements in HandList
    {
      // If Hand is left
      if (hands_in_frame[i].isLeft())
      {
	ros_msg.left_hand = true;					// set ros_msg.left_hand TRUE
	left_hand = hands_in_frame[i];					// set this hand as left_hand
	printf("Lefty HERE! Sphere radius: %.1f; Pinch strength: %f\n", left_hand.sphereRadius(), left_hand.pinchStrength());		// FYI
	
	// convert palm position into meters and copy to ros_msg.left_palm_pos
	ros_msg.left_palm_pose.position.x = left_hand.palmPosition().x/1000;
	ros_msg.left_palm_pose.position.y = left_hand.palmPosition().y/1000;
	ros_msg.left_palm_pose.position.z = left_hand.palmPosition().z/1000;
	
	// Get hand's roll-pitch-yam and convert them into quaternion.
	// NOTE: Leap Motion roll-pith-yaw is from the perspective of human, so I am mapping it so that roll is about x-, pitch about y-, and yaw about z-axis.
	float l_yaw = left_hand.palmNormal().roll();
	float l_roll = left_hand.direction().pitch();
	float l_pitch = -left_hand.direction().yaw();			// Negating to comply with the right hand rule.
	printf("Lefty HERE! Roll: %.2f; Pitch: %.2f; Yaw: %.2f\n", l_roll, l_pitch, l_yaw);
	ros_msg.left_palm_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(l_roll, l_pitch, l_yaw);

	// Put sphere radius and pinch strength into ROS message
	ros_msg.left_hand_sphere_radius = left_hand.sphereRadius();
	ros_msg.left_hand_pinch_strength = left_hand.pinchStrength();
      }
      
      // If Hand is right
      else if (hands_in_frame[i].isRight())
      {
	ros_msg.right_hand = true;					// set ros_msg.right_hand true
	right_hand = hands_in_frame[i];					// set this hand as right_hand
	printf("Righty HERE! Sphere radius: %.1f; Pinch strength: %f\n", right_hand.sphereRadius(), right_hand.pinchStrength());	// FYI
	
	// Convert palm position into meters and copy to ros_msg.right_palm_pos
	ros_msg.right_palm_pose.position.x = right_hand.palmPosition().x/1000;
	ros_msg.right_palm_pose.position.y = right_hand.palmPosition().y/1000;
	ros_msg.right_palm_pose.position.z = right_hand.palmPosition().z/1000;

	// Get hand's roll-pitch-yam and convert them into quaternion.
	// NOTE: Leap Motion roll-pith-yaw is from the perspective of human, so I am mapping it so that roll is about x-, pitch about y-, and yaw about z-axis.
	float r_yaw = right_hand.palmNormal().roll();
	float r_roll = right_hand.direction().pitch();
	float r_pitch = -right_hand.direction().yaw();			// Negating to comply with the right hand rule.
	printf("Righty HERE! Roll: %.2f; Pitch: %.2f; Yaw: %.2f\n", r_roll, r_pitch, r_yaw);
	ros_msg.right_palm_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(r_roll, r_pitch, r_yaw);

	// Put sphere radius and pinch strength into ROS message
	ros_msg.right_hand_sphere_radius = right_hand.sphereRadius();
	ros_msg.right_hand_pinch_strength = right_hand.pinchStrength();
      }
    } // for
  } // else if (hands_in_frame.count() > 0)
  
  // Getting gestures
  // TODO Do I actually need gestures in this ROS driver or at all. If the answer is YES, perhaps add some other gestures, e.g. swipe and circle.
  // Maybe enable gestures with a YAML file
  // leap_motion_controller_data:
  //	-left_hand_frame:	"leap_motion"	# that makes sense if you start using PoseStamped for palm, this way you can have different frames from different hands if you want
  //	-right_hand_frame:	"leap_motion"
  //	-enable_key_tap:	true
  //	-enable_swipe:		true
  // etc ...
  Leap::GestureList gestures_in_frame = work_frame.gestures();
  ros_msg.left_hand_key_tap = false;					// by default set KEY_TAP gestures to false on both hands
  ros_msg.right_hand_key_tap = false;
  
  // Since only KEY_TAP gestures have been enabled, any detected gesure is a KEY_TAP gesture.
  for (int j = 0; j < gestures_in_frame.count(); j++)			// go through all the gestures in the list
  {
    // if the hand of j-th gesture is valid and left
    if (gestures_in_frame[j].hands()[0].isValid() && gestures_in_frame[j].hands()[0].isLeft())
    {
      ros_msg.left_hand_key_tap = true;					// report that there was a lefthand_key_tap
    } // end if
    // if the hand of j-th gesture is valid and right
    if (gestures_in_frame[j].hands()[0].isValid() && gestures_in_frame[j].hands()[0].isRight())
    {
      ros_msg.right_hand_key_tap = true;				// report that there was a righthand_key_tap
    } // end if
  } // end for
   
  // Publish the ROS message based on this Leap Motion Controller frame.
  ros_publisher_.publish( ros_msg );
 
} // end LeapListener::onFrame()

/** Main method. */
int main(int argc, char** argv)
{
  // ROS init
  ros::init(argc, argv, "leap_motion");
  
  // ROS node handle
  ros::NodeHandle nh;

  // Instance of LeapListener
  LeapListener listener;

  // Set up ROS publisher for LeapListener
  // TODO change the topic name when doing code cleanup
  listener.ros_publisher_ = nh.advertise<leap_motion_controller::LeapMotionOutput>("leapmotion_general"/*"leap_motion_output"*/, 10);

  // Instance of LEAP Controller
  Leap::Controller controller;
  
  // Have the listener receive events from the controller
  controller.addListener(listener);

  // Do ros::spin() until CTRL+C is pressed
  ros::spin();
 
  // Remove the listener when done
  controller.removeListener(listener);

  return 0;
} // main