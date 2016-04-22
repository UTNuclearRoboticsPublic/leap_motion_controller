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

int msg_ready = 0;					///< This falg is set to 1, when new leap_motion_controller::LeapMotionOutput message is ready to be sent.
leap_motion_controller::LeapMotionOutput send_msg;	///< Next leap_motion_controller::LeapMotionOutput message to be published.

/** Implementation of Leap::Listener class. */
class LeapListener : public Leap::Listener {
  public:
    virtual void onInit(const Leap::Controller&);
    virtual void onConnect(const Leap::Controller&);
    virtual void onDisconnect(const Leap::Controller&);
    virtual void onExit(const Leap::Controller&);
    virtual void onFrame(const Leap::Controller&);

  private:
};

void LeapListener::onInit(const Leap::Controller& controller) {
  std::cout << "Leap Motion Controller Initialized" << std::endl;
}

void LeapListener::onConnect(const Leap::Controller& controller) {
  std::cout << "Leap Motion Controller Connected" << std::endl;
  controller.enableGesture(Leap::Gesture::TYPE_KEY_TAP);		// enable recognition of the key tap gesture
  // Possible to configure KeyTapGesture parameters
  controller.config().setFloat("Gesture.KeyTap.MinDownVelocity", 50.0);
  controller.config().setFloat("Gesture.KeyTap.HistorySeconds", 0.1);
  controller.config().setFloat("Gesture.KeyTap.MinDistance", 3.0);
  controller.config().save();
}

void LeapListener::onDisconnect(const Leap::Controller& controller) {
    std::cout << "Leap Motion Controller Disconnected" << std::endl;
}

void LeapListener::onExit(const Leap::Controller& controller) {
    std::cout << "Leap Motion Controller Exited" << std::endl;
}

/** This method is executed when new Leap motion frame is available. */
void LeapListener::onFrame(const Leap::Controller& controller) {
  
  // Get the most recent frame and report some basic information
  const Leap::Frame work_frame = controller.frame();

  // Create new instance of Leapmsg
  leap_motion_controller::LeapMotionOutput msg;
  
  // Add ROS timestamp to msg.header
  msg.header.stamp = ros::Time::now();

  // Set frame id
  msg.header.frame_id = "leap_motion";

  // FYI
  std::cout << "Frame id: " << work_frame.id()
            << ", timestamp: " << work_frame.timestamp()
            << ", hands: " << work_frame.hands().count()
            << ", extended fingers: " << work_frame.fingers().extended().count()
            << ", gestures: " << work_frame.gestures().count() << std::endl;
  
  // Getting hands
  Leap::HandList hands_in_frame = work_frame.hands();			// get HandList from frame
  msg.left_hand = false;						// by default set both hands false in msg;
  msg.right_hand = false;
  if (hands_in_frame.count() > 2) ROS_INFO("WHAT?? WAY TOO MANY HANDS!!");// if, for some reason, there are more than 2 hands, that's NOT OK, imho
  else if (hands_in_frame.count() > 0) {				// if there are more than 0 hands
    Leap::Hand left_hand, right_hand;					// create empty objects for left_hand and right_hand
    for(int i = 0; i < hands_in_frame.count(); i++) {			// go thru all the elements in HandList
      if (hands_in_frame[i].isLeft()) {					// if Hand is left
	msg.left_hand = true;						// set msg.left_hand true
	left_hand = hands_in_frame[i];					// set this hand as left_hand
	printf("Lefty HERE! Sphere radius: %.1f; Pinch strength: %f\n", left_hand.sphereRadius(), left_hand.pinchStrength());		// FYI
	msg.left_palm_pose.position.x = left_hand.palmPosition().x/1000;		// convert palm position into meters and copy to msg.left_palm_pos
	msg.left_palm_pose.position.y = left_hand.palmPosition().y/1000;
	msg.left_palm_pose.position.z = left_hand.palmPosition().z/1000;
	
	// Get hand's roll-pitch-yam and convert them into quaternion.
	// NOTE: Leap Motion roll-pith-yaw is from the perspective of human, so I am mapping it so that roll is about x-, pitch about y-, and yaw about z-axis.
	float l_yaw = left_hand.palmNormal().roll();
	float l_roll = left_hand.direction().pitch();
	float l_pitch = -left_hand.direction().yaw();			// Negating to comply with the right hand rule.
	printf("Lefty HERE! Roll: %.2f; Pitch: %.2f; Yaw: %.2f\n", l_roll, l_pitch, l_yaw);
	msg.left_palm_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(l_roll, l_pitch, l_yaw);

	msg.left_hand_sphere_radius = left_hand.sphereRadius();
	msg.left_hand_pinch_strength = left_hand.pinchStrength();
      }
      if (hands_in_frame[i].isRight()) {				// if Hand is right
	msg.right_hand = true;						// set msg.right_hand true
	right_hand = hands_in_frame[i];					// set this hand as right_hand
	printf("Righty HERE! Sphere radius: %.1f; Pinch strength: %f\n", right_hand.sphereRadius(), right_hand.pinchStrength());	// FYI
	msg.right_palm_pose.position.x = right_hand.palmPosition().x/1000;	// convert palm position into meters and copy to msg.right_palm_pos
	msg.right_palm_pose.position.y = right_hand.palmPosition().y/1000;
	msg.right_palm_pose.position.z = right_hand.palmPosition().z/1000;

	// Get hand's roll-pitch-yam and convert them into quaternion.
	// NOTE: Leap Motion roll-pith-yaw is from the perspective of human, so I am mapping it so that roll is about x-, pitch about y-, and yaw about z-axis.
	float r_yaw = right_hand.palmNormal().roll();
	float r_roll = right_hand.direction().pitch();
	float r_pitch = -right_hand.direction().yaw();			// Negating to comply with the right hand rule.
	printf("Righty HERE! Roll: %.2f; Pitch: %.2f; Yaw: %.2f\n", r_roll, r_pitch, r_yaw);
	msg.right_palm_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(r_roll, r_pitch, r_yaw);

	msg.right_hand_sphere_radius = right_hand.sphereRadius();
	msg.right_hand_pinch_strength = right_hand.pinchStrength();

      }
    } // for
  } // else if
  
  // Getting gestures
  Leap::GestureList gestures_in_frame = work_frame.gestures();
  msg.left_hand_key_tap = false;						// by default set KEY_TAP gestures to false on both hands
  msg.right_hand_key_tap = false;
  int left_count = 0;							// number of gestures with left hand
  int right_count = 0;							// number of gestures with right hand
  for (int j = 0; j < gestures_in_frame.count(); j++) {			// go through all the gestures in the list
    if (gestures_in_frame[j].hands()[0].isValid() && gestures_in_frame[j].hands()[0].isLeft()) {	// if the hand of j-th gesture is valid and left
      left_count++;							// increment number of gestures with left hand
      msg.left_hand_key_tap = true;					// report that there was a lefthand_key_tap
    } // end if
    if (gestures_in_frame[j].hands()[0].isValid() && gestures_in_frame[j].hands()[0].isRight()) {	// if the hand of j-th gesture is valid and right
      right_count++;							// increment number of gestures with right hand
      msg.right_hand_key_tap = true;					// report that there was a righthand_key_tap
    } // end if
  } // end for
   
  // Copy msg to global send_msg and set msg_ready to 1. main() is using this data to publish ROS messages
  send_msg = msg;
  msg_ready = 1;
 
} // end onFrame

/** Main method. */
int main(int argc, char** argv) {

  // ROS stuff
  ros::init(argc, argv, "leap_motion");
  ros::AsyncSpinner spinner(1);		// using async spinner
  spinner.start();			// starting async spinner
  ros::NodeHandle nh;			// ROS node handle

  // Creates publisher that advertises Leapmsg messages on rostopic /leapmotion_general
  ros::Publisher pub_leapmsg = nh.advertise<leap_motion_controller::LeapMotionOutput>("leapmotion_general", 10);

  // Create a listener and controller
  LeapListener listener;
  Leap::Controller controller;

  // Have the listener receive events from the controller
  controller.addListener(listener);

  // Keep this process running until ros is ok();
  while(ros::ok()) {
    if (msg_ready == 1) {		// if msg_ready flag has been set
      pub_leapmsg.publish(send_msg);	// publish the most recent Leapmsg
      msg_ready = 0;			// set msg_ready flag to zero
    } // if

  } // while

  // Remove the listener when done
  controller.removeListener(listener);

  return 0;
} // main