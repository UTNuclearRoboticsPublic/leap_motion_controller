# For position, Leap uses '_optical' axis orientation, i.e.:
#    z forward
#    x right
#    y down

std_msgs/Header header

bool left_hand					# TRUE if a left hand has been detected, FALSE otherwise.
geometry_msgs/Pose left_palm_pose		# Contains the center position of the palm in meters from the Leap Motion Controller origin and the quaternion orientation of the palm.
geometry_msgs/Vector3 left_palm_velocity	# The rate of change of the left palm position in meters per second. 
float32 left_hand_sphere_radius			# The radius of a sphere fit to the curvature of this hand. This sphere is placed roughly as if the hand were holding a ball. Thus the size of the sphere decreases as the fingers are curled into a fist.
float32 left_hand_pinch_strength		# The strength is zero for an open hand, and blends to 1.0 when a pinching hand pose is recognized.
bool left_hand_key_tap				# TRUE if KEY_TAP gesture has been detected on left hand, FALSE otherwise.

bool right_hand					# TRUE if a right hand has been detected, FALSE otherwise.	
geometry_msgs/Pose right_palm_pose		# Contains the center position of the palm in meters from the Leap Motion Controller origin and the quaternion orientation of the palm.
geometry_msgs/Vector3 right_palm_velocity	# The rate of change of the right palm position in meters per second. 
float32 right_hand_sphere_radius		# The radius of a sphere fit to the curvature of this hand. This sphere is placed roughly as if the hand were holding a ball. Thus the size of the sphere decreases as the fingers are curled into a fist.
float32 right_hand_pinch_strength		# The strength is zero for an open hand, and blends to 1.0 when a pinching hand pose is recognized.
bool right_hand_key_tap				# TRUE if KEY_TAP gesture has been detected on right hand, FALSE otherwise.