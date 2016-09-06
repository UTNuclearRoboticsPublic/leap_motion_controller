# For position, Leap Motion Controller uses '_optical' axis orientation, i.e.:
#    z forward
#    x right
#    y down

bool is_present					# TRUE if is a valid hand (typically left or right), FALSE otherwise.
geometry_msgs/PoseStamped palm_pose		# Contains the center position of the palm in meters from the Leap Motion Controller origin and the quaternion orientation of the palm.
geometry_msgs/Vector3Stamped palm_velocity	# The rate of change of the left palm position in meters per second. 
float32 sphere_radius				# The radius of a sphere fit to the curvature of this hand. This sphere is placed roughly as if the hand were holding a ball. Thus the size of the sphere decreases as the fingers are curled into a fist.
float32 pinch_strength				# The strength is zero for an open hand, and blends to 1.0 when a pinching hand pose is recognized.
bool key_tap					# TRUE if KEY_TAP gesture has been detected on left hand, FALSE otherwise.