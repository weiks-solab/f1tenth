uint8 SPEED_UNKNOWN=0
uint8 SPEED_ODOM=1
uint8 SPEED_TF_ESTIMATE=2

uint8 STEERING_UNKNOWN=0
uint8 STEERING_MEASURED=1
uint8 STEERING_SERVO_ESTIMATE=2

# The enclosing RelativeState defines the frame and alignment time.
geometry_msgs/Pose2D pose
float64 speed_mps
float64 steering_angle_rad
builtin_interfaces/Time steering_stamp
bool steering_valid
uint8 steering_source
float64 s_m
float64 d_m
float64 e_psi_rad
float64 jacobian
builtin_interfaces/Time pose_stamp
builtin_interfaces/Time speed_stamp
bool valid
uint8 speed_source
