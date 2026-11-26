uint8 UNKNOWN=0
uint8 TRACKED=1
uint8 ABSENT=2
uint8 LOST=3

std_msgs/Header header
string state_id
string geometry_id
VehicleState ego
VehicleState opponent
float64 delta_s_local_m
float64 delta_s_unwrapped_m
float64 relative_speed_mps
uint8 opponent_status
string reason
# Values aligned to the ego pose stamp. Positive steering is left, SI units.
bool rollout_valid
float64 yaw_rate_radps
float64 slip_angle_rad
SteeringState steering
