uint8 STOP=0
uint8 CRUISE=1
uint8 FOLLOW=2
uint8 PASS=3
uint8 RETURN=4
uint8 PACE=5

std_msgs/Header header
string plan_id
string state_id
string geometry_id
builtin_interfaces/Time valid_until
uint8 candidate_id
uint8 mode
geometry_msgs/Point[] points
float64 target_speed_mps
string reason
