# In the ego body frame. Report the last command applied after arbitration.
std_msgs/Header header
float64 measured_angle_rad
float64 held_command_rad
# Two steering delay samples, newest first, in road-wheel radians.
float64[2] delay_buffer_rad
# Full 10 ms predictor ticks elapsed in the current 50 ms controller period (0..4).
uint8 phase_ticks
