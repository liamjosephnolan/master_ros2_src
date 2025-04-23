std_msgs/Header header

# Encoder positions (e.g. ticks)
int32 roll_encoder
int32 pitch_encoder
int32 insertion_encoder

# Commanded motor speeds (e.g. RPM)
float32 roll_motor_command_speed
float32 pitch_motor_command_speed
float32 insertion_motor_command_speed

# Actual joint speeds (e.g. RPM)
float32 roll_actual_speed
float32 pitch_actual_speed
float32 insertion_actual_speed

# Actual joint angles (e.g. degrees or radians)
float32 roll_angle
float32 pitch_angle
float32 insertion_angle