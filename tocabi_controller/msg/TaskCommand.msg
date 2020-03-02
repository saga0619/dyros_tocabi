# CoM/Pelvis Control
float64 ratio # left foot 0 ----- 1 right foot
float64 height
float64 angle

# Control Duration And Mode
float64 time
int32 mode

# Left/Right Arm Control
float64 l_x
float64 l_y
float64 l_z
float64 l_roll
float64 l_pitch
float64 l_yaw

float64 r_x
float64 r_y
float64 r_z
float64 r_roll
float64 r_pitch
float64 r_yaw

# Walking
uint32 walking_enable
uint32 pattern
uint32 ik_mode
bool dob
uint32 first_foot_step
float32 x
float32 y
float32 z
float32 walking_height
float32 theta
float32 step_length_x
float32 step_length_y

# Control Duration And Mode
float64 time
int32 mode
