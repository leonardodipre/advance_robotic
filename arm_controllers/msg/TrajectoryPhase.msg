float64 t_start         # Start time of the phase
float64 t_end           # End time of the phase
int32[] fixed_joints    # List of joint indices to fix
float64[] joint_angles  # List of angles for the fixed joints
bool use_dynamic_goal   # Flag to indicate if dynamic goal is used
float64[] goal_position # Goal position (x, y, z)