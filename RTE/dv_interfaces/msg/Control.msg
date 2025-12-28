bool TORQUE_PERCENTAGE = 0
bool SPEED_KMH = 1

bool move_type

# SPEED_KMH mode:
#   - movement = target speed [m/s] (legacy)
# TORQUE_PERCENTAGE mode:
#   - movement is ignored
#   - torque_* are per-wheel torque commands in [%] of max_torque (same scaling as before)
float32 movement

# per-wheel torque commands (used when move_type==TORQUE_PERCENTAGE)
float32 torque_fl
float32 torque_fr
float32 torque_rl
float32 torque_rr

float32 steeringAngle_rad
uint8 serviceBrake
bool finished
