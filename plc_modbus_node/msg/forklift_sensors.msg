# Constant values for lift arm commands
uint16 CMD_LIFT_UP=1
uint16 CMD_LIFT_DOWN=2
uint16 CMD_NO_LIFT=0
uint16 CMD_IR=1
uint16 CMD_NO_IR=0

# Variables
uint16 lift_cmd
uint16 ir_cmd
uint16 ir_dist_left
uint16 ir_dist_right 
float32 angle
bool mount_status
bool alignment
bool busy_status