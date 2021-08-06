#Constant value for estop command
int16 A_ESTOP = 1  # A --> ACTIVATE
int16 D_ESTOP = 0  # B --> DEACTIVATE

# Variables
bool heartbeat
bool estop_status

float32 twentyfour_volt_measure
float32 nineteen_volt_measure
float32 twelve_volt_measure