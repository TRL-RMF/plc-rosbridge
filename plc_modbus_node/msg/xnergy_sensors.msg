# Constant values for lift arm commands
uint16 toggle= 1
uint16 START_CHARGE= 1
uint16 STOP_CHARGE = 0

# variables
float32 xnergy_runtime_voltage 
float32 xnergy_runtime_current
uint16 rcu_temp
float32 batt_output_current
float32 battery_volt
uint16 error_code
bool toggle_state # true= toggle state on thus can modify charging state , false= toggle state is off
bool charge_state # true= charging, false=not charging