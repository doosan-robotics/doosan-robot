int32       robot_state
string      robot_state_str
float64[6]  current_posj
float64[6]  current_posx
int8[16]    ctrlbox_digital_input
int8[16]    ctrlbox_digital_output
int8[6]     flange_digital_input
int8[6]     flange_digital_output
#####       io_modbus           # GJH
ModbusState[] modbus_state
#####       error               # GJH
int32 	    access_control
bool	    homming_completed
bool	    tp_initialized
float64[6]  joint_speed
float64[6]  task_speed
int8	    mastering_need
bool	    drl_stopped
bool	    disconnected