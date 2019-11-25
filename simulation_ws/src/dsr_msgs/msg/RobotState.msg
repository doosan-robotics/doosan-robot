int32       robot_state             # 0 : STATE_INITIALIZING,   1 : STATE_STANDBY,   2 : STATE_MOVING,    3 : STATE_SAFE_OFF
                                    # 4 : STATE_TEACHING,       5 : STATE_SAFE_STOP, 6 : STATE_EMERGENCY_STOP,
                                    # 7 : STATE_EMERGENCY_STOP, 8 : STATE_HOMMING,   9 : STATE_RECOVERY,  10: STATE_SAFE_STOP2,     
                                    # 11: STATE_SAFE_OFF2,      12: STATE_RESERVED1, 13: STATE_RESERVED2, 14: STATE_RESERVED3,
                                    # 15: STATE_NOT_READY       16: STATE_LAST
string      robot_state_str         # Convert robot_state id to string
int8        actual_mode             # position control: 0, torque control: 1
int8        actual_space            # joint space: 0, task space: 1
float64[6]  current_posj            # current joint angle list [degree] 
float64[6]  current_velj            # current joint velocity list []
float64[6]  joint_abs               # Position Actual Value in ABS
float64[6]  joint_err               # Joint Error
float64[6]  target_posj             # target joint angle list [degree] 
float64[6]  target_velj             # target joint velocity list []
float64[6]  current_posx            # current task angle list []
float64[6]  current_velx            # current task velocity list []
float64[6]  task_err                # Task Error
float64[6]  target_velx             # target task velocity list []       
float64[6]  target_posx             # target task position list []
int8        solution_space          # Solution Space (0 ~ 7)
std_msgs/Float64MultiArray[] rotation_matrix  # Rotation Matrix
float64[6]  dynamic_tor             # dynamic torque
float64[6]  actual_jts              # joint torque sensor
float64[6]  actual_ejt              # external joint torque
float64[6]  actual_ett              # external tool torque
float64     sync_time               # internal clock counter   
int8[6]     actual_bk               # brake status 
int8[5]     actual_bt               # cockpit(robot button) info.
float64[6]  actual_mc               # motor current 
float64[6]  actual_mt               # motor temperature
int8[16]    ctrlbox_digital_input   # Digital Input in Control Box(0 ~ 15 ; 0 : ON, 1 : OFF)
int8[16]    ctrlbox_digital_output  # Digital Output in Control Box(0 ~ 15 ; 0 : ON, 1 : OFF)
int8[6]     flange_digital_input    # Digital Input in Flange(0 ~ 5 ; 0 : ON, 1 : OFF) x1 port : 0 ~ 2, x2 port : 3 ~ 5
int8[6]     flange_digital_output   # Digital Output in Flange(0 ~ 5 ; 0 : ON, 1 : OFF)
ModbusState[] modbus_state          # Custom msg for modbus state(refer to ModbusState.msg)
int32 	    access_control          # 
bool	    homming_completed       #
bool	    tp_initialized          #
int8	    mastering_need          #
bool	    drl_stopped             #
bool	    disconnected            #