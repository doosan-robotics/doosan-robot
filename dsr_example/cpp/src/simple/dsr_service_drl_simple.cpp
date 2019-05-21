/*
 * [c++ example simple] DRL(Doosan Robot Language) test for doosan robot
 * Author: Jin Hyuk Gong (jinhyuk.gong@doosan.com)
 *
 * Copyright (c) 2019 Doosan Robotics
 * Use of this source code is governed by the BSD, see LICENSE
*/

#include <ros/ros.h>
#include <signal.h>
#include <sstream>
#include <std_msgs/String.h>

#include "dsr_util.h"
#include "dsr_robot.h"

using namespace std;
using namespace DSR_Robot;

std::string temp_data;
//----- set tartget robot----------------------------------------------------
string ROBOT_ID     = "dsr01";
string ROBOT_MODEL  = "m1013";
void SET_ROBOT(string id, string model) {ROBOT_ID = id; ROBOT_MODEL= model;}   
//---------------------------------------------------------------------------
CDsrRobot *temp;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void msgRobotState_cb(const dsr_msgs::RobotState::ConstPtr& msg)
{
    static int sn_cnt =0;
    
    sn_cnt++;
    if(0==(sn_cnt % 100))
    {  
        ROS_INFO("________ ROBOT STATUS ________");
        ROS_INFO("  robot_state           : %d", msg->robot_state);
        ROS_INFO("  robot_state_str       : %s", msg->robot_state_str.c_str());
        ROS_INFO("  current_posj          :  %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f",msg->current_posj[0] ,msg->current_posj[1] ,msg->current_posj[2]
                                                                             ,msg->current_posj[3] ,msg->current_posj[4] ,msg->current_posj[5] );
        ROS_INFO("  current_posx          :  %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f",msg->current_posx[0] ,msg->current_posx[1] ,msg->current_posx[2]
                                                                             ,msg->current_posx[3] ,msg->current_posx[4] ,msg->current_posx[5] );              
        temp_data = "";
        for(int i = 0; i < 16; i++){
            temp_data += (std::to_string(msg->ctrlbox_digital_input[i]) + " ");
        }
        ROS_INFO("  ctrlbox_digital_input : %s", temp_data.c_str());

        temp_data = "";
        for(int i = 0; i < 16; i++){
            temp_data += (std::to_string(msg->ctrlbox_digital_output[i]) + " ");
        }
        ROS_INFO("  ctrlbox_digital_output : %s", temp_data.c_str());

        temp_data = "";
        for(int i = 0; i < 6; i++){
            temp_data += (std::to_string(msg->flange_digital_input[i]) + " ");
        }
        ROS_INFO("  flange_digital_input : %s", temp_data.c_str());
        
        temp_data = "";
        for(int i = 0; i < 6; i++){
            temp_data += (std::to_string(msg->flange_digital_output[i]) + " ");
        }
        ROS_INFO("  flange_digital_output : %s", temp_data.c_str());

        temp_data = "";
        for(int i = 0; i < msg->modbus_state.size(); i++){
            temp_data += ("[" + msg->modbus_state[i].modbus_symbol + " , " + std::to_string(msg->modbus_state[i].modbus_value) + "] ");
        }
        ROS_INFO("  modbus_state : %s", temp_data.c_str());
        ROS_INFO("  access_control        : %d", msg->access_control);
        ROS_INFO("  homming_completed     : %d", msg->homming_completed);
        ROS_INFO("  tp_initialized        : %d", msg->tp_initialized);
        ROS_INFO("  joint_speed           :  %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f",msg->current_velj[0] ,msg->current_velj[1] ,msg->current_velj[2]
                                                                             ,msg->current_velj[3] ,msg->current_velj[4] ,msg->current_velj[5] );
        ROS_INFO("  task_speed            :  %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f",msg->current_velx[0] ,msg->current_velx[1] ,msg->current_velx[2]
                                                                             ,msg->current_velx[3] ,msg->current_velx[4] ,msg->current_velx[5] );
        ROS_INFO("  mastering_need        : %d", msg->mastering_need);
        ROS_INFO("  drl_stopped           : %d", msg->drl_stopped);
        ROS_INFO("  disconnected          : %d", msg->disconnected);
    }
} 

static void thread_subscriber()
{
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
    ///ros::Subscriber subRobotState = node->subscribe("/dsr01m1013/state", 100, msgRobotState_cb);
    ///ros::spin();
    ros::MultiThreadedSpinner spinner(2);
    spinner.spin();
}

void SigHandler(int sig)
{
    // Do some custom action.
    // For example, publish a stop message to some other nodes.
    temp->drl_stop();
    temp->set_robot_mode(ROBOT_MODE_MANUAL);
    ROS_INFO("DRL_STOP !!!!!!");
    // All the default sigint handler does is call shutdown()
    ROS_INFO("shutdown time!");
    ROS_INFO("shutdown time!");
    ROS_INFO("shutdown time!");
    //ros::ServiceClient srvMoveStop = nh.serviceClient<dsr_msgs::MoveStop>("/dsr01m1013/motion/move_stop");

    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
    ros::Publisher pubRobotStop = node->advertise<dsr_msgs::RobotStop>("/"+ROBOT_ID +ROBOT_MODEL+"/stop",100);
    
    dsr_msgs::RobotStop msg;
    
    msg.stop_mode  = STOP_TYPE_QUICK;
    pubRobotStop.publish(msg);
    
    ros::shutdown();
}

int main(int argc, char** argv)
{   
    //----- set target robot --------------- 
    string my_robot_id    = "dsr01";
    string my_robot_model = "m1013";
    if(1 == argc){
        ROS_INFO("default arguments: dsr01 m1013");
    }
    else{
        if(3 != argc){
            ROS_ERROR("invalid arguments: <ns> <model> (ex) dsr01 m1013");
            exit(1);
        }
        for (int i = 1; i < argc; i++){
            printf("argv[%d] = %s\n", i, argv[i]);
        }
        my_robot_id    = argv[1];
        my_robot_model = argv[2];
    }  
    //std::cout << "my_robot_id= " << my_robot_id << ", my_robot_model= " << my_robot_model << endl;
    SET_ROBOT(my_robot_id, my_robot_model);

    //----- init ROS ---------------------- 
    ros::init(argc, argv, "dsr_service_drl_cpp", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh("~");
    // Override the default ros sigint handler.
    // This must be set after the first NodeHandle is created.
    signal(SIGINT, SigHandler);
    ros::Publisher  pubRobotStop = nh.advertise<dsr_msgs::RobotStop>("/"+ROBOT_ID +ROBOT_MODEL+"/stop",10);
    ///ros::Subscriber subRobotState = nh.subscribe("/dsr01m1013/state", 100, msgRobotState_cb);
 
    //----- create DsrRobot --------------- 
    CDsrRobot robot(nh, my_robot_id, my_robot_model);

     // run subscriber thread (for monitoring)
    boost::thread thread_sub(thread_subscriber);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    std::string drlCodeMove = "set_velj(50)\nset_accj(50)\nmovej([0,0,90,0,90,0])\n";
    std::string drlCodeReset = "movej([0,0,0,0,0,0])\n";
    std::string mode;
    int _rate;
    robot.set_robot_mode(ROBOT_MODE_AUTONOMOUS);
    robot.drl_start(ROBOT_SYSTEM_REAL, drlCodeMove + drlCodeReset);    
    temp = &robot;
    while(ros::ok())
    {            
    }

    thread_sub.join();
    ROS_INFO("dsr_service_test finished !!!!!!!!!!!!!!!!!!!!!");
    return 0;
}
