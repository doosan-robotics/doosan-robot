

#include <ros/ros.h>
#include <signal.h>
#include <boost/thread/thread.hpp>
#include <cstdlib>
#include <array>
#include <vector>
#include <algorithm>
#include <string>
#include <iostream>

#include <dsr_msgs/RobotState.h>
#include <dsr_msgs/RobotStop.h>

#include <dsr_msgs/MoveJoint.h>
#include <dsr_msgs/MoveLine.h>
#include <dsr_msgs/MoveJointx.h>
#include <dsr_msgs/MoveCircle.h>
#include <dsr_msgs/MoveSplineJoint.h>
#include <dsr_msgs/MoveSplineTask.h>
#include <dsr_msgs/MoveBlending.h>
#include <dsr_msgs/MoveSpiral.h>
#include <dsr_msgs/MovePeriodic.h>
#include <dsr_msgs/MoveWait.h>

#include <dsr_msgs/ConfigCreateTcp.h>
#include <dsr_msgs/ConfigDeleteTcp.h>
#include <dsr_msgs/GetCurrentTcp.h>
#include <dsr_msgs/SetCurrentTcp.h>

#include <dsr_msgs/SetCurrentTool.h>
#include <dsr_msgs/GetCurrentTool.h>
#include <dsr_msgs/ConfigCreateTool.h>
#include <dsr_msgs/ConfigDeleteTool.h>

#include <dsr_msgs/SetCtrlBoxDigitalOutput.h>
#include <dsr_msgs/GetCtrlBoxDigitalInput.h>
#include <dsr_msgs/SetToolDigitalOutput.h>
#include <dsr_msgs/GetToolDigitalInput.h>
#include <dsr_msgs/SetCtrlBoxAnalogOutput.h>
#include <dsr_msgs/GetCtrlBoxAnalogInput.h>
#include <dsr_msgs/SetCtrlBoxAnalogOutputType.h>
#include <dsr_msgs/SetCtrlBoxAnalogInputType.h>

#include <dsr_msgs/SetModbusOutput.h>
#include <dsr_msgs/GetModbusInput.h>
#include <dsr_msgs/ConfigCreateModbus.h>
#include <dsr_msgs/ConfigDeleteModbus.h>

#include <dsr_msgs/DrlPause.h>
#include <dsr_msgs/DrlStart.h>
#include <dsr_msgs/DrlStop.h>
#include <dsr_msgs/DrlResume.h>

#include "DRFL.h"
#include "DRFC.h"
#include "DRFS.h"

using namespace std;

//----- set tartget robot----------------------------------------------------
string ROBOT_ID     = "dsr01";
string ROBOT_MODEL  = "m1013";
void SET_ROBOT(string id, string model) {ROBOT_ID = id; ROBOT_MODEL= model;}   
//---------------------------------------------------------------------------

int movej(float fTargetPos[NUM_JOINT],
          float fTargetVel,
          float fTargetAcc,
          float fTargetTime = 0.f,
          int   nMoveMode = MOVE_MODE_ABSOLUTE,
          float fBlendingRadius = 0.f,
          int   nBlendingType = BLENDING_SPEED_TYPE_DUPLICATE,
          int nSyncType = 0)             
{
      
    //ros::ServiceClient srvMoveJoint = nh.serviceClient<dsr_msgs::MoveJoint>("/dsr01m1013/motion/move_joint");
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
    ros::ServiceClient srvMoveJoint = node->serviceClient<dsr_msgs::MoveJoint>("/"+ROBOT_ID +ROBOT_MODEL+"/motion/move_joint");

    dsr_msgs::MoveJoint srv;

    for(int i=0; i<NUM_JOINT; i++)
        srv.request.pos[i] = fTargetPos[i];
    srv.request.vel = fTargetVel;
    srv.request.acc = fTargetAcc;
    srv.request.time = fTargetTime;
    srv.request.mode = nMoveMode;
    srv.request.radius = fBlendingRadius;
    srv.request.blendType = nBlendingType; 
    srv.request.syncType = nSyncType;
    if(srvMoveJoint.call(srv))
    {         
        ROS_INFO("receive srv, srv.response.success: %ld\n", (long int)srv.response.success);
        return (srv.response.success);
    }
    else
    {    
        ROS_ERROR("Failed to call service dr_control_service : move_joint\n");
        ros::shutdown();  
        return -1;
    }

    return 0; 
}

int set_modbus_output(string strName,
                      int nValue)
{
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
    ros::ServiceClient srvSetModbusOutput = node->serviceClient<dsr_msgs::SetModbusOutput>("/"+ROBOT_ID +ROBOT_MODEL+"/modbus/set_modbus_output");
    dsr_msgs::SetModbusOutput srv;

    srv.request.name = strName;
    srv.request.value = nValue;

    if(srvSetModbusOutput.call(srv))
    {         
        ROS_INFO("receive srv, srv.response.success: %ld\n", (long int)srv.response.success);
        return (srv.response.success);
    }
    else
    {    
        ROS_ERROR("Failed to call service dr_control_service : set_modbus_output\n");
        ros::shutdown();  
        return -1;
    }
    return 0;
}

int get_modbus_input(string strName)
{
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
    ros::ServiceClient srvGetModbusInput = node->serviceClient<dsr_msgs::GetModbusInput>("/"+ROBOT_ID +ROBOT_MODEL+"/modbus/get_modbus_input");
    dsr_msgs::GetModbusInput srv;

    srv.request.name = strName;
    if(srvGetModbusInput.call(srv))
    {         
        ROS_INFO("receive srv, srv.response.value: %ld\n", (long int)srv.response.value);
        return (srv.response.value);
    }
    else
    {    
        ROS_ERROR("Failed to call service dr_control_service : get_modbus_input\n");
        ros::shutdown();  
        return -1;
    }
}

int config_create_modbus(string strName, 
                       string strIP, 
                       int nPort, 
                       int nRegType, 
                       int nRegIndex, 
                       int nRegValue = 0)
{
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
    ros::ServiceClient srvConfigCreateModbus = node->serviceClient<dsr_msgs::ConfigCreateModbus>("/"+ROBOT_ID +ROBOT_MODEL+"/modbus/config_create_modbus");
    dsr_msgs::ConfigCreateModbus srv;

    srv.request.name = strName;
    srv.request.ip = strIP;
    srv.request.port = nPort;
    srv.request.reg_type = nRegType;
    srv.request.index = nRegIndex;
    srv.request.value = nRegValue;

    if(srvConfigCreateModbus.call(srv))
    {         
        ROS_INFO("receive srv, srv.response.success: %ld\n", (long int)srv.response.success);
        return (srv.response.success);
    }
    else
    {    
        ROS_ERROR("Failed to call service dr_control_service : config_create_modbus\n");
        ros::shutdown();  
        return -1;
    }
    return 0;
}

int config_delete_modbus(string strName)
{
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
    ros::ServiceClient srvConfigDeleteModbus = node->serviceClient<dsr_msgs::ConfigDeleteModbus>("/"+ROBOT_ID +ROBOT_MODEL+"/modbus/config_delete_modbus");
    dsr_msgs::ConfigDeleteModbus srv;

    srv.request.name = strName;

    if(srvConfigDeleteModbus.call(srv))
    {         
        ROS_INFO("receive srv, srv.response.success: %ld\n", (long int)srv.response.success);
        return (srv.response.success);
    }
    else
    {    
        ROS_ERROR("Failed to call service dr_control_service : config_delete_modbus\n");
        ros::shutdown();  
        return -1;
    }
    return 0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void mySigintHandler(int sig)
{
    // Do some custom action.
    // For example, publish a stop message to some other nodes.
  
    // All the default sigint handler does is call shutdown()
    ROS_INFO("shutdown time!");
    ROS_INFO("shutdown time!");
    ROS_INFO("shutdown time!");
    //ros::ServiceClient srvMoveStop = nh.serviceClient<dsr_msgs::MoveStop>("/dsr01m1013/motion/move_stop");
    //nh;

    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
    ros::Publisher pubRobotStop = node->advertise<dsr_msgs::RobotStop>("/"+ROBOT_ID +ROBOT_MODEL+"/stop",100);
    
    dsr_msgs::RobotStop msg;
    
    msg.stop_mode  = STOP_TYPE_QUICK;
    pubRobotStop.publish(msg);
    
    ros::shutdown();
}


void thread_subscriber()
{
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
    ros::MultiThreadedSpinner spinner(2);
    spinner.spin();
}

int main(int argc, char** argv)
{
    //----- set target robot --------------- 
    const string my_robot_id    = "dsr01";
    const string my_robot_model = "m1013";
    SET_ROBOT(my_robot_id, my_robot_model);

    int rate_sub = 1;    // 1 Hz = 1 sec
    int nPubRate = 100;  // 100 Hz (10ms)

    int i=0, nRes=0; 
    ros::init(argc, argv, "dsr_service_modbus_basic_cpp", ros::init_options::NoSigintHandler);  
    ros::NodeHandle nh("~");

    ros::Publisher  pubRobotStop = nh.advertise<dsr_msgs::RobotStop>("/"+ROBOT_ID +ROBOT_MODEL+"/stop",10);

    // spawn another thread
    boost::thread thread_sub(thread_subscriber);

    signal(SIGINT, mySigintHandler);
    /* 
    if(argc !=3)
    {
        ROS_INFO("cmd : rosrun dsr_control dsr_control_service arg0 arg1");
        ROS_INFO("arg0: double number, arg1: double number");
        return 1;
    }
    */
    config_create_modbus("di1", "192.168.137.50", 502, MODBUS_REGISTER_TYPE_DISCRETE_INPUTS, 3);
    config_create_modbus("do1", "192.168.137.50", 502, MODBUS_REGISTER_TYPE_COILS, 3);
    config_create_modbus("ro1", "192.168.137.50", 502, MODBUS_REGISTER_TYPE_HOLDING_REGISTER, 7);
    config_create_modbus("ro2", "192.168.137.50", 502, MODBUS_REGISTER_TYPE_HOLDING_REGISTER, 6);
    
    while(ros::ok())
    {
        set_modbus_output("ro2", 80);
        set_modbus_output("do1", 1);
        ROS_INFO("do1 : %d", get_modbus_input("do1"));
        ROS_INFO("di1 : %d", get_modbus_input("di1"));
        if(get_modbus_input("di1") == 1){
            set_modbus_output("ro1", 30);
        }
    }

    ros::shutdown();
    // wait the second thread to finish
    ///thread_sub.join();
    ROS_INFO("dsr_service_test finished !!!!!!!!!!!!!!!!!!!!!");
    return 0;
}
