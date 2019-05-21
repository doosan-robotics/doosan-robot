/*
 * [c++ example basic] DRL(Doosan Robot Language) test for doosan robot
 * Author: Jin Hyuk Gong (jinhyuk.gong@doosan.com)
 *
 * Copyright (c) 2019 Doosan Robotics
 * Use of this source code is governed by the BSD, see LICENSE
*/

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

int config_create_tcp(string strName, float fTargetPos[NUM_TASK])
{
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
    ros::ServiceClient srvConfigCreateTcp = node->serviceClient<dsr_msgs::ConfigCreateTcp>("/"+ROBOT_ID +ROBOT_MODEL+"/tcp/config_create_tcp");
    dsr_msgs::ConfigCreateTcp srv;

    srv.request.name = strName;
    for(int i=0; i<NUM_TASK; i++){
        srv.request.pos[i] = fTargetPos[i];
    }

    if(srvConfigCreateTcp.call(srv))
    {         
        //ROS_INFO("receive srv, srv.response.success: %ld\n", (long int)srv.response.success);
        return (srv.response.success);
    }
    else
    {    
        ROS_ERROR("Failed to call service dr_control_service : config_create_tcp\n");
        ros::shutdown();  
        return -1;
    }
    return 0;
}

int set_current_tcp(string strName)
{
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
    ros::ServiceClient srvSetCurrentTcp = node->serviceClient<dsr_msgs::SetCurrentTcp>("/"+ROBOT_ID +ROBOT_MODEL+"/tcp/set_current_tcp");
    dsr_msgs::SetCurrentTcp srv;

    srv.request.name = strName;

    if(srvSetCurrentTcp.call(srv))
    {         
        //ROS_INFO("receive srv, srv.response.success: %ld\n", (long int)srv.response.success);
        return (srv.response.success);
    }
    else
    {    
        ROS_ERROR("Failed to call service dr_control_service : set_current_tcp\n");
        ros::shutdown();  
        return -1;
    }
    return 0;
}

string get_current_tcp()
{
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
    ros::ServiceClient srvGetCurrentTcp = node->serviceClient<dsr_msgs::GetCurrentTcp>("/"+ROBOT_ID +ROBOT_MODEL+"/tcp/get_current_tcp");
    dsr_msgs::GetCurrentTcp srv;

    if(srvGetCurrentTcp.call(srv))
    {         
        //ROS_INFO("receive srv, srv.response.success: %ld\n", (long int)srv.response.success);
        return (srv.response.info);
    }
    else
    {    
        ROS_ERROR("Failed to call service dr_control_service : get_current_tcp\n");
        ros::shutdown();  
        return NULL;
    }
    return NULL;
}

int config_create_tool(string strName,
                       float fTargetWeight,
                       float fTargetCog[3],
                       float fTargetInertia[NUM_TASK])
{
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
    ros::ServiceClient srvConfigCreateTool = node->serviceClient<dsr_msgs::ConfigCreateTool>("/"+ROBOT_ID +ROBOT_MODEL+"/tool/config_create_tool");
    dsr_msgs::ConfigCreateTool srv;

    srv.request.name = strName;
    srv.request.weight = fTargetWeight;
    for(int i=0; i<3; i++){
        srv.request.cog[i] = fTargetCog[i];
    }
    for(int i=0; i<NUM_TASK; i++){
        srv.request.inertia[i] = fTargetInertia[i];
    }

    if(srvConfigCreateTool.call(srv))
    {         
        //ROS_INFO("receive srv, srv.response.success: %ld\n", (long int)srv.response.success);
        return (srv.response.success);
    }
    else
    {    
        ROS_ERROR("Failed to call service dr_control_service : config_create_tool\n");
        ros::shutdown();  
        return -1;
    }
    return 0;
}

int set_current_tool(string strName)
{
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
    ros::ServiceClient srvSetCurrentTool = node->serviceClient<dsr_msgs::SetCurrentTool>("/"+ROBOT_ID +ROBOT_MODEL+"/tool/set_current_tool");
    dsr_msgs::SetCurrentTool srv;

    srv.request.name = strName;
    
    if(srvSetCurrentTool.call(srv))
    {         
        //ROS_INFO("receive srv, srv.response.success: %ld\n", (long int)srv.response.success);
        return (srv.response.success);
    }
    else
    {    
        ROS_ERROR("Failed to call service dr_control_service : set_current_tool\n");
        ros::shutdown();  
        return -1;
    }
    return 0;
}

string get_current_tool()
{
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
    ros::ServiceClient srvGetCurrentTool = node->serviceClient<dsr_msgs::GetCurrentTool>("/"+ROBOT_ID +ROBOT_MODEL+"/tool/get_current_tool");
    dsr_msgs::GetCurrentTool srv;

    if(srvGetCurrentTool.call(srv))
    {         
        //ROS_INFO("receive srv, srv.response.success: %ld\n", (long int)srv.response.success);
        return (srv.response.info);
    }
    else
    {    
        ROS_ERROR("Failed to call service dr_control_service : get_current_tool\n");
        ros::shutdown();  
        return NULL;
    }
    return NULL;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void thread_subscriber()
{
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
    ros::MultiThreadedSpinner spinner(2);
    spinner.spin();
}

void SigHandler(int sig)
{
    // Do some custom action.
    // For example, publish a stop message to some other nodes.
  
    // All the default sigint handler does is call shutdown()
    ROS_INFO("shutdown time! sig=%d",sig);
    ROS_INFO("shutdown time! sig=%d",sig);
    ROS_INFO("shutdown time! sig=%d",sig);
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
    int rate_sub = 1;    // 1 Hz = 1 sec
    int nPubRate = 100;  // 100 Hz (10ms)
    int i=0, nRes=0; 
    ros::init(argc, argv, "dsr_service_drl_basic_cpp", ros::init_options::NoSigintHandler);  
    ros::NodeHandle nh("~");
    // Override the default ros sigint handler.
    // This must be set after the first NodeHandle is created.
    signal(SIGINT, SigHandler);
    ros::Publisher  pubRobotStop = nh.advertise<dsr_msgs::RobotStop>("/"+ROBOT_ID +ROBOT_MODEL+"/stop",10);

    // spawn another thread
    boost::thread thread_sub(thread_subscriber);


    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    float fCog[3] = {10.0, 10.0, 10.0};
    float fInertia[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    float p0[6] = {0, 0, 0, 0, 0, 0};
    config_create_tool("tool1", 5.3, fCog, fInertia);
    config_create_tcp("tcp1", p0);
    set_current_tool("tool1");
    set_current_tcp("tcp1");

    ROS_INFO("current tool : %s", get_current_tool().c_str());
    ROS_INFO("current tcp : %s", get_current_tcp().c_str());

    while(ros::ok())
    {
    }

    ros::shutdown();

    ROS_INFO("dsr_service_drl_basic_cpp finished !!!!!!!!!!!!!!!!!!!!!");
    return 0;
}
