

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

int set_digital_output(int nGpioIndex,
                       bool bGpioValue)
{
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
    ros::ServiceClient srvSetCtrlBoxDigitalOutput = node->serviceClient<dsr_msgs::SetCtrlBoxDigitalOutput>("/"+ROBOT_ID +ROBOT_MODEL+"/io/set_digital_output");
    dsr_msgs::SetCtrlBoxDigitalOutput srv;

    srv.request.index = nGpioIndex;
    srv.request.value = bGpioValue;
    
    if(srvSetCtrlBoxDigitalOutput.call(srv))
    {         
        ROS_INFO("receive srv, srv.response.success: %ld\n", (long int)srv.response.success);
        return (srv.response.success);
    }
    else
    {    
        ROS_ERROR("Failed to call service dr_control_service : set_digital_output\n");
        ros::shutdown();  
        return -1;
    }
    return 0;
}

int get_digital_input(int nGpioIndex)
{
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
    ros::ServiceClient srvGetCtrlBoxDigitalInput = node->serviceClient<dsr_msgs::GetCtrlBoxDigitalInput>("/"+ROBOT_ID +ROBOT_MODEL+"/io/get_digital_input");
    dsr_msgs::GetCtrlBoxDigitalInput srv;

    srv.request.index = nGpioIndex;

    if(srvGetCtrlBoxDigitalInput.call(srv))
    {         
        ROS_INFO("receive srv, srv.response.success: %ld\n", (long int)srv.response.value);
        return (srv.response.value);
    }
    else
    {    
        ROS_ERROR("Failed to call service dr_control_service : get_digital_input\n");
        ros::shutdown();  
        return -1;
    }
    return 0;
}

int set_tool_digital_output(int nGpioIndex,
                            bool bGpioValue)
{
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
    ros::ServiceClient srvSetToolDigitalOutput = node->serviceClient<dsr_msgs::SetToolDigitalOutput>("/"+ROBOT_ID +ROBOT_MODEL+"/io/set_tool_digital_output");
    dsr_msgs::SetToolDigitalOutput srv;

    srv.request.index = nGpioIndex;
    srv.request.value = bGpioValue;

    if(srvSetToolDigitalOutput.call(srv))
    {         
        ROS_INFO("receive srv, srv.response.success: %ld\n", (long int)srv.response.success);
        return (srv.response.success);
    }
    else
    {    
        ROS_ERROR("Failed to call service dr_control_service : set_tool_digital_output\n");
        ros::shutdown();  
        return -1;
    }
    return 0;
}

int get_tool_digital_input(int nGpioIndex)
{
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
    ros::ServiceClient srvGetToolDigitalInput = node->serviceClient<dsr_msgs::GetToolDigitalInput>("/"+ROBOT_ID +ROBOT_MODEL+"/io/get_tool_digital_input");
    dsr_msgs::GetToolDigitalInput srv;

    srv.request.index = nGpioIndex;

    if(srvGetToolDigitalInput.call(srv))
    {         
        ROS_INFO("receive srv, srv.response.success: %ld\n", (long int)srv.response.value);
        return (srv.response.value);
    }
    else
    {    
        ROS_ERROR("Failed to call service dr_control_service : get_tool_digital_input\n");
        ros::shutdown();  
        return -1;
    }
    return 0;
}

int set_analog_output(int nGpioChannel,
                      float fGpioValue)
{
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
    ros::ServiceClient srvSetCtrlBoxAnalogOutput = node->serviceClient<dsr_msgs::SetCtrlBoxAnalogOutput>("/"+ROBOT_ID +ROBOT_MODEL+"/io/set_analog_output");
    dsr_msgs::SetCtrlBoxAnalogOutput srv;

    srv.request.channel = nGpioChannel;
    srv.request.value = fGpioValue;

    if(srvSetCtrlBoxAnalogOutput.call(srv))
    {         
        ROS_INFO("receive srv, srv.response.success: %ld\n", (long int)srv.response.success);
        return (srv.response.success);
    }
    else
    {    
        ROS_ERROR("Failed to call service dr_control_service : set_analog_output\n");
        ros::shutdown();  
        return -1;
    }
    return 0;
}

int get_analog_input(int nGpioChannel)
{
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
    ros::ServiceClient srvGetCtrlBoxAnalogInput = node->serviceClient<dsr_msgs::GetCtrlBoxDigitalInput>("/"+ROBOT_ID +ROBOT_MODEL+"/io/get_analog_input");
    dsr_msgs::GetCtrlBoxAnalogInput srv;

    srv.request.channel = nGpioChannel;

    if(srvGetCtrlBoxAnalogInput.call(srv))
    {         
        ROS_INFO("receive srv, srv.response.success: %ld\n", (long int)srv.response.value);
        return (srv.response.value);
    }
    else
    {    
        ROS_ERROR("Failed to call service dr_control_service : get_analog_input\n");
        ros::shutdown();  
        return -1;
    }
    return 0;
}

int set_analog_output_type(int nGpioChannel,
                           int nGpioMode)
{
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
    ros::ServiceClient srvSetCtrlBoxAnalogOutputType = node->serviceClient<dsr_msgs::SetCtrlBoxAnalogOutputType>("/"+ROBOT_ID +ROBOT_MODEL+"/io/set_analog_output_type");
    dsr_msgs::SetCtrlBoxAnalogOutputType srv;

    srv.request.channel = nGpioChannel;
    srv.request.mode = nGpioMode;

    if(srvSetCtrlBoxAnalogOutputType.call(srv))
    {         
        ROS_INFO("receive srv, srv.response.success: %ld\n", (long int)srv.response.success);
        return (srv.response.success);
    }
    else
    {    
        ROS_ERROR("Failed to call service dr_control_service : set_analog_output_type\n");
        ros::shutdown();  
        return -1;
    }
    return 0;
}

int set_analog_input_type(int nGpioChannel,
                           int nGpioMode)
{
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
    ros::ServiceClient srvSetCtrlBoxAnalogInputType = node->serviceClient<dsr_msgs::SetCtrlBoxAnalogInputType>("/"+ROBOT_ID +ROBOT_MODEL+"/io/set_analog_input_type");
    dsr_msgs::SetCtrlBoxAnalogInputType srv;

    srv.request.channel = nGpioChannel;
    srv.request.mode = nGpioMode;

    if(srvSetCtrlBoxAnalogInputType.call(srv))
    {         
        ROS_INFO("receive srv, srv.response.success: %ld\n", (long int)srv.response.success);
        return (srv.response.success);
    }
    else
    {    
        ROS_ERROR("Failed to call service dr_control_service : set_analog_input_type\n");
        ros::shutdown();  
        return -1;
    }
    return 0;
}

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
    ros::init(argc, argv, "dsr_service_io_basic_cpp", ros::init_options::NoSigintHandler);  
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
    set_digital_output(2, true);
    if(get_digital_input(2) == 1){
        set_digital_output(11, true);
    }
  
    while(ros::ok())
    {
    }

    ros::shutdown();
    // wait the second thread to finish
    //thread_sub.join();
    ROS_INFO("dsr_service_test finished !!!!!!!!!!!!!!!!!!!!!");
    return 0;
}
