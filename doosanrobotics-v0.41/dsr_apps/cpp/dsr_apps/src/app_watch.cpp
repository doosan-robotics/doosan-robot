#include <ros/ros.h>
#include <stdlib.h>
#include "dsr_robot.h"

//using namespace std;
//using namespace DSR_Robot;

#define DSR_CTL_PUB_RATE  100 // [hz] , 10ms     
#define DEFAULT_RATE    0.1   //[sec]  
#define MINIMUM_RATE    0.1   //[sec]  

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class CMonitor{
    public:
        void msgRobotStateCallback(const dsr_msgs::RobotState::ConstPtr& msg);  //every 10[ms] 
        void msgRobotErrorCallback(const dsr_msgs::RobotError::ConstPtr& msg);   
        CMonitor(std::string strRoobotName, double d);
    private:
        double m_dRate;
        int m_nCnt;
        int m_nLimit;
        std::string m_strRoobotName;
};
CMonitor::CMonitor(std::string strRoobotName, double d){ 
    m_strRoobotName = strRoobotName;
    m_dRate = d; 
    m_nCnt = 0;
}

void CMonitor::msgRobotStateCallback(const dsr_msgs::RobotState::ConstPtr& msg)
{
    // This function is called every 10 msec
    m_nCnt++;
    if( 0==( m_nCnt % ((int)(m_dRate * DSR_CTL_PUB_RATE)) ) )
    {  
        m_nCnt = 0;
        system("clear");
        ROS_INFO("________ ROBOT(%s) STATUS ________",m_strRoobotName.c_str());
        //ROS_INFO("  robot_state       : %d", msg->robot_state);
        ROS_INFO("  robot_state       : %s", msg->robot_state_str.c_str());
        ROS_INFO("  current_posj      :  %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f\r",msg->current_posj[0] ,msg->current_posj[1] ,msg->current_posj[2]
                                                                             ,msg->current_posj[3] ,msg->current_posj[4] ,msg->current_posj[5] );
        ROS_INFO("  io_control_box    : %d", msg->io_control_box);
        //ROS_INFO("  io_modbus         : %d\r", msg->io_modbus);l 
        //ROS_INFO("  error             : %d\r", msg->error);
        ROS_INFO("  access_control    : %d", msg->access_control);
        ROS_INFO("  homming_completed : %d", msg->homming_completed);
        ROS_INFO("  tp_initialized    : %d", msg->tp_initialized);
        ROS_INFO("  speed             : %d", msg->speed);
        ROS_INFO("  mastering_need    : %d", msg->mastering_need);
        ROS_INFO("  drl_stopped       : %d", msg->drl_stopped);
        ROS_INFO("  disconnected      : %d", msg->disconnected);
    }
} 

void CMonitor::msgRobotErrorCallback(const dsr_msgs::RobotError::ConstPtr& msg)
{
    // This function is called when an error occurs.    
    char strLevel[256]={0,};
    char strGroup[256]={0,};

    switch(msg->level){
        case 1: strcpy(strLevel,"INFO"); break; 
        case 2: strcpy(strLevel,"WARN"); break;
        case 3: strcpy(strLevel,"ERROR"); break;
        default: strcpy(strLevel,"NONE"); break;        
    }
    switch(msg->group){
        case 1: strcpy(strGroup,"SYSTEM"); break; 
        case 2: strcpy(strGroup,"MOTION"); break;
        case 3: strcpy(strGroup,"INVERTER"); break;
        case 4: strcpy(strGroup,"SAFETY_CONTROLLER"); break;        
        default: strcpy(strGroup,"NONE"); break;        
    }

    ROS_INFO("________ ROBOT ERROR ________");
    ROS_INFO("  level       : %s", strLevel);
    ROS_INFO("  group       : %s", strGroup);
    ROS_INFO("  code        : %d", msg->code);
    if(msg->msg1[0] != 0) ROS_INFO("  msg1        : %s", msg->msg1.c_str());
    if(msg->msg2[0] != 0) ROS_INFO("  msg2        : %s", msg->msg2.c_str());
    if(msg->msg3[0] != 0) ROS_INFO("  msg3        : %s", msg->msg3.c_str());

    ros::shutdown();
} 

int main(int argc, char* argv[])
{
    std::string MsgNamePrefix  = "/dsr01/m1013";

    double dRate = DEFAULT_RATE; //sec   
    if(argc==3) 
    {
        MsgNamePrefix = argv[1];
        ROS_INFO("MsgNamePrefix = %s",MsgNamePrefix.c_str());

        dRate = (double)atof(argv[2]);
        if(dRate <= MINIMUM_RATE) dRate = MINIMUM_RATE;  
    }
    else
    {
        ROS_INFO("[ERROR] invalid argments: <robot_id> <duraion>");
        ROS_INFO("<ex> rosrun dsr_app_cpp app_watch /dsr01m1013 1.0");
        return -1;
    }

    ros::init(argc, argv, "app_watch_cpp");  
    ros::NodeHandle nh("~");

    ROS_INFO("________ Monitor every [%.3f]sec ________",dRate);
    ros::Duration(0.5).sleep(); //[sec]
 
    CMonitor monitor(MsgNamePrefix, dRate);

    //ros::Subscriber subRobotState = nh.subscribe("/doosan_robot/state", 100, msgRobotState_cb);
    ros::Subscriber subRobotState  = nh.subscribe(MsgNamePrefix +"/state", 100, &CMonitor::msgRobotStateCallback, &monitor);
    ros::Subscriber subRobotError = nh.subscribe(MsgNamePrefix +"/error", 100, &CMonitor::msgRobotErrorCallback, &monitor);
 
    ros::spin();
  
    ROS_INFO("app_watch_cpp finished !!!");
    return 0;
}
