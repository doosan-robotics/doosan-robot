#include <ros/ros.h>
#include <signal.h>

#include "dsr_robot.h"
using namespace DSR_Robot;
using namespace std;

//----- set tartget robot----------------------------------------------------
string ROBOT_ID     = "dsr01";
string ROBOT_MODEL  = "m1013";

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void SigHandler(int sig)
{
    // Do some custom action.
    // For example, publish a stop message to some other nodes.
  
    // All the default sigint handler does is call shutdown()
    ROS_INFO("shutdown time!");
    ROS_INFO("shutdown time!");
    ROS_INFO("shutdown time!");
    //ros::ServiceClient srvMoveStop = nh.serviceClient<dsr_msgs::MoveStop>("/doosan_robot/motion/move_stop");

    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
    //ros::Publisher pubRobotStop = node->advertise<dsr_msgs::RobotStop>("/doosan_robot/stop",100);
    ros::Publisher pubRobotStop = node->advertise<dsr_msgs::RobotStop>("/"+ROBOT_ID  +ROBOT_MODEL +"/stop",100);
    
    dsr_msgs::RobotStop msg;
    
    msg.stop_mode  = STOP_TYPE_QUICK;
    pubRobotStop.publish(msg);
    
    ros::shutdown();
}

void msgRobotError_cb(const dsr_msgs::RobotError::ConstPtr& msg)
{

    ROS_INFO("________ ROBOT ERROR ________");
    ROS_INFO("  level       : %d", msg->level);
    ROS_INFO("  group       : %d", msg->group);
    ROS_INFO("  code        : %d", msg->code);
    ROS_INFO("  msg1        : %s", msg->msg1.c_str());
    ROS_INFO("  msg2        : %s", msg->msg2.c_str());
    ROS_INFO("  msg3        : %s", msg->msg3.c_str());
 
} 

void msgRobotState_cb(const dsr_msgs::RobotState::ConstPtr& msg)
{
    static int sn_cnt =0;
    
    sn_cnt++;
    if(0==(sn_cnt % 100))
    {  
        ROS_INFO("________ ROBOT STATUS ________");
        ROS_INFO("  robot_state       : %d", msg->robot_state);
        ROS_INFO("  robot_state_str   : %s", msg->robot_state_str.c_str());
        ROS_INFO("  current_posj      :  %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f",msg->current_posj[0] ,msg->current_posj[1] ,msg->current_posj[2]
                                                                             ,msg->current_posj[3] ,msg->current_posj[4] ,msg->current_posj[5] );
        ROS_INFO("  io_control_box    : %d", msg->io_control_box);
        //ROS_INFO("  io_modbus         : %d", msg->io_modbus);
        //ROS_INFO("  error             : %d", msg->error);
        ROS_INFO("  access_control    : %d", msg->access_control);
        ROS_INFO("  homming_completed : %d", msg->homming_completed);
        ROS_INFO("  tp_initialized    : %d", msg->tp_initialized);
        ROS_INFO("  speed             : %d", msg->speed);
        ROS_INFO("  mastering_need    : %d", msg->mastering_need);
        ROS_INFO("  drl_stopped       : %d", msg->drl_stopped);
        ROS_INFO("  disconnected      : %d", msg->disconnected);
    }
} 

static void thread_subscriber()
{
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
    ///ros::Subscriber subRobotState = node->subscribe("/doosan_robot/state", 100, msgRobotState_cb);
    ros::Subscriber subRobotError = node->subscribe("/"+ROBOT_ID  +ROBOT_MODEL +"/error", 100, msgRobotError_cb);
   
    ///ros::spin();
   
    ros::MultiThreadedSpinner spinner(2);
    spinner.spin();

    ROS_INFO("thread_subscriber() good-bye!!!");    
}

int main(int argc, char** argv)
{
    signal(SIGINT, SigHandler);

    ros::init(argc, argv, "dsr_error_motion_cpp", ros::init_options::NoSigintHandler);  
    ros::NodeHandle nh("~");

    ros::Publisher  pubRobotStop = nh.advertise<dsr_msgs::RobotStop>("/"+ROBOT_ID  +ROBOT_MODEL +"/stop",10);
    ros::Subscriber subRobotState = nh.subscribe("/"+ROBOT_ID  +ROBOT_MODEL +"/state", 100, msgRobotState_cb);
    ros::Subscriber subRobotError = nh.subscribe("/"+ROBOT_ID  +ROBOT_MODEL +"/error", 100, msgRobotError_cb);

    // run subscriber thread (for monitoring)
    boost::thread thread_sub(thread_subscriber);

    CDsrRobot robot(nh);

    float p1[6] = {5.30, 17.56, 95.91, 0.00, 66.53, 55.30};
    float p2[6] = {90.30, 17.56, 170.91, 0.00, 66.53, 55.30};

    robot.movej(p1, 30, 30);
    robot.movej(p2, 30, 30);

    ROS_INFO("dsr_error_motion 111");

    ros::Duration(3.0).sleep(); //sec

    ROS_INFO("dsr_error_motion 222");

    ///if(&robot) delete (&robot); 
    ///thread_sub.join();

    ROS_INFO("dsr_error_motion 333");

    ROS_INFO("dsr_error_motion finished !!!");
    return 0;
}
