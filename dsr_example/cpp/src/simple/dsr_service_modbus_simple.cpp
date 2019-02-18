#include <ros/ros.h>
#include <signal.h>
#include "dsr_robot.h"

using namespace DSR_Robot;
using namespace std;

//----- set tartget robot----------------------------------------------------
string ROBOT_ID     = "dsr01";
string ROBOT_MODEL  = "m1013";
void SET_ROBOT(string id, string model) {ROBOT_ID = id; ROBOT_MODEL= model;}   
//---------------------------------------------------------------------------

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void SigHandler(int sig)
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

static void thread_subscriber()
{
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
    ///ros::Subscriber subRobotState = node->subscribe("/dsr01m1013/state", 100, msgRobotState_cb);
    ///ros::spin();
    ros::MultiThreadedSpinner spinner(2);
    spinner.spin();
}

int main(int argc, char** argv)
{
    signal(SIGINT, SigHandler);

    ros::init(argc, argv, "dsr_service_test_cpp", ros::init_options::NoSigintHandler);  
    ros::NodeHandle nh("~");
    ros::Publisher  pubRobotStop = nh.advertise<dsr_msgs::RobotStop>("/"+ROBOT_ID +ROBOT_MODEL+"/stop",10);
    ///ros::Subscriber subRobotState = nh.subscribe("/dsr01m1013/state", 100, msgRobotState_cb);
 
    //----- set target robot --------------- 
    const string my_robot_id    = "dsr01";
    const string my_robot_model = "m1013";
    SET_ROBOT(my_robot_id, my_robot_model);
    CDsrRobot robot(nh, my_robot_id, my_robot_model);

 
    // run subscriber thread (for monitoring)
    boost::thread thread_sub(thread_subscriber);

    robot.config_create_modbus("di1", "192.168.137.50", 502, MODBUS_REGISTER_TYPE_DISCRETE_INPUTS, 3);
    robot.config_create_modbus("do1", "192.168.137.50", 502, MODBUS_REGISTER_TYPE_COILS, 3);
    robot.config_create_modbus("ro1", "192.168.137.50", 502, MODBUS_REGISTER_TYPE_HOLDING_REGISTER, 7);
    robot.config_create_modbus("ro2", "192.168.137.50", 502, MODBUS_REGISTER_TYPE_HOLDING_REGISTER, 6);

    while(ros::ok())
    {
        robot.set_modbus_output("ro2", 80);
        robot.set_modbus_output("do1", 1);
        if(robot.get_modbus_input("di1") == 1){
            robot.set_modbus_output("ro1", 30);
        }
    }
    thread_sub.join();
    // wait the second thread to finish
    ROS_INFO("dsr_service_test finished !!!!!!!!!!!!!!!!!!!!!");
    return 0;
}
