#include <ros/ros.h>
#include <signal.h>
#include "dsr_robot.h"

using namespace DSR_Robot;
using namespace std;

//----- set tartget robot----------------------------------------------------
string ROBOT_ID     = "dsr01";
string ROBOT_MODEL  = "m0617";
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
    //ros::ServiceClient srvMoveStop = nh.serviceClient<dsr_msgs::MoveStop>("/dsr01m0617/motion/move_stop");

    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
    ros::Publisher pubRobotStop = node->advertise<dsr_msgs::RobotStop>("/"+ROBOT_ID +ROBOT_MODEL+"/stop",100);
    
    dsr_msgs::RobotStop msg;
    
    msg.stop_mode  = STOP_TYPE_QUICK;
    pubRobotStop.publish(msg);
    
    ros::shutdown();
}

int main(int argc, char** argv)
{
    signal(SIGINT, SigHandler);

    ros::init(argc, argv, "dance_m0617_cpp", ros::init_options::NoSigintHandler);  
    ros::NodeHandle nh("~");
    ros::Publisher  pubRobotStop = nh.advertise<dsr_msgs::RobotStop>("/"+ROBOT_ID +ROBOT_MODEL+"/stop",10);

    //----- set target robot --------------- 
    const string my_robot_id    = "dsr01";
    const string my_robot_model = "m0617";
    SET_ROBOT(my_robot_id, my_robot_model);
    CDsrRobot robot(nh, my_robot_id, my_robot_model);

    /////////POS DEFINE
    
    float JReady[6] = {0, -20, 110, 0, 60, 0}; //posj
    
    float TCP_POS[6] = {0, 0, 0,0 ,0, 0};
    float J00[6] = {-180, 0, -145, 0, -35, 0};

    float J01r[6] = {-180.0, 71.4, -135.0, 0.0, -9.7, 0.0};
    float J02r[6] = {-180.0, 65.7, -135.2, 0.0, 69.5, 0.0};
    float J03r[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    float J04r[6] = {-90.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    float J05r[6] = {-144.0, -4.0, -84.8, -90.9, 54.0, -1.1};
    float J06r[6];
    float J07r[6] = {-152.4, 12.4, -78.6, 18.7, -68.3, -37.7};
    float J08r[6] = {-90.0, 30.0, -120.0, -90.0, -90.0, 0.0};

    float dREL1[6] = {0, 0, 450, 0, 0, 0};
    float dREL2[6] = {0, 0, -450, 0, 0, 0};

    float velx[2] = {0,0};
    float accx[2] = {0, 0};

    ///////////////////


    while(ros::ok())
    {
        robot.movej(JReady, 20, 20);
        robot.config_create_tcp("TCP_ZERO", TCP_POS);
        robot.set_current_tcp("TCP_ZERO");
        robot.movej(J00, 0, 0, 6);

        robot.movej(J01r, 0, 0, 2, MOVE_MODE_ABSOLUTE, 100);
        robot.movej(J02r, 0, 0, 2, MOVE_MODE_ABSOLUTE, 50); 
        robot.movej(J03r, 0, 0, 2);

        robot.movej(J04r, 0, 0, 1.5);
        robot.movej(J05r, 0, 0, 2.5, MOVE_MODE_ABSOLUTE, 100); 
        robot.movel(dREL1, velx, accx, 1, MOVE_MODE_ABSOLUTE, MOVE_REFERENCE_TOOL, 50); 
        robot.movel(dREL2, velx, accx, 1.5, MOVE_MODE_ABSOLUTE, MOVE_REFERENCE_TOOL, 100); 

        robot.movej(J07r, 0, 0, 1.5, MOVE_MODE_ABSOLUTE, 100); 
        robot.movej(J08r, 0, 0, 2);

        robot.movej(JReady, 20, 20, 5);


    }

    ROS_INFO("dance_m0617 finished !!!!!!!!!!!!!!!!!!!!!");
    return 0;
}
