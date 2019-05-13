/*
 * [c++ demo] muliti robot sync test 
 * Author: Kab Kyoum Kim (kabkyoum.kim@doosan.com)
 *
 * Copyright (c) 2019 Doosan Robotics
 * Use of this source code is governed by the BSD, see LICENSE
*/

#include <ros/ros.h>
#include <signal.h>

#include "dsr_util.h"
#include "dsr_robot.h"

using namespace std;
using namespace DSR_Util;
using namespace DSR_Robot;

//----- set tartget robot----------------------------------------------------
#define NUM_ROBOT   6 
string ROBOT_ID     = "dsr01";
string ROBOT_MODEL  = "m1013";
string ROBOT_ID2    = "dsr02";
string ROBOT_MODEL2 = "m1013";

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


CRobotSync RobotSync(NUM_ROBOT);

float JReady[6] = {0, -20, 110, 0, 60, 0}; //posj   
float TCP_POS[6] = {0, 0, 0,0 ,0, 0};
float J00[6] = {-180, 0, -145, 0, -35, 0};
float J01r[6] = {-180.0, 71.4, -145.0, 0.0, -9.7, 0.0};

void thread_robot1(ros::NodeHandle nh)
{
    int nRobotID = 0;    
    CDsrRobot r1(nh,"dsr01","m1013");

    RobotSync.Wait(nRobotID);
    r1.movej(JReady, 20, 20);

    while(1)
    {
        RobotSync.Wait(nRobotID);
        r1.movej(J00, 20, 20);

        RobotSync.Wait(nRobotID);
        r1.movej(J01r, 20, 20);
    }
}

void thread_robot2(ros::NodeHandle nh)
{
    int nRobotID = 1;    
    CDsrRobot r2(nh,"dsr02","m1013");

    RobotSync.Wait(nRobotID);
    r2.movej(JReady, 20, 20);

    while(1)
    {
        RobotSync.Wait(nRobotID);
        r2.movej(J00, 20, 20);

        RobotSync.Wait(nRobotID);
        r2.movej(J01r, 20, 20);
    }

}

void thread_robot3(ros::NodeHandle nh)
{
    int nRobotID = 2;    
    while(1)
    {
        //ROS_INFO("thread_robot3 running...");
        RobotSync.Wait(nRobotID);
        ROS_INFO("R3 do somethind...");
        time_sleep(0.1);
        ///r.sleep();
    }
}

void thread_robot4(ros::NodeHandle nh)
{
    int nRobotID = 3;    
    while(1)
    {
        //ROS_INFO("thread_robot4 running...");
        RobotSync.Wait(nRobotID);
        ROS_INFO("R4 do somethind...");
        time_sleep(0.1);
        ///r.sleep();
    }
}

void thread_robot5(ros::NodeHandle nh)
{
    int nRobotID = 4;    
    while(1)
    {
        //ROS_INFO("thread_robot5 running...");
        RobotSync.Wait(nRobotID);
        ROS_INFO("R5 do somethind...");
        time_sleep(0.1);
        ///r.sleep();
    }
}

void thread_robot6(ros::NodeHandle nh)
{
    int nRobotID = 5;    
    while(1)
    {
        //ROS_INFO("thread_robot6 running...");
        RobotSync.Wait(nRobotID);
        ROS_INFO("R6 do somethind...");
        time_sleep(0.1);
    }
}

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
    ros::Publisher pubRobotStop2= node->advertise<dsr_msgs::RobotStop>("/"+ROBOT_ID2 +ROBOT_MODEL2+"/stop",100);
    
    dsr_msgs::RobotStop msg;
    
    msg.stop_mode  = STOP_TYPE_QUICK;

    pubRobotStop.publish(msg);
    pubRobotStop2.publish(msg);
    
    ros::shutdown();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "m1013x2_sync_cpp", ros::init_options::NoSigintHandler);  
    ros::NodeHandle nh("~");
    // Override the default ros sigint handler.
    // This must be set after the first NodeHandle is created.
    signal(SIGINT, SigHandler);

    ros::Publisher  pubRobotStop  = nh.advertise<dsr_msgs::RobotStop>("/"+ROBOT_ID  +ROBOT_MODEL +"/stop",10);
    ros::Publisher  pubRobotStop2 = nh.advertise<dsr_msgs::RobotStop>("/"+ROBOT_ID2 +ROBOT_MODEL2+"/stop",10);

    // spawn another thread
    boost::thread thread_r1(thread_robot1, nh);
    boost::thread thread_r2(thread_robot2, nh);
    boost::thread thread_r3(thread_robot3, nh);
    boost::thread thread_r4(thread_robot4, nh);
    boost::thread thread_r5(thread_robot5, nh);
    boost::thread thread_r6(thread_robot6, nh);

    while(1)
    {
        time_sleep(0.01);
        RobotSync.WakeUpAll();
    }

    ROS_INFO("m1013x2_sync2_cpp finished !!!!!!!!!!!!!!!!!!!!!");
    return 0;
}
