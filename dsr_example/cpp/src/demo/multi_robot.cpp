/*
 * [c++ demo]  muliti robot sync test 
 * Author: Kab Kyoum Kim (kabkyoum.kim@doosan.com)
 *
 * Copyright (c) 2019 Doosan Robotics
 * Use of this source code is governed by the BSD, see LICENSE
*/

#include <ros/ros.h>
#include <signal.h>
#include <time.h>
#include <chrono>

#include "dsr_util.h"
#include "dsr_robot.h"

using namespace std;
using namespace DSR_Util;
using namespace DSR_Robot;

//----- set tartget robot----------------------------------------------------
#define NUM_ROBOT   2 
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
float J02r[6] = {-180.0, 67.7, -144.0, 0.0, 76.3, 0.0};
float J03r[6] = {-180.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float J04r[6] = {-90.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float J04r1[6] = {-90.0, 30.0, -60.0, 0.0, 30.0, -0.0};
float J04r2[6] = {-90.0, -45.0, 90.0, 0.0, -45.0, -0.0};
float J04r3[6] = {-90.0, 60.0, -120.0, 0.0, 60.0, -0.0};
float J04r4[6] = {-90.0, 0.0, -0.0, 0.0, 0.0, -0.0};
float J05r[6] = {-144.0, -4.0, -84.8, -90.9, 54.0, -1.1};
float J06r[6];
float J07r[6] = {-152.4, 12.4, -78.6, 18.7, -68.3, -37.7};
float J08r[6] = {-90.0, 30.0, -120.0, -90.0, -90.0, 0.0};
float JEnd[6] = {0.0, -12.6, 101.1, 0.0, 91.5, -0.0};
float dREL1[6] = {0, 0, 350, 0, 0, 0};
float dREL2[6] = {0, 0, -350, 0, 0, 0};
float velx[2] = {0,0};
float accx[2] = {0, 0};
float vel_spi[2] = {400, 400};
float acc_spi[2] = {150, 150};
float J1[6] = {81.2, 20.8, 127.8, 162.5, 56.1, -37.1};
float X0[6] = {-88.7, 799.0, 182.3, 95.7, 93.7, 133.9};
float X1[6] = {304.2, 871.8, 141.5, 99.5, 84.9, 133.4};
float X2[6] = {437.1, 876.9, 362.1, 99.6, 84.0, 132.1};
float X3[6] = {-57.9, 782.4, 478.4, 99.6, 84.0, 132.1};
float amp[6] = {0, 0, 0, 30, 30, 0};
float period[6] = {0, 0, 0, 3, 6, 0};
float x01[6] = {423.6, 334.5, 651.2, 84.7, -180.0, 84.7};
float x02[6] = {423.6, 34.5, 951.2, 68.2, -180.0, 68.2};
float x03[6] = {423.6, -265.5, 651.2, 76.1, -180.0, 76.1};
float x04[6] = {423.6, 34.5, 351.2, 81.3, -180.0, 81.3};
float x0204c[2][6];
///////////////////

void thread_robot1(ros::NodeHandle nh)
{
    int nRobotID = 0;    
    CDsrRobot r(nh,"dsr01","m1013");

    while(ros::ok())
    {
        RobotSync.Wait(nRobotID);
        r.movej(JReady, 20, 20);

        RobotSync.Wait(nRobotID);
        r.config_create_tcp("TCP_ZERO", TCP_POS);

        RobotSync.Wait(nRobotID);
        r.set_current_tcp("TCP_ZERO");

        RobotSync.Wait(nRobotID);
        r.movej(J1, 20, 20, 0);

        RobotSync.Wait(nRobotID);
        r.movel(X3, velx, accx, 2.5);
        
        for(int i=0; i<2; i++){
            RobotSync.Wait(nRobotID);
            r.movel(X2, velx, accx, 0, MOVE_MODE_ABSOLUTE, MOVE_REFERENCE_BASE);

            RobotSync.Wait(nRobotID);
            r.movel(X1, velx, accx, 0, MOVE_MODE_ABSOLUTE, MOVE_REFERENCE_BASE);

            RobotSync.Wait(nRobotID);
            r.movel(X0, velx, accx, 0);

            RobotSync.Wait(nRobotID);
            r.movel(X1, velx, accx, 0, MOVE_MODE_ABSOLUTE, MOVE_REFERENCE_BASE);

            RobotSync.Wait(nRobotID);
            r.movel(X2, velx, accx, 0, MOVE_MODE_ABSOLUTE, MOVE_REFERENCE_BASE);

            RobotSync.Wait(nRobotID);
            r.movel(X3, velx, accx, 0, MOVE_MODE_ABSOLUTE, MOVE_REFERENCE_BASE);
        }

        RobotSync.Wait(nRobotID);
        r.movej(J00, 60, 60);

        RobotSync.Wait(nRobotID);
        r.movej(J01r, 30, 30, 0, MOVE_MODE_ABSOLUTE);

        RobotSync.Wait(nRobotID);
        r.movej(J02r, 30, 30, 0, MOVE_MODE_ABSOLUTE); 

        RobotSync.Wait(nRobotID);
        r.movej(J03r, 30, 30, 0);

        RobotSync.Wait(nRobotID);
        r.movej(J04r, 30, 30, 0);

        RobotSync.Wait(nRobotID);
        r.movej(J04r1, 30, 30, 0, MOVE_MODE_ABSOLUTE);

        RobotSync.Wait(nRobotID);
        r.movej(J04r2, 30, 30, 0, MOVE_MODE_ABSOLUTE);

        RobotSync.Wait(nRobotID);
        r.movej(J04r3, 30, 30, 0, MOVE_MODE_ABSOLUTE);

        RobotSync.Wait(nRobotID);
        r.movej(J04r4, 30, 30, 0);

        RobotSync.Wait(nRobotID);
        r.movej(J05r, 30, 30, 0, MOVE_MODE_ABSOLUTE); 

        RobotSync.Wait(nRobotID);
        r.movel(dREL1, velx, accx, 0, MOVE_MODE_ABSOLUTE, MOVE_REFERENCE_TOOL); 

        RobotSync.Wait(nRobotID);
        r.movel(dREL2, velx, accx, 0, MOVE_MODE_ABSOLUTE, MOVE_REFERENCE_TOOL); 

        RobotSync.Wait(nRobotID);
        r.movej(J07r, 60, 60, 0, MOVE_MODE_ABSOLUTE); 

        RobotSync.Wait(nRobotID);
        r.movej(J08r, 60, 60);

        RobotSync.Wait(nRobotID);
        r.movej(JEnd, 60, 60);

        RobotSync.Wait(nRobotID);
        r.move_periodic(amp, period, 0, 2, MOVE_REFERENCE_TOOL);

        RobotSync.Wait(nRobotID);
        r.move_spiral(3, 200, 100, vel_spi, acc_spi, 0, TASK_AXIS_X, MOVE_REFERENCE_TOOL);

        RobotSync.Wait(nRobotID);
        r.movel(x01, velx, accx);

        RobotSync.Wait(nRobotID);
        r.movel(x04, velx, accx, 0, MOVE_MODE_ABSOLUTE, MOVE_REFERENCE_BASE);

        RobotSync.Wait(nRobotID);
        r.movel(x03, velx, accx, 0, MOVE_MODE_ABSOLUTE, MOVE_REFERENCE_BASE);

        RobotSync.Wait(nRobotID);
        r.movel(x02, velx, accx, 0, MOVE_MODE_ABSOLUTE, MOVE_REFERENCE_BASE);

        RobotSync.Wait(nRobotID);
        r.movel(x01, velx, accx);

        for(int i=0; i<6; i++){
            x0204c[0][i] = x02[i];
            x0204c[1][i] = x04[i];
        }

        RobotSync.Wait(nRobotID);
        r.movec(x0204c, velx, accx, 0 , MOVE_MODE_ABSOLUTE, MOVE_REFERENCE_BASE);
    }
}

void thread_robot2(ros::NodeHandle nh)
{
    int nRobotID = 1;    
    CDsrRobot r(nh,"dsr02","m1013");

    while(ros::ok())
    {
        RobotSync.Wait(nRobotID);
        r.movej(JReady, 20, 20);

        RobotSync.Wait(nRobotID);
        r.config_create_tcp("TCP_ZERO", TCP_POS);

        RobotSync.Wait(nRobotID);
        r.set_current_tcp("TCP_ZERO");

        RobotSync.Wait(nRobotID);
        r.movej(J1, 20, 20, 0);

        RobotSync.Wait(nRobotID);
        r.movel(X3, velx, accx, 2.5);
        
        for(int i=0; i<2; i++){
            RobotSync.Wait(nRobotID);
            r.movel(X2, velx, accx, 0, MOVE_MODE_ABSOLUTE, MOVE_REFERENCE_BASE);

            RobotSync.Wait(nRobotID);
            r.movel(X1, velx, accx, 0, MOVE_MODE_ABSOLUTE, MOVE_REFERENCE_BASE);

            RobotSync.Wait(nRobotID);
            r.movel(X0, velx, accx, 0);

            RobotSync.Wait(nRobotID);
            r.movel(X1, velx, accx, 0, MOVE_MODE_ABSOLUTE, MOVE_REFERENCE_BASE);

            RobotSync.Wait(nRobotID);
            r.movel(X2, velx, accx, 0, MOVE_MODE_ABSOLUTE, MOVE_REFERENCE_BASE);

            RobotSync.Wait(nRobotID);
            r.movel(X3, velx, accx, 0, MOVE_MODE_ABSOLUTE, MOVE_REFERENCE_BASE);
        }

        RobotSync.Wait(nRobotID);
        r.movej(J00, 60, 60);

        RobotSync.Wait(nRobotID);
        r.movej(J01r, 30, 30, 0, MOVE_MODE_ABSOLUTE);

        RobotSync.Wait(nRobotID);
        r.movej(J02r, 30, 30, 0, MOVE_MODE_ABSOLUTE); 

        RobotSync.Wait(nRobotID);
        r.movej(J03r, 30, 30, 0);

        RobotSync.Wait(nRobotID);
        r.movej(J04r, 30, 30, 0);

        RobotSync.Wait(nRobotID);
        r.movej(J04r1, 30, 30, 0, MOVE_MODE_ABSOLUTE);

        RobotSync.Wait(nRobotID);
        r.movej(J04r2, 30, 30, 0, MOVE_MODE_ABSOLUTE);

        RobotSync.Wait(nRobotID);
        r.movej(J04r3, 30, 30, 0, MOVE_MODE_ABSOLUTE);

        RobotSync.Wait(nRobotID);
        r.movej(J04r4, 30, 30, 0);

        RobotSync.Wait(nRobotID);
        r.movej(J05r, 30, 30, 0, MOVE_MODE_ABSOLUTE); 

        RobotSync.Wait(nRobotID);
        r.movel(dREL1, velx, accx, 0, MOVE_MODE_ABSOLUTE, MOVE_REFERENCE_TOOL); 

        RobotSync.Wait(nRobotID);
        r.movel(dREL2, velx, accx, 0, MOVE_MODE_ABSOLUTE, MOVE_REFERENCE_TOOL); 

        RobotSync.Wait(nRobotID);
        r.movej(J07r, 60, 60, 0, MOVE_MODE_ABSOLUTE); 

        RobotSync.Wait(nRobotID);
        r.movej(J08r, 60, 60);

        RobotSync.Wait(nRobotID);
        r.movej(JEnd, 60, 60);

        RobotSync.Wait(nRobotID);
        r.move_periodic(amp, period, 0, 2, MOVE_REFERENCE_TOOL);

        RobotSync.Wait(nRobotID);
        r.move_spiral(3, 200, 100, vel_spi, acc_spi, 0, TASK_AXIS_X, MOVE_REFERENCE_TOOL);

        RobotSync.Wait(nRobotID);
        r.movel(x01, velx, accx);

        RobotSync.Wait(nRobotID);
        r.movel(x04, velx, accx, 0, MOVE_MODE_ABSOLUTE, MOVE_REFERENCE_BASE);

        RobotSync.Wait(nRobotID);
        r.movel(x03, velx, accx, 0, MOVE_MODE_ABSOLUTE, MOVE_REFERENCE_BASE);

        RobotSync.Wait(nRobotID);
        r.movel(x02, velx, accx, 0, MOVE_MODE_ABSOLUTE, MOVE_REFERENCE_BASE);

        RobotSync.Wait(nRobotID);
        r.movel(x01, velx, accx);

        for(int i=0; i<6; i++){
            x0204c[0][i] = x02[i];
            x0204c[1][i] = x04[i];
        }

        RobotSync.Wait(nRobotID);
        r.movec(x0204c, velx, accx, 0 , MOVE_MODE_ABSOLUTE, MOVE_REFERENCE_BASE);
    }
}

void thread_robot3(ros::NodeHandle nh)
{
    int nRobotID = 2;    
    while(ros::ok())
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
    while(ros::ok())
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
    while(ros::ok())
    {
        //ROS_INFO("thread_robot5 running...");
        RobotSync.Wait(nRobotID);
        //ROS_INFO("R5 do somethind...");
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
        //ROS_INFO("R6 do somethind...");
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
    
    ///delete(&RobotSync);
    ros::shutdown();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "multi_robot_cpp", ros::init_options::NoSigintHandler);  
    ros::NodeHandle nh("~");
    // Override the default ros sigint handler.
    // This must be set after the first NodeHandle is created.
    signal(SIGINT, SigHandler);

    ros::Publisher  pubRobotStop  = nh.advertise<dsr_msgs::RobotStop>("/"+ROBOT_ID  +ROBOT_MODEL +"/stop",10);
    ros::Publisher  pubRobotStop2 = nh.advertise<dsr_msgs::RobotStop>("/"+ROBOT_ID2 +ROBOT_MODEL2+"/stop",10);

    // spawn another thread
    boost::thread thread_r1(thread_robot1, nh);
    boost::thread thread_r2(thread_robot2, nh);
    //boost::thread thread_r3(thread_robot3, nh);
    //boost::thread thread_r4(thread_robot4, nh);
    //boost::thread thread_r5(thread_robot5, nh);
    //boost::thread thread_r6(thread_robot6, nh);

    time_sleep(1);

    double max_time = 0.0;
    double cur_time = 0.0;

    while(ros::ok())
    {
        time_sleep(0.01);
        RobotSync.WakeUpAll();      
    }

    ROS_INFO("multi_robot_cpp finished !!!!!!!!!!!!!!!!!!!!!");
    return 0;
}
