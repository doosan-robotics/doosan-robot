/*
 * [c++ gripper example] object pick and place for doosan robot
 * Author: Jin Hyuk Gong (jinhyuk.gong@doosan.com)
 *
 * Copyright (c) 2019 Doosan Robotics
 * Use of this source code is governed by the BSD, see LICENSE
*/

#include <ros/ros.h>
#include <signal.h>
#include <boost/thread/thread.hpp>

#include "dsr_util.h"
#include "dsr_robot.h"

using namespace std;
using namespace DSR_Robot;

//----- set tartget robot----------------------------------------------------
string ROBOT_ID     = "dsr01";
string ROBOT_MODEL  = "m1013";
void SET_ROBOT(string id, string model) {ROBOT_ID = id; ROBOT_MODEL= model;}   
//---------------------------------------------------------------------------

int gripper_move(float width)             
{
      
    //ros::ServiceClient srvMoveJoint = nh.serviceClient<dsr_msgs::MoveJoint>("/dsr01m1013/motion/move_joint");
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
    ros::ServiceClient srvRobotiq2FMove = node->serviceClient<dsr_msgs::Robotiq2FMove>("/"+ROBOT_ID + ROBOT_MODEL+"/gripper/robotiq_2f_move");
    dsr_msgs::Robotiq2FMove srv;
    srv.request.width = width;

    if(srvRobotiq2FMove.call(srv))
    {         
        //ROS_INFO("receive srv, srv.response.success: %ld\n", (long int)srv.response.success);
        return (srv.response.success);
    }
    else
    {    
        ROS_ERROR("Failed to call service dr_control_service : gripper_move\n");
        ros::shutdown();  
        return -1;
    }

    return 0; 
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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
    ros::init(argc, argv, "object_pick_and_place_simple_cpp", ros::init_options::NoSigintHandler);  
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
    float p0[6]={0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    float p1[6]={0.0, 0.0, 90.0, 0.0, 90.0 , 0.0};                             //joint
    float p2[6]={180.0, 0.0, 90, 0.0, 90.0, 0.0};   //joint
    
    float x1[6]={0, 0, -200, 0, 0, 0}; //task
    float x2[6]={0, 0, 200, 0, 0, 0}; //task
    float velx[2]={50, 50};     // 태스크 속도를 50(mm/sec), 50(deg/sec)로 설정
    float accx[2]={100, 100};   // 태스크 가속도를 100(mm/sec2), 100(deg/sec2)로 설정

    while(ros::ok())
    {
        robot.movej(p0, 60, 30);
        robot.movej(p1, 60, 30);
        
        robot.movel(x1, velx, accx, 2, 0.f, MOVE_REFERENCE_BASE, MOVE_MODE_RELATIVE);
        gripper_move(0.4); //0.0 - close;
        ros::Duration(1).sleep();
        robot.movel(x2, velx, accx, 2, 0.f, MOVE_REFERENCE_BASE, MOVE_MODE_RELATIVE);

        robot.movej(p2, 60, 30, 3);
        robot.movel(x1, velx, accx, 2, 0.f, MOVE_REFERENCE_BASE, MOVE_MODE_RELATIVE);
        gripper_move(0.0); //0.8 - open
        ros::Duration(1).sleep();
        robot.movel(x2, velx, accx, 2, 0.f, MOVE_REFERENCE_BASE, MOVE_MODE_RELATIVE);
    }

    ROS_INFO("object_pick_and_place_simple_cpp finished !!!!!!!!!!!!!!!!!!!!!");
    ROS_INFO("object_pick_and_place_simple_cpp finished !!!!!!!!!!!!!!!!!!!!!");
    ROS_INFO("object_pick_and_place_simple_cpp finished !!!!!!!!!!!!!!!!!!!!!");

    ///if(&robot) delete (&robot); 
    thread_sub.join();
    
    ROS_INFO("object_pick_and_place_simple_cpp finished !!!!!!!!!!!!!!!!!!!!!");
    return 0;
}
