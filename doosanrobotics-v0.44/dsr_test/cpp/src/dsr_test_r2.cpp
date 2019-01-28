#include <ros/ros.h>
#include <signal.h>
#include <boost/thread/thread.hpp>
#include <cstdlib>
#include <array>
#include <vector>
#include <algorithm>

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

#include "dsr_robot.h"

int movej(float fTargetPos[NUM_JOINT],
          float fTargetVel,
          float fTargetAcc,
          float fTargetTime = 0.f,
          int   nMoveMode = MOVE_MODE_ABSOLUTE,
          float fBlendingRadius = 0.f,
          int   nBlendingType = BLENDING_SPEED_TYPE_DUPLICATE)             
{
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
    ros::ServiceClient srvMoveJoint = node->serviceClient<dsr_msgs::MoveJoint>("/dsr01m1013/motion/move_joint");

    dsr_msgs::MoveJoint srv;

    for(int i=0; i<NUM_JOINT; i++)
        srv.request.pos[i] = fTargetPos[i];
    srv.request.vel = fTargetVel;
    srv.request.acc = fTargetAcc;
    srv.request.time = fTargetTime;
    srv.request.mode = nMoveMode;
    srv.request.radius = fBlendingRadius;
    srv.request.blendType = nBlendingType; 

    ROS_INFO("service call: /dsr01m1013/motion/move_joint");
    ROS_INFO("  <pos> %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f",srv.request.pos[0],srv.request.pos[1],srv.request.pos[2],srv.request.pos[3],srv.request.pos[4],srv.request.pos[5]);
    ROS_INFO("  <vel> %7.3f , <acc> %7.3f, <time> %7.3f",srv.request.vel, srv.request.acc, srv.request.time);
    ROS_INFO("  <mode> %d , <radius> %7.3f, <blendType> %d",srv.request.mode, srv.request.radius, srv.request.blendType);

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

int movej2(float fTargetPos[NUM_JOINT],
          float fTargetVel,
          float fTargetAcc,
          float fTargetTime = 0.f,
          int   nMoveMode = MOVE_MODE_ABSOLUTE,
          float fBlendingRadius = 0.f,
          int   nBlendingType = BLENDING_SPEED_TYPE_DUPLICATE)             
{
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
    ros::ServiceClient srvMoveJoint = node->serviceClient<dsr_msgs::MoveJoint>("/dsr02m0609/motion/move_joint");

    dsr_msgs::MoveJoint srv;

    for(int i=0; i<NUM_JOINT; i++)
        srv.request.pos[i] = fTargetPos[i];
    srv.request.vel = fTargetVel;
    srv.request.acc = fTargetAcc;
    srv.request.time = fTargetTime;
    srv.request.mode = nMoveMode;
    srv.request.radius = fBlendingRadius;
    srv.request.blendType = nBlendingType; 

    ROS_INFO("service call: /dsr02m0609/motion/move_joint");
    ROS_INFO("  <pos> %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f",srv.request.pos[0],srv.request.pos[1],srv.request.pos[2],srv.request.pos[3],srv.request.pos[4],srv.request.pos[5]);
    ROS_INFO("  <vel> %7.3f , <acc> %7.3f, <time> %7.3f",srv.request.vel, srv.request.acc, srv.request.time);
    ROS_INFO("  <mode> %d , <radius> %7.3f, <blendType> %d",srv.request.mode, srv.request.radius, srv.request.blendType);

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

int movel(float fTargetPos[NUM_JOINT],
          float fTargetVel[2],
          float fTargetAcc[2],
          float fTargetTime = 0.f,
          int   nMoveMode = MOVE_MODE_ABSOLUTE,
          int   nMoveReference = MOVE_REFERENCE_BASE,
          float fBlendingRadius = 0.f,
          int   nBlendingType = BLENDING_SPEED_TYPE_DUPLICATE)             
{
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();  
    ros::ServiceClient srvMoveLine = node->serviceClient<dsr_msgs::MoveLine>("/dsr01m1013/motion/move_line");
    dsr_msgs::MoveLine srv;

    for(int i=0; i<NUM_JOINT; i++)
        srv.request.pos[i] = fTargetPos[i];
    srv.request.vel[0] = fTargetVel[0];
    srv.request.vel[1] = fTargetVel[1];
    srv.request.acc[0] = fTargetAcc[0];
    srv.request.acc[1] = fTargetAcc[1];
    srv.request.time = fTargetTime;
    srv.request.mode = nMoveMode;
    srv.request.ref  = nMoveReference;
    srv.request.radius = fBlendingRadius;
    srv.request.blendType = nBlendingType; 

    ROS_INFO("service call: /dsr01m1013/motion/move_line");
    ROS_INFO("  <pos> %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f",srv.request.pos[0],srv.request.pos[1],srv.request.pos[2],srv.request.pos[3],srv.request.pos[4],srv.request.pos[5]);
    ROS_INFO("  <vel> %7.3f,%7.3f <acc> %7.3f,%7.3f <time> %7.3f",srv.request.vel[0],srv.request.vel[1],srv.request.acc[0],srv.request.acc[1], srv.request.time);
    ROS_INFO("  <mode> %d, <ref> %d, <radius> %7.3f, <blendType> %d",srv.request.mode,srv.request.ref, srv.request.radius, srv.request.blendType);

    if(srvMoveLine.call(srv))
    {         
        ROS_INFO("receive srv, srv.response.success: %ld\n", (long int)srv.response.success);
        return (srv.response.success);
    }
    else
    {     
        ROS_ERROR("Failed to call service dr_control_service : move_line\n");
        ros::shutdown();
        return -1;
    }

  return 0; 
}

int movel2(float fTargetPos[NUM_JOINT],
          float fTargetVel[2],
          float fTargetAcc[2],
          float fTargetTime = 0.f,
          int   nMoveMode = MOVE_MODE_ABSOLUTE,
          int   nMoveReference = MOVE_REFERENCE_BASE,
          float fBlendingRadius = 0.f,
          int   nBlendingType = BLENDING_SPEED_TYPE_DUPLICATE)             
{
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();  
    ros::ServiceClient srvMoveLine = node->serviceClient<dsr_msgs::MoveLine>("/dsr02m0609/motion/move_line");
    dsr_msgs::MoveLine srv;

    for(int i=0; i<NUM_JOINT; i++)
        srv.request.pos[i] = fTargetPos[i];
    srv.request.vel[0] = fTargetVel[0];
    srv.request.vel[1] = fTargetVel[1];
    srv.request.acc[0] = fTargetAcc[0];
    srv.request.acc[1] = fTargetAcc[1];
    srv.request.time = fTargetTime;
    srv.request.mode = nMoveMode;
    srv.request.ref  = nMoveReference;
    srv.request.radius = fBlendingRadius;
    srv.request.blendType = nBlendingType; 

    ROS_INFO("service call: /dsr02m0609/motion/move_line");
    ROS_INFO("  <pos> %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f",srv.request.pos[0],srv.request.pos[1],srv.request.pos[2],srv.request.pos[3],srv.request.pos[4],srv.request.pos[5]);
    ROS_INFO("  <vel> %7.3f,%7.3f <acc> %7.3f,%7.3f <time> %7.3f",srv.request.vel[0],srv.request.vel[1],srv.request.acc[0],srv.request.acc[1], srv.request.time);
    ROS_INFO("  <mode> %d, <ref> %d, <radius> %7.3f, <blendType> %d",srv.request.mode,srv.request.ref, srv.request.radius, srv.request.blendType);

    if(srvMoveLine.call(srv))
    {         
        ROS_INFO("receive srv, srv.response.success: %ld\n", (long int)srv.response.success);
        return (srv.response.success);
    }
    else
    {     
        ROS_ERROR("Failed to call service dr_control_service : move_line\n");
        ros::shutdown();
        return -1;
    }

  return 0; 
}

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
    //nh;

    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
    ros::Publisher pubRobotStop  = node->advertise<dsr_msgs::RobotStop>("/dsr01m1013/stop",100);
    ros::Publisher pubRobotStop2 = node->advertise<dsr_msgs::RobotStop>("/dsr02m0609/stop",100);
    
    dsr_msgs::RobotStop msg;
    
    msg.stop_mode  = STOP_TYPE_QUICK;
    pubRobotStop.publish(msg);
    pubRobotStop2.publish(msg);
    
    ros::shutdown();
}

void msgRobotState_cb(const dsr_msgs::RobotState::ConstPtr& msg)
{
    static int sn_cnt =0;
    
    sn_cnt++;
    if(0==(sn_cnt % 100))
    {  
        ROS_INFO("________ ROBOT[1] STATUS ________");
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
void msgRobotState_cb2(const dsr_msgs::RobotState::ConstPtr& msg)
{
    static int sn_cnt =0;
    
    sn_cnt++;
    if(0==(sn_cnt % 100))
    {  
        ROS_INFO("________ ROBOT[2] STATUS ________");
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

void thread_subscriber()
{
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
    ros::Subscriber subRobotState = node->subscribe("/dsr01m1013/state", 100, msgRobotState_cb);
    ///ros::spin();
    ros::MultiThreadedSpinner spinner(2);
    spinner.spin();
}
void thread_subscriber2()
{
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
    ros::Subscriber subRobotState = node->subscribe("/dsr02m0609/state", 100, msgRobotState_cb2);
    ///ros::spin();
    ros::MultiThreadedSpinner spinner(2);
    spinner.spin();
}

void thread_robot2()
{
    float p1[6]={0.0,};                             //joint
    float p2[6]={0.0, 0.0, 90.0, 0.0, 90.0, 0.0};   //joint
    
    float x1[6]={400, 500, 800.0, 0.0, 180.0, 0.0}; //task
    float x2[6]={400, 500, 500.0, 0.0, 180.0, 0.0}; //task
    float velx[2]={50, 50};     // 태스크 속도를 50(mm/sec), 50(deg/sec)로 설정
    float accx[2]={100, 100};   // 태스크 가속도를 100(mm/sec2), 100(deg/sec2)로 설정

    while(ros::ok())
    {
      movej2(p1, 30, 30); 
      movej2(p2, 100, 100);
      
      movel2(x1, velx, accx);
      movel2(x2, velx, accx);
    }
}

int main(int argc, char** argv)
{
    int rate_sub = 1;    // 1 Hz = 1 sec
    int nPubRate = 100;  // 100 Hz (10ms)

    int i=0, nRes=0; 
    ros::init(argc, argv, "dsr_test_r2_cpp", ros::init_options::NoSigintHandler);  
    ros::NodeHandle nh("~");

    ros::Publisher  pubRobotStop  = nh.advertise<dsr_msgs::RobotStop>("/dsr01m1013/stop",10);
    ros::Publisher  pubRobotStop2 = nh.advertise<dsr_msgs::RobotStop>("/dsr02m0609/stop",10);
 
    ///ros::Subscriber subRobotState = nh.subscribe("/doosan_robot/state", 100, msgRobotState_cb);
    
    ros::ServiceClient srvMoveJoint  = nh.serviceClient<dsr_msgs::MoveJoint>("/dsr01m1013/motion/move_joint");
    ros::ServiceClient srvMoveJoint2 = nh.serviceClient<dsr_msgs::MoveJoint>("/dsr02m0609/motion/move_joint");
 
    ros::ServiceClient srvMoveLine   = nh.serviceClient<dsr_msgs::MoveLine>("/dsr01m1013/motion/move_line");
    ros::ServiceClient srvMoveLine2  = nh.serviceClient<dsr_msgs::MoveLine>("/dsr02m0609/motion/move_line");
 
    // spawn another thread
    boost::thread thread_sub(thread_subscriber);
    boost::thread thread_sub2(thread_subscriber2);
    boost::thread thread_sub3(thread_robot2);

    signal(SIGINT, SigHandler);

    float p1[6]={0.0,};                             //joint
    float p2[6]={0.0, 0.0, 90.0, 0.0, 90.0, 0.0};   //joint
    
    float x1[6]={400, 500, 800.0, 0.0, 180.0, 0.0}; //task
    float x2[6]={400, 500, 500.0, 0.0, 180.0, 0.0}; //task
    float velx[2]={50, 50};     // 태스크 속도를 50(mm/sec), 50(deg/sec)로 설정
    float accx[2]={100, 100};   // 태스크 가속도를 100(mm/sec2), 100(deg/sec2)로 설정

    while(ros::ok())
    {
      movej(p1, 30, 30); 
      movej2(p1, 30, 30); 

      movej(p2, 100, 100);
      movej2(p2, 100, 100);
      
      movel(x1, velx, accx);
      movel2(x1, velx, accx);

      movel(x2, velx, accx);
      movel2(x2, velx, accx);
    }

    // wait the second thread to finish
    thread_sub.join();
 
    ROS_INFO("dsr_service_test finished !!!!!!!!!!!!!!!!!!!!!");
    return 0;
}
