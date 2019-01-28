#include <ros/ros.h>
#include <signal.h>
#include <boost/thread/thread.hpp>
#include <cstdlib>
#include <array>
#include <vector>
#include <algorithm>

#include <std_msgs/Float64.h>

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

#ifndef PI
#define PI 3.14159265359
#endif
#define deg2rad(deg)  ((deg) * PI / 180.0)
#define rad2deg(rad)  ((rad) * 180.0 / PI)

int movej(float fTargetPos[NUM_JOINT],
          float fTargetVel,
          float fTargetAcc,
          float fTargetTime = 0.f,
          int   nMoveMode = MOVE_MODE_ABSOLUTE,
          float fBlendingRadius = 0.f,
          int   nBlendingType = BLENDING_SPEED_TYPE_DUPLICATE)             
{
      
    //ros::ServiceClient srvMoveJoint = nh.serviceClient<dsr_msgs::MoveJoint>("/doosan_robot/motion/move_joint");
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
    ros::ServiceClient srvMoveJoint = node->serviceClient<dsr_msgs::MoveJoint>("/doosan_robot/motion/move_joint");

    dsr_msgs::MoveJoint srv;

    for(int i=0; i<NUM_JOINT; i++)
        srv.request.pos[i] = fTargetPos[i];
    srv.request.vel = fTargetVel;
    srv.request.acc = fTargetAcc;
    srv.request.time = fTargetTime;
    srv.request.mode = nMoveMode;
    srv.request.radius = fBlendingRadius;
    srv.request.blendType = nBlendingType; 

    ROS_INFO("service call: /doosan_robot/motion/move_joint");
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
    ros::ServiceClient srvMoveLine = node->serviceClient<dsr_msgs::MoveLine>("/doosan_robot/motion/move_line");
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

    ROS_INFO("service call: /doosan_robot/motion/move_line");
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

int movec(float fTargetPos[2][NUM_TASK],
          float fTargetVel[2],
          float fTargetAcc[2],
          float fTargetTime = 0.f,
          int   nMoveMode = MOVE_MODE_ABSOLUTE,
          int   nMoveReference = MOVE_REFERENCE_BASE,
          float fBlendingRadius = 0.f,
          int   nBlendingType = BLENDING_SPEED_TYPE_DUPLICATE)
{
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();  
    ros::ServiceClient srvMoveCircle = node->serviceClient<dsr_msgs::MoveCircle>("/doosan_robot/motion/move_circle");

    dsr_msgs::MoveCircle srv; 
    std::vector<std_msgs::Float64MultiArray> poses;
    std_msgs::Float64MultiArray pos;
    for(int i = 0; i < 2; i++){
        pos.data.clear();
        for(int j = 0; j < NUM_TASK; j++){
            pos.data.push_back(fTargetPos[i][j]);
        }
        poses.push_back(pos);
    }

    ROS_INFO("  <xxx pos1> %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f",fTargetPos[0][0],fTargetPos[0][1],fTargetPos[0][2],fTargetPos[0][3],fTargetPos[0][4],fTargetPos[0][5]);
    ROS_INFO("  <xxx pos2> %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f",fTargetPos[1][0],fTargetPos[1][1],fTargetPos[1][2],fTargetPos[1][3],fTargetPos[1][4],fTargetPos[1][5]);

    ROS_INFO("  <pos1> %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f",poses[0].data[0],poses[0].data[1],poses[0].data[2],poses[0].data[3],poses[0].data[4],poses[0].data[5]);
    ROS_INFO("  <pos2> %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f",poses[1].data[0],poses[1].data[1],poses[1].data[2],poses[1].data[3],poses[1].data[4],poses[1].data[5]);

    srv.request.pos = poses;
    srv.request.vel[0] = fTargetVel[0];
    srv.request.vel[1] = fTargetVel[1];
    srv.request.acc[0] = fTargetAcc[0];
    srv.request.acc[1] = fTargetAcc[1];
    srv.request.time = fTargetTime;
    srv.request.mode = nMoveMode;
    srv.request.ref  = nMoveReference;
    srv.request.radius = fBlendingRadius;
    srv.request.blendType = nBlendingType;

    ROS_INFO("service call: /doosan_robot/motion/move_circle");
    ROS_INFO("  <pos1> %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f",srv.request.pos[0].data[0],srv.request.pos[0].data[1],srv.request.pos[0].data[2],srv.request.pos[0].data[3],srv.request.pos[0].data[4],srv.request.pos[0].data[5]);
    ROS_INFO("  <pos2> %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f",srv.request.pos[1].data[0],srv.request.pos[1].data[1],srv.request.pos[1].data[2],srv.request.pos[1].data[3],srv.request.pos[1].data[4],srv.request.pos[1].data[5]);
    ROS_INFO("  <vel> %7.3f,%7.3f <acc> %7.3f,%7.3f <time> %7.3f",srv.request.vel[0],srv.request.vel[1],srv.request.acc[0],srv.request.acc[1], srv.request.time);
    ROS_INFO("  <mode> %d, <ref> %d, <radius> %7.3f, <blendType> %d",srv.request.mode,srv.request.ref, srv.request.radius, srv.request.blendType);

    if(srvMoveCircle.call(srv))
    {
        ROS_INFO("receive srv, srv.response.success: %ld\n", (long int)srv.response.success);
        return (srv.response.success);
    }
    else
    {
        ROS_ERROR("Failed to call service dr_control_service : move_circle\n");
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
    ros::Publisher pubRobotStop = node->advertise<dsr_msgs::RobotStop>("/doosan_robot/stop",100);
    
    dsr_msgs::RobotStop msg;
    
    msg.stop_mode  = STOP_TYPE_QUICK;
    pubRobotStop.publish(msg);
    
    ros::shutdown();
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

void thread_subscriber()
{
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
    ///ros::Subscriber subRobotState = node->subscribe("/doosan_robot/state", 100, msgRobotState_cb);
    ///ros::spin();
    ros::MultiThreadedSpinner spinner(2);
    spinner.spin();
}

int main(int argc, char** argv)
{
    int rate_sub = 1;    // 1 Hz = 1 sec
    int nPubRate = 100;  // 100 Hz (10ms)

    int i=0, nRes=0; 
    ros::init(argc, argv, "dsr_gazebo_cpp", ros::init_options::NoSigintHandler);  
    ros::NodeHandle nh("~");

    ros::Publisher  pubJoint1 = nh.advertise<std_msgs::Float64>("/dsr01/dsr_joint1_position/command",10);
    ros::Publisher  pubJoint2 = nh.advertise<std_msgs::Float64>("/dsr/dsr_joint2_position/command",10);
    ros::Publisher  pubJoint3 = nh.advertise<std_msgs::Float64>("/dsr01/dsr_joint3_position/command",10);
    ros::Publisher  pubJoint4 = nh.advertise<std_msgs::Float64>("/dsr01/joint4_position/command",10);
    ros::Publisher  pubJoint5 = nh.advertise<std_msgs::Float64>("/dsr/joint5_position/command",10);
    ros::Publisher  pubJoint6 = nh.advertise<std_msgs::Float64>("/dsr01/joint6_position/command",10);
   
    ///ros::Subscriber subRobotState = nh.subscribe("/doosan_robot/state", 100, msgRobotState_cb);
    
    // spawn another thread
    boost::thread thread_sub(thread_subscriber);

    signal(SIGINT, SigHandler);

    std_msgs::Float64 msg[6];   
    for(int i=0; i<6; i++) 
        msg[i].data = 0.0;


    ros::Rate loop_rate(1); //[hz]
    while(ros::ok())
    {
        msg[0].data = deg2rad(0.0);    
        msg[1].data = deg2rad(0.0);    
        msg[2].data = deg2rad(0.0);    
        msg[3].data = deg2rad(0.0);    
        msg[4].data = deg2rad(0.0);    
        msg[5].data = deg2rad(0.0);    

        pubJoint1.publish(msg[0]);
        pubJoint2.publish(msg[1]);
        pubJoint3.publish(msg[2]);
        pubJoint4.publish(msg[3]);
        pubJoint5.publish(msg[4]);
        pubJoint6.publish(msg[5]);
        loop_rate.sleep();

        msg[0].data = deg2rad(0.0);    
        msg[1].data = deg2rad(0.0);    
        msg[2].data = deg2rad(90.0);    
        msg[3].data = deg2rad(0.0);    
        msg[4].data = deg2rad(90.0);    
        msg[5].data = deg2rad(0.0);    

        pubJoint1.publish(msg[0]);
        pubJoint2.publish(msg[1]);
        pubJoint3.publish(msg[2]);
        pubJoint4.publish(msg[3]);
        pubJoint5.publish(msg[4]);
        pubJoint6.publish(msg[5]);
        loop_rate.sleep();

        msg[0].data = deg2rad(90.0);    
        msg[1].data = deg2rad(0.0);    
        msg[2].data = deg2rad(90.0);    
        msg[3].data = deg2rad(0.0);    
        msg[4].data = deg2rad(90.0);    
        msg[5].data = deg2rad(0.0);    

        pubJoint1.publish(msg[0]);
        pubJoint2.publish(msg[1]);
        pubJoint3.publish(msg[2]);
        pubJoint4.publish(msg[3]);
        pubJoint5.publish(msg[4]);
        pubJoint6.publish(msg[5]);
        loop_rate.sleep();

    }

    // wait the second thread to finish
    //thread_sub.join();
 
    ROS_INFO("dsr_service_test finished !!!!!!!!!!!!!!!!!!!!!");
    return 0;
}
