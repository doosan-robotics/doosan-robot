/*
 * [c++ example basic] single robot basic test
 * Author: Kab Kyoum Kim (kabkyoum.kim@doosan.com)
 *
 * Copyright (c) 2019 Doosan Robotics
 * Use of this source code is governed by the BSD, see LICENSE
*/

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

int movej(float fTargetPos[NUM_JOINT], float fTargetVel, float fTargetAcc, float fTargetTime = 0.f, float fBlendingRadius = 0.f,
          int nMoveMode = MOVE_MODE_ABSOLUTE, int nBlendingType = BLENDING_SPEED_TYPE_DUPLICATE, int nSyncType = 0)             
{     
    //ros::ServiceClient srvMoveJoint = nh.serviceClient<dsr_msgs::MoveJoint>("/dsr01m1013/motion/move_joint");
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
    ros::ServiceClient srvMoveJoint = node->serviceClient<dsr_msgs::MoveJoint>("/"+ROBOT_ID + ROBOT_MODEL+"/motion/move_joint");

    dsr_msgs::MoveJoint srv;

    for(int i=0; i<NUM_JOINT; i++)
        srv.request.pos[i] = fTargetPos[i];
    srv.request.vel = fTargetVel;
    srv.request.acc = fTargetAcc;
    srv.request.time = fTargetTime;
    srv.request.radius = fBlendingRadius;
    srv.request.mode = nMoveMode;
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
}
int amovej(float fTargetPos[NUM_JOINT], float fTargetVel, float fTargetAcc, float fTargetTime = 0.f, float fBlendingRadius = 0.f,
          int nMoveMode = MOVE_MODE_ABSOLUTE, int nBlendingType = BLENDING_SPEED_TYPE_DUPLICATE, int nSyncType = 1)             
{
    return movej(fTargetPos, fTargetVel, fTargetAcc, fTargetTime, fBlendingRadius, nMoveMode, nBlendingType, /*nSyncType=*/ 1);             
}

int movel(float fTargetPos[NUM_JOINT], float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, float fBlendingRadius = 0.f,
          int nMoveReference = MOVE_REFERENCE_BASE, int nMoveMode = MOVE_MODE_ABSOLUTE, int nBlendingType = BLENDING_SPEED_TYPE_DUPLICATE, int nSyncType = 0)             
{
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();  
    ros::ServiceClient srvMoveLine = node->serviceClient<dsr_msgs::MoveLine>("/"+ROBOT_ID +ROBOT_MODEL+"/motion/move_line");
    dsr_msgs::MoveLine srv;

    for(int i=0; i<NUM_JOINT; i++)
        srv.request.pos[i] = fTargetPos[i];
    for(int i=0; i<2; i++){
        srv.request.vel[i] = fTargetVel[i];
        srv.request.acc[i] = fTargetAcc[i];
    }
    srv.request.time = fTargetTime;
    srv.request.radius = fBlendingRadius;
    srv.request.ref  = nMoveReference;
    srv.request.mode = nMoveMode;
    srv.request.blendType = nBlendingType; 
    srv.request.syncType = nSyncType;

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
}
int amovel(float fTargetPos[NUM_JOINT], float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, float fBlendingRadius = 0.f,
          int nMoveReference = MOVE_REFERENCE_BASE, int nMoveMode = MOVE_MODE_ABSOLUTE, int nBlendingType = BLENDING_SPEED_TYPE_DUPLICATE, int nSyncType = 1)             
{
    return movel(fTargetPos, fTargetVel, fTargetAcc, fTargetTime, fBlendingRadius, nMoveReference, nMoveMode, nBlendingType, /*nSyncType=*/ 1);
}


int movejx(float fTargetPos[NUM_TASK], float fTargetVel, float fTargetAcc, float fTargetTime=0.f, float fBlendingRadius = 0.f,
           int nMoveReference = MOVE_REFERENCE_BASE, int nMoveMode = MOVE_MODE_ABSOLUTE, int nBlendingType = BLENDING_SPEED_TYPE_DUPLICATE, int nSolSpace = 0, int nSyncType = 0)
{
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();  
    ros::ServiceClient srvMoveJointx = node->serviceClient<dsr_msgs::MoveJointx>("/"+ROBOT_ID +ROBOT_MODEL+"/motion/move_jointx");
    dsr_msgs::MoveJointx srv; 

    for(int i=0; i<NUM_TASK; i++)
        srv.request.pos[i] = fTargetPos[i];
    srv.request.vel = fTargetVel;
    srv.request.acc = fTargetAcc;
    srv.request.time = fTargetTime;
    srv.request.radius = fBlendingRadius;
    srv.request.ref = nMoveReference;
    srv.request.mode = nMoveMode;
    srv.request.blendType = nBlendingType;
    srv.request.sol = nSolSpace;

    if(srvMoveJointx.call(srv))
    {         
        ROS_INFO("receive srv, srv.response.success: %ld\n", (long int)srv.response.success);
        return (srv.response.success);
    }
    else
    {     
        ROS_ERROR("Failed to call service dr_control_service : move_jointx\n");
        ros::shutdown();
        return -1;
    }
}
int amovejx(float fTargetPos[NUM_TASK], float fTargetVel, float fTargetAcc, float fTargetTime=0.f, float fBlendingRadius = 0.f,
           int nMoveReference = MOVE_REFERENCE_BASE, int nMoveMode = MOVE_MODE_ABSOLUTE, int nBlendingType = BLENDING_SPEED_TYPE_DUPLICATE, int nSolSpace = 0, int nSyncType = 1)
{
    return movejx(fTargetPos, fTargetVel, fTargetAcc, fTargetTime, fBlendingRadius, nMoveReference, nMoveMode, nBlendingType, nSolSpace, /*nSyncType=*/ 1);
}


int movec(float fTargetPos[2][NUM_TASK], float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, float fBlendingRadius = 0.f,
          int nMoveReference = MOVE_REFERENCE_BASE, int nMoveMode = MOVE_MODE_ABSOLUTE, int nBlendingType = BLENDING_SPEED_TYPE_DUPLICATE, int nSyncType = 0)
{
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();  
    ros::ServiceClient srvMoveCircle = node->serviceClient<dsr_msgs::MoveCircle>("/"+ROBOT_ID +ROBOT_MODEL+"/motion/move_circle");

    dsr_msgs::MoveCircle srv; 
    std::vector<std_msgs::Float64MultiArray> poses;
    std_msgs::Float64MultiArray pos;
    for(int i=0; i<2; i++){
        pos.data.clear();
        for(int j = 0; j < NUM_TASK; j++){
            pos.data.push_back(fTargetPos[i][j]);
        }
        poses.push_back(pos);
    }
    srv.request.pos = poses;

    for(int i=0; i<2; i++){
        srv.request.vel[i] = fTargetVel[i];
        srv.request.acc[i] = fTargetAcc[i];
    }
    srv.request.time = fTargetTime;
    srv.request.radius = fBlendingRadius;
    srv.request.ref  = nMoveReference;
    srv.request.mode = nMoveMode;
    srv.request.blendType = nBlendingType;
    srv.request.syncType = nSyncType;

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
}
int amovec(float fTargetPos[2][NUM_TASK], float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, float fBlendingRadius = 0.f,
          int nMoveReference = MOVE_REFERENCE_BASE, int nMoveMode = MOVE_MODE_ABSOLUTE, int nBlendingType = BLENDING_SPEED_TYPE_DUPLICATE, int nSyncType = 1)
{
    return movec(fTargetPos, fTargetVel, fTargetAcc, fTargetTime, fBlendingRadius, nMoveReference, nMoveMode, nBlendingType, /*nSyncType=*/ 1);
}


int movesj(float fTargetPos[MAX_SPLINE_POINT][NUM_JOINT], int nPosCount, float fTargetVel, float fTargetAcc, float fTargetTime = 0.f,
           int nMoveMode = MOVE_MODE_ABSOLUTE, int nSyncType = 0)
{
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();  
    ros::ServiceClient srvMoveSplineJoint = node->serviceClient<dsr_msgs::MoveSplineJoint>("/"+ROBOT_ID +ROBOT_MODEL+"/motion/move_spline_joint");
    dsr_msgs::MoveSplineJoint srv;
    std::vector<std_msgs::Float64MultiArray> poses;
    std_msgs::Float64MultiArray pos;

    for(int i = 0; i < MAX_SPLINE_POINT; i++){
        pos.data.clear();
        for(int j = 0; j < NUM_JOINT; j++){
            pos.data.push_back(fTargetPos[i][j]);
        }
        poses.push_back(pos);
    }
    srv.request.pos = poses;
    srv.request.posCnt = nPosCount;

    srv.request.vel = fTargetVel;
    srv.request.acc = fTargetAcc;
    srv.request.time = fTargetTime;
    srv.request.mode = nMoveMode;
    srv.request.syncType = nSyncType;

    if(srvMoveSplineJoint.call(srv))
    {
        ROS_INFO("receive srv, srv.response.success: %ld\n", (long int)srv.response.success);
        return (srv.response.success);
    }
    else
    {
        ROS_ERROR("Failed to call service dr_control_service : move_spline_joint\n");
        ros::shutdown();
        return -1;
    }
}
int amovesj(float fTargetPos[MAX_SPLINE_POINT][NUM_JOINT], int nPosCount, float fTargetVel, float fTargetAcc, float fTargetTime = 0.f,
           int nMoveMode = MOVE_MODE_ABSOLUTE, int nSyncType = 1)
{
    return movesj(fTargetPos, nPosCount, fTargetVel, fTargetAcc, fTargetTime, nMoveMode, /*nSyncType=*/ 1);
}


int movesx(float fTargetPos[MAX_SPLINE_POINT][NUM_TASK], int nPosCount, float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f,
           int nMoveReference = MOVE_REFERENCE_BASE, int nMoveMode = MOVE_MODE_ABSOLUTE, int nVelOpt = SPLINE_VELOCITY_OPTION_DEFAULT, int nSyncType = 0)
{
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();  
    ros::ServiceClient srvMoveSplineTask = node->serviceClient<dsr_msgs::MoveSplineTask>("/"+ROBOT_ID +ROBOT_MODEL+"/motion/move_spline_task");
    dsr_msgs::MoveSplineTask srv;
    std::vector<std_msgs::Float64MultiArray> poses;
    std_msgs::Float64MultiArray pos;

    for(int i = 0; i < MAX_SPLINE_POINT; i++){
        pos.data.clear();
        for(int j = 0; j < NUM_TASK; j++){
            pos.data.push_back(fTargetPos[i][j]);
        }
        poses.push_back(pos);
    }
    srv.request.pos = poses;
    srv.request.posCnt = nPosCount;
 
    for(int i=0; i<2; i++){
        srv.request.vel[i] = fTargetVel[i];
        srv.request.acc[i] = fTargetAcc[i];
    }
    srv.request.time = fTargetTime;
    srv.request.ref = nMoveReference;
    srv.request.mode = nMoveMode;
    srv.request.opt = nVelOpt;
    srv.request.syncType = nSyncType;

    if(srvMoveSplineTask.call(srv))
    {
        ROS_INFO("receive srv, srv.response.success: %ld\n", (long int)srv.response.success);
        return (srv.response.success);
    }
    else
    {
        ROS_ERROR("Failed to call service dr_control_service : move_spline_task\n");
        ros::shutdown();
        return -1;
    }
}
int amovesx(float fTargetPos[MAX_SPLINE_POINT][NUM_TASK], int nPosCount, float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f,
           int nMoveReference = MOVE_REFERENCE_BASE, int nMoveMode = MOVE_MODE_ABSOLUTE, int nVelOpt = SPLINE_VELOCITY_OPTION_DEFAULT, int nSyncType = 1)
{
    return movesx(fTargetPos, nPosCount, fTargetVel,  fTargetAcc, fTargetTime, nMoveReference, nMoveMode, nVelOpt, /*nSyncType=*/ 1);
}


int moveb(MOVE_POSB* fTargetPos, int nPosCount, float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f,
          int nMoveReference = MOVE_REFERENCE_BASE, int nMoveMode = MOVE_MODE_ABSOLUTE, int nSyncType = 0)
{
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();  
    ros::ServiceClient srvMoveBlending = node->serviceClient<dsr_msgs::MoveBlending>("/"+ROBOT_ID +ROBOT_MODEL+"/motion/move_blending");
    dsr_msgs::MoveBlending srv;

    std::vector<std_msgs::Float64MultiArray> segments;
    std_msgs::Float64MultiArray segment;

    for(int i=0; i<nPosCount; i++){
        segment.data.clear();

        for(int j=0; j<NUM_TASK; j++)
            segment.data.push_back( fTargetPos[i]._fTargetPos[0][j]);
        for(int j=0; j<NUM_TASK; j++)
            segment.data.push_back( fTargetPos[i]._fTargetPos[1][j]);

        segment.data.push_back( fTargetPos[i]._iBlendType );
        segment.data.push_back( fTargetPos[i]._fBlendRad  );  

        segments.push_back(segment);
    }
    srv.request.segment = segments;
    srv.request.posCnt = nPosCount;    

    for(int i=0; i<2; i++){
        srv.request.vel[i] = fTargetVel[i];
        srv.request.acc[i] = fTargetAcc[i];
    }
    srv.request.time = fTargetTime;    
    srv.request.ref = nMoveReference;
    srv.request.mode = nMoveMode;
    srv.request.syncType = nSyncType;

    if(srvMoveBlending.call(srv))
    {
        ROS_INFO("receive srv, srv.response.success: %ld\n", (long int)srv.response.success);
        return (srv.response.success);
    }
    else
    {
        ROS_ERROR("Failed to call service dr_control_service : move_spline_blending\n");
        ros::shutdown();
        return -1;
    }

    return 0; 
}
int amoveb(MOVE_POSB* fTargetPos, int nPosCount, float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f,
          int nMoveReference = MOVE_REFERENCE_BASE, int nMoveMode = MOVE_MODE_ABSOLUTE, int nSyncType = 1)
{
    return moveb(fTargetPos, nPosCount, fTargetVel, fTargetAcc, fTargetTime, nMoveReference, nMoveMode, /*nSyncType=*/ 1);
}


int move_spiral(float fRevolution, float fMaxRadius, float fMaxLength, float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f,
               int nTaskAxis = TASK_AXIS_Z, int nMoveReference = MOVE_REFERENCE_TOOL, int nSyncType = 0)
{
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();  
    ros::ServiceClient srvMoveSpiral = node->serviceClient<dsr_msgs::MoveSpiral>("/"+ROBOT_ID +ROBOT_MODEL+"/motion/move_spiral");
    dsr_msgs::MoveSpiral srv;

    srv.request.revolution = fRevolution;
    srv.request.maxRadius = fMaxRadius;
    srv.request.maxLength = fMaxLength;
    for(int i=0; i<2; i++){
        srv.request.vel[i] = fTargetVel[i];
        srv.request.acc[i] = fTargetAcc[i];
    }
    srv.request.time = fTargetTime;
    srv.request.taskAxis = nTaskAxis;
    srv.request.ref = nMoveReference;      
    srv.request.syncType = nSyncType;

    if(srvMoveSpiral.call(srv)){
        ROS_INFO("receive srv, srv.response.success: %ld\n", (long int)srv.response.success);
        return (srv.response.success);
    }
    else{
        ROS_ERROR("Failed to call service dr_control_service : move_spiral\n");
        ros::shutdown();
        return -1;
    } 
}
int amove_spiral(float fRevolution, float fMaxRadius, float fMaxLength, float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f,
               int nTaskAxis = TASK_AXIS_Z, int nMoveReference = MOVE_REFERENCE_TOOL, int nSyncType = 1)
{
    return move_spiral(fRevolution, fMaxRadius, fMaxLength, fTargetVel, fTargetAcc, fTargetTime, nTaskAxis, nMoveReference, /*nSyncType=*/ 1);
}


int move_periodic(float fAmplitude[NUM_TASK], float fPeriodic[NUM_TASK], float fAccelTime = 0.f, int nRepeat = 1, int nMoveReference = MOVE_REFERENCE_TOOL, int nSyncType = 0)
{
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();  
    ros::ServiceClient srvMovePeriodic = node->serviceClient<dsr_msgs::MovePeriodic>("/"+ROBOT_ID +ROBOT_MODEL+"/motion/move_periodic");
    dsr_msgs::MovePeriodic srv;

    for(int i=0; i<NUM_TASK; i++){
        srv.request.amp[i] = fAmplitude[i];
        srv.request.periodic[i] = fPeriodic[i];
    }
    srv.request.acc = fAccelTime;
    srv.request.repeat = nRepeat;
    srv.request.ref = nMoveReference;
    srv.request.syncType = nSyncType;

    if(srvMovePeriodic.call(srv))
    {
        ROS_INFO("receive srv, srv.response.success: %ld\n", (long int)srv.response.success);
        return (srv.response.success);
    }
    else
    {
        ROS_ERROR("Failed to call service dr_control_service : move_periodic\n");
        ros::shutdown();
        return -1;
    }

    return 0;
}
int amove_periodic(float fAmplitude[NUM_TASK], float fPeriodic[NUM_TASK], float fAccelTime = 0.f, int nRepeat = 1, int nMoveReference = MOVE_REFERENCE_TOOL, int nSyncType = 1)
{
    return move_periodic(fAmplitude, fPeriodic, fAccelTime, nRepeat, nMoveReference, /*nSyncType=*/ 1);
}


int move_wait()
{
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();  
    ros::ServiceClient srvMoveWait = node->serviceClient<dsr_msgs::MoveWait>("/"+ROBOT_ID +ROBOT_MODEL+"/motion/move_wait");
    dsr_msgs::MoveWait srv;

    if(srvMoveWait.call(srv))
    {
        ROS_INFO("receive srv, srv.response.success: %ld\n", (long int)srv.response.success);
        return (srv.response.success);
    }
    else
    {
        ROS_ERROR("Failed to call service dr_control_service : move_wait\n");
        ros::shutdown();
        return -1;
    }

    return 0; 
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void thread_subscriber()
{
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
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
    int rate_sub = 1;    // 1 Hz = 1 sec
    int nPubRate = 100;  // 100 Hz (10ms)
    int i=0, nRes=0; 
    ros::init(argc, argv, "single_robot_basic_cpp", ros::init_options::NoSigintHandler);  
    ros::NodeHandle nh("~");
    // Override the default ros sigint handler.
    // This must be set after the first NodeHandle is created.
    signal(SIGINT, SigHandler);
    ros::Publisher  pubRobotStop = nh.advertise<dsr_msgs::RobotStop>("/"+ROBOT_ID +ROBOT_MODEL+"/stop",10);

    // spawn another thread
    boost::thread thread_sub(thread_subscriber);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    float velx[2]={250.0, 80.625};     // 태스크 속도를 250(mm/sec), 80.625(deg/sec)로 설정
    float accx[2]={1000.0, 322.5};   // 태스크 가속도를 1000(mm/sec2), 322.5(deg/sec2)로 설정

    float j1[6]={0.0, 0.0, 90.0, 0.0, 90.0, 0.0};   //joint
    float sj1[2][6]={{10.00, 0.00, 0.00, 0.00, 10.00, 20.00},{15.00, 0.00, -10.00, 0.00, 10.00, 20.00}};
    float x1[6]={0.0, 0.0, -100.0, 0.0, 0.0, 0.0}; //task
    float x2[6]={545,100,514,0,-180,0}; //jx task
    float cx1[2][6]={{544.00, 100.00, 500.00, 0.00, -180.00, 0.00},{543.00, 106.00, 479.00, 7.00, -180.00, 7.00}};
    float sx1[2][6]={{10.00, -10.00, 20.00, 0.00, 10.00, 0.00},{15.00, 10.00, -10.00, 0.00, 10.00, 0.00}};
    float bx1[2][6]={{564.00, 200.00, 690.00, 0.00, 180.00, 0.00},{0, 0, 0, 0, 0, 0}};
    float bx2[2][6]={{564.00, 100.00, 590.00, 0.00, 180.00, 0.00},{564.00, 150.00, 590.00, 0.00, 180.00, 0.00}};

    float amp[6]={10.00, 0.00, 20.00, 0.00, 0.50, 0.00};
    float period[6]={1.00, 0.00, 1.50, 0.00, 0.00, 0.00};
    MOVE_POSB posb[2];

    while(ros::ok())
    {
        /*
        set_velj(60.0)
        set_accj(100.0)
        set_velx(250.0, 80.625)
        set_accx(1000.0, 322.5)
        gLoopCountRev = 0 */
        //movej(posj(0.00, 0.00, 90.00, 0.00, 90.00, 0.00), radius=0.00, ra=DR_MV_RA_DUPLICATE)
        movej(j1, 60, 30);
        //movel(posx(0.00, 0.00, -100.00, 0.00, 0.00, 0.00), radius=0.00, ref=DR_BASE, mod=DR_MV_MOD_REL,ra=DR_MV_RA_DUPLICATE)
        movel(x1, velx, accx, 0, 0, MOVE_REFERENCE_BASE, MOVE_MODE_RELATIVE);
        //movejx
        movejx(x2, 60, 30, 2, 0, MOVE_REFERENCE_BASE, MOVE_MODE_ABSOLUTE, BLENDING_SPEED_TYPE_DUPLICATE, 2);
        //movec(posx(544.00, 100.00, 500.00, 0.00, -180.00, 0.00), posx(543.00, 106.00, 479.00, 7.00, -180.00, 7.00), radius=0.00, angle=[0.00,0.00],ra=DR_MV_RA_DUPLICATE)
        movec(cx1, velx, accx);
        //movesj([posj(10.00, 0.00, 0.00, 0.00, 10.00, 20.00), posj(15.00, 0.00, -10.00, 0.00, 10.00, 20.00)], mod=DR_MV_MOD_REL)
        movesj(sj1, 2, 60, 30, 0, MOVE_MODE_RELATIVE);
        //movesx([posx(10.00, -10.00, 20.00, 0.00, 10.00, 0.00), posx(15.00, 10.00, -10.00, 0.00, 10.00, 0.00)], mod=DR_MV_MOD_REL)
        movesx(sx1, 2, velx, accx, 0, MOVE_MODE_RELATIVE);
        //moveb([posb(DR_LINE, posx(564.00, 200.00, 690.00, 0.00, 180.00, 0.00), radius=40.0), posb(DR_CIRCLE, posx(564.00, 100.00, 590.00, 0.00, 180.00, 0.00), posx(564.00, 150.00, 590.00, 0.00, 180.00, 0.00), radius=20.0)], ref=DR_BASE, mod=DR_MV_MOD_ABS)
        for(int i=0; i<2; i++){
            for(int j=0; j<6; j++){
                posb[0]._fTargetPos[i][j] = bx1[i][j];
                posb[1]._fTargetPos[i][j] = bx2[i][j];
            }
        }

        posb[0]._iBlendType = 0;    // LINE
        posb[1]._iBlendType = 1;    // CIRCLE

        posb[0]._fBlendRad = 40.0;
        posb[1]._fBlendRad = 20.0;

        moveb(posb, 2, velx, accx);
        //move_spiral(rev=1.00, rmax=20.00, lmax=20.00, time=5.00, axis=DR_AXIS_Z, ref=DR_TOOL)
        move_spiral(1.00, 20.00, 20.00, velx, accx, 5, TASK_AXIS_Z, MOVE_REFERENCE_TOOL);
        //move_periodic(amp=[10.00, 0.00, 20.00, 0.00, 0.50, 0.00], period=[1.00, 0.00, 1.50, 0.00, 0.00, 0.00], atime=0.50, repeat=3, ref=DR_BASE)
        move_periodic(amp, period, 0.5, 3, MOVE_REFERENCE_BASE);
   
        //---- async motions --------------------------------------------------------------------------------- 
        amovej(j1, 60, 30, 0, 0, MOVE_MODE_ABSOLUTE, BLENDING_SPEED_TYPE_DUPLICATE);
        move_wait();
   
        amovel(x1, velx, accx, 0, 0, MOVE_REFERENCE_BASE, MOVE_MODE_RELATIVE, BLENDING_SPEED_TYPE_DUPLICATE);
        move_wait();

        amovejx(x2, 60, 30, 2, MOVE_MODE_ABSOLUTE,MOVE_REFERENCE_BASE, 0, BLENDING_SPEED_TYPE_DUPLICATE, 2);
        move_wait();

        amovec(cx1, velx, accx, 0, 0, MOVE_REFERENCE_BASE, MOVE_MODE_ABSOLUTE, BLENDING_SPEED_TYPE_DUPLICATE);
        move_wait();

        amovesj(sj1, 2, 60, 30, 0, MOVE_MODE_RELATIVE);
        move_wait();

        amovesx(sx1, 2, velx, accx, 0, MOVE_MODE_RELATIVE, MOVE_REFERENCE_BASE, SPLINE_VELOCITY_OPTION_DEFAULT);
        move_wait();

        amoveb(posb, 2, velx, accx, 0, MOVE_MODE_ABSOLUTE, MOVE_REFERENCE_BASE);
        move_wait();

        amove_spiral(1.00, 20.00, 20.00, velx, accx, 5, TASK_AXIS_Z, MOVE_REFERENCE_TOOL);
        move_wait();

        amove_periodic(amp, period, 0.5, 3, MOVE_REFERENCE_BASE);
        move_wait();
    }

    ros::shutdown();
    // wait the second thread to finish
    ///thread_sub.join();
    ROS_INFO("single_robot_basic_cpp finished !!!!!!!!!!!!!!!!!!!!!");
    return 0;
}
