

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

int movej(float fTargetPos[NUM_JOINT],
          float fTargetVel,
          float fTargetAcc,
          float fTargetTime = 0.f,
          int   nMoveMode = MOVE_MODE_ABSOLUTE,
          float fBlendingRadius = 0.f,
          int   nBlendingType = BLENDING_SPEED_TYPE_DUPLICATE,
          int nSyncType = 0)             
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
    srv.request.mode = nMoveMode;
    srv.request.radius = fBlendingRadius;
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

    return 0; 
}

int movel(float fTargetPos[NUM_JOINT],
          float fTargetVel[2],
          float fTargetAcc[2],
          float fTargetTime = 0.f,
          int   nMoveMode = MOVE_MODE_ABSOLUTE,
          int   nMoveReference = MOVE_REFERENCE_BASE,
          float fBlendingRadius = 0.f,
          int   nBlendingType = BLENDING_SPEED_TYPE_DUPLICATE,
          int   nSyncType = 0)             
{
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();  
    ros::ServiceClient srvMoveLine = node->serviceClient<dsr_msgs::MoveLine>("/"+ROBOT_ID +ROBOT_MODEL+"/motion/move_line");
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
    return 0; 
}

int movec(float fTargetPos[2][NUM_TASK],
          float fTargetVel[2],
          float fTargetAcc[2],
          float fTargetTime = 0.f,
          int   nMoveMode = MOVE_MODE_ABSOLUTE,
          int   nMoveReference = MOVE_REFERENCE_BASE,
          float fBlendingRadius = 0.f,
          int   nBlendingType = BLENDING_SPEED_TYPE_DUPLICATE,
          int   nSyncType = 0)
{
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();  
    ros::ServiceClient srvMoveCircle = node->serviceClient<dsr_msgs::MoveCircle>("/"+ROBOT_ID +ROBOT_MODEL+"/motion/move_circle");

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

    return 0;
}

int movejx(float fTargetPos[NUM_TASK],
           float fTargetVel, 
           float fTargetAcc, 
           float fTargetTime=0.f,
           int   nMoveMode = MOVE_MODE_ABSOLUTE,
           int   nMoveReference = MOVE_REFERENCE_BASE,
           float fBlendingRadius = 0.f,
           int   nBlendingType = BLENDING_SPEED_TYPE_DUPLICATE,
           int   nSolSpace = 0,
           int    nSyncType = 0)
{
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();  
    ros::ServiceClient srvMoveJointx = node->serviceClient<dsr_msgs::MoveJointx>("/"+ROBOT_ID +ROBOT_MODEL+"/motion/move_jointx");
    dsr_msgs::MoveJointx srv; 

    for(int i=0; i<NUM_TASK; i++)
        srv.request.pos[i] = fTargetPos[i];
    srv.request.vel = fTargetVel;
    srv.request.acc = fTargetAcc;
    srv.request.time = fTargetTime;
    srv.request.mode = nMoveMode;
    srv.request.ref = nMoveReference;
    srv.request.radius = fBlendingRadius;
    srv.request.sol = nSolSpace;
    srv.request.blendType = nBlendingType;

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

int moveperiodic(float fAmplitude[NUM_TASK],
                 float fPeriodic[NUM_TASK],
                 float fAccelTime = 0.f,
                 int nRepeat = 1,
                 int nMoveReference = MOVE_REFERENCE_TOOL,
                 int nSyncType = 0)
{
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();  
    ros::ServiceClient srvMovePeriodic = node->serviceClient<dsr_msgs::MovePeriodic>("/"+ROBOT_ID +ROBOT_MODEL+"/motion/move_periodic");
    dsr_msgs::MovePeriodic srv;

    for(int i=0; i<NUM_TASK; i++)
        srv.request.amp[i] = fAmplitude[i];
    for(int i=0; i<NUM_TASK; i++)
        srv.request.periodic[i] = fPeriodic[i];
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

int movespiral(int nTaskAxis,
               float fRevolution,
               float fMaxRadius,
               float fMaxLength,
               float fTargetVel[2],
               float fTargetAcc[2],
               float fTargetTime = 0.f,
               int nMoveReference = MOVE_REFERENCE_TOOL,
               int nSyncType = 0)
{
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();  
    ros::ServiceClient srvMoveSpiral = node->serviceClient<dsr_msgs::MoveSpiral>("/"+ROBOT_ID +ROBOT_MODEL+"/motion/move_spiral");
    dsr_msgs::MoveSpiral srv;

    srv.request.taskAxis = nTaskAxis;
    srv.request.revolution = fRevolution;
    srv.request.maxRadius = fMaxRadius;
    srv.request.maxLength = fMaxLength;
    for(int i=0; i<2; i++)
        srv.request.vel[i] = fTargetVel[i];
    for(int i=0; i<2; i++)
        srv.request.acc[i] = fTargetAcc[i];
    srv.request.time = fTargetTime;
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

int movesj(float fTargetPos[MAX_SPLINE_POINT][NUM_JOINT],
           int nPosCount,
           float fTargetVel,
           float fTargetAcc,
           float fTargetTime = 0.f,
           int nMoveMode = MOVE_MODE_ABSOLUTE,
           int nSyncType = 0)
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

int movesx(float fTargetPos[MAX_SPLINE_POINT][NUM_TASK],
           int nPosCount,
           float fTargetVel[2],
           float fTargetAcc[2],
           float fTargetTime = 0.f,
           int nMoveMode = MOVE_MODE_ABSOLUTE,
           int nMoveReference = MOVE_REFERENCE_BASE,
           int nVelOpt = SPLINE_VELOCITY_OPTION_DEFAULT,
           int nSyncType = 0)
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
    for(int i=0; i<2; i++)
        srv.request.vel[i] = fTargetVel[i];
    for(int i=0; i<2; i++)
        srv.request.acc[i] = fTargetAcc[i];
    srv.request.time = fTargetTime;
    srv.request.mode = nMoveMode;
    srv.request.ref = nMoveReference;
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

int moveb(MOVE_POSB* fTargetPos,
          int nPosCount,
          float fTargetVel[2],
          float fTargetAcc[2],
          float fTargetTime = 0.f,
          int nMoveMode = MOVE_MODE_ABSOLUTE,
          int nMoveReference = MOVE_REFERENCE_BASE,
          int nSyncType = 0)
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

    for(int i=0; i<2; i++)
        srv.request.vel[i] = fTargetVel[i];
    for(int i=0; i<2; i++)
        srv.request.acc[i] = fTargetAcc[i];

    srv.request.posCnt = nPosCount;    
    srv.request.time = fTargetTime;    
    srv.request.mode = nMoveMode;
    srv.request.ref = nMoveReference;
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
void mySigintHandler(int sig)
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


void thread_subscriber()
{
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
    ros::MultiThreadedSpinner spinner(2);
    spinner.spin();
}

int main(int argc, char** argv)
{
    //----- set target robot --------------- 
    const string my_robot_id    = "dsr01";
    const string my_robot_model = "m1013";
    SET_ROBOT(my_robot_id, my_robot_model);

    int rate_sub = 1;    // 1 Hz = 1 sec
    int nPubRate = 100;  // 100 Hz (10ms)
    int i=0, nRes=0; 

    ros::init(argc, argv, "dsr_service_motion_basic_cpp", ros::init_options::NoSigintHandler);  
    ros::NodeHandle nh("~");

    ros::Publisher  pubRobotStop = nh.advertise<dsr_msgs::RobotStop>("/"+ROBOT_ID +ROBOT_MODEL+"/stop",10);

    // spawn another thread
    boost::thread thread_sub(thread_subscriber);

    signal(SIGINT, mySigintHandler);
    /* 
    if(argc !=3)
    {
        ROS_INFO("cmd : rosrun dsr_control dsr_control_service arg0 arg1");
        ROS_INFO("arg0: double number, arg1: double number");
        return 1;
    }
    */
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
        movel(x1, velx, accx, 0, MOVE_MODE_RELATIVE);
        //movejx
        movejx(x2, 60, 30, 2, MOVE_MODE_ABSOLUTE,MOVE_REFERENCE_BASE, 0, BLENDING_SPEED_TYPE_DUPLICATE, 2);
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
        movespiral(2, 1.00, 20.00, 20.00, velx, accx, 5, MOVE_REFERENCE_TOOL);
        //move_periodic(amp=[10.00, 0.00, 20.00, 0.00, 0.50, 0.00], period=[1.00, 0.00, 1.50, 0.00, 0.00, 0.00], atime=0.50, repeat=3, ref=DR_BASE)
        moveperiodic(amp, period, 0.5, 3, MOVE_REFERENCE_BASE);
        //amovej
        movej(j1, 60, 30, 0, MOVE_MODE_ABSOLUTE, 0, BLENDING_SPEED_TYPE_DUPLICATE, 1);
        //wait
        move_wait();
        //amovel
        movel(x1, velx, accx, 0, MOVE_MODE_RELATIVE, MOVE_REFERENCE_BASE, 0, BLENDING_SPEED_TYPE_DUPLICATE, 1);
        //amovejx
        movejx(x2, 60, 30, 2,MOVE_MODE_ABSOLUTE,MOVE_REFERENCE_BASE, 0, BLENDING_SPEED_TYPE_DUPLICATE, 2, 1);
        //wait
        move_wait();
        //amovec
        movec(cx1, velx, accx, 0, MOVE_MODE_ABSOLUTE, MOVE_REFERENCE_BASE, 0, BLENDING_SPEED_TYPE_DUPLICATE, 1);
        //wait
        move_wait();
        //amovesj
        movesj(sj1, 2, 60, 30, 0, MOVE_MODE_RELATIVE, 1);
        //wait
        move_wait();
        //amovesx
        movesx(sx1, 2, velx, accx, 0, MOVE_MODE_RELATIVE, MOVE_REFERENCE_BASE, SPLINE_VELOCITY_OPTION_DEFAULT, 1);
        //wait
        move_wait();
        //amoveb
        moveb(posb, 2, velx, accx, 0, MOVE_MODE_ABSOLUTE, MOVE_REFERENCE_BASE, 1);
        //wait
        move_wait();
        //amovespiral
        movespiral(2, 1.00, 20.00, 20.00, velx, accx, 5, MOVE_REFERENCE_TOOL, 1);
        //wait
        move_wait();
        //amoveperiodic
        moveperiodic(amp, period, 0.5, 3, MOVE_REFERENCE_BASE, 1);
        //wait
        move_wait();
    }

    ros::shutdown();
    // wait the second thread to finish
    ///thread_sub.join();
    ROS_INFO("dsr_service_test finished !!!!!!!!!!!!!!!!!!!!!");
    return 0;
}
