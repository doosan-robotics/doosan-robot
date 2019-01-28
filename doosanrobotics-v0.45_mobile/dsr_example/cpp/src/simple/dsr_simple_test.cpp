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


    float p1[6]={0.0,};                             //joint
    float p2[6]={0.0, 0.0, 90.0, 0.0, 90.0, 0.0};   //joint
    
    float x1[6]={400, 500, 800.0, 0.0, 180.0, 0.0}; //task
    float x2[6]={400, 500, 500.0, 0.0, 180.0, 0.0}; //task
    float velx[2]={50, 50};     // 태스크 속도를 50(mm/sec), 50(deg/sec)로 설정
    float accx[2]={100, 100};   // 태스크 가속도를 100(mm/sec2), 100(deg/sec2)로 설정


    float veljx[6] = {50, 50, 50, 50, 50, 50};
    float accjx[6] = {100, 100, 100, 100, 100, 100};
    float fCirclePos[2][6] = {{559,434.5,651.5,0,180,0}, {559,434.5,251.5,0,180,0}};

    float amp[6] = {10,0,0,0,30,0};
    float periodic[6] = {1,0,1.5,0,0,0};
    
    float fSJPos[3][6] = {{10, -10, 20, -30, 10, 20}, {25, 0, 10, -50, 20, 40}, {50, 50, 50, 50, 50, 50}};
    float fSXPos[3][6] = {{600, 600, 600, 0, 175, 0},{600, 750, 600, 0, 175, 0},{150, 600, 450, 0, 175, 0}};

    float bx1[2][6] = {{370, 670, 650, 0, 180, 0}, {370, 670, 650, 0, 180, 0}};
    float bx2[2][6] = {{370, 670, 400, 0, 180, 0},{370, 545, 400, 0, 180, 0}};
    float bx3[2][6] = {{370, 595, 400, 0, 180, 0}, {370, 595, 400, 0, 180, 0}};

    MOVE_POSB posb[3];// = {bx1, 0, 40}, {bx2, 1, 40}, {bx3, 1, 40};
    for(int i=0; i<2; i++){
        for(int j=0; j<6; j++){
            posb[0]._fTargetPos[i][j] = bx1[i][j];
        }
    }
    for(int i=0; i<2; i++){
        for(int j=0; j<6; j++){
            posb[1]._fTargetPos[i][j] = bx2[i][j];
        }
    }
    for(int i=0; i<2; i++){
        for(int j=0; j<6; j++){
            posb[2]._fTargetPos[i][j] = bx3[i][j];
        }
    }
    posb[0]._iBlendType = 0;
    posb[1]._iBlendType = 1;
    posb[2]._iBlendType = 0;
    posb[0]._fBlendRad = 20;
    posb[1]._fBlendRad = 20;
    posb[2]._fBlendRad = 20;

    while(ros::ok())
    {
        robot.movej(p1, 30, 30); 
        robot.movej(p2, 30, 30);
        robot.movel(x1, velx, accx);
        robot.movel(x2, velx, accx);

        robot.movec(fCirclePos, velx, accx);
        robot.move_periodic(amp, periodic, 0.5, 3, 0);  
        robot.move_spiral(TASK_AXIS_Z,9.5, 20.0, 50.0, velx, accx, 20.0);
        robot.movesj(fSJPos,3, 30, 100);
        robot.movesx(fSXPos, 3, velx, accx);
        
        robot.moveb(posb, 3, velx, accx);

    }

    ROS_INFO("dsr_service_test finished !!!!!!!!!!!!!!!!!!!!!");
    ROS_INFO("dsr_service_test finished !!!!!!!!!!!!!!!!!!!!!");
    ROS_INFO("dsr_service_test finished !!!!!!!!!!!!!!!!!!!!!");

    ///if(&robot) delete (&robot); 
    thread_sub.join();
    
    ROS_INFO("dsr_service_test finished !!!!!!!!!!!!!!!!!!!!!");
    return 0;
}
