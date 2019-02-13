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
        robot.movej(j1, 60, 30);
        //movel(posx(0.00, 0.00, -100.00, 0.00, 0.00, 0.00), radius=0.00, ref=DR_BASE, mod=DR_MV_MOD_REL,ra=DR_MV_RA_DUPLICATE)
        robot.movel(x1, velx, accx, 0, MOVE_MODE_RELATIVE);
        //movejx
        robot.movejx(x2, 60, 30, 2, MOVE_MODE_ABSOLUTE,MOVE_REFERENCE_BASE, 0, BLENDING_SPEED_TYPE_DUPLICATE, 2);
        //movec(posx(544.00, 100.00, 500.00, 0.00, -180.00, 0.00), posx(543.00, 106.00, 479.00, 7.00, -180.00, 7.00), radius=0.00, angle=[0.00,0.00],ra=DR_MV_RA_DUPLICATE)
        robot.movec(cx1, velx, accx);
        //movesj([posj(10.00, 0.00, 0.00, 0.00, 10.00, 20.00), posj(15.00, 0.00, -10.00, 0.00, 10.00, 20.00)], mod=DR_MV_MOD_REL)
        robot.movesj(sj1, 2, 60, 30, 0, MOVE_MODE_RELATIVE);
        //movesx([posx(10.00, -10.00, 20.00, 0.00, 10.00, 0.00), posx(15.00, 10.00, -10.00, 0.00, 10.00, 0.00)], mod=DR_MV_MOD_REL)
        robot.movesx(sx1, 2, velx, accx, 0, MOVE_MODE_RELATIVE);
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

        robot.moveb(posb, 2, velx, accx);
        //move_spiral(rev=1.00, rmax=20.00, lmax=20.00, time=5.00, axis=DR_AXIS_Z, ref=DR_TOOL)
        robot.move_spiral(2, 1.00, 20.00, 20.00, velx, accx, 5, MOVE_REFERENCE_TOOL);
        //move_periodic(amp=[10.00, 0.00, 20.00, 0.00, 0.50, 0.00], period=[1.00, 0.00, 1.50, 0.00, 0.00, 0.00], atime=0.50, repeat=3, ref=DR_BASE)
        robot.move_periodic(amp, period, 0.5, 3, MOVE_REFERENCE_BASE);
        //amovej
        robot.amovej(j1, 60, 30, 0, MOVE_MODE_ABSOLUTE, 0, BLENDING_SPEED_TYPE_DUPLICATE);
        //wait
        robot.move_wait();
        //amovel
        robot.amovel(x1, velx, accx, 0, MOVE_MODE_RELATIVE, MOVE_REFERENCE_BASE, 0, BLENDING_SPEED_TYPE_DUPLICATE);
        //amovejx
        robot.amovejx(x2, 60, 30, 2,MOVE_MODE_ABSOLUTE,MOVE_REFERENCE_BASE, 0, BLENDING_SPEED_TYPE_DUPLICATE, 2);
        //wait
        robot.move_wait();
        //amovec
        robot.amovec(cx1, velx, accx, 0, MOVE_MODE_ABSOLUTE, MOVE_REFERENCE_BASE, 0, BLENDING_SPEED_TYPE_DUPLICATE);
        //wait
        robot.move_wait();
        //amovesj
        robot.amovesj(sj1, 2, 60, 30, 0, MOVE_MODE_RELATIVE);
        //wait
        robot.move_wait();
        //amovesx
        robot.amovesx(sx1, 2, velx, accx, 0, MOVE_MODE_RELATIVE, MOVE_REFERENCE_BASE, SPLINE_VELOCITY_OPTION_DEFAULT);
        //wait
        robot.move_wait();
        //amoveb
        robot.amoveb(posb, 2, velx, accx, 0, MOVE_MODE_ABSOLUTE, MOVE_REFERENCE_BASE);
        //wait
        robot.move_wait();
        //amovespiral
        robot.amove_spiral(2, 1.00, 20.00, 20.00, velx, accx, 5, MOVE_REFERENCE_TOOL);
        //wait
        robot.move_wait();
        //amoveperiodic
        robot.amove_periodic(amp, period, 0.5, 3, MOVE_REFERENCE_BASE);
        //wait
        robot.move_wait();
    }
    thread_sub.join();
    // wait the second thread to finish
    ROS_INFO("dsr_service_test finished !!!!!!!!!!!!!!!!!!!!!");
    return 0;
}
