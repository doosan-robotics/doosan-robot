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

int main(int argc, char** argv)
{
    signal(SIGINT, SigHandler);

    ros::init(argc, argv, "dance_m1013_cpp", ros::init_options::NoSigintHandler);  
    ros::NodeHandle nh("~");
    ros::Publisher  pubRobotStop = nh.advertise<dsr_msgs::RobotStop>("/"+ROBOT_ID +ROBOT_MODEL+"/stop",10);

    //----- set target robot --------------- 
    const string my_robot_id    = "dsr01";
    const string my_robot_model = "m1013";
    SET_ROBOT(my_robot_id, my_robot_model);
    CDsrRobot robot(nh, my_robot_id, my_robot_model);

    /////////POS DEFINE
    
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


    while(ros::ok())
    {
        robot.movej(JReady, 20, 20);
        robot.config_create_tcp("TCP_ZERO", TCP_POS);
        robot.set_current_tcp("TCP_ZERO");

        robot.movej(J1, 0, 0, 3);
        robot.movel(X3, velx, accx, 2.5);
        
        for(int i=0; i<2; i++){
            robot.movel(X2, velx, accx, 2.5, MOVE_MODE_ABSOLUTE, MOVE_REFERENCE_BASE, 50);
            robot.movel(X1, velx, accx, 1.5, MOVE_MODE_ABSOLUTE, MOVE_REFERENCE_BASE, 50);
            robot.movel(X0, velx, accx, 2.5);
            robot.movel(X1, velx, accx, 2.5, MOVE_MODE_ABSOLUTE, MOVE_REFERENCE_BASE, 50);
            robot.movel(X2, velx, accx, 1.5, MOVE_MODE_ABSOLUTE, MOVE_REFERENCE_BASE, 50);
            robot.movel(X3, velx, accx, 2.5, MOVE_MODE_ABSOLUTE, MOVE_REFERENCE_BASE, 50);
        }

        robot.movej(J00, 60, 60, 6);

        robot.movej(J01r, 0, 0, 2, MOVE_MODE_ABSOLUTE, 100);
        robot.movej(J02r, 0, 0, 2, MOVE_MODE_ABSOLUTE, 50); 
        robot.movej(J03r, 0, 0, 2);

        robot.movej(J04r, 0, 0, 1.5);
        robot.movej(J04r1, 0, 0, 2, MOVE_MODE_ABSOLUTE, 50);
        robot.movej(J04r2, 0, 0, 4, MOVE_MODE_ABSOLUTE, 50);
        robot.movej(J04r3, 0, 0, 4, MOVE_MODE_ABSOLUTE, 50);
        robot.movej(J04r4, 0, 0, 2);

        robot.movej(J05r, 0, 0, 2.5, MOVE_MODE_ABSOLUTE, 100); 
        robot.movel(dREL1, velx, accx, 1, MOVE_MODE_ABSOLUTE, MOVE_REFERENCE_TOOL, 50); 
        robot.movel(dREL2, velx, accx, 1.5, MOVE_MODE_ABSOLUTE, MOVE_REFERENCE_TOOL, 100); 

        robot.movej(J07r, 60, 60, 1.5, MOVE_MODE_ABSOLUTE, 100); 
        robot.movej(J08r, 60, 60, 2);

        robot.movej(JEnd, 60, 60, 4);

        robot.move_periodic(amp, period, 0, 1, MOVE_REFERENCE_TOOL);
        robot.move_spiral(0, 3, 200, 100, vel_spi, acc_spi, 0, MOVE_REFERENCE_TOOL);

        robot.movel(x01, velx, accx, 2);
        robot.movel(x04, velx, accx, 2, MOVE_MODE_ABSOLUTE, MOVE_REFERENCE_BASE, 100);
        robot.movel(x03, velx, accx, 2, MOVE_MODE_ABSOLUTE, MOVE_REFERENCE_BASE, 100);
        robot.movel(x02, velx, accx, 2, MOVE_MODE_ABSOLUTE, MOVE_REFERENCE_BASE, 100);
        robot.movel(x01, velx, accx, 2);
        for(int i=0; i<6; i++){
            x0204c[0][i] = x02[i];
            x0204c[1][i] = x04[i];
        }
        robot.movec(x0204c, velx, accx, 4 , MOVE_MODE_ABSOLUTE, MOVE_REFERENCE_BASE, 360);

    }

    ROS_INFO("dance_m1013 finished !!!!!!!!!!!!!!!!!!!!!");
    return 0;
}
