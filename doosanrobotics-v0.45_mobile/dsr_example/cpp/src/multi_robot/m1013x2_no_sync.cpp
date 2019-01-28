#include <ros/ros.h>
#include <signal.h>
#include "dsr_robot.h"

using namespace DSR_Robot;
using namespace std;

//----- set tartget robot----------------------------------------------------
string ROBOT_ID     = "dsr01";
string ROBOT_MODEL  = "m1013";
string ROBOT_ID2    = "dsr02";
string ROBOT_MODEL2 = "m1013";

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

boost::mutex io_mutex;
boost::mutex::scoped_lock lock(io_mutex);
boost::condition_variable condition;

boost::mutex io_mutex2;
boost::mutex::scoped_lock lock2(io_mutex2);
boost::condition_variable condition2;

bool bIswait_R1 = false; 
bool bIswait_R2 = false; 

int WakeUp_R2()
{
    while(1)
    {  
        if(true == bIswait_R2)
        {     
            std::cout << ">>>>>>>>>>>>>>>>>>>> send wake-up [R2]" << std::endl;    
            condition.notify_one();
            //condition.notify_all();
            break;    
        }
        //boost::this_thread::sleep(boost::posix_time::seconds(1));
        boost::this_thread::sleep(boost::posix_time::milliseconds(10));
    }            
    return 0;
}
int WaitSignalFrom_R1()
{
    std::cout << "<<<<<<<<<<<<<<<<<<<< wait signal form [R1]" << std::endl;

    bIswait_R2 = true;
    condition.wait(lock);
    bIswait_R2 = false;
    return 0;
}

int WakeUp_R1()
{
    while(1)
    {  
        if(true == bIswait_R1)
        {     
            std::cout << ">>>>>>>>>>>>>>>>>>>> send wake-up [R1]" << std::endl;
            //condition2.notify_one();
            condition2.notify_all();
            break;
        }
        //boost::this_thread::sleep(boost::posix_time::seconds(1));
        boost::this_thread::sleep(boost::posix_time::milliseconds(10));
    }
    return 0;
}
int WaitSignalFrom_R2()
{
    std::cout << "<<<<<<<<<<<<<<<<<<<< wait signal form [R2]" << std::endl;
    bIswait_R1 = true;
    condition2.wait(lock2);
    bIswait_R1 = false;
    return 0;
}

void thread_robot1(ros::NodeHandle nh)
{
    ros::Rate r(100); // 100 hz

    float JReady[6] = {0, -20, 110, 0, 60, 0}; //posj   
    float TCP_POS[6] = {0, 0, 0,0 ,0, 0};
    float J00[6] = {-180, 0, -145, 0, -35, 0};
    float J01r[6] = {-180.0, 71.4, -145.0, 0.0, -9.7, 0.0};

    CDsrRobot r1(nh,"dsr01","m1013");

    WakeUp_R2();
    r1.movej(JReady, 20, 20);
    WaitSignalFrom_R2();

    while(1)
    {
        //ROS_INFO("thread_robot1 running...");

        WakeUp_R2();
        r1.movej(J00, 20, 20);
        WaitSignalFrom_R2();

        WakeUp_R2();
        r1.movej(J01r, 20, 20);
        WaitSignalFrom_R2();

        ///r.sleep();
    }
}

void thread_robot2(ros::NodeHandle nh)
{
    ros::Rate r(100); // 100 hz

    float JReady[6] = {0, -20, 110, 0, 60, 0}; //posj   
    float TCP_POS[6] = {0, 0, 0,0 ,0, 0};
    float J00[6] = {-180, 0, -145, 0, -35, 0};
    float J01r[6] = {-180.0, 71.4, -145.0, 0.0, -9.7, 0.0};

    CDsrRobot r2(nh,"dsr02","m1013");

    WaitSignalFrom_R1();
    r2.movej(JReady, 20, 20);
    WakeUp_R1();

    while(1)
    {
        //ROS_INFO("thread_robot2 running...");

        WaitSignalFrom_R1();
        r2.movej(J00, 20, 20);
        WakeUp_R1();

        WaitSignalFrom_R1();
        r2.movej(J01r, 20, 20);
        WakeUp_R1();

        ///r.sleep();
    }
}

int main(int argc, char** argv)
{
    signal(SIGINT, SigHandler);

    ros::init(argc, argv, "m1013x2_no_sync_cpp", ros::init_options::NoSigintHandler);  
    ros::NodeHandle nh("~");
    ros::Publisher  pubRobotStop  = nh.advertise<dsr_msgs::RobotStop>("/"+ROBOT_ID  +ROBOT_MODEL +"/stop",10);
    ros::Publisher  pubRobotStop2 = nh.advertise<dsr_msgs::RobotStop>("/"+ROBOT_ID2 +ROBOT_MODEL2+"/stop",10);

    // spawn another thread
    boost::thread thread_sub(thread_robot1, nh);
    boost::thread thread_sub2(thread_robot2, nh);

    //--------------------------------------------------------
    /*
    boost::this_thread::sleep(boost::posix_time::seconds(5));   
    worker_is_done = true;
    std::cout << "Notifying condition..." << std::endl;
    //condition.notify_one();
    condition.notify_all();
    */
    //--------------------------------------------------------

    //CDsrRobot robot(nh);
    CDsrRobot r1(nh,"dsr01","m1013");
    CDsrRobot r2(nh,"dsr02","m1013");

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

    ros::Rate r(100); // 100 hz

    while(ros::ok())
    {
        r1.amovej(JReady, 20, 20);
        r2.amovej(JReady, 20, 20);
        r1.move_wait(); r2.move_wait();

        r1.config_create_tcp("TCP_ZERO", TCP_POS);
        r2.config_create_tcp("TCP_ZERO", TCP_POS);
        r1.set_current_tcp("TCP_ZERO");
        r2.set_current_tcp("TCP_ZERO");

        r1.amovej(J1, 0, 0, 3);
        r2.amovej(J1, 0, 0, 3);
        r1.move_wait(); r2.move_wait();

        r1.amovel(X3, velx, accx, 2.5);
        r2.amovel(X3, velx, accx, 2.5);
        r1.move_wait(); r2.move_wait();
        
        for(int i=0; i<2; i++){
            r1.amovel(X2, velx, accx, 2.5, MOVE_MODE_ABSOLUTE, MOVE_REFERENCE_BASE, 50);
            r2.amovel(X2, velx, accx, 2.5, MOVE_MODE_ABSOLUTE, MOVE_REFERENCE_BASE, 50);
            r1.move_wait(); r2.move_wait();

            r1.amovel(X1, velx, accx, 1.5, MOVE_MODE_ABSOLUTE, MOVE_REFERENCE_BASE, 50);
            r2.amovel(X1, velx, accx, 1.5, MOVE_MODE_ABSOLUTE, MOVE_REFERENCE_BASE, 50);
            r1.move_wait(); r2.move_wait();

            r1.amovel(X0, velx, accx, 2.5);
            r2.amovel(X0, velx, accx, 2.5);
            r1.move_wait(); r2.move_wait();

            r1.amovel(X1, velx, accx, 2.5, MOVE_MODE_ABSOLUTE, MOVE_REFERENCE_BASE, 50);
            r2.amovel(X1, velx, accx, 2.5, MOVE_MODE_ABSOLUTE, MOVE_REFERENCE_BASE, 50);
            r1.move_wait(); r2.move_wait();

            r1.amovel(X2, velx, accx, 1.5, MOVE_MODE_ABSOLUTE, MOVE_REFERENCE_BASE, 50);
            r2.amovel(X2, velx, accx, 1.5, MOVE_MODE_ABSOLUTE, MOVE_REFERENCE_BASE, 50);
            r1.move_wait(); r2.move_wait();

            r1.amovel(X3, velx, accx, 2.5, MOVE_MODE_ABSOLUTE, MOVE_REFERENCE_BASE, 50);
            r2.amovel(X3, velx, accx, 2.5, MOVE_MODE_ABSOLUTE, MOVE_REFERENCE_BASE, 50);
            r1.move_wait(); r2.move_wait();
        }

        r1.amovej(J00, 60, 60, 6);
        r2.amovej(J00, 60, 60, 6);
        r1.move_wait(); r2.move_wait();

        r1.amovej(J01r, 0, 0, 2, MOVE_MODE_ABSOLUTE, 100);
        r2.amovej(J01r, 0, 0, 2, MOVE_MODE_ABSOLUTE, 100);
        r1.move_wait(); r2.move_wait();

        r1.amovej(J02r, 0, 0, 2, MOVE_MODE_ABSOLUTE, 50); 
        r2.amovej(J02r, 0, 0, 2, MOVE_MODE_ABSOLUTE, 50); 
        r1.move_wait(); r2.move_wait();

        r1.amovej(J03r, 0, 0, 2);
        r2.amovej(J03r, 0, 0, 2);
        r1.move_wait(); r2.move_wait();

        r1.amovej(J04r, 0, 0, 1.5);
        r2.amovej(J04r, 0, 0, 1.5);
        r1.move_wait(); r2.move_wait();

        r1.amovej(J04r1, 0, 0, 2, MOVE_MODE_ABSOLUTE, 50);
        r2.amovej(J04r1, 0, 0, 2, MOVE_MODE_ABSOLUTE, 50);
        r1.move_wait(); r2.move_wait();

        r1.amovej(J04r2, 0, 0, 4, MOVE_MODE_ABSOLUTE, 50);
        r2.amovej(J04r2, 0, 0, 4, MOVE_MODE_ABSOLUTE, 50);
        r1.move_wait(); r2.move_wait();

        r1.amovej(J04r3, 0, 0, 4, MOVE_MODE_ABSOLUTE, 50);
        r2.amovej(J04r3, 0, 0, 4, MOVE_MODE_ABSOLUTE, 50);
        r1.move_wait(); r2.move_wait();

        r1.amovej(J04r4, 0, 0, 2);
        r2.amovej(J04r4, 0, 0, 2);
        r1.move_wait(); r2.move_wait();

        r1.amovej(J05r, 0, 0, 2.5, MOVE_MODE_ABSOLUTE, 100); 
        r2.amovej(J05r, 0, 0, 2.5, MOVE_MODE_ABSOLUTE, 100); 
        r1.move_wait(); r2.move_wait();

        r1.amovel(dREL1, velx, accx, 1, MOVE_MODE_ABSOLUTE, MOVE_REFERENCE_TOOL, 50); 
        r2.amovel(dREL1, velx, accx, 1, MOVE_MODE_ABSOLUTE, MOVE_REFERENCE_TOOL, 50); 
        r1.move_wait(); r2.move_wait();

        r1.amovel(dREL2, velx, accx, 1.5, MOVE_MODE_ABSOLUTE, MOVE_REFERENCE_TOOL, 100); 
        r2.amovel(dREL2, velx, accx, 1.5, MOVE_MODE_ABSOLUTE, MOVE_REFERENCE_TOOL, 100); 
        r1.move_wait(); r2.move_wait();

        r1.amovej(J07r, 60, 60, 1.5, MOVE_MODE_ABSOLUTE, 100); 
        r2.amovej(J07r, 60, 60, 1.5, MOVE_MODE_ABSOLUTE, 100); 
        r1.move_wait(); r2.move_wait();

        r1.amovej(J08r, 60, 60, 2);
        r2.amovej(J08r, 60, 60, 2);
        r1.move_wait(); r2.move_wait();

        r1.amovej(JEnd, 60, 60, 4);
        r2.amovej(JEnd, 60, 60, 4);
        r1.move_wait(); r2.move_wait();

        r1.amove_periodic(amp, period, 0, 1, MOVE_REFERENCE_TOOL);
        r2.amove_periodic(amp, period, 0, 1, MOVE_REFERENCE_TOOL);
        r1.move_wait(); r2.move_wait();

        r1.amove_spiral(0, 3, 200, 100, vel_spi, acc_spi, 0, MOVE_REFERENCE_TOOL);
        r2.amove_spiral(0, 3, 200, 100, vel_spi, acc_spi, 0, MOVE_REFERENCE_TOOL);
        r1.move_wait(); r2.move_wait();

        r1.amovel(x01, velx, accx, 2);
        r2.amovel(x01, velx, accx, 2);
        r1.move_wait(); r2.move_wait();

        r1.amovel(x04, velx, accx, 2, MOVE_MODE_ABSOLUTE, MOVE_REFERENCE_BASE, 100);
        r2.amovel(x04, velx, accx, 2, MOVE_MODE_ABSOLUTE, MOVE_REFERENCE_BASE, 100);
        r1.move_wait(); r2.move_wait();

        r1.amovel(x03, velx, accx, 2, MOVE_MODE_ABSOLUTE, MOVE_REFERENCE_BASE, 100);
        r2.amovel(x04, velx, accx, 2, MOVE_MODE_ABSOLUTE, MOVE_REFERENCE_BASE, 100);
        r1.move_wait(); r2.move_wait();

        r1.amovel(x02, velx, accx, 2, MOVE_MODE_ABSOLUTE, MOVE_REFERENCE_BASE, 100);
        r2.amovel(x02, velx, accx, 2, MOVE_MODE_ABSOLUTE, MOVE_REFERENCE_BASE, 100);
        r1.move_wait(); r2.move_wait();

        r1.amovel(x01, velx, accx, 2);
        r2.amovel(x01, velx, accx, 2);
        r1.move_wait(); r2.move_wait();

        for(int i=0; i<6; i++){
            x0204c[0][i] = x02[i];
            x0204c[1][i] = x04[i];
        }
        r1.amovec(x0204c, velx, accx, 4 , MOVE_MODE_ABSOLUTE, MOVE_REFERENCE_BASE, 360);
        r2.amovec(x0204c, velx, accx, 4 , MOVE_MODE_ABSOLUTE, MOVE_REFERENCE_BASE, 360);
        r1.move_wait(); r2.move_wait();

        r.sleep();
    }

    ROS_INFO("m1013x2_sync_cpp finished !!!!!!!!!!!!!!!!!!!!!");
    return 0;
}
