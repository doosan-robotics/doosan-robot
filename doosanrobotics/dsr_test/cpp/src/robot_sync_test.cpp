#include <ros/ros.h>
#include <signal.h>
#include "dsr_robot.h"
#include <time.h>
#include <chrono>

using namespace DSR_Robot;
using namespace std;

//----- set tartget robot----------------------------------------------------
#define NUM_ROBOT   2 
string ROBOT_ID     = "dsr01";
string ROBOT_MODEL  = "m1013";
string ROBOT_ID2    = "dsr02";
string ROBOT_MODEL2 = "m1013";

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void time_sleep(float x){boost::this_thread::sleep( boost::posix_time::milliseconds(int(x*1000)));}

class CRobotSync{
    int m_nRobot; 
    bool m_nIsRun; 
    bool m_bIsWait[NUM_ROBOT]; 
    unsigned int m_nWaitBit, m_nCurBit;


    boost::mutex m_io_mutex[NUM_ROBOT];
    boost::mutex::scoped_lock* m_pLock[NUM_ROBOT];
    boost::condition_variable m_condition[NUM_ROBOT];

    public:
        CRobotSync(int r){
            m_nRobot = r;    
            m_nIsRun = true;
            m_nWaitBit = m_nCurBit = 0x00;

            for(int i=0; i<m_nRobot; i++){
                m_nWaitBit |= (0x1<<i);
                m_bIsWait[i] = false;
                m_pLock[i] = new boost::mutex::scoped_lock(m_io_mutex[i]);
            }    
        }
        virtual ~CRobotSync(){
            printf("~CRobotSync()\n");
            printf("~CRobotSync()\n");
            printf("~CRobotSync()\n");
            /*
            for(int i=0; i<m_nRobot; i++){
                if(true == m_bIsWait[i])
                    m_condition[i].notify_one();
            }
            */
            m_nIsRun = false;
            /*    
            for(int i=0; i<m_nRobot; i++){
                //delete &m_condition[i];
                //m_io_mutex[i].release();        
                //if(m_pLock[i]) delete m_pLock[i]; 
            }
            */
        }    
        int Wait(int nId){
            m_bIsWait[nId] = true;
            m_condition[nId].wait( *m_pLock[nId] );
            m_bIsWait[nId] = false;
        }
        int WakeUp(int nId){ 
            while(m_nIsRun){
                if(true == m_bIsWait[nId]){                       
                    m_condition[nId].notify_one();
                    break;
                }
                time_sleep(0.01);    
            }
            return 0;
        } 
        int WakeUpAll(){ 
            m_nCurBit=0;
            while(m_nIsRun){
                for(int i=0; i<m_nRobot; i++){
                    if(true == m_bIsWait[i])
                        m_nCurBit |= (0x1<<i);
                }    
                if(m_nWaitBit == m_nCurBit)
                    break;
                time_sleep(0.01);    
            }
            for(int i=0; i<m_nRobot; i++)
                m_condition[i].notify_one();
            return 0;
        } 
};
CRobotSync RobotSync(NUM_ROBOT);

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


std::chrono::system_clock::time_point time_1;
std::chrono::system_clock::time_point time_2;

void thread_robot1(ros::NodeHandle nh)
{
    int nRobotID = 0;    
 
    while(ros::ok())
    {
        RobotSync.Wait(nRobotID);
        //ROS_INFO("thread_robot1 running...");
        //time_1 = clock();
        time_1 = std::chrono::system_clock::now();
        time_sleep(0.001);
    }
}
void thread_robot2(ros::NodeHandle nh)
{
    int nRobotID = 1;    
 
    while(ros::ok())
    {
        RobotSync.Wait(nRobotID);
        //ROS_INFO("thread_robot2 running...");
        //time_2 = clock();
        time_2 = std::chrono::system_clock::now();
        time_sleep(0.001);
    }
}

void thread_robot3(ros::NodeHandle nh)
{
    int nRobotID = 2;    
    while(1)
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
    while(1)
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
    while(1)
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

int main(int argc, char** argv)
{
    signal(SIGINT, SigHandler);

    ros::init(argc, argv, "robot_sync_test_cpp", ros::init_options::NoSigintHandler);  
    ros::NodeHandle nh("~");
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

    //clock_t t1, t2;
    std::chrono::system_clock::time_point t1, t2; 
    std::chrono::milliseconds mill;  


    double max_time = 0.0;
    double cur_time = 0.0;

    while(ros::ok())
    {
        
        time_sleep(0.001);
        RobotSync.WakeUpAll();
        std::chrono::nanoseconds nano = time_1 - time_2;
        cur_time =  double( abs(nano.count()/1000000.f));       
        if (cur_time > max_time)
        {
            max_time = cur_time;
            printf("Max Sync Time = %.3f ms\n", max_time);
        }  
    }

    ROS_INFO("robot_sync_test_cpp finished !!!!!!!!!!!!!!!!!!!!!");
    return 0;
}
