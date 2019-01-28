/*

*/
#include <signal.h>
#include "dsr_control/dsr_hw_interface.h"
#include <controller_manager/controller_manager.h>

using namespace dsr_control;

int g_nKill_dsr_control = false; 
void dsr_control_sigintHandler(int sig)
{
    // Do some custom action.
    // For example, publish a stop message to some other nodes.
  
    // All the default sigint handler does is call shutdown()
    ROS_INFO("[dsr_control] shutdown time! (sig = %d)",sig);
    ROS_INFO("[dsr_control] shutdown time! (sig = %d)",sig);
    ROS_INFO("[dsr_control] shutdown time! (sig = %d)",sig);
    
    g_nKill_dsr_control = true;
    ///usleep(1000*1000);
    ros::shutdown();
}

int main(int argc, char** argv)
{
    signal(SIGINT, dsr_control_sigintHandler);

    ///ros::init(argc, argv, "dsr_control_node");
    ros::init(argc, argv, "dsr_control_node", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh("~");
    ///ros::NodeHandle nh("/dsr_control");

    ///dsr_control::DRHWInterface arm(nh);
    DRHWInterface* pArm = NULL;
    pArm = new DRHWInterface(nh);

    ///if(!arm.init() ){
    if(!pArm->init() ){
        ROS_ERROR("[dsr_control] Error initializing robot model, quitting");
        return -1;
    }
    ///controller_manager::ControllerManager cm(&arm, nh);
    controller_manager::ControllerManager cm(pArm, nh);
    ros::AsyncSpinner spinner(1);
    spinner.start();

    int rate;
    ros::param::param<int>("~rate", rate, 50);
    ros::Rate r(rate);
    ros::Time last_time;
    ros::Time curr_time;
    ros::Duration elapsed;
    last_time = ros::Time::now();

    ROS_INFO("[dsr_control] controller_manager is updating!");

    while(ros::ok() && (g_nKill_dsr_control==false))
    ///while(g_nKill_dsr_control==false)
    {
        try{
            ///ROS_INFO("[dsr_control] Running...(g_nKill_dsr_control=%d)",g_nKill_dsr_control);
            curr_time = ros::Time::now();
            elapsed = curr_time - last_time;
            if(pArm) pArm->read(elapsed);
            cm.update(ros::Time::now(), elapsed);
            if(pArm) pArm->write(elapsed);
            r.sleep();
        }
        catch(std::runtime_error& ex)
        {
            ROS_ERROR("[dsr_control] Exception: [%s]", ex.what());
            ROS_ERROR("[dsr_control] Exception: [%s]", ex.what());
            ROS_ERROR("[dsr_control] Exception: [%s]", ex.what());
            break;
        }
    }

    ROS_INFO("[dsr_control] Good-bye!");
    ROS_INFO("[dsr_control] Good-bye!");
    ROS_INFO("[dsr_control] Good-bye!");

    spinner.stop();

///kkk    if(pArm) delete(pArm);    //멀티로봇에서 if(pArm) delete(pArm) 하면 에러가 날까???????????????????????

    return 0;
}