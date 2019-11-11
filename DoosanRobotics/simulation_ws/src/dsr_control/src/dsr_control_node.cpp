/*
 * dsr_control_node 
 * Author: Kab Kyoum Kim (kabkyoum.kim@doosan.com)
 *
 * Copyright (c) 2019 Doosan Robotics
 * Use of this source code is governed by the BSD, see LICENSE
*/

#include <signal.h>
#include "dsr_control/dsr_hw_interface.h"
#include <controller_manager/controller_manager.h>

using namespace dsr_control;

int g_nKill_dsr_control = false; 
void SigHandler(int sig)
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
    //----- init ROS ---------------------- 
    ///ros::init(argc, argv, "dsr_control_node");
    ros::init(argc, argv, "dsr_control_node", ros::init_options::NoSigintHandler);
    ros::NodeHandle private_nh("~");
    ///ros::NodeHandle nh("/dsr_control");
    // Override the default ros sigint handler.
    // This must be set after the first NodeHandle is created.
    signal(SIGINT, SigHandler);

    //----- get param ---------------------
    int rate;
    private_nh.param<int>("rate", rate, 100);
    ROS_INFO("rate is %d\n", rate);
    ros::Rate r(rate);

    ///dsr_control::DRHWInterface arm(nh);
    DRHWInterface* pArm = NULL;
    pArm = new DRHWInterface(private_nh);

    if(!pArm->init() ){
        ROS_ERROR("[dsr_control] Error initializing robot");
        return -1;
    }
    ///controller_manager::ControllerManager cm(&arm, nh);
    controller_manager::ControllerManager cm(pArm, private_nh);
    ros::AsyncSpinner spinner(1);
    spinner.start();

//    int rate;
//    private_nh.param<int>("rate", rate, 50);
//    ROS_INFO("rate is %d\n", rate);
//    ros::Rate r(rate);

    ros::Time last_time;
    ros::Time curr_time;
    ros::Duration elapsed;
    last_time = ros::Time::now();

    ROS_INFO("[dsr_control] controller_manager is updating!");

    while(ros::ok() && (false==g_nKill_dsr_control))
    ///while(g_nKill_dsr_control==false)
    {
        try{
            ///ROS_INFO("[dsr_control] Running...(g_nKill_dsr_control=%d)",g_nKill_dsr_control);
            curr_time = ros::Time::now();
            elapsed = curr_time - last_time;
            if(pArm) pArm->read(elapsed);
            cm.update(ros::Time::now(), elapsed);
            if(pArm) pArm->write(elapsed);
            r.sleep();	//(1000/rate)[sec], default: 10ms 
        }
        catch(std::runtime_error& ex)
        {
            ROS_ERROR("[dsr_control] Exception: [%s]", ex.what());
            ROS_ERROR("[dsr_control] Exception: [%s]", ex.what());
            ROS_ERROR("[dsr_control] Exception: [%s]", ex.what());
            break;
        }
    }

    spinner.stop();
    //if(pArm) delete(pArm);

    ROS_INFO("[dsr_control] Good-bye!");

    return 0;
}