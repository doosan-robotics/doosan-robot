/*
 * dsr_control_node
 * Author: Kab Kyoum Kim (kabkyoum.kim@doosan.com)
 *
 * Copyright (c) 2019 Doosan Robotics
 * Use of this source code is governed by the BSD, see LICENSE
*/

#include <controller_manager/controller_manager.h>
#include <sensor_msgs/JointState.h>
#include <signal.h>
#include <boost/assign/list_of.hpp>
#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>
#include "dsr_control/dsr_hw_interface.h"
#include <cstdlib> 
#include <chrono>
#include <thread>

using namespace dsr_control;

int g_nKill_dsr_control = false;

bool isEmulatorRunning() {
    FILE *cmd_pipe = popen("docker ps -q --filter \"name=dsr_emulator\"", "r");
    if (!cmd_pipe) {
        ROS_ERROR("[dsr_control] An error occurred while executing the command.");
        return false;
    }

    char result[1024];
    if (fgets(result, sizeof(result), cmd_pipe) != nullptr) {
        pclose(cmd_pipe);
        return true; // In case of a container named 'dsr_emulator' running...
    }

    pclose(cmd_pipe);
    return false; // In case of a container named 'dsr_emulator' not running...
}

bool stopEmulator() {
    int result = system("docker stop dsr_emulator");
    if (result != 0) {
        ROS_ERROR("[dsr_control] Error occurred while stopping the emulator container.");
        return false;
    }
    return true;
}


void SigHandler(int sig) {
    ROS_INFO("[dsr_control] shutdown time! (sig = %d)", sig);
    ROS_INFO("[dsr_control] shutdown time! (sig = %d)", sig);
    ROS_INFO("[dsr_control] shutdown time! (sig = %d)", sig);
    if (isEmulatorRunning()) {
        ROS_INFO("[dsr_control] The emulator container is running. Attempting to stop it...");
        if (stopEmulator()) {
            ROS_INFO("[dsr_control] The emulator container has been successfully stopped.");
        } else {
            ROS_ERROR("[dsr_control] The emulator container failed to stop.");
        }
    } else {
        ROS_WARN("[dsr_control] The emulator container is not running.");
    }
    // g_nKill_dsr_control = true;
    ros::shutdown();
}

int main(int argc, char** argv) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
    ros::init(argc, argv, "dsr_control_node",
              ros::init_options::NoSigintHandler);
    ros::NodeHandle private_nh("~");
    signal(SIGINT, SigHandler);
    //----- get param ---------------------
    int rate;
    private_nh.param<int>("rate", rate, 100);
    ROS_INFO("rate is %d\n", rate);
    ros::Rate r(rate);

    ros::V_string arm_joint_names;
    arm_joint_names = boost::assign::list_of("joint1")("joint2")("joint3")(
                          "joint4")("joint5")("joint6")
                          .convert_to_container<ros::V_string>();

    DRHWInterface* pArm = NULL;
    pArm = new DRHWInterface(private_nh);
    private_nh.getParam("name", pArm->m_strRobotName);
    private_nh.getParam("model", pArm->m_strRobotModel);
    private_nh.getParam("moveit", pArm->m_moveit);
    ros::Publisher PubJointState =
        private_nh.advertise<sensor_msgs::JointState>(
            "/" + pArm->m_strRobotName + pArm->m_strRobotModel + "/joint_states", 1);

    if (!pArm->init()) {
        ROS_ERROR("[dsr_control] Error initializing robot");
        return -1;
    }
    controller_manager::ControllerManager cm(pArm, private_nh);
    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    ros::Time last_time;
    ros::Time curr_time;
    ros::Duration elapsed;
    last_time = ros::Time::now();

    ROS_INFO("[dsr_control] controller_manager is updating!");

    while (ros::ok() && (false==g_nKill_dsr_control)) {
        try {
            sensor_msgs::JointState joint_state;

            curr_time = ros::Time::now();
            elapsed = curr_time - last_time;
            if (pArm)
                pArm->read(elapsed);
            cm.update(ros::Time::now(), elapsed);
            for (int i = 0; i < 6; i++) {
                joint_state.header.stamp = curr_time;
                joint_state.name.push_back(arm_joint_names[i]);
                joint_state.position.push_back(pArm->joints[i].pos);
                joint_state.velocity.push_back(pArm->joints[i].vel);
                joint_state.effort.push_back(pArm->joints[i].eff);
            }
            if (!pArm->m_moveit)
                PubJointState.publish(joint_state);
            if (pArm)
                pArm->write(elapsed);
            r.sleep();
        } catch (std::runtime_error& ex) {
            ROS_ERROR("[dsr_control] Exception: [%s]", ex.what());
            ROS_ERROR("[dsr_control] Exception: [%s]", ex.what());
            ROS_ERROR("[dsr_control] Exception: [%s]", ex.what());
            break;
        }
    }

    spinner.stop();

    ROS_INFO("[dsr_control] Good-bye!");

    return 0;
}