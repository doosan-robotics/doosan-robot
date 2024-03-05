# [Doosan Robotics](http://www.doosanrobotics.com/kr/)
[![license - apache 2.0](https://img.shields.io/:license-Apache%202.0-yellowgreen.svg)](https://opensource.org/licenses/Apache-2.0)
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)
[![support level: community](https://img.shields.io/badge/support%20level-community-lightgray.png)](http://rosindustrial.org/news/2016/10/7/better-supporting-a-growing-ros-industrial-software-platform)

# *overview*

[Doosan ROS Video](https://www.youtube.com/watch?v=mE24X5PhZ4M&feature=youtu.be)   
<img src="https://user-images.githubusercontent.com/47092672/113229859-c1f86100-92d2-11eb-8242-3aa7e7f7ef88.png" width="50%">  
[Doosan ROS Online Lecture(Kor)](https://www.youtube.com/watch?v=TpvBziOb--A)   
[Doosan ROS Online Lecture(Eng)](https://www.youtube.com/watch?v=KkzoS5VORPc) 


# *install*

#### virtual mode
To utilize the new emulator in virtual mode, Docker is required . Please ensure Docker is installed beforehand if virtual mode is required.
##### *install Docker https://docs.docker.com/engine/install/ubuntu/*
#### dependency packages
    sudo apt-get install ros-noetic-rqt* ros-noetic-moveit* ros-noetic-gazebo-ros-control ros-noetic-joint-state-controller ros-noetic-effort-controllers ros-noetic-position-controllers ros-noetic-ros-controllers ros-noetic-ros-control ros-noetic-joint-state-publisher-gui ros-noetic-joint-state-publisher


# *build* 
##### *Doosan Robot ROS Package is implemented at ROS-Noetic.*
    ### We recommand the /home/<user_home>/catkin_ws/src
    cd ~/catkin_ws/src
    git clone https://github.com/doosan-robotics/doosan-robot
    rosdep install --from-paths doosan-robot --ignore-src --rosdistro noetic -r -y
    ##### Emulator Download
    sh doosan-robot/doosan_robot/install_emulator.sh 
    
    ##### Serial Package source build
    ### Noetic distro does not support serial package, so you have to install it manually.
    cd ~/catkin_ws/src
    git clone https://github.com/wjwwood/serial.git
    
    cd ~/catkin_ws
    catkin_make
    source ./devel/setup.bash


# *usage* <a id="chapter-3"></a>
#### Operation Mode
##### Virtual Mode
If you are driveing without a real robot, use __virtual mode__   
When ROS launches in virtual mode, the emulator(DRCF) runs automatically.
> (DRCF) location: docker image

```bash
roslaunch dsr_launcher single_robot_gazebo.launch mode:=virtual
```
_One emulator is required for each robot_

##### Real Mode
Use __real mode__ to drive a real robot   
The default IP of the robot controller is _192.168.137.100_ and the port is _12345_.
```bash
roslaunch dsr_launcher single_robot_gazebo.launch mode:=real host:=192.168.137.100 port:=12345
```
___
#### dsr_description
```bash
roslaunch dsr_description m0609.launch    
roslaunch dsr_description m1013.launch color:=blue # Change Color
roslaunch dsr_description m1509.launch 
roslaunch dsr_description m0617.launch color:=blue # change color 
roslaunch dsr_description a0509.launch 
roslaunch dsr_description e0509.launch    
```

> $ _roslaunch dsr_description m1013.launch_ 
<img src="https://user-images.githubusercontent.com/47092672/55622394-0f708f00-57db-11e9-8625-a344513a5d3a.png" width="70%">

> + In dsr_description, the user can use joint_state_publisher to move the robot.
> + [Joint_state_publisher](http://wiki.ros.org/joint_state_publisher)

> $ _roslaunch dsr_description m0617.launch color:=blue 

<img src="https://user-images.githubusercontent.com/47092672/55624467-f7037300-57e0-11e9-930a-ec929de3a0fa.png" width="70%">

___
#### dsr_moveit_config
> ###### __arguments__
   > color:= ROBOT_COLOR <white / blue> deflaut = white  (e0509 has only white)
    
    roslaunch moveit_config_m0609 m0609.launch
    roslaunch moveit_config_m0617 m0617.launch
    roslaunch moveit_config_m1013 m1013.launch 
    roslaunch moveit_config_m1509 m1509.launch
    roslaunch moveit_config_a0509 a0509.launch
    roslaunch moveit_config_a0509 e0509.launch
    
    
<img src="https://user-images.githubusercontent.com/47092672/55613994-fd84f100-57c6-11e9-97eb-49d1d7c9e32c.png" width="70%">

___
#### dsr_control _(default model:= m1013, default mode:= virtual)_
> ###### __arguments__                    
>host := ROBOT_IP deflaut = 127.0.0.1 
port := ROBOT_PORT default = 12345  
mode := OPERATION MODE <virtual  /  real> deflaut = virtual  
model := ROBOT_MODEL <m0609  /  0617 /  m1013  /  m1509 / a0509 / a0912 / h2017 / h2515 / e0509> deflaut = m1013  
color := ROBOT_COLOR <white  /  blue> deflaut = white  
gripper := USE_GRIPPER <none  /  robotiq_2f> deflaut = none  
mobile := USE_MOBILE <none  /  husky> deflaut = none  
rviz := USE_RVIZ_GUI <true  /  false> deflaut = true  

#### dsr_moveit
    roslaunch dsr_launcher dsr_moveit.launch
    roslaunch dsr_launcher dsr_moveit.launch model:=m0609 mode:=virtual
    roslaunch dsr_launcher dsr_moveit.launch model:=m0617 mode:=virtual
    roslaunch dsr_launcher dsr_moveit.launch model:=m1013 mode:=virtual
    roslaunch dsr_launcher dsr_moveit.launch model:=m1509 mode:=virtual
    roslaunch dsr_launcher dsr_moveit.launch model:=a0509 mode:=virtual

### dsr_moveit + gazebo
    roslaunch dsr_launcher dsr_moveit_gazebo.launch

##### *How to use MoveIt Commander*
###### _You can run Moveit with CLI commands through the moveit commander package._
###### _You can install the "moveit_commander" package using below command._
    sudo apt-get install ros-noetic-moveit-commander
##### *MoveitCommander usage example*
	roslaunch dsr_launcher dsr_moveit.launch model:=m1013
	In another terminal 
	ROS_NAMESPACE=/dsr01m1013 rosrun moveit_commander moveit_commander_cmdline.py robot_description:=/dsr01m1013/robot_description   
###### *moveit commander CLI is executed.*    
    > use arm 
    > goal0 = [0 0 0 0 0 0]        # save the home position to variable "goal0"
    > goal1 = [0 0 1.57 0 1.57 0]  # save the target position to varialbe "goal1" / radian
    > go goal1                     # plan & excute (the robot is going to move target position)
    > go goal0                     # paln & excute (the robot is going to move home position)
___
#### dsr_launcher

__If you don`t have real doosan controller, you must execute emulator before run dsr_launcer.__
> ###### __arguments__    
   >host:= ROBOT_IP deflaut = 127.0.0.1  ##controller IP = 192.168.137.100 
    port:= ROBOT_PORT default = 12345  
    mode:= OPERATION MODE <virtual  /  real> deflaut = virtual  
    model:= ROBOT_MODEL <m0609 / m0617 / m1013 / m1509 / a0509 / e0509> deflaut = m1013  
    color:= ROBOT_COLOR <white / blue> deflaut = white  
    gripper:= USE_GRIPPER <none / robotiq_2f> deflaut = none  
    mobile:= USE_MOBILE <none / husky> deflaut = none 
    rviz:= USE Rviz <true / false > deflaut = true

    roslaunch dsr_launcher single_robot_rviz.launch host:=127.0.0.1 port:=12345 mode:=virtual model:=m1013 color:=blue gripper:=none mobile:=none
    roslaunch dsr_launcher single_robot_gazebo.launch host:=192.168.137.100
    roslaunch dsr_launcher single_robot_rviz_gazebo.launch gripper:=robotiq_2f mobile:=husky
    
___
#### dsr_example
###### single robot
    <launch>
      - single robot in rviz : 
      roslaunch dsr_launcher single_robot_rviz.launch model:=m1013 color:=white
      - single robot in gazebo : 
      roslaunch dsr_launcher single_robot_gazebo.launch model:=m1013 color:=blue
      - single robot in rviz + gazebo : 
      roslaunch dsr_launcher single_robot_rviz_gazebo.launch model:=m1013 color:=white
    <run application node>
      rosrun dsr_example_py single_robot_simple.py dsr01 m1013
    <ex>
      roslaunch dsr_launcher single_robot_rviz_gazebo.launch model:=m1013 color:=white
      rosrun dsr_example_py single_robot_simple.py

> _$ roslaunch dsr_launcher single_robot_rviz_gazebo.launch_

> _$ rosrun dsr_example_py single_robot_simple.py_
> <img src="https://user-images.githubusercontent.com/47092672/55624471-fbc82700-57e0-11e9-8c1f-4fe9f526944b.png" width="70%">
    
#### gazebo+rviz+virtual
    roslaunch dsr_launcher single_robot_rviz_gazebo.launch
    rosrun dsr_example_py single_robot_simple.py
```bash
  <include file="$(find dsr_gazebo)/launch/dsr_base.launch">
    <arg name="ns" value="dsr01"/> # Robot ID
    <arg name="model" value="m1013"/> # Robot Model
    <arg name="host" value="192.168.137.100"/> # Robot IP
    <arg name="port" value="12345"/> # Robot Port
    <arg name="mode" value="virtual"/> # Robot Controller Mode 
    # Position & Posture in Gazebo
    <arg name="x" value="2"/>
    <arg name="y" value="-4"/>
    <arg name="yaw" value="0.7"/>
  </include>
  <include file="$(find dsr_gazebo)/launch/dsr_base.launch">
    <arg name="ns" value="dsr02"/> # Secondary Robot ID
    <arg name="model" value="m1013"/> # Secondary Robot Model
    <arg name="host" value="192.168.137.102"/> # Secondary Robot IP
    <arg name="port" value="12346"/> # Robot Port
    <arg name="mode" value="virtual"/> # Secondary Robot Controller Mode
    # Secondary Position & Posture in Gazebo
    <arg name="x" value="2"/>
    <arg name="y" value="-4"/>
    <arg name="yaw" value="0.7"/>
  </include>
```  

#### Service Call
```bash
rosservice call /dsr01m1013/motion/move_joint "pos: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
vel: 0.0
acc: 0.0
time: 0.0
radius: 0.0
mode: 0
blendType: 0
syncType: 0"
```
___
# manuals

[Manual(Kor)](http://wiki.ros.org/doosan-robotics?action=AttachFile&do=get&target=Doosan_Robotics_ROS_Manual_ver1.12_20200522A%28Kor.%29.pdf)


[Manual(Eng)](http://wiki.ros.org/doosan-robotics?action=AttachFile&do=get&target=Doosan_Robotics_ROS_Manual_ver1.12_20200522%28EN.%29.pdf)

