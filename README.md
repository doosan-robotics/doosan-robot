# [Doosan robotics](http://www.doosanrobotics.com/kr/)


## Contents
  
  #### [Build](#chapter-2)
  
  #### [Usage](#chapter-3)

# *build* <a id="chapter-2"></a>
    mkdir -p /home/path/to/your/workspace/src   ### We recoomand the /home/<user_home>/catkin_ws/src
    cd /home/path/to/your/workspace/src
    catkin_init_workspace
    git clone https://github.com/doosan-robotics/doosan-robot
    rosdep install --from-paths doosan-robot --ignore-src --rosdistro kinetic -r -y 
    catkin_make
    source ./devel/setup.bash
#### package list
    sudo apt-get install ros-kinetic-rqt* ros-kinetic-moveit* ros-kinetic-industrial-core ros-kinetic-gazebo-ros-control ros-kinetic-joint-state-controller ros-kinetic-effort-controllers ros-kinetic-position-controllers ros-kinetic-ros-controllers ros-kinetic-ros-control ros-kinetic-serial ros-kinetic-lms1xx ros-kinetic-interactive-marker-twist-server ros-kinetic-twist-mux ros-kinetic-imu-tools ros-kinetic-controller-manager ros-kinetic-robot-localization

# *usage* <a id="chapter-3"></a>
#### DRCF Emulator
If you don`t have real doosan controller, you must excute our emulator. 
Emulator has local IP(127.0.0.1).
```bash
cd /home/path/to/workspace/doosan-robot/common/bin/DRCF
sudo ./DRCF64 <port>   ## 64bits OS
or 
sudo ./DRCF32 <port>   ## 32bits OS
``` 
#### dsr_description
```bash
roslaunch dsr_description m0609.launch    
roslaunch dsr_description m1013.launch color:=blue # Change Color
roslaunch dsr_description m1509.launch gripper:=robotiq_2f # insert robotiq gripper
roslaunch dsr_description m0617.launch color:=blue gripper:=robotiq_2f # change color & insert robotiq gripper 필요!
```
#### dsr_moveit_config
> ###### __arguments__
   >color:= ROBOT_COLOR <white  /  blue> defalut = white
   
    roslaunch moveit_config_m0609 m0609.launch
    roslaunch moveit_config_m0617 m0617.launch
    roslaunch moveit_config_m1013 m1013.launch color:=blue
    roslaunch moveit_config_m1509 m1509.launch
    
#### dsr_control _(default model:= m1013, default mode:= virtual)_
> ###### __arguments__                    
>host:= ROBOT_IP defalut = 192,168.137.100   
host:= ROBOT_PORT default = 12345
mode:= OPERATION MODE <virtual  /  real> defalut = virtual  
model:= ROBOT_MODEL <m0609  /  0617/  m1013  /  m1509> defalut = m1013  
color:= ROBOT_COLOR <white  /  blue> defalut = white  
gripper:= USE_GRIPPER <none  /  robotiq_2f> defalut = none  
mobile:= USE_MOBILE <none  /  husky> defalut = none  

#### dsr_control + dsr_moveit_config
    roslaunch dsr_control dsr_moveit.launch
    roslaunch dsr_control dsr_moveit.launch model:=m0609 mode:=virtual
    roslaunch dsr_control dsr_moveit.launch model:=m0617 mode:=virtual
    roslaunch dsr_control dsr_moveit.launch model:=m1013 mode:=virtual
    roslaunch dsr_control dsr_moveit.launch model:=m1509 mode:=virtual
      
#### dsr_launcher
---
__If you don`t have real doosan controller, you must execute emulator before run dsr_launcer.__
> ###### __arguments__    
   >host:= ROBOT_IP defalut = 192.168.137.100  ##Emulator IP = 127.0.0.1   
    port:= ROBOT_PORT default = 12345  
    mode:= OPERATION MODE <virtual  /  real> defalut = virtual  
    model:= ROBOT_MODEL <m0609  /  0617/  m1013  /  m1509> defalut = m1013  
    color:= ROBOT_COLOR <white  /  blue> defalut = white  
    gripper:= USE_GRIPPER <none  /  robotiq_2f> defalut = none  
    mobile:= USE_MOBILE <none  /  husky> defalut = none  

    roslaunch dsr_launcher single_robot_rviz.launch host:=127.0.0.1 port:=12345 mode:=virtual model:=m1013 color:=blue gripper:=none mobile:=none
    roslaunch dsr_launcher single_robot_gazebo.launch host:=192.168.137.100
    roslaunch dsr_launcher single_robot_rviz_gazebo.launch gripper:=robotiq_2f mobile:=husky
    roslaunch dsr_launcher multi_robot_rviz.launch
    roslaunch dsr_launcher multi_robot_gazebo.launch model:=m0609
    roslaunch dsr_launcher multi_robot_rviz_gazebo.launch
---
#### dsr_example
###### single robot
    <launch>
      single robot in rviz : roslaunch dsr_launcher single_robot_rviz.launch model:=m1013 color:=white
      single robot in gazebo : roslaunch dsr_launcher single_robot_gazebo.launch model:=m1013 color:=blue
      single robot in rviz + gazebo : roslaunch dsr_launcher single_robot_rviz_gazebo.launch model:=m1013 color:=white
    <run application node>
      <cpp>
        basic example : rosrun dsr_example_cpp single_robot_basic dsr01 m1013
        simple example : rosrun dsr_example_cpp single_robot_simple dsr01 m1013
      <py>
        basic example : rosrun dsr_example_py single_robot_basic.py dsr01 m1013
        simple example : rosrun dsr_example_py single_robot_simple.py dsr01 m1013
    <ex>
      roslaunch dsr_launcher single_robot_rviz_gazebo.launch model:=m1013 color:=white
      rosrun dsr_example_cpp single_robot_simple dsr01 m1013

###### multi robot
    <launch>
      multi robot in rviz : roslaunch dsr_launcher multi_robot_rviz.launch
      multi robot in gazebo : roslaunch dsr_launcher multi_robot_gazebo.launch
      multi robot in rviz + gazebo : roslaunch dsr_launcher multi_robot_rviz_gazebo.launch
    <run application node>
      <cpp>
        basic example : rosrun dsr_example_cpp multi_robot_basic
        simple example : rosrun dsr_example_cpp multi_robot_simple
      <py>
        basic example : rosrun dsr_example_py multi_robot_basic.py
        simple example : rosrun dsr_example_py multi_robot_simple.py
      <ex>
        roslaunch dsr_launcher multi_robot_rviz_gazebo.launch
        rosrun dsr_example_cpp multi_robot  

###### robot + mobile
> insert argument mobile:=husky
- single robot on mobile
```bash
roslaunch dsr_launcher single_robot_rviz.launch host:=192.168.137.100 mode:=virtual model:=m1013 color:=blue mobile:=husky
  
<run application node>
  <cpp>
    rosrun dsr_example_cpp single_robot_mobile
  <python>
    rosrun dsr_example_py single_robot_mobile
```
- multi robot on mobile
```bash
roslaunch dsr_launcher multi_robot_rviz.launch host:=192.168.137.100 mode:=virtual model:=m1013 color:=blue mobile:=husky

<run application node>
  <cpp>
    rosrun dsr_example_cpp multi_robot_mobile
  <python>
    rosrun dsr_example_py multi_robot_mobile  
```
###### robot + gripper
> insert argument gripper:=robotiq_2f  
- single robot + gripper
```bash
roslaunch dsr_launcher single_robot_rviz.launch host:=192.168.137.100 mode:=virtual model:=m1013 color:=blue gripper:=robotiq_2f

<run application node>
  <cpp>
    rosrun dsr_example_cpp pick_and_place_simple
  <python>
    rosrun dsr_example_py pick_and_place_simple
```
- Serial Test(Loopback)
```bash
rosrun dsr_example_cpp serial_example_node ttyUSB0 115200
rostopic echo /serial_read
rostopic pub /serial_write std_msgs/String 'data: 100'
```

    
#### gazebo+rviz+virtual
    roslaunch dsr_example test.launch
    rosrun dsr_test_cpp dsr_test
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

#### Run multi-robot by command line
```bash
roslaunch dsr_launcher test.launch
rostopic pub /dsr01m1013/joint_position_controller/command std_msgs/Float64MultiArray "layout:
  dim:
  - label: ''
    size: 0
    stride: 0
    data_offset: 0
data: [10, 10, 40, 10, 60, 10]"
```
#### Service Call
```bash
rosservice call /dsr/set_joint_move "jointAngle: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
jointVelocity: [50.0, 0.0, 0.0, 0.0, 0.0, 0.0]
jointAcceleration: [50.0, 0.0, 0.0, 0.0, 0.0, 0.0]
radius: 0.0"
```

# diagnostic

# etc
