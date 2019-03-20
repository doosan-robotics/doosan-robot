
[doosan robotics](http://www.doosanrobotics.com/kr/)
================
## Contents
  [Installation](#chapter-1)
  [Build](#chapter-2)
  [Usage](#chapter-3)

## installation <a id="chapter-1"></a>
#### package update
apt-get update
#### ssh-server 설치
apt-get install openssh-server
gedit /etc/ssh/sshd_config
> PermitRootLogin Yes
#### FTP 설치
service ssh restart
apt-get install vsftpd
gedit /etc/vsftpd.conf

> write_enable=YES
> local_umask=022

service vsftpd start
#### packages.ros.org의 Software 설치 허용
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu
(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
#### Key 입력
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
#### ROS Kinetic 설치
sudo apt-get install ros-kinetic-desktop-full
#### ROS Kinetic 설치 확인
apt-cache search ros-kinetic
#### Environment Setup
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc source ~/.bashrc

## build <a id="chapter-2"></a>
  mkdir -p dr_ws/src & cd dr_ws/src
   git clone or download & unzip
3. cd ..
4. rosdep install --from-paths rs --ignore-src --rosdistro kinetic -r -y 
5. catkin_make
6. source ./devel/setup.bash
   

## usage <a id="chapter-3"></a>

1.  <dsr_description>
    roslaunch dsr_description m0609.launch
    roslaunch dsr_description m1013.launch color:=blue # 색 변경
    roslaunch dsr_description m1509.launch gripper:=robotiq_2f #robotiq 그리퍼 추가 -> 머지하면서 확인 필요!
    roslaunch dsr_description m0617.launch color:=blue gripper:=robotiq_2f #색 변경 및 그리퍼 추가 -> 머지하면서 확인 필요!

2.  <dsr_moveit_config>
    - arguments
    color:= ROBOT_COLOR <white  /  blue> defalut = white

    roslaunch moveit_config_m0609 m0609.launch
    roslaunch moveit_config_m0617 m0617.launch
    roslaunch moveit_config_m1013 m1013.launch color:=blue
    roslaunch moveit_config_m1509 m1509.launch
    
3.  <dsr_control> (default model:= m1013, default mode:= virtual)

    - arguments
    host:= ROBOT_IP defalut = 192,168.137.100
    mode:= OPERATION MODE <virtual  /  real> defalut = virtual
    model:= ROBOT_MODEL <m0609  /  0617/  m1013  /  m1509> defalut = m1013
    color:= ROBOT_COLOR <white  /  blue> defalut = white
    gripper:= USE_GRIPPER <none  /  robotiq_2f> defalut = none
    mobile:= USE_MOBILE <none  /  husky> defalut = none

    (2) dsr_control + dsr_moveit_config
      roslaunch dsr_control dsr_moveit.launch
      roslaunch dsr_control dsr_moveit.launch model:=m0609 mode:=virtual
      roslaunch dsr_control dsr_moveit.launch model:=m0617 mode:=virtual
      roslaunch dsr_control dsr_moveit.launch model:=m1013 mode:=virtual
      roslaunch dsr_control dsr_moveit.launch model:=m1509 mode:=virtual
      
4.  <dsr_launcher>
    - arguments
    host:= ROBOT_IP defalut = 192,168.137.100
    mode:= OPERATION MODE <virtual  /  real> defalut = virtual
    model:= ROBOT_MODEL <m0609  /  0617/  m1013  /  m1509> defalut = m1013
    color:= ROBOT_COLOR <white  /  blue> defalut = white
    gripper:= USE_GRIPPER <none  /  robotiq_2f> defalut = none
    mobile:= USE_MOBILE <none  /  husky> defalut = none

    roslaunch dsr_launcher single_robot_rviz.launch
    roslaunch dsr_launcher single_robot_gazebo.launch
    roslaunch dsr_launcher single_robot_rviz_gazebo.launch
    roslaunch dsr_launcher multi_robot_rviz.launch
    roslaunch dsr_launcher multi_robot_gazebo.launch
    roslaunch dsr_launcher multi_robot_rviz_gazebo.launch

4.  <dsr_example>
  4.1 single robot
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
  4.2 multi robot
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

  4.3 robot + mobile
    - insert argument mobile:=true
    4.3.1 single robot on mobile
        - roslaunch dsr_launcher rviz_single_robot.launch host:=192.168.137.100 mode:=virtual model:=m1013 color:=blue mobile:=husky
        <run  application  node>
          <cpp>
            rosrun dsr_example_cpp single_robot_mobile
          <python>
            rosrun dsr_example_py single_robot_mobile

   4.3.2 multi robot on mobile
       - roslaunch dsr_launcher rviz_multi_robot.launch host:=192.168.137.100 mode:=virtual model:=m1013 color:=blue mobile:=husky
        <run application node>
          <cpp>
            rosrun dsr_example_cpp multi_robot_mobile
          <python>
            rosrun dsr_example_py multi_robot_mobile  

  4.4 using gripper
    - insert argument gripper:=true

5.  <dsr_apps>
  - rosrun dsr_apps_cpp app_watch
  - rosrun dsr_apps_cpp app_watch.py

6.  <dsr_test>
  - rosrun dsr_test_cpp dsr_test

### gazebo+rviz+virtual(dsr)함께 실행 하는
1. roslaunch dsr_example test.launch
2. rosrun dsr_test_cpp dsr_test
```bash
  <include file="$(find dsr_gazebo)/launch/dsr_base.launch">
    <arg name="ns" value="dsr01"/> 고유 ID
    <arg name="model" value="m1013"/> 로봇 모델
    <arg name="host" value="192.168.137.100"/> 로봇 제어기 IP주소
    <arg name="mode" value="virtual"/> 로봇 제어기 모드 
    가제보에서의 위치와 자세
    <arg name="x" value="2"/>
    <arg name="y" value="-4"/>
    <arg name="yaw" value="0.7"/>
  </include>
  <include file="$(find dsr_gazebo)/launch/dsr_base.launch">
    <arg name="ns" value="dsr02"/> 두번째 고유 ID
    <arg name="model" value="m1013"/> 두번째 로봇 모델
    <arg name="host" value="192.168.137.102"/> 두번째 로봇 제어기 IP주소
    <arg name="mode" value="virtual"/> 두번째 로봇 제어기 모드
두번째 위치와 자세
    <arg name="x" value="2"/>
    <arg name="y" value="-4"/>
    <arg name="yaw" value="0.7"/>
  </include>
```  

### moveit 실행
```bash
roslaunch dsr_control dsr_moveit.launch model:=m0609
```

### 멀티 로봇 Command로 실행
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
### call service
```bash
rosservice call /dsr/set_joint_move "jointAngle: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
jointVelocity: [50.0, 0.0, 0.0, 0.0, 0.0, 0.0]
jointAcceleration: [50.0, 0.0, 0.0, 0.0, 0.0, 0.0]
radius: 0.0"
```

### 모바일 로봇 실행
추가 패키지 설치 for kinetic
sudo apt install ros-kinetic-interactive-markers
sudo apt install ros-kinetic-interactive-marker-twist-server
sudo apt install ros-kinetic-twist-mux ros-kinetic-twist-mux-msgs ros-kinetic-robot-localization

추가 패키지 설치 for melodic
sudo apt install ros-melodic-interactive-markers
#sudo apt install ros-melodic-interactive-marker-twist-server (not-exist) need form src install
sudo apt install ros-melodic-twist-mux ros-melodic-twist-mux-msgs ros-melodic-robot-localization

#ros-melodic-interactive-marker-twist-server for source install
cd ~/catkin_ws/src
git clone https://github.com/ros-visualization/interactive_marker_twist_server.git
rosdep install --from-paths ~/catkin_ws/src --ignore-src
cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash
```bash
#roslaunch dsr_launcher multi_gazebo.launch mobile:=true
roslaunch dsr_launcher mobile_m1013.launch
or
roslaunch dsr_launcher mobile_robot.launch model:=m1013 color:=white
rosrun dsr_example_cpp m1013_on_mobile
```

# diagnostic

# etc