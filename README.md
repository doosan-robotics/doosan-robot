# [Doosan robotics](http://www.doosanrobotics.com/kr/)

# *overview*

[Doosan ROS Video](https://www.youtube.com/watch?v=mE24X5PhZ4M&feature=youtu.be)

# *build* 
##### *Doosan Robot ROS Package is implemented at ROS-Kinetic.*
    ### We recoomand the /home/<user_home>/catkin_ws/src
    cd ~/catkin_ws/src
    git clone https://github.com/doosan-robotics/doosan-robot
    rosdep install --from-paths doosan-robot --ignore-src --rosdistro kinetic -r -y
    cd ~/catkin_ws
    catkin_make
    source ./devel/setup.bash

#### package list
    sudo apt-get install ros-kinetic-rqt* ros-kinetic-moveit* ros-kinetic-industrial-core ros-kinetic-gazebo-ros-control ros-kinetic-joint-state-controller ros-kinetic-effort-controllers ros-kinetic-position-controllers ros-kinetic-ros-controllers ros-kinetic-ros-control ros-kinetic-serial
    
__packages for mobile robot__

    sudo apt-get ros-kinetic-lms1xx ros-kinetic-interactive-marker-twist-server ros-kinetic-twist-mux ros-kinetic-imu-tools ros-kinetic-controller-manager ros-kinetic-robot-localization


# *usage* <a id="chapter-3"></a>
#### DRCF Emulator
If you don`t have real doosan controller, you must excute our emulator. 
Emulator has local IP(127.0.0.1) default port=12345. 
```bash
cd ~/catkin_ws/src/doosan-robot/common/bin/DRCF
./DRCF64 <port>   ## 64bits OS
or 
./DRCF32 <port>   ## 32bits OS
``` 

> _$ ./DRCF64 12345_

> <img src="https://user-images.githubusercontent.com/47092672/63494967-dc186d80-c4f9-11e9-967d-519656457499.jpg" width="80%">


#### dsr_description
```bash
roslaunch dsr_description m0609.launch    
roslaunch dsr_description m1013.launch color:=blue # Change Color
roslaunch dsr_description m1509.launch gripper:=robotiq_2f # insert robotiq gripper
roslaunch dsr_description m0617.launch color:=blue gripper:=robotiq_2f # change color & insert robotiq gripper
```

> $ _roslaunch dsr_description m1013.launch_ 
<img src="https://user-images.githubusercontent.com/47092672/55622394-0f708f00-57db-11e9-8625-a344513a5d3a.png" width="70%">

> + In dsr_description, the user can use joint_state_publisher to move the robot.
> + [Joint_state_publisher](http://wiki.ros.org/joint_state_publisher)

> $ _roslaunch dsr_description m0617.launch color:=blue gripper:=robotiq_2f_ 

<img src="https://user-images.githubusercontent.com/47092672/55624467-f7037300-57e0-11e9-930a-ec929de3a0fa.png" width="70%">


#### dsr_moveit_config
> ###### __arguments__
   > color:= ROBOT_COLOR <white / blue> defalut = white  
    
    roslaunch moveit_config_m0609 m0609.launch
    roslaunch moveit_config_m0617 m0617.launch
    roslaunch moveit_config_m1013 m1013.launch color:=blue
    roslaunch moveit_config_m1509 m1509.launch
    
    
<img src="https://user-images.githubusercontent.com/47092672/55613994-fd84f100-57c6-11e9-97eb-49d1d7c9e32c.png" width="70%">

##### *How to use MoveIt Commander*
###### _You can run Moveit with CLI commands through the moveit commander package._
###### _You can install the "moveit_commander" package using below command._
    sudo apt-get insatll ros-kinetic-moveit-commander
##### *MoveitCommander usage example*
	ROS_NAMESPACE=/dsr01m1013 rosrun moveit_commander moveit_commander_cmdline.py robot_description:=/dsr01m1013/robot_description   
###### *moveit commander CLI is executed.*    
    > use arm 
    > goal0 = [0 0 0 0 0 0]        # save the home position to variable "goal0"
    > goal1 = [0 0 1.57 0 1.57 0]  # save the target position to varialbe "goal1" / radian
    > go goal1                     # plan & excute (the robot is going to move target position)
    > go goal0                     # paln & excute (the robot is going to move home position)

#### dsr_control _(default model:= m1013, default mode:= virtual)_
> ###### __arguments__                    
>host := ROBOT_IP defalut = 192.168.127.100   
port := ROBOT_PORT default = 12345  
mode := OPERATION MODE <virtual  /  real> defalut = virtual  
model := ROBOT_MODEL <m0609  /  0617/  m1013  /  m1509> defalut = m1013  
color := ROBOT_COLOR <white  /  blue> defalut = white  
gripper := USE_GRIPPER <none  /  robotiq_2f> defalut = none  
mobile := USE_MOBILE <none  /  husky> defalut = none  

#### dsr_control + dsr_moveit_config
    roslaunch dsr_control dsr_moveit.launch
    roslaunch dsr_control dsr_moveit.launch model:=m0609 mode:=virtual
    roslaunch dsr_control dsr_moveit.launch model:=m0617 mode:=virtual
    roslaunch dsr_control dsr_moveit.launch model:=m1013 mode:=virtual
    roslaunch dsr_control dsr_moveit.launch model:=m1509 mode:=virtual
      
#### dsr_launcher

__If you don`t have real doosan controller, you must execute emulator before run dsr_launcer.__
> ###### __arguments__    
   >host:= ROBOT_IP defalut = 192.168.127.100  ##Emulator IP = 127.0.0.1   
    port:= ROBOT_PORT default = 12345  
    mode:= OPERATION MODE <virtual  /  real> defalut = virtual  
    model:= ROBOT_MODEL <m0609  /  0617/  m1013  /  m1509> defalut = m1013  
    color:= ROBOT_COLOR <white  /  blue> defalut = white  
    gripper:= USE_GRIPPER <none  /  robotiq_2f> defalut = none  
    mobile:= USE_MOBILE <none  /  husky> defalut = none  

    roslaunch dsr_launcher single_robot_rviz.launch host:=127.0.0.1 port:=12345 mode:=virtual model:=m1013 color:=blue gripper:=none mobile:=none
    roslaunch dsr_launcher single_robot_gazebo.launch host:=192.168.127.100
    roslaunch dsr_launcher single_robot_rviz_gazebo.launch gripper:=robotiq_2f mobile:=husky
    roslaunch dsr_launcher multi_robot_rviz.launch
    roslaunch dsr_launcher multi_robot_gazebo.launch model:=m0609
    roslaunch dsr_launcher multi_robot_rviz_gazebo.launch
    
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


###### multi robot
    <launch>
      - multi robot in rviz : 
      roslaunch dsr_launcher multi_robot_rviz.launch
      - multi robot in gazebo : 
      roslaunch dsr_launcher multi_robot_gazebo.launch
      - multi robot in rviz + gazebo : 
      roslaunch dsr_launcher multi_robot_rviz_gazebo.launch
    <run application node>
      rosrun dsr_example_py multi_robot_simple.py
    <ex>
        roslaunch dsr_launcher multi_robot_rviz_gazebo.launch
        rosrun dsr_example_py multi_robot.py  

> _$ roslaunch dsr_launcher multi_robot_rviz_gazebo.launch_

> _$ rosrun dsr_example_py multi_robot.py_
> <img src="https://user-images.githubusercontent.com/47092672/55622398-10092580-57db-11e9-8a23-b9dae4131897.png" width="70%">

###### robot + gripper
> insert argument gripper:=robotiq_2f  
- single robot + gripper
```bash
roslaunch dsr_launcher single_robot_rviz.launch gripper:=robotiq_2f

<run application node>
  rosrun dsr_example_py pick_and_place.py
```
- Serial Test(Loopback)
```bash
rosrun serial_example_node serial_example_node ttyUSB0 115200
rostopic echo /serial_read
rostopic pub /serial_write std_msgs/String 'data: 100'
```



###### robot + mobile
> insert argument mobile:=husky
- single robot on mobile
```bash
roslaunch dsr_launcher single_robot_rviz.launch mobile:=husky
  
<run application node>
  rosrun dsr_example_py single_robot_mobile.py
```

> _$ roslaunch dsr_launcher single_robot_rviz mobile:=husky color:=blue_  
> <img src="https://user-images.githubusercontent.com/47092672/55622399-10092580-57db-11e9-9ee0-f3c04a5569de.png" width="70%">

- multi robot on mobile
```bash
roslaunch dsr_launcher multi_robot_rviz.launch mobile:=husky

<run application node>
  rosrun dsr_example_py multi_robot_mobile.py  
```

> _$ roslaunch dsr_launcher multi_robot_rviz mobile:=husky_
> <img src="https://user-images.githubusercontent.com/47092672/55622397-10092580-57db-11e9-8fe8-4d711725ac45.png" width="70%">

    
#### gazebo+rviz+virtual
    roslaunch dsr_launcher single_robot_rviz_gazebo.launch
    rosrun dsr_example_py single_robot_simple.py
```bash
  <include file="$(find dsr_gazebo)/launch/dsr_base.launch">
    <arg name="ns" value="dsr01"/> # Robot ID
    <arg name="model" value="m1013"/> # Robot Model
    <arg name="host" value="192.168.127.100"/> # Robot IP
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
    <arg name="host" value="192.168.127.102"/> # Secondary Robot IP
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
# manuals

[Manual(Kor)](http://wiki.ros.org/doosan-robotics?action=AttachFile&do=get&target=Doosan_Robotics_ROS_Manual_ver0.96_191121A%28Kor.pdf)


[Manual(Eng)](http://wiki.ros.org/doosan-robotics?action=AttachFile&do=get&target=Doosan_Robotics_ROS_Manual_ver0.96_191121A%28EN.PDF)

# demo

### Doosan-Robots In Gazebo

<img src="https://user-images.githubusercontent.com/47092672/55624381-9f650780-57e0-11e9-80aa-0f26ec528987.png" width="80%">

### Doosan-Robots & Mobile in Rviz 

<img src="https://user-images.githubusercontent.com/47092672/55624380-9ecc7100-57e0-11e9-8854-f6d8ca3561e7.png" width="80%">
