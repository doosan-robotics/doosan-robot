[doosan robotics](http://www.doosanrobotics.com/kr/)
================

### build
1. mkdir -p dr_ws/src & cd dr_ws/src
2. git clone or download & unzip
3. cd .. 
4. rosdep install --from-paths rs --ignore-src --rosdistro kinetic -r -y
5. catkin_make
6. source ./devel/setup.bash 

### usage
1. <dsr_description>
    roslaunch dsr_description m0609.launch
    roslaunch dsr_description m0617.launch
    roslaunch dsr_description m1013.launch
    roslaunch dsr_description m1509.launch

2. <dsr_moveit_config>
    #old roslaunch dsr_moveit_config m0609.launch
    #old roslaunch dsr_moveit_config m0617.launch
    #old roslaunch dsr_moveit_config m1013.launch
    #old roslaunch dsr_moveit_config m1509.launch
    roslaunch dsr_control dsr_moveit.launch model:=m0609
    roslaunch dsr_control dsr_moveit.launch model:=m0617
    roslaunch dsr_control dsr_moveit.launch model:=m1013
    roslaunch dsr_control dsr_moveit.launch model:=m1509

3. <dsr_control> (default model:= m1013, default mode:= virtual)
    (1) dsr_control + dsr_description
      roslaunch dsr_control dsr_control.launch 
      roslaunch dsr_control dsr_control.launch model:=m0609 mode:=virtual
      roslaunch dsr_control dsr_control.launch model:=m0617 mode:=virtual
      roslaunch dsr_control dsr_control.launch model:=m1013 mode:=virtual
      roslaunch dsr_control dsr_control.launch model:=m1509 mode:=virtual
    (2) dsr_control + dsr_moveit_config
      roslaunch dsr_control dsr_moveit.launch
      roslaunch dsr_control dsr_moveit.launch model:=m0609 mode:=virtual
      roslaunch dsr_control dsr_moveit.launch model:=m0617 mode:=virtual 
      roslaunch dsr_control dsr_moveit.launch model:=m1013 mode:=virtual
      roslaunch dsr_control dsr_moveit.launch model:=m1509 mode:=virtual

4. <dsr_example>  
  - rosrun dsr_example_cpp dsr_servie_motion_basic
  - rosrun dsr_example_py dsr_servie_motion_basic.py
  - roslaunch dsr_example multi_gazebo.launch
  - roslaunch dsr_example multi_virtual.launch
  - roslaunch dsr_example multi_real.launch

5. <dsr_apps>  
  - rosrun dsr_apps_cpp app_watch
  - rosrun dsr_apps_cpp app_watch.py

6. <dsr_test>
  - rosrun dsr_test_cpp dsr_test

### gazebo+rviz+virtual(dsr)함께 실행 하는 방법 
1. roslaunch dsr_example test.launch 
2. rosrun dsr_test_cpp dsr_test
```bash 
  <include file="$(find dsr_gazebo)/launch/dsr_base.launch">
    <arg name="ns"            value="dsr01"/> 고유 ID
    <arg name="model"         value="m1013"/> 로봇 모델 
    <arg name="host"          value="192.168.137.100"/> 로봇 제어기 IP주소
    <arg name="mode"          value="virtual"/> 로봇 제어기 모드 
    가제보에서의 위치와 자세 
    <arg name="x"             value="2"/> 
    <arg name="y"             value="-4"/>
    <arg name="yaw"           value="0.7"/>
  </include>
  <include file="$(find dsr_gazebo)/launch/dsr_base.launch">
    <arg name="ns"            value="dsr02"/> 두번째 고유 ID
    <arg name="model"         value="m1013"/> 두번째 로봇 모델 
    <arg name="host"          value="192.168.137.102"/> 두번째 로봇 제어기 IP주소
    <arg name="mode"          value="virtual"/> 두번째 로봇 제어기 모드
    두번째 위치와 자세  
    <arg name="x"             value="2"/> 
    <arg name="y"             value="-4"/>
    <arg name="yaw"           value="0.7"/>
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

# diagnostic

# etc

