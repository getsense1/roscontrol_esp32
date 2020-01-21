## roscontrol_esp32, tested at the Kinetic on Ubuntu Trusty 14.04 LTS


This [roscontrol](http://wiki.ros.org/ros_control) that includes controller interfaces, 
controller managers and hardware_interfaces to joint on ESP32 board 



```
how work
git clone https://github.com/getsense1/roscontrol_esp32
cd ./rosserial_esp32, read README.md, install firmware to ESP32 board

cd ./roscontrol_ROS
this is workspace, 
set ROS environment: source /opt/ros/kinetic/setup.bash
make package: control_heater, node: control_heater_node
run node "control_heater_node": roslaunch control_heater control_heater.launch
get joint state(using JointStatesController->JointStateInterface): rostopic echo /control_heater/joint_states
set joint command(using JointPositionController->PositionJointInterface): rostopic pub /control_heater/heater_cmd_controller/command  std_msgs/Float64 5.98

src comments:
- JointStateInterface js_heater_interface, it was registered in the ROS JointStatesController
- PositionJointInterface pj_heater_interface, it was registered in the ROS JointPositionController
- ros::Publisher _heater_pub, send command to rosserial node(joint ESP32 board)
- ros::Subscriber _heater_sub, get state from rosserial node(joint ESP32 board)
- heater_main.cpp:
	register interfaces in controller_manager::ControllerManager 
	ros::AsyncSpinner run Subscriber callback function(get states)
	send command to rosserial node 
```

