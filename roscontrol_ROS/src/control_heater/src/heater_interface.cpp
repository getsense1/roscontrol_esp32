
#include "control_heater/heater_interface.h"
#include <string>
 #include <boost/thread/mutex.hpp>

_Heater *g_heater_h;
static boost::mutex mutex_;

_Heater::_Heater()
  : name_("heater_hw_interface") {
}

void _Heater::init(){

  ROS_INFO_STREAM_NAMED(name_, "Loading joint handle: heater_0");

  hardware_interface::JointStateHandle   _jointstatehandle("heater_hw_interface", &pos, &vel, &eff);
  js_heater_interface.registerHandle(_jointstatehandle);

  hardware_interface::JointHandle cmd_handle_h0(_jointstatehandle, &cmd);
  pj_heater_interface.registerHandle(cmd_handle_h0);

  registerInterface(&js_heater_interface);
  registerInterface(&pj_heater_interface);


  reset();

  _heater_pub = nh.advertise<std_msgs::Float64>("cmd_heater_tx", 1000);
  _heater_sub = nh.subscribe("cmd_heater_rx", 1000, _heater_subCallback);



  ROS_INFO_STREAM_NAMED(name_, "->Ready");
}


void _Heater::reset(){
  cmd=OFF_HEATER;
}

void _Heater::read(void){
  ros::spinOnce();
}

void _Heater::write(void){
std_msgs::Float64 _msg0;
boost::mutex::scoped_lock lock(mutex_);

    _msg0.data = cmd;
    _heater_pub.publish(_msg0);

    ROS_INFO_STREAM_NAMED("_cmd_heater_tx", "send Float64 msg->"+boost::lexical_cast<std::string>(cmd));

}

ros::Duration _Heater::get_period(){

    clock_gettime(CLOCK_MONOTONIC, &current_time_);
    ros::Duration elapsed_time_ = ros::Duration(current_time_.tv_sec - last_time_.tv_sec, current_time_.tv_nsec - last_time_.tv_nsec);
    last_time_ = current_time_;

    return elapsed_time_;
}


void _Heater::_heater_subCallback(const std_msgs::Float64MultiArray::ConstPtr& msg){

boost::mutex::scoped_lock lock(mutex_);

    g_heater_h->pos = msg->data[0];
    g_heater_h->vel = msg->data[1];
    g_heater_h->eff = msg->data[2];

    ROS_INFO_STREAM_NAMED("_cmd_heater_rx", "get Float64Array msg->"+boost::lexical_cast<std::string>(g_heater_h->pos));
}



