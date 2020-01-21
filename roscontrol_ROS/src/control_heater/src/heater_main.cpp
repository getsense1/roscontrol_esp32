#include "control_heater/heater_interface.h"
#include "ros/spinner.h"



int main(int argc, char** argv)
{
  ros::init(argc, argv, "heater_hw_interface");


  _Heater _heater_h;
  g_heater_h = &_heater_h;
  _heater_h.init();

  ros::AsyncSpinner spinner(4);
  spinner.start();

  controller_manager::ControllerManager cm(&_heater_h);


  while (ros::ok())
  {
     //_heater_h.read();
     cm.update(ros::Time::now(), _heater_h.get_period());
     _heater_h.write();
     sleep(1);
  }
}
