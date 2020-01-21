#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_state_interface.h>
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"


#define  ON_HEATER  0.1
#define  OFF_HEATER 0.2

#define BILLION (1000000000.0)



class _Heater : public hardware_interface::RobotHW
{
public:
  /**
   * \brief Constructor
   * \param nh - Node handle for topics.
   * \param urdf - optional pointer to a parsed robot model
   */
  _Heater();

  /** \brief Destructor */
  virtual ~_Heater() {}

  /** \brief Initialize the hardware interface */
  virtual void init();

  /** \brief Read the state from the robot hardware. */
  virtual void read();

  /** \brief Write the command to the robot hardware. */
  virtual void write();

  /** \brief Set all members to default values */
  virtual void reset();

  /**
   * \brief Check (in non-realtime) if given controllers could be started and stopped from the
   * current state of the RobotHW
   * with regard to necessary hardware interface switches. Start and stop list are disjoint.
   * This is just a check, the actual switch is done in doSwitch()
   */
  virtual bool canSwitch(const std::list<hardware_interface::ControllerInfo> &start_list,
                         const std::list<hardware_interface::ControllerInfo> &stop_list) const
  {
    return true;
  }

  /**
   * \brief Perform (in non-realtime) all necessary hardware interface switches in order to start
   * and stop the given controllers.
   * Start and stop list are disjoint. The feasability was checked in canSwitch() beforehand.
   */
  virtual void doSwitch(const std::list<hardware_interface::ControllerInfo> &start_list,
                        const std::list<hardware_interface::ControllerInfo> &stop_list)
  {
  }

  ros::Duration get_period(void);


protected:

  // Short name of this class
  std::string name_;

  // Startup and shutdown of the internal node inside a roscpp program
  ros::NodeHandle nh;
  ros::Publisher _heater_pub;
  ros::Subscriber _heater_sub;



  hardware_interface::JointStateInterface js_heater_interface;
  hardware_interface::PositionJointInterface pj_heater_interface;

  double cmd;
  double pos;
  double vel;
  double eff;


  struct timespec last_time_;
  struct timespec current_time_;

  static void _heater_subCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
};


extern _Heater *g_heater_h;
