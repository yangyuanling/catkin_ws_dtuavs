#ifndef PX4_GET_DATA_SET_WRAPPER_H
#define PX4_GET_DATA_SET_WRAPPER_H
#include <pthread.h>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistStamped.h"
#include "mavros_msgs/RCIn.h"
#include "mavros_msgs/State.h"
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <std_msgs/Bool.h>
#include "ros/ros.h"
#include "iostream"

class px4_get_data_set_wrapper
{
public:
  px4_get_data_set_wrapper();
  void tgt_vel_msg_sub_cb(const geometry_msgs::TwistStampedConstPtr& msg);
  void px4_state_msg_sub_cb(const mavros_msgs::StateConstPtr& msg);
  void px4_rc_msg_sub_cb(const mavros_msgs::RCInConstPtr& msg);
  static void *offboard_run(void *args);
private:
  std::string _px4_ref_vel_msg_pub_topic;//The Message will Publish to PX4
  std::string _start_mission_msg_pub_topic;//The Message will Publish to Mission Points
  std::string _chage_mission_msg_pub_topic;//The Message will Publish to Mission Points

  std::string _tgt_vel_msg_sub_topic;//The Message from Other Controller
  std::string _px4_state_msg_sub_topic;//The Message from PX4--about the current state of px4
  std::string _px4_rc_msg_sub_topic;//The Message from PX4--about the rc information of px4

  std::string _px4_arming_client_topic;//The Client Message to PX4 -- arm the vehicle
  std::string _px4_set_mode_client_topic;//The Client Message to PX4 -- set the vehicle mode to offboard control

  ros::Subscriber _tgt_vel_msg_sub;//The Subscriber for Subscribe Other Control Publish Target Velocity Message
  ros::Subscriber _px4_state_msg_sub;//The Subscriber for Subscribe PX4 State
  ros::Subscriber _px4_rc_msg_sub;//The Subscriber for Subscribe RC Information

  ros::Publisher _px4_ref_vel_msg_pub;//The Publisher publish the target velocity to PX4
  ros::Publisher _start_mission_msg_pub;//The Publisher Publish the Mission Start to Mission Points
  ros::Publisher _change_mission_msg_pub;//The Publisher Publish the Change Mission to Mission Points

  ros::ServiceClient _px4_arming_client;//The ServiceClient to Arm the Vehecle
  ros::ServiceClient _px4_set_mode_client;//The ServiceClient to Set the Offboard Control Mode to Vehecle

  int _rc_start_ch;//The RC Channel for Start Arm Vehecle and Enter Offboard Control Mode
  int _rc_land_ch;//The RC Channel for Land the Vehecle
  int _rc_start_mission_ch;//The RC Channel for Start Mission
  int _rc_change_mission_ch;//The RC Channel for Change Mission

  float _ref_velocity_x;//The Target Value of Velocity X
  float _ref_velocity_y;//The Target Value of Velocity Y
  float _ref_velocity_z;//The Target Value of Velocity Z

  bool _start_offboard_flag;//The Flag for Start Vehicle Offboard Control
  bool _is_land_flag;//The Flag for Land Vehicle

  mavros_msgs::State _px4_current_state;
  mavros_msgs::SetMode _px4_set_mode;
  mavros_msgs::CommandBool _px4_arming;

  geometry_msgs::Twist _px4_ref_vel;


  pthread_t _offboard_control_run;
  float _run_frequen_hz;

  bool _is_change;


};

#endif // PX4_GET_DATA_SET_WRAPPER_H
