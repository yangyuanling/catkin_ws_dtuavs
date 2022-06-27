#ifndef PX4_VELOCITY_CONTROL_WRAPPER_H
#define PX4_VELOCITY_CONTROL_WRAPPER_H
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
#include "geometry_msgs/PoseStamped.h"
#include "dt_message_package/save_image.h"
#include "std_msgs/Bool.h"
class px4_velocity_control_wrapper
{
public:
  px4_velocity_control_wrapper();
  void tgt_vel_msg_sub_cb(const geometry_msgs::TwistStampedConstPtr& msg);
  void px4_state_msg_sub_cb(const mavros_msgs::StateConstPtr& msg);
  void px4_rc_msg_sub_cb(const mavros_msgs::RCInConstPtr& msg);
  void px4_local_pose_sub_cb(const geometry_msgs::PoseStampedConstPtr& msg);
  static void *offboard_run(void *args);
  void cmd_reset_time_cb(const ros::TimerEvent& event);
  void network_data_get_cb(const std_msgs::BoolConstPtr& msg);
private:
  std::string _px4_ref_vel_msg_pub_topic;//The Message will Publish to PX4

  std::string _tgt_vel_msg_sub_topic;//The Message from Other Controller
  std::string _px4_state_msg_sub_topic;//The Message from PX4--about the current state of px4
  std::string _px4_rc_msg_sub_topic;//The Message from PX4--about the rc information of px4

  std::string _px4_arming_client_topic;//The Client Message to PX4 -- arm the vehicle
  std::string _px4_set_mode_client_topic;//The Client Message to PX4 -- set the vehicle mode to offboard control

  ros::Subscriber _tgt_vel_msg_sub;//The Subscriber for Subscribe Other Control Publish Target Velocity Message
  ros::Subscriber _px4_state_msg_sub;//The Subscriber for Subscribe PX4 State
  ros::Subscriber _px4_rc_msg_sub;//The Subscriber for Subscribe RC Information

  ros::Publisher _px4_ref_vel_msg_pub;//The Publisher publish the target velocity to PX4

  ros::Subscriber _px4_local_pos_msg_sub;
  ros::Publisher _local_pos_msg_pub;
  ros::Subscriber _network_data_case_sub;

  ros::ServiceClient _px4_arming_client;//The ServiceClient to Arm the Vehecle
  ros::ServiceClient _px4_set_mode_client;//The ServiceClient to Set the Offboard Control Mode to Vehecle

  ros::ServiceClient _save_image_client;
  bool _is_req_save_image;

  int _rc_start_ch;//The RC Channel for Start Arm Vehecle and Enter Offboard Control Mode
  int _rc_land_ch;//The RC Channel for Land the Vehecle
  int _rc_save_image_ch;

  float _ref_velocity_x;//The Target Value of Velocity X
  float _ref_velocity_y;//The Target Value of Velocity Y
  float _ref_velocity_z;//The Target Value of Velocity Z

  bool _start_offboard_flag;//The Flag for Start Vehicle Offboard Control
  bool _is_land_flag;//The Flag for Land Vehicle
  bool _is_take_off;
  bool _is_first_takeoff;
  bool _is_get_data;

  ros::Timer _cmd_reset_timer;
  double _cmd_reset_duration;

  mavros_msgs::State _px4_current_state;
  mavros_msgs::SetMode _px4_set_mode;
  mavros_msgs::CommandBool _px4_arming;

  bool _is_get_local_pose;
  geometry_msgs::PoseStamped _local_pose;
  geometry_msgs::PoseStamped _start_local_pose;


  geometry_msgs::Twist _px4_ref_vel;


  pthread_t _offboard_control_run;
  float _run_frequen_hz;


};

#endif // PX4_VELOCITY_CONTROL_WRAPPER_H












