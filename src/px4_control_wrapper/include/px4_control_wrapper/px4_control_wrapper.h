#ifndef PX4_CONTROL_WRAPPER_H
#define PX4_CONTROL_WRAPPER_H
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
#include "std_msgs/Int8.h"
class px4_control_wrapper
{
public:
  px4_control_wrapper();
  void tgt_vel_msg_sub_cb(const geometry_msgs::TwistStampedConstPtr& msg);
  void tgt_pos_msg_sub_cb(const geometry_msgs::PoseStampedConstPtr& msg);
  void px4_rc_msg_sub_cb(const mavros_msgs::RCInConstPtr& msg);
  void data_valid_sub_cb(const std_msgs::BoolConstPtr& msg);
  void uav_pos_msg_sub_cb(const geometry_msgs::PoseStampedConstPtr& msg);
  void vr_control_msg_sub_cb(const std_msgs::BoolConstPtr& msg);
  void arm_msg_sub_cb(const std_msgs::BoolConstPtr& msg);
  void px4_state_msg_sub_cb(const mavros_msgs::StateConstPtr& msg);
  void flight_mode_msg_sub_cb(const std_msgs::Int8ConstPtr& msg);
  static void *offboard_run(void *args);
  static void *auto_run(void *args);
private:
  std::string _px4_ref_vel_msg_pub_topic;//The Message will Publish to PX4
  std::string _tgt_vel_msg_sub_topic;//The Message from Other Controller
  std::string _px4_rc_msg_sub_topic;//The Message from PX4--about the rc information of px4
  std::string _data_valid_sub_topic;

  bool _data_valid;
  bool _is_check_data_valid;
  bool _is_rc_control;

  ros::Subscriber _data_valid_sub;
  ros::Subscriber _tgt_vel_msg_sub;//The Subscriber for Subscribe Other Control Publish Target Velocity Message
  ros::Subscriber _tgt_pos_msg_sub;

  ros::Subscriber _px4_rc_msg_sub;//The Subscriber for Subscribe RC Information
  ros::Subscriber _uav_pos_msg_sub;

  ros::Subscriber _arm_com_sub;
  ros::Subscriber _target_fmode_sub;
  ros::Subscriber _vr_control_sub;
  ros::Subscriber _computer_cmd_sub;
  ros::Subscriber _px4_state_msg_sub;//The Subscriber for Subscribe PX4 State
  ros::Subscriber _flight_mode_msg_sub;

  ros::Publisher _uav_local_msg_pub;
  ros::Publisher _px4_ref_vel_msg_pub;//The Publisher publish the target velocity to PX4
  ros::Publisher _px4_ref_pos_msg_pub;

  ros::ServiceClient _px4_arming_client;//The ServiceClient to Arm the Vehecle
  ros::ServiceClient _px4_set_mode_client;//The ServiceClient to Set the Offboard Control Mode to Vehecle

  mavros_msgs::State _px4_current_state;
  mavros_msgs::SetMode _px4_set_mode;
  mavros_msgs::CommandBool _px4_arming;

  int _rc_start_ch;//The RC Channel for Start Arm Vehecle and Enter Offboard Control Mode
  int _rc_land_ch;//The RC Channel for Land the Vehecle

  float _ref_velocity_x;//The Target Value of Velocity X
  float _ref_velocity_y;//The Target Value of Velocity Y
  float _ref_velocity_z;//The Target Value of Velocity Z

  float _ref_pos_x;
  float _ref_pos_y;
  float _ref_pos_z;

  float _ref_rot_x;
  float _ref_rot_y;
  float _ref_rot_z;
  float _ref_rot_w;

  bool _is_land_flag;//The Flag for Land Vehicle
  bool _is_start_other_control_flag;
  bool _is_local_pos;
  bool _initStartPos;
  bool _reset_start_pos;
  bool _is_set_rotation;

  float _start_pos_x;
  float _start_pos_y;
  float _start_pos_z;

  float _current_height;

  int _control_mode;



  geometry_msgs::Twist _px4_ref_vel;
  geometry_msgs::PoseStamped _px4_ref_pos;


  pthread_t _offboard_control_run;
  pthread_t _auto_run;
  bool _is_start_thread;

  float _run_frequen_hz;

  bool _is_set_start;
  bool _is_start_vr_control;
  bool _is_vr_control;
};

#endif // PX4_CONTROL_WRAPPER_H
