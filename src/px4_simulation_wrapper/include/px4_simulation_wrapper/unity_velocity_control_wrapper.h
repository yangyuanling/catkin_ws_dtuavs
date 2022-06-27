#ifndef UNITY_VELOCITY_CONTROL_WRAPPER_H
#define UNITY_VELOCITY_CONTROL_WRAPPER_H
#include "ros/ros.h"
#include "geometry_msgs/TwistStamped.h"
#include "std_msgs/Bool.h"
class unity_velocity_control_wrapper
{
public:
  unity_velocity_control_wrapper();
  void tgt_vel_msg_sub_cb(const geometry_msgs::TwistStampedConstPtr& msg);
  void cmd_reset_time_cb(const ros::TimerEvent& event);
  static void *offboard_run(void *args);
  void network_data_get_cb(const std_msgs::BoolConstPtr& msg);
private:
   ros::Subscriber _tgt_vel_msg_sub;

   ros::Publisher _px4_ref_vel_msg_pub;

   ros::Subscriber _network_data_case_sub;

   ros::Timer _cmd_reset_timer;
   double _cmd_reset_duration;

   bool _is_get_data;

   pthread_t _offboard_control_run;
   float _run_frequen_hz;

   float _ref_velocity_x;//The Target Value of Velocity X
   float _ref_velocity_y;//The Target Value of Velocity Y
   float _ref_velocity_z;//The Target Value of Velocity Z
};

#endif // UNITY_VELOCITY_CONTROL_WRAPPER_H
