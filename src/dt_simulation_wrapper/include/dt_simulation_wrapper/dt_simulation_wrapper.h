#ifndef DT_SIMULATION_WRAPPER_H
#define DT_SIMULATION_WRAPPER_H
#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <dt_message_package/uavs_pose_vel.h>
#include "pthread.h"
#include <dt_message_package/CloudMessage.h>
#include <dt_common/define_common.h>
using namespace DTUAV;
class dt_simulation_wrapper
{
public:
  dt_simulation_wrapper();
  static void *run(void *args);
  void uav1_local_pos_sub_cb(const geometry_msgs::PoseStampedConstPtr& msg);
  void uav1_vel_info_sub_cb(const geometry_msgs::TwistStampedConstPtr& msg);

  void uav2_local_pos_sub_cb(const geometry_msgs::PoseStampedConstPtr& msg);
  void uav2_vel_info_sub_cb(const geometry_msgs::TwistStampedConstPtr& msg);

  void uav3_local_pos_sub_cb(const geometry_msgs::PoseStampedConstPtr& msg);
  void uav3_vel_info_sub_cb(const geometry_msgs::TwistStampedConstPtr& msg);

  void uav4_local_pos_sub_cb(const geometry_msgs::PoseStampedConstPtr& msg);
  void uav4_vel_info_sub_cb(const geometry_msgs::TwistStampedConstPtr& msg);

  void uav5_local_pos_sub_cb(const geometry_msgs::PoseStampedConstPtr& msg);
  void uav5_vel_info_sub_cb(const geometry_msgs::TwistStampedConstPtr& msg);

  void uav6_local_pos_sub_cb(const geometry_msgs::PoseStampedConstPtr& msg);
  void uav6_vel_info_sub_cb(const geometry_msgs::TwistStampedConstPtr& msg);

  void target_local_pos_sub_cb(const geometry_msgs::PoseStampedConstPtr& msg);
  void target_vel_info_sub_cb(const geometry_msgs::TwistStampedConstPtr& msg);

  void cloudMsgCb(const dt_message_package::CloudMessageConstPtr& msg);

private:
  ros::Subscriber _uav1_pose_sub;
  ros::Subscriber _uav2_pose_sub;
  ros::Subscriber _uav3_pose_sub;
  ros::Subscriber _uav4_pose_sub;
  ros::Subscriber _uav5_pose_sub;
  ros::Subscriber _uav6_pose_sub;
  ros::Subscriber _target_pose_sub;

  ros::Subscriber _uav1_vel_sub;
  ros::Subscriber _uav2_vel_sub;
  ros::Subscriber _uav3_vel_sub;
  ros::Subscriber _uav4_vel_sub;
  ros::Subscriber _uav5_vel_sub;
  ros::Subscriber _uav6_vel_sub;
  ros::Subscriber _target_vel_sub;

  ros::Subscriber _cloudMsgSub;

  ros::Publisher _uavs_pos_vel_pub;

  ros::Publisher _uav1_pose_pub;
  ros::Publisher _uav2_pose_pub;
  ros::Publisher _uav3_pose_pub;
  ros::Publisher _uav4_pose_pub;
  ros::Publisher _uav5_pose_pub;
  ros::Publisher _uav6_pose_pub;
  ros::Publisher _target_pose_pub;


  geometry_msgs::PoseStamped _uav1_pose;
  geometry_msgs::PoseStamped _uav2_pose;
  geometry_msgs::PoseStamped _uav3_pose;
  geometry_msgs::PoseStamped _uav4_pose;
  geometry_msgs::PoseStamped _uav5_pose;
  geometry_msgs::PoseStamped _uav6_pose;
  geometry_msgs::PoseStamped _target_pose;

  geometry_msgs::TwistStamped _uav1_vel;
  geometry_msgs::TwistStamped _uav2_vel;
  geometry_msgs::TwistStamped _uav3_vel;
  geometry_msgs::TwistStamped _uav4_vel;
  geometry_msgs::TwistStamped _uav5_vel;
  geometry_msgs::TwistStamped _uav6_vel;
  geometry_msgs::TwistStamped _target_vel;

  pthread_t _runThread;
  float _msgPubHz;
  int _mode;

  bool _isPubTarget;

};

#endif // DT_SIMULATION_WRAPPER_H














