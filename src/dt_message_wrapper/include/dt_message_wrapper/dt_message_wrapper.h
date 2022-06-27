#ifndef DT_MESSAGE_WRAPPER_H
#define DT_MESSAGE_WRAPPER_H
#include "ros/ros.h"
#include "iostream"
#include <dt_common/define_common.h>
#include "sensor_msgs/NavSatFix.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include "mavros_msgs/State.h"
#include "mavros_msgs/GlobalPositionTarget.h"
#include "sensor_msgs/BatteryState.h"
#include "x2struct/x2struct.hpp"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Int8.h"
#include <dt_message_package/CloudMessage.h>
#include <dt_message_package/uavs_pose_vel.h>
#include <dt_message_package/NetworkStateMsg.h>
#include "pthread.h"
#include <mutex>
#include <tf/tf.h>
#include <unordered_set>
using namespace DTUAV;

typedef struct
{
  int id;
  float pos_x;
  float pos_y;
  float pos_z;
  float vel_x;
  float vel_y;
  float vel_z;
} UavStateInfo;


class dt_message_wrapper
{
public:
  dt_message_wrapper();
  static void *run(void *args);
  static void *run_pub_heart_msg(void *args);
  void local_pos_sub_cb(const geometry_msgs::PoseStampedConstPtr& msg);
  void global_pos_sub_cb(const sensor_msgs::NavSatFixConstPtr& msg);
  void battery_info_sub_cb(const sensor_msgs::BatteryStateConstPtr& msg);
  void state_info_sub_cb(const mavros_msgs::StateConstPtr &msg);
  void vel_info_sub_cb(const geometry_msgs::TwistStampedConstPtr& msg);
  void cloud_msg_cb(const dt_message_package::CloudMessageConstPtr& msg);
private:
  ros::Subscriber _local_pos_sub;
  ros::Subscriber _global_pos_sub;
  ros::Subscriber _battery_info_sub;
  ros::Subscriber _state_info_sub;
  ros::Subscriber _vel_info_sub;
  ros::Subscriber _cloud_msg_sub;

  ros::Publisher _cloud_msg_pub;
  ros::Publisher _target_vel_pub;
  ros::Publisher _target_pos_pub;
  ros::Publisher _arm_com_pub;
  ros::Publisher _target_fmode_pub;
  ros::Publisher _vr_control_pub;
  ros::Publisher _computer_cmd_pub;
  ros::Publisher _apply_cam_pub;
  ros::Publisher _other_uavs_state_pub;
  ros::Publisher _network_state_pub;
  ros::Publisher _target_obj_pos_pub;

  std::unordered_set<int> _updatedUavsId;
  std::vector<UavStateInfo> _otherUavsState;
  bool _isAllUpdate;
  int _numStateUpdate;
  int _uavNum;
  bool _isFirstSend;

  bool _isStart;
  bool _isLinkServer;
  std::vector<bool> _isLinkUavs;

  int _sourceID;
  int _targetID;
  int _dtObjectID;
  pthread_t _runPubHeartTh;
  float _heartMsgHz;

  pthread_t _runThread;
  float _msgPubHz;
  std::mutex m;

  float _startPosX;
  float _startPosY;
  float _startPosZ;

  float _curPosX;
  float _curPosY;
  float _curPosZ;

  //The Position, Rotation, Linear Velocity, Angular Velocity
  double _latitude;
  double _longitude;
  double _altitude;
  double _lposX;
  double _lposY;
  double _lposZ;
  double _rotX;
  double _rotY;
  double _rotZ;
  double _rotW;
  double _lVelX;
  double _lVelY;
  double _lVelZ;
  double _aVelX;
  double _aVelY;
  double _aVelZ;
  //The Safe Checking
  bool _netPx4;//the connective of the computer and PX4
  //The Status of UAV
  bool _isArm;
  /*
         * 0: the manual mode
         * 1: the stabilizing mode
         * 2: the altitude mode
         * 3: the position mode
         * 4: the offboard mode
         * 5: the return mode
         * -1: the undefined mode
         */
  int _fMode;//the flight mode of UAV:
  float _voltage;//the current voltage of UAV
  float _remaining;//the remaining of battery
};

#endif // DT_MESSAGE_WRAPPER_H
