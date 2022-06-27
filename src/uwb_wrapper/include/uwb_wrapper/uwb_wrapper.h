#ifndef UWB_WRAPPER_H
#define UWB_WRAPPER_H
#include "ros/ros.h"
#include "iostream"
#include "geometry_msgs/PoseStamped.h"
#include "nlink_parser/LinktrackNodeframe2.h"
#include "sensor_msgs/ChannelFloat32.h"
#include "sensor_msgs/Range.h"
#include "pthread.h"
using namespace std;
class uwb_wrapper
{
public:
  uwb_wrapper();
  void uwbInformationCallback(const nlink_parser::LinktrackNodeframe2ConstPtr& msg);
  void distanceMsgCallback(const sensor_msgs::RangeConstPtr& msg);
  void localPosMsgCallback(const geometry_msgs::PoseStampedConstPtr& msg);
  static void *run(void *args);
private:
  string _uwbInforSubTopic;
  string _uwbPosPubTopic;
  string _uwbDistanceTopic;
  string _distanceMsgTopic;
  string _localPosMsgTopic;

  float _uwbInstallHeiht;
  float _distanceHeiht;
  float _lastDistanceHeight;
  float _error;
  bool _isLocalPositionModel;
  bool _isInit;
  bool _isHeightInit;
  double _homePositionX;
  double _homePositionY;
  double _homePositionZ;

  ros::Subscriber _uwbInforSub;
  ros::Subscriber _distanceMsgSub;
  ros::Subscriber _localPoseMsgSub;
  ros::Publisher _uwbPosPub;
  ros::Publisher _uwbDistancePub;

  geometry_msgs::Quaternion _rotation;
  geometry_msgs::PoseStamped _pose_msg;
  pthread_t run_thread;
  float _poseHz;

};

#endif // UWB_WRAPPER_H
