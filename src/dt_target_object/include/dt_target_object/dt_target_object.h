#ifndef DT_TARGET_OBJECT_H
#define DT_TARGET_OBJECT_H
#include "ros/ros.h"
#include "iostream"
#include <dt_common/define_common.h>
#include <geometry_msgs/PoseStamped.h>
#include "x2struct/x2struct.hpp"
#include <dt_message_package/CloudMessage.h>
#include "pthread.h"
#include <mutex>
#include <geometry_msgs/TwistStamped.h>
#include "../dt_target_object/matplotlibcpp.h"
#include <dt_message_package/uavs_pose_vel.h>
#include "boost/circular_buffer.hpp"
using namespace DTUAV;
namespace plt = matplotlibcpp;
typedef struct{
  boost::circular_buffer<float> xs;
  boost::circular_buffer<float> ys;
  boost::circular_buffer<float> zs;
  boost::circular_buffer<float> vxs;
  boost::circular_buffer<float> vys;
  boost::circular_buffer<float> vzs;
} UAVPOSSTU;

class dt_target_object
{
public:
  dt_target_object();
  static void *run(void *args);
  void localPosSubCb(const geometry_msgs::PoseStampedConstPtr& msg);
  void cloudMsgCb(const dt_message_package::CloudMessageConstPtr& msg);
  void uavsPoseVelCb(const dt_message_package::uavs_pose_velConstPtr& msg);
  static void *runPlot(void *args);
private:
  ros::Subscriber _localPoseSub;
  ros::Subscriber _cloudMsgSub;
  ros::Subscriber _uavsPoseVelMsgSub;

  ros::Publisher _cloudMsgPub;
  ros::Publisher _targetVelPub;

  geometry_msgs::PoseStamped _localPose;

  int _sourceID;
  int _targetID;
  int _dtObjectID;

  int _objectNum;
  int _getDataNum;
  bool _isPlot;
  bool _isShowTargetObje;

  int _plotPointNum;
  vector<UAVPOSSTU> _allUavState;

  float _startPosX;
  float _startPosY;
  float _startPosZ;


  pthread_t _runThread;
  float _msgPubHz;
  std::mutex m;

  pthread_t _runPlotThread;
  float _plotHz;
  std::mutex _plotHzM;
};

#endif // DT_TARGET_OBJECT_H
