#include "../include/uwb_wrapper/uwb_wrapper.h"

void uwb_wrapper::localPosMsgCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
  _rotation = msg.get()->pose.orientation;
}

void *uwb_wrapper::run(void *args)
{
  uwb_wrapper* node = (uwb_wrapper*)(args);
  ros::Rate rate(node->_poseHz);
  while(ros::ok())
  {
    if(node->_isInit&&node->_isHeightInit)
    {
       node->_uwbPosPub.publish(node->_pose_msg);
      rate.sleep();
    }

  }
  pthread_join(node->run_thread,NULL);

}

uwb_wrapper::uwb_wrapper()
{
  ros::NodeHandle n("~");
  n.getParam("UwbPosMsgTopic",_uwbPosPubTopic);//
  n.getParam("UwbInforMsgTopic",_uwbInforSubTopic);//
  n.getParam("UwbInstallHeigh",_uwbInstallHeiht);
  n.getParam("UwbDistanceTopic",_uwbDistanceTopic);
  n.getParam("DistanceMsgTopic",_distanceMsgTopic);
  n.getParam("LocalPositionModel",_isLocalPositionModel);
  n.getParam("LocalPosMsgTopic",_localPosMsgTopic);
  n.getParam("poseHz",_poseHz);
  n.getParam("error",_error);
  _distanceHeiht = 0;
  _distanceMsgSub = n.subscribe(_distanceMsgTopic,1,&uwb_wrapper::distanceMsgCallback,this);
  _uwbPosPub = n.advertise<geometry_msgs::PoseStamped>(_uwbPosPubTopic,1);
  _uwbDistancePub = n.advertise<sensor_msgs::ChannelFloat32>(_uwbDistanceTopic,1);
  _uwbInforSub = n.subscribe(_uwbInforSubTopic,1,&uwb_wrapper::uwbInformationCallback,this);
  _localPoseMsgSub = n.subscribe(_localPosMsgTopic,1,&uwb_wrapper::localPosMsgCallback,this);
  _isInit = false;
  _isHeightInit = false;
  if(_isLocalPositionModel)
  {
    ROS_INFO("------------Local Position Model--------------------");
  }
  else
  {
    ROS_INFO("-----------Global Position Model--------------------");
  }

  int flag_thread = pthread_create(&run_thread,NULL,&uwb_wrapper::run,this);
  if (flag_thread < 0) {
    ROS_ERROR("pthread_create ros_process_thread failed: %d\n", flag_thread);
  }
}
void uwb_wrapper::distanceMsgCallback(const sensor_msgs::RangeConstPtr &msg)
{
  if(!_isHeightInit)
  {
    if(msg.get()->range<=0.2)
    {
      _lastDistanceHeight = msg.get()->range;
      _isHeightInit = true;
       ROS_INFO("Get Current Height");
    }
     std::cout<<"Current Height: " << _lastDistanceHeight <<std::endl;
  }
  else
  {
    if(abs(msg.get()->range - _lastDistanceHeight)<0.2)
    {
      _distanceHeiht = msg.get()->range;
      _lastDistanceHeight = _distanceHeiht;
    }
  }


}

void uwb_wrapper::uwbInformationCallback(const nlink_parser::LinktrackNodeframe2ConstPtr &msg)
{
  sensor_msgs::ChannelFloat32 distance_msg;
  distance_msg.name = "uwb_distance";
  distance_msg.values.resize(4);
  if(_isInit&&_isHeightInit)
  {
    if(msg.get()->eop_3d.at(0)<_error&&msg.get()->eop_3d.at(1)<_error/*&&msg.get()->eop_3d.at(2)<_error*/)
    {
      if(_isLocalPositionModel)
      {
        _pose_msg.header.stamp = ros::Time::now();
        _pose_msg.pose.position.x = msg.get()->pos_3d.at(0) - _homePositionX;
        _pose_msg.pose.position.y = msg.get()->pos_3d.at(1) - _homePositionY;
        _pose_msg.pose.position.z = _distanceHeiht;
        _pose_msg.pose.orientation.w = _rotation.w;//msg.get()->quaternion.at(0);
        _pose_msg.pose.orientation.x = _rotation.x;//msg.get()->quaternion.at(1);
        _pose_msg.pose.orientation.y = _rotation.y;//msg.get()->quaternion.at(2);
        _pose_msg.pose.orientation.z = _rotation.z;//msg.get()->quaternion.at(3);
       // _uwbPosPub.publish(_pose_msg);
      }
      else
      {
        _pose_msg.header.stamp = ros::Time::now();
        _pose_msg.pose.position.x = msg.get()->pos_3d.at(0);
        _pose_msg.pose.position.y = msg.get()->pos_3d.at(1);
        _pose_msg.pose.position.z = _distanceHeiht;
        _pose_msg.pose.orientation.w = _rotation.w;//msg.get()->quaternion.at(0);
        _pose_msg.pose.orientation.x = _rotation.x;//msg.get()->quaternion.at(1);
        _pose_msg.pose.orientation.y = _rotation.y;//msg.get()->quaternion.at(2);
        _pose_msg.pose.orientation.z = _rotation.z;//msg.get()->quaternion.at(3);
       // _uwbPosPub.publish(_pose_msg);
      }
    }
  }
  else
  {
    if(msg.get()->eop_3d.at(0)<0.2&&msg.get()->eop_3d.at(1)<0.2&&msg.get()->eop_3d.at(2)<0.2)
    {
      _isInit = true;
      _homePositionX = msg.get()->pos_3d.at(0);
      _homePositionY = msg.get()->pos_3d.at(1);
      _homePositionZ = msg.get()->pos_3d.at(2);
      ROS_INFO("Get Home Position By Uwb");
      std::cout<<"Home Position: " << "( " << _homePositionX << "," << _homePositionY << "," << _homePositionZ << " )" <<std::endl;
    }
  }

  if(msg.get()->nodes.size()>=1)
  {
    for(int i = 0; i<msg.get()->nodes.size();i++)
    {
      if(i==0)
      {
        distance_msg.values.at(0) = msg.get()->nodes.at(i).dis;
      }
      else if(i==1)
      {
        distance_msg.values.at(1) = msg.get()->nodes.at(i).dis;
      }
      else if(i==2)
      {
        distance_msg.values.at(2) = msg.get()->nodes.at(i).dis;
      }
      else if(i==3)
      {
        distance_msg.values.at(3) = msg.get()->nodes.at(i).dis;
      }
    }
    _uwbDistancePub.publish(distance_msg);
  }
}
