#include "../include/dt_simulation_wrapper/dt_simulation_wrapper.h"

dt_simulation_wrapper::dt_simulation_wrapper()
{
  ros::NodeHandle n("~");
  std::string UAV1PosSubTopic = "/UAV1/mavros/local_position/pose";
  n.getParam("UAV1LocalPositionMessageSubTopic",UAV1PosSubTopic);
  std::string UAV2PosSubTopic = "/UAV2/mavros/local_position/pose";
  n.getParam("UAV2LocalPositionMessageSubTopic",UAV2PosSubTopic);
  std::string UAV3PosSubTopic = "/UAV3/mavros/local_position/pose";
  n.getParam("UAV3LocalPositionMessageSubTopic",UAV3PosSubTopic);
  std::string UAV4PosSubTopic = "/UAV4/mavros/local_position/pose";
  n.getParam("UAV4LocalPositionMessageSubTopic",UAV4PosSubTopic);
  std::string UAV5PosSubTopic = "/UAV5/mavros/local_position/pose";
  n.getParam("UAV5LocalPositionMessageSubTopic",UAV5PosSubTopic);
  std::string UAV6PosSubTopic = "/UAV6/mavros/local_position/pose";
  n.getParam("UAV6LocalPositionMessageSubTopic",UAV6PosSubTopic);

  std::string UAV1PosPubTopic = "/UAV1/mavros/local_position/pose";
  n.getParam("UAV1LocalPositionMessagepubTopic",UAV1PosPubTopic);
  std::string UAV2PosPubTopic = "/UAV2/mavros/local_position/pose";
  n.getParam("UAV2LocalPositionMessagepubTopic",UAV2PosPubTopic);
  std::string UAV3PosPubTopic = "/UAV3/mavros/local_position/pose";
  n.getParam("UAV3LocalPositionMessagepubTopic",UAV3PosPubTopic);
  std::string UAV4PosPubTopic = "/UAV4/mavros/local_position/pose";
  n.getParam("UAV4LocalPositionMessagepubTopic",UAV4PosPubTopic);
  std::string UAV5PosPubTopic = "/UAV5/mavros/local_position/pose";
  n.getParam("UAV5LocalPositionMessagepubTopic",UAV5PosPubTopic);
  std::string UAV6PosPubTopic = "/UAV6/mavros/local_position/pose";
  n.getParam("UAV6LocalPositionMessagepubTopic",UAV6PosPubTopic);


  std::string UAV1VelSubTopic = "/UAV1/mavros/local_position/velocity";
  n.getParam("UAV1LocalVelocityMessageSubTopic",UAV1VelSubTopic);
  std::string UAV2VelSubTopic = "/UAV2/mavros/local_position/velocity";
  n.getParam("UAV2LocalVelocityMessageSubTopic",UAV2VelSubTopic);
  std::string UAV3VelSubTopic = "/UAV3/mavros/local_position/velocity";
  n.getParam("UAV3LocalVelocityMessageSubTopic",UAV3VelSubTopic);
  std::string UAV4VelSubTopic = "/UAV4/mavros/local_position/velocity";
  n.getParam("UAV4LocalVelocityMessageSubTopic",UAV4VelSubTopic);
  std::string UAV5VelSubTopic = "/UAV5/mavros/local_position/velocity";
  n.getParam("UAV5LocalVelocityMessageSubTopic",UAV5VelSubTopic);
  std::string UAV6VelSubTopic = "/UAV6/mavros/local_position/velocity";
  n.getParam("UAV6LocalVelocityMessageSubTopic",UAV6VelSubTopic);

  std::string TargetLocalPosPubTopic = "/target/mavros/local_position/pose";
  n.getParam("TargetLocalPositionMessagepubTopic",TargetLocalPosPubTopic);


  std::string TargetPosSubTopic = "/target/mavros/local_position/velocity";
  n.getParam("TargetLocalPositionMsgSubTopic",TargetPosSubTopic);


  std::string TargetVelSubTopic = "/target/mavros/local_position/velocity";
  n.getParam("TargetLocalVelocityMsgSubTopic",TargetVelSubTopic);

  std::string UAVsPosVelPubTopic = "/other_uavs/state";
  n.getParam("UAVsPosVelMessagePubTopic",UAVsPosVelPubTopic);

  if(!n.getParam("MessagePubFrequency",_msgPubHz))
  {
    _msgPubHz = 10;
  }
  _mode = 0;

  _isPubTarget = false;
  n.getParam("IsPubTarget",_isPubTarget);

  std::string CloudMsgSubTopic = "/R_UAV_0/Message_from_Cloud";
  if(!n.getParam("CloudMessageSubTopic",CloudMsgSubTopic))
  {
    std::cout<<"dt_target_object--CloudMessageSubTopic No Configure"<<std::endl;
  }
  std::cout<<"dt_target_object--CloudMessageSubTopic: "<<CloudMsgSubTopic<<std::endl;


  _uav1_pose_sub = n.subscribe(UAV1PosSubTopic,5,&dt_simulation_wrapper::uav1_local_pos_sub_cb,this);
  _uav2_pose_sub = n.subscribe(UAV2PosSubTopic,5,&dt_simulation_wrapper::uav2_local_pos_sub_cb,this);
  _uav3_pose_sub = n.subscribe(UAV3PosSubTopic,5,&dt_simulation_wrapper::uav3_local_pos_sub_cb,this);
  _uav4_pose_sub = n.subscribe(UAV4PosSubTopic,5,&dt_simulation_wrapper::uav4_local_pos_sub_cb,this);
  _uav5_pose_sub = n.subscribe(UAV5PosSubTopic,5,&dt_simulation_wrapper::uav5_local_pos_sub_cb,this);
  _uav6_pose_sub = n.subscribe(UAV6PosSubTopic,5,&dt_simulation_wrapper::uav6_local_pos_sub_cb,this);
  _target_pose_sub = n.subscribe(TargetPosSubTopic,5,&dt_simulation_wrapper::target_local_pos_sub_cb,this);

  _uav1_vel_sub = n.subscribe(UAV1VelSubTopic,5,&dt_simulation_wrapper::uav1_vel_info_sub_cb,this);
  _uav2_vel_sub = n.subscribe(UAV2VelSubTopic,5,&dt_simulation_wrapper::uav2_vel_info_sub_cb,this);
  _uav3_vel_sub = n.subscribe(UAV3VelSubTopic,5,&dt_simulation_wrapper::uav3_vel_info_sub_cb,this);
  _uav4_vel_sub = n.subscribe(UAV4VelSubTopic,5,&dt_simulation_wrapper::uav4_vel_info_sub_cb,this);
  _uav5_vel_sub = n.subscribe(UAV5VelSubTopic,5,&dt_simulation_wrapper::uav5_vel_info_sub_cb,this);
  _uav6_vel_sub = n.subscribe(UAV6VelSubTopic,5,&dt_simulation_wrapper::uav6_vel_info_sub_cb,this);
  _target_vel_sub = n.subscribe(TargetVelSubTopic,5,&dt_simulation_wrapper::target_vel_info_sub_cb,this);

  _cloudMsgSub = n.subscribe(CloudMsgSubTopic,1,&dt_simulation_wrapper::cloudMsgCb,this);

  _uavs_pos_vel_pub = n.advertise<dt_message_package::uavs_pose_vel>(UAVsPosVelPubTopic,10);
  _uav1_pose_pub = n.advertise<geometry_msgs::PoseStamped>(UAV1PosPubTopic,10);
  _uav2_pose_pub = n.advertise<geometry_msgs::PoseStamped>(UAV2PosPubTopic,10);
  _uav3_pose_pub = n.advertise<geometry_msgs::PoseStamped>(UAV3PosPubTopic,10);
  _uav4_pose_pub = n.advertise<geometry_msgs::PoseStamped>(UAV4PosPubTopic,10);
  _uav5_pose_pub = n.advertise<geometry_msgs::PoseStamped>(UAV5PosPubTopic,10);
  _uav6_pose_pub = n.advertise<geometry_msgs::PoseStamped>(UAV6PosPubTopic,10);
  _target_pose_pub = n.advertise<geometry_msgs::PoseStamped>(TargetLocalPosPubTopic,10);

  int flag_thread = pthread_create(&_runThread,NULL,&dt_simulation_wrapper::run,this);
  if (flag_thread < 0) {
    ROS_ERROR("message_wrapper--pthread_create ros_process_thread failed: %d\n", flag_thread);
  }

}

void dt_simulation_wrapper::target_local_pos_sub_cb(const geometry_msgs::PoseStampedConstPtr& msg)
{
  _target_pose = *msg;

}
void dt_simulation_wrapper::target_vel_info_sub_cb(const geometry_msgs::TwistStampedConstPtr& msg)
{
  _target_vel = *msg;
}

void *dt_simulation_wrapper::run(void *args)
{
  dt_simulation_wrapper* ptr = (dt_simulation_wrapper*)(args);
  ros::Rate rate(ptr->_msgPubHz);
  dt_message_package::uavs_pose_vel pubMsg;
  int size = 0;
  if(ptr->_isPubTarget) {size = 7;pubMsg.uav_id = {1,2,3,4,5,6,7};}
  else {size = 6;pubMsg.uav_id = {1,2,3,4,5,6};
  }
  pubMsg.position.resize(size);
  pubMsg.velocity.resize(size);
  while(ros::ok())
  {
    pubMsg.position.at(0) = ptr->_uav1_pose.pose.position;
    pubMsg.position.at(1) = ptr->_uav2_pose.pose.position;
    pubMsg.position.at(2) = ptr->_uav3_pose.pose.position;
    pubMsg.position.at(3) = ptr->_uav4_pose.pose.position;
    pubMsg.position.at(4) = ptr->_uav5_pose.pose.position;
    pubMsg.position.at(5) = ptr->_uav6_pose.pose.position;

    pubMsg.velocity.at(0) = ptr->_uav1_vel.twist.linear;
    pubMsg.velocity.at(1) = ptr->_uav2_vel.twist.linear;
    pubMsg.velocity.at(2) = ptr->_uav3_vel.twist.linear;
    pubMsg.velocity.at(3) = ptr->_uav4_vel.twist.linear;
    pubMsg.velocity.at(4) = ptr->_uav5_vel.twist.linear;
    pubMsg.velocity.at(5) = ptr->_uav6_vel.twist.linear;

    if(ptr->_isPubTarget){
      pubMsg.position.at(6) = ptr->_target_pose.pose.position;
      pubMsg.velocity.at(6) = ptr->_target_vel.twist.linear;
    }

    ptr->_uavs_pos_vel_pub.publish(pubMsg);
    rate.sleep();
  }
  delete ptr;
  pthread_join(ptr->_runThread,NULL);
}
void dt_simulation_wrapper::uav1_local_pos_sub_cb(const geometry_msgs::PoseStampedConstPtr& msg)
{
  _uav1_pose = *msg;
}
void dt_simulation_wrapper::uav1_vel_info_sub_cb(const geometry_msgs::TwistStampedConstPtr& msg)
{
  _uav1_vel = *msg;
}

void dt_simulation_wrapper::uav2_local_pos_sub_cb(const geometry_msgs::PoseStampedConstPtr& msg)
{
  _uav2_pose = *msg;
}
void dt_simulation_wrapper::uav2_vel_info_sub_cb(const geometry_msgs::TwistStampedConstPtr& msg)
{
  _uav2_vel = *msg;
}

void dt_simulation_wrapper::uav3_local_pos_sub_cb(const geometry_msgs::PoseStampedConstPtr& msg)
{
  _uav3_pose = *msg;
}
void dt_simulation_wrapper::uav3_vel_info_sub_cb(const geometry_msgs::TwistStampedConstPtr& msg)
{
  _uav3_vel = *msg;
}

void dt_simulation_wrapper::uav4_local_pos_sub_cb(const geometry_msgs::PoseStampedConstPtr& msg)
{
  _uav4_pose = *msg;
}
void dt_simulation_wrapper::uav4_vel_info_sub_cb(const geometry_msgs::TwistStampedConstPtr& msg)
{
  _uav4_vel = *msg;
}

void dt_simulation_wrapper::uav5_local_pos_sub_cb(const geometry_msgs::PoseStampedConstPtr& msg)
{
  _uav5_pose = *msg;
}
void dt_simulation_wrapper::uav5_vel_info_sub_cb(const geometry_msgs::TwistStampedConstPtr& msg)
{
  _uav5_vel = *msg;
}

void dt_simulation_wrapper::uav6_local_pos_sub_cb(const geometry_msgs::PoseStampedConstPtr& msg)
{
  _uav6_pose = *msg;
}
void dt_simulation_wrapper::uav6_vel_info_sub_cb(const geometry_msgs::TwistStampedConstPtr& msg)
{
  _uav6_vel = *msg;
}

void dt_simulation_wrapper::cloudMsgCb(const dt_message_package::CloudMessageConstPtr &msg)
{
  switch (msg.get()->MessageID) {
  case AllUavStateMessageID:
  {
    AllUavStateMessage uavsMsg;
    bool isLoad = x2struct::X::loadjson(msg.get()->MessageData,uavsMsg,false);
    if(isLoad)
    {

      geometry_msgs::PoseStamped poseMsg1;
      poseMsg1.header.frame_id ="uav";
      poseMsg1.header.stamp = ros::Time::now();
      poseMsg1.pose.position.x = uavsMsg.xs.at(0);
      poseMsg1.pose.position.y = uavsMsg.ys.at(0);
      poseMsg1.pose.position.z = uavsMsg.zs.at(0);
      _uav1_pose_pub.publish(poseMsg1);

      poseMsg1.pose.position.x = uavsMsg.xs.at(1);
      poseMsg1.pose.position.y = uavsMsg.ys.at(1);
      poseMsg1.pose.position.z = uavsMsg.zs.at(1);
      _uav2_pose_pub.publish(poseMsg1);

      poseMsg1.pose.position.x = uavsMsg.xs.at(2);
      poseMsg1.pose.position.y = uavsMsg.ys.at(2);
      poseMsg1.pose.position.z = uavsMsg.zs.at(2);
      _uav3_pose_pub.publish(poseMsg1);

      poseMsg1.pose.position.x = uavsMsg.xs.at(3);
      poseMsg1.pose.position.y = uavsMsg.ys.at(3);
      poseMsg1.pose.position.z = uavsMsg.zs.at(3);
      _uav4_pose_pub.publish(poseMsg1);

      poseMsg1.pose.position.x = uavsMsg.xs.at(4);
      poseMsg1.pose.position.y = uavsMsg.ys.at(4);
      poseMsg1.pose.position.z = uavsMsg.zs.at(4);
      _uav5_pose_pub.publish(poseMsg1);

      poseMsg1.pose.position.x = uavsMsg.xs.at(5);
      poseMsg1.pose.position.y = uavsMsg.ys.at(5);
      poseMsg1.pose.position.z = uavsMsg.zs.at(5);
      _uav6_pose_pub.publish(poseMsg1);
    }
  }
  break;
  default:
    break;
}
}
