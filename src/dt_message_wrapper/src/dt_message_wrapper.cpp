#include "../include/dt_message_wrapper/dt_message_wrapper.h"

dt_message_wrapper::dt_message_wrapper()
{
  ros::NodeHandle n("~");
  if(!n.getParam("SourceID",_sourceID))
  {
    _sourceID = 0;
    std::cout<<"message_wrapper--SourceID No Configure"<<std::endl;
  }
  std::cout<<"message_wrapper--SourceID: "<<_sourceID<<std::endl;

  if(!n.getParam("TargetID",_targetID))
  {
    _targetID = 100;
    std::cout<<"message_wrapper--TargetID No Configure"<<std::endl;
  }
  std::cout<<"message_wrapper--TargetID: "<<_targetID<<std::endl;

  if(!n.getParam("DtObjectID",_dtObjectID))
  {
    _dtObjectID = 100;
    std::cout<<"message_wrapper--DtObjectID No Configure"<<_dtObjectID<<std::endl;
  }
  std::cout<<"message_wrapper--DtObjectID: "<<_dtObjectID<<std::endl;

  std::string CloudMsgSubTopic = "/R_UAV_0/Message_from_Cloud";
  if(!n.getParam("CloudMessageSubTopic",CloudMsgSubTopic))
  {
    std::cout<<"message_wrapper--CloudMessageSubTopic No Configure"<<std::endl;
  }
  std::cout<<"message_wrapper--CloudMessageSubTopic: "<<CloudMsgSubTopic<<std::endl;

  std::string GlobalPosMsgSubTopic = "/mavros/global_position/global";

  if(!n.getParam("GlobalPositionMessageSubTopic",GlobalPosMsgSubTopic))
  {
    std::cout<<"message_wrapper--GlobalPositionMessageSubTopic No Configure"<<std::endl;
  }
  std::cout<<"message_wrapper--GlobalPositionMessageSubTopic: "<<GlobalPosMsgSubTopic<<std::endl;

  std::string LocalPosMsgSubTopic = "/mavros/local_position/pose";
  if(!n.getParam("LocalPositionMessageSubTopic",LocalPosMsgSubTopic))
  {
    std::cout<<"message_wrapper--LocalPositionMessageSubTopic No Configure"<<std::endl;
  }
  std::cout<<"message_wrapper--LocalPositionMessageSubTopic: "<<LocalPosMsgSubTopic<<std::endl;

  std::string LocalVelMsgSubTopic = "/mavros/local_position/velocity";
  if(!n.getParam("LocalVelocityMessageSubTopic",LocalVelMsgSubTopic))
  {
    std::cout<<"message_wrapper--LocalVelocityMessageSubTopic No Configure"<<std::endl;
  }
  std::cout<<"message_wrapper--LocalVelocityMessageSubTopic: "<<LocalVelMsgSubTopic<<std::endl;

  std::string UavStateMsgSubTopic = "/mavros/state";
  if(!n.getParam("UavStateMessageSubTopic",UavStateMsgSubTopic))
  {
    std::cout<<"message_wrapper--UavStateMessageSubTopic No Configure"<<std::endl;
  }
  std::cout<<"message_wrapper--UavStateMessageSubTopic: "<<UavStateMsgSubTopic<<std::endl;

  std::string BatteryMsgSubTopic = "/mavros/battery";
  if(!n.getParam("BatteryMessageSubTopic",BatteryMsgSubTopic))
  {
    std::cout<<"message_wrapper--BatteryMessageSubTopic No Configure"<<std::endl;
  }
  std::cout<<"message_wrapper--BatteryMessageSubTopic: "<<BatteryMsgSubTopic<<std::endl;

  if(!n.getParam("MessagePubFrequency",_msgPubHz))
  {
    _msgPubHz = 10;
    std::cout<<"message_wrapper--MessagePubFrequency No Configure"<<std::endl;
  }
  std::cout<<"message_wrapper--MessagePubFrequency: "<<_msgPubHz<<std::endl;


  _local_pos_sub = n.subscribe(LocalPosMsgSubTopic,1,&dt_message_wrapper::local_pos_sub_cb,this);
  _global_pos_sub = n.subscribe(GlobalPosMsgSubTopic,1,&dt_message_wrapper::global_pos_sub_cb,this);
  _battery_info_sub = n.subscribe(BatteryMsgSubTopic,1,&dt_message_wrapper::battery_info_sub_cb,this);
  _state_info_sub = n.subscribe(UavStateMsgSubTopic,1,&dt_message_wrapper::state_info_sub_cb,this);
  _vel_info_sub = n.subscribe(LocalVelMsgSubTopic,1,&dt_message_wrapper::vel_info_sub_cb,this);
  _cloud_msg_sub = n.subscribe(CloudMsgSubTopic,1,&dt_message_wrapper::cloud_msg_cb,this);

  std::string CloudMsgPubTopic = "/R_UAV_0/Message_to_Cloud";
  if(!n.getParam("CloudMessagePubTopic",CloudMsgPubTopic))
  {
    std::cout<<"message_wrapper--CloudMessagePubTopic No Configure"<<std::endl;
  }
  std::cout<<"message_wrapper--CloudMessagePubTopic: "<<CloudMsgPubTopic<<std::endl;

  std::string TargetVelPubTopic = "/R_UAV_0/Target/Velocity";
  if(!n.getParam("TargetVelocityMessagePubTopic",TargetVelPubTopic))
  {
    std::cout<<"message_wrapper--TargetVelocityMessagePubTopic No Configure"<<std::endl;
  }
  std::cout<<"message_wrapper--TargetVelocityMessagePubTopic: "<<TargetVelPubTopic<<std::endl;

  std::string TargetPosPubTopic = "/R_UAV_0/Target/Position";
  if(!n.getParam("TargetPositionMessagePubTopic",TargetPosPubTopic))
  {
    std::cout<<"message_wrapper--TargetPositionMessagePubTopic No Configure"<<std::endl;
  }
  std::cout<<"message_wrapper--TargetPositionMessagePubTopic: "<<TargetPosPubTopic<<std::endl;

  std::string ArmComPubTopic = "/R_UAV_0/Set/Arm";
  if(!n.getParam("ArmCommandMessagePubTopic",ArmComPubTopic))
  {
    std::cout<<"message_wrapper--ArmCommandMessagePubTopic No Configure"<<std::endl;
  }
  std::cout<<"message_wrapper--ArmCommandMessagePubTopic: "<<ArmComPubTopic<<std::endl;

  std::string TargetFModePubTopic = "/R_UAV_0/Set/FMode";
  if(!n.getParam("TargetFModeMessagePubTopic",TargetFModePubTopic))
  {
    std::cout<<"message_wrapper--TargetFModeMessagePubTopic No Configure"<<std::endl;
  }
  std::cout<<"message_wrapper--TargetFModeMessagePubTopic: "<<ArmComPubTopic<<std::endl;

  std::string VRControlPubTopic = "/R_UAV_0/VR/Control";
  if(!n.getParam("VRControlMessagePubTopic",VRControlPubTopic))
  {
    std::cout<<"message_wrapper--VRControlPubTopic No Configure"<<std::endl;
  }
  std::cout<<"message_wrapper--VRControlPubTopic: "<<VRControlPubTopic<<std::endl;

  std::string ComputerCmdPubTopic = "/R_UAV_0/Computer/Cmd";
  if(!n.getParam("ComputerCmdMessagePubTopic",ComputerCmdPubTopic))
  {
    std::cout<<"message_wrapper--ComputerCmdMessagePubTopic No Configure"<<std::endl;
  }
  std::cout<<"message_wrapper--ComputerCmdMessagePubTopic: "<<ComputerCmdPubTopic<<std::endl;

  std::string ApplyCamPubTopic = "/R_UAV_0/Apply/Camera";
  if(!n.getParam("ApplyCameraMessagePubTopic",ApplyCamPubTopic))
  {
    std::cout<<"message_wrapper--ApplyCameraMessagePubTopic No Configure"<<std::endl;
  }
  std::cout<<"message_wrapper--ApplyCameraMessagePubTopic: "<<ApplyCamPubTopic<<std::endl;

  std::string OtherUavStateMsgPubTopic = "/OtherUavs/State";
  if(!n.getParam("OtherUavStateMsgPubTopic",OtherUavStateMsgPubTopic))
  {
    std::cout<<"message_wrapper--OtherUavStateMsgPubTopic No Configure"<<std::endl;
  }
  std::cout<<"message_wrapper--OtherUavStateMsgPubTopic: "<<OtherUavStateMsgPubTopic<<std::endl;

  if(!n.getParam("UavNum",_uavNum))
  {
    _uavNum = 5;
    std::cout<<"message_wrapper--UavNum No Configure"<<std::endl;
  }
  std::cout<<"message_wrapper--UavNum: "<<_uavNum<<std::endl;

  if(!n.getParam("IsAllUpdate",_isAllUpdate))
  {
    _isAllUpdate = false;
    std::cout<<"message_wrapper--IsAllUpdate No Configure"<<std::endl;
  }
  std::cout<<"message_wrapper--IsAllUpdate: "<<_isAllUpdate<<std::endl;

  _heartMsgHz = 0.5;
  if(!n.getParam("HeartMessagePubHz",_heartMsgHz))
  {
    std::cout<<"message_wrapper--HeartMessagePubHz No Configure"<<std::endl;
  }
  std::cout<<"message_wrapper--HeartMessagePubHz: "<<_heartMsgHz<<std::endl;

  std::string NetworkStateMsgPubTopic ="/Network/State";
  if(!n.getParam("NetworkStateMsgPubTopic",NetworkStateMsgPubTopic))
  {
    std::cout<<"message_wrapper--NetworkStateMsgPubTopic No Configure"<<std::endl;
  }
  std::cout<<"message_wrapper--NetworkStateMsgPubTopic: "<<NetworkStateMsgPubTopic<<std::endl;

  if(!n.getParam("StartPositionX",_startPosX))
  {
    std::cout<<"message_wrapper--StartPositionX No Configure"<<std::endl;
  }
  std::cout<<"message_wrapper--StartPositionX: "<<_startPosX<<std::endl;

  if(!n.getParam("StartPositionY",_startPosY))
  {
    std::cout<<"message_wrapper--StartPositionY No Configure"<<std::endl;
  }
  std::cout<<"message_wrapper--StartPositionY: "<<_startPosY<<std::endl;

  if(!n.getParam("StartPositionZ",_startPosZ))
  {
    std::cout<<"message_wrapper--StartPositionZ No Configure"<<std::endl;
  }
  std::cout<<"message_wrapper--StartPositionZ: "<<_startPosZ<<std::endl;

  std::string TargetObjPosMsgPubTopic = "/target_object/pose";
  if(!n.getParam("TargetObjPosMsgPubTopic",TargetObjPosMsgPubTopic))
  {
    std::cout<<"message_wrapper--TargetObjPosMsgPubTopic No Configure"<<std::endl;
  }
  std::cout<<"message_wrapper--TargetObjPosMsgPubTopic: "<<TargetObjPosMsgPubTopic<<std::endl;


  _cloud_msg_pub= n.advertise<dt_message_package::CloudMessage>(CloudMsgPubTopic,20);
  _target_pos_pub = n.advertise<geometry_msgs::PoseStamped>(TargetPosPubTopic,10);
  _target_vel_pub = n.advertise<geometry_msgs::TwistStamped>(TargetVelPubTopic,10);
  _arm_com_pub = n.advertise<std_msgs::Bool>(ArmComPubTopic,10);
  _target_fmode_pub = n.advertise<std_msgs::Int8>(TargetFModePubTopic,10);
  _vr_control_pub = n.advertise<std_msgs::Bool>(VRControlPubTopic,10);
  _computer_cmd_pub = n.advertise<std_msgs::Bool>(ComputerCmdPubTopic,10);
  _apply_cam_pub = n.advertise<std_msgs::Bool>(ApplyCamPubTopic,10);
  _other_uavs_state_pub = n.advertise<dt_message_package::uavs_pose_vel>(OtherUavStateMsgPubTopic,50);
  _network_state_pub = n.advertise<dt_message_package::NetworkStateMsg>(NetworkStateMsgPubTopic,10);

  _target_obj_pos_pub = n.advertise<geometry_msgs::PoseStamped>(TargetObjPosMsgPubTopic,10);

  _otherUavsState.resize(_uavNum);
  _isLinkUavs.resize(_uavNum);
  _isLinkUavs.at(_sourceID-1) = true;
  _numStateUpdate = 0;
  _isFirstSend = true;
  _isStart = false;
  _isLinkServer = false;

  int flag_thread = pthread_create(&_runThread,NULL,&dt_message_wrapper::run,this);
  if (flag_thread < 0) {
    ROS_ERROR("message_wrapper--pthread_create ros_process_thread failed: %d\n", flag_thread);
  }

  flag_thread = pthread_create(&_runPubHeartTh,NULL,&dt_message_wrapper::run_pub_heart_msg,this);
  if (flag_thread < 0) {
    ROS_ERROR("message_wrapper--pthread_create heart_publish_message_thread failed: %d\n", flag_thread);
  }
}

void *dt_message_wrapper::run_pub_heart_msg(void *args)
{
  dt_message_wrapper* dtvrPtr = (dt_message_wrapper*)(args);
  ros::Rate rate(dtvrPtr->_heartMsgHz);
  dt_message_package::CloudMessage pubMsg;
  pubMsg.SourceID = dtvrPtr->_sourceID;
  pubMsg.TargetID = dtvrPtr->_targetID;
  pubMsg.MessageID = HeartMsgID;
  HeartMsg hit;
  hit.is_get = true;
  pubMsg.MessageData = x2struct::X::tojson(hit);

  dt_message_package::NetworkStateMsg networkStateMsg;
  networkStateMsg.uavs.resize(dtvrPtr->_uavNum);
  while(ros::ok())
  {
    pubMsg.TimeStamp = ros::Time::now().toNSec();
    dtvrPtr->_cloud_msg_pub.publish(pubMsg);

    networkStateMsg.header.stamp = ros::Time::now();
    networkStateMsg.isServer = dtvrPtr->_isLinkServer;
    for(int i=0;i<dtvrPtr->_uavNum;i++)
    {
      networkStateMsg.uavs.at(i) = dtvrPtr->_isLinkUavs.at(i);
    }
    dtvrPtr->_network_state_pub.publish(networkStateMsg);
    {
      std::lock_guard<mutex> guard(dtvrPtr->m);
      for(int i=0;i<dtvrPtr->_uavNum;i++)
      {
        if(i!=dtvrPtr->_dtObjectID-1)
          dtvrPtr->_isLinkUavs.at(i) = false;
      }
    }
    rate.sleep();
  }
  pthread_join(dtvrPtr->_runPubHeartTh,NULL);
}

void *dt_message_wrapper::run(void *args)
{
  dt_message_wrapper* dtvrPtr = (dt_message_wrapper*)(args);
  ros::Rate rate(dtvrPtr->_msgPubHz);
  dt_message_package::CloudMessage pubMsg;
  pubMsg.SourceID = dtvrPtr->_sourceID;
  pubMsg.TargetID = dtvrPtr->_targetID;
  pubMsg.MessageID = UavInfoID;
  UavInfo info;
  while(ros::ok())
  {
    if(dtvrPtr->_uavNum==1)
    {
      dt_message_package::uavs_pose_vel uavsPosVelMsg;
      uavsPosVelMsg.header.frame_id = "UAV";
      uavsPosVelMsg.header.stamp = ros::Time::now();
      geometry_msgs::Point pos;
      geometry_msgs::Vector3 vel;
      uavsPosVelMsg.uav_id.push_back(dtvrPtr->_sourceID);
      pos.x = dtvrPtr->_curPosX;
      pos.y = dtvrPtr->_curPosY;
      pos.z = dtvrPtr->_curPosZ;
      vel.x = dtvrPtr->_lVelX;
      vel.y = dtvrPtr->_lVelY;
      vel.z = dtvrPtr->_lVelZ;
      uavsPosVelMsg.position.push_back(pos);
      uavsPosVelMsg.velocity.push_back(vel);
      dtvrPtr->_other_uavs_state_pub.publish(uavsPosVelMsg);
    }

    pubMsg.TimeStamp = ros::Time::now().toNSec();
    {
      std::lock_guard<mutex> guard(dtvrPtr->m);
      info.AVelX = dtvrPtr->_aVelX;
      info.AVelY = dtvrPtr->_aVelY;
      info.AVelZ = dtvrPtr->_aVelZ;
      info.FMode = dtvrPtr->_fMode;
      info.IsArm = dtvrPtr->_isArm;
      info.LVelX = dtvrPtr->_lVelX;
      info.LVelY = dtvrPtr->_lVelY;
      info.LVelZ = dtvrPtr->_lVelZ;
      info.NetPx4 = dtvrPtr->_netPx4;
      info.PosX = dtvrPtr->_lposX;
      info.PosY = dtvrPtr->_lposY;
      info.PosZ = dtvrPtr->_lposZ;
      info.Remaining = dtvrPtr->_remaining;
      info.RotW = dtvrPtr->_rotW;
      info.RotX = dtvrPtr->_rotX;
      info.RotY = dtvrPtr->_rotY;
      info.RotZ = dtvrPtr->_rotZ;
      info.Voltage = dtvrPtr->_voltage;
      info.IsStart = dtvrPtr->_isStart;
    }
    pubMsg.MessageData = x2struct::X::tojson(info);
    dtvrPtr->_cloud_msg_pub.publish(pubMsg);
    rate.sleep();
  }
  pthread_join(dtvrPtr->_runThread,NULL);
}

void dt_message_wrapper::vel_info_sub_cb(const geometry_msgs::TwistStampedConstPtr& msg)
{
  _lVelX = msg.get()->twist.linear.x;
  _lVelY = msg.get()->twist.linear.y;
  _lVelZ = msg.get()->twist.linear.z;
  _aVelX = msg.get()->twist.angular.x;
  _aVelY = msg.get()->twist.angular.y;
  _aVelZ = msg.get()->twist.angular.z;
}

void dt_message_wrapper::local_pos_sub_cb(const geometry_msgs::PoseStampedConstPtr& msg)
{
  _lposX = msg.get()->pose.position.x + _startPosX;
  _lposY = msg.get()->pose.position.y + _startPosY;
  _lposZ = msg.get()->pose.position.z + _startPosZ;
  _rotX = msg.get()->pose.orientation.x;
  _rotY = msg.get()->pose.orientation.y;
  _rotZ = msg.get()->pose.orientation.z;
  _rotW = msg.get()->pose.orientation.w;

  _curPosX = _lposX;
  _curPosY = _lposY;
  _curPosZ = _lposZ;
}

void dt_message_wrapper::global_pos_sub_cb(const sensor_msgs::NavSatFixConstPtr& msg)
{
  _latitude = msg.get()->latitude;
  _longitude = msg.get()->longitude;
  _altitude = msg.get()->altitude;
}

void dt_message_wrapper::battery_info_sub_cb(const sensor_msgs::BatteryStateConstPtr &msg)
{
  _voltage = msg.get()->voltage;
  _remaining = msg.get()->percentage;
}

void dt_message_wrapper::state_info_sub_cb(const mavros_msgs::StateConstPtr &msg)
{
  if(msg.get()->connected)
    _netPx4 = true;
  else
    _netPx4 = false;
  if(msg.get()->armed)
    _isArm = true;
  else
    _isArm = false;
  std::string mode = msg.get()->mode;
  if(mode == msg.get()->MODE_PX4_MANUAL)
    _fMode = 0;
  else if(mode == msg.get()->MODE_PX4_STABILIZED)
    _fMode = 1;
  else if(mode == msg.get()->MODE_PX4_ALTITUDE)
    _fMode = 2;
  else if(mode == msg.get()->MODE_PX4_POSITION)
    _fMode = 3;
  else if(mode == msg.get()->MODE_PX4_OFFBOARD)
    _fMode = 4;
  else if(mode == msg.get()->MODE_PX4_RTL)
    _fMode = 5;
  else
    _fMode = -1;
}

void dt_message_wrapper::cloud_msg_cb(const dt_message_package::CloudMessageConstPtr &msg)
{
  switch (msg.get()->MessageID) {
  case UavControlID:
  {
    UavControl uavControlMsg;
    bool isLoad = x2struct::X::loadjson(msg.get()->MessageData,uavControlMsg,false);
    if(isLoad)
    {
      if(uavControlMsg.Mode==0)
      {
        //position mode
        geometry_msgs::PoseStamped targetPosMsg;
        targetPosMsg.header.stamp = ros::Time::now();
        targetPosMsg.pose.position.x = uavControlMsg.ComLX;
        targetPosMsg.pose.position.y = uavControlMsg.ComLY;
        targetPosMsg.pose.position.z = uavControlMsg.ComLZ;
        tf::Quaternion qua = tf::createQuaternionFromRPY(uavControlMsg.ComAX,uavControlMsg.ComAY,uavControlMsg.ComAZ);
        targetPosMsg.pose.orientation.x = qua.x();
        targetPosMsg.pose.orientation.y = qua.y();
        targetPosMsg.pose.orientation.z = qua.z();
        targetPosMsg.pose.orientation.w = qua.w();
        _target_pos_pub.publish(targetPosMsg);
      }
      else if(uavControlMsg.Mode==1)
      {
        //velocity mode
        geometry_msgs::TwistStamped targetVelMsg;
        targetVelMsg.header.stamp = ros::Time::now();
        targetVelMsg.twist.linear.x = uavControlMsg.ComLX;
        targetVelMsg.twist.linear.y = uavControlMsg.ComLY;
        targetVelMsg.twist.linear.z = uavControlMsg.ComLZ;
        targetVelMsg.twist.angular.x = uavControlMsg.ComAX;
        targetVelMsg.twist.angular.y = uavControlMsg.ComAY;
        targetVelMsg.twist.angular.z = uavControlMsg.ComAZ;
        _target_vel_pub.publish(targetVelMsg);
        //std::cout<<"dddddd"<<std::endl;
      }
    }
    else
      ROS_INFO("Unpack UavControl Message Fail!!!");
  }
    break;
  case UavCommandID:
  {
    UavCommand uavCommandMsg;
    bool isLoad = x2struct::X::loadjson(msg.get()->MessageData,uavCommandMsg,false);
    if(isLoad)
    {
      if(uavCommandMsg.ComMode == 1)
      {
        std_msgs::Int8 targetFModeMsg;
        if(uavCommandMsg.IsOffboard)
          targetFModeMsg.data = 4;//Offboard
        else
          targetFModeMsg.data = 3;//Position
        _target_fmode_pub.publish(targetFModeMsg);
      }
      else if(uavCommandMsg.ComMode == 2)
      {
        std_msgs::Bool armMsg;
        if(uavCommandMsg.IsArm)
          armMsg.data = true;
        else
          armMsg.data = false;
        _arm_com_pub.publish(armMsg);
      }
      else if (uavCommandMsg.ComMode == 3)
      {
        std_msgs::Int8 targetFModeMsg;
        if(uavCommandMsg.IsOffboard)
          targetFModeMsg.data = 4;//Offboard
        else
          targetFModeMsg.data = 3;//Position
        _target_fmode_pub.publish(targetFModeMsg);
        std_msgs::Bool armMsg;
        if(uavCommandMsg.IsArm)
          armMsg.data = true;
        else
          armMsg.data = false;
        _arm_com_pub.publish(armMsg);
      }
      else if(uavCommandMsg.ComMode == 4)
      {
        std_msgs::Bool startMsg;
        startMsg.data = uavCommandMsg.IsStart;
        _isStart = uavCommandMsg.IsStart;
        _vr_control_pub.publish(startMsg);
      }
      else if(uavCommandMsg.ComMode == 5)
      {
        std_msgs::Int8 targetFModeMsg;
        if(uavCommandMsg.IsOffboard)
          targetFModeMsg.data = 4;//Offboard
        else
          targetFModeMsg.data = 3;//Position
        _target_fmode_pub.publish(targetFModeMsg);

        std_msgs::Bool startMsg;
        startMsg.data = uavCommandMsg.IsStart;
        _vr_control_pub.publish(startMsg);
      }

      else if(uavCommandMsg.ComMode == 6)
      {
        std_msgs::Bool startMsg;
        startMsg.data = uavCommandMsg.IsStart;
        _vr_control_pub.publish(startMsg);

        std_msgs::Bool armMsg;
        if(uavCommandMsg.IsArm)
          armMsg.data = true;
        else
          armMsg.data = false;
        _arm_com_pub.publish(armMsg);
      }

      else if(uavCommandMsg.ComMode == 7)
      {
        std_msgs::Bool armMsg;
        if(uavCommandMsg.IsArm)
          armMsg.data = true;
        else
          armMsg.data = false;
        _arm_com_pub.publish(armMsg);

        std_msgs::Int8 targetFModeMsg;
        if(uavCommandMsg.IsOffboard)
          targetFModeMsg.data = 4;//Offboard
        else
          targetFModeMsg.data = 3;//Position
        _target_fmode_pub.publish(targetFModeMsg);

        std_msgs::Bool startMsg;
        startMsg.data = uavCommandMsg.IsStart;
        _vr_control_pub.publish(startMsg);
      }
    }
    else
      ROS_INFO("Unpack UavCommand Message Fail!!!");

  }
    break;
  case ComputerControlID:
  {
    ComputerControl computerControlMsg;
    bool isLoad = x2struct::X::loadjson(msg.get()->MessageData,computerControlMsg,false);
    if(isLoad)
    {
      std_msgs::Bool computerCmdMsg;
      computerCmdMsg.data = computerControlMsg.IsClose;
      _computer_cmd_pub.publish(computerCmdMsg);
    }
  }
    break;
  case ApplyCameraID:
  {
    ApplyCameraMsg applyCamMsg;
    bool isLoad = x2struct::X::loadjson(msg.get()->MessageData,applyCamMsg,false);
    if(isLoad)
    {
      std_msgs::Bool apply;
      apply.data = applyCamMsg.isOpen;
      _apply_cam_pub.publish(apply);
    }
  }
    break;
  case UavInfoID:
  {
    int ownInDex = _sourceID-1;
    _otherUavsState.at(ownInDex).id = _sourceID;
    _otherUavsState.at(ownInDex).pos_x = _curPosX;
    _otherUavsState.at(ownInDex).pos_y = _curPosY;
    _otherUavsState.at(ownInDex).pos_z = _curPosZ;
    _otherUavsState.at(ownInDex).vel_x = _lVelX;
    _otherUavsState.at(ownInDex).vel_y = _lVelY;
    _otherUavsState.at(ownInDex).vel_z = _lVelZ;
    UavInfo uavInfoMsg;
    bool isLoad = x2struct::X::loadjson(msg.get()->MessageData,uavInfoMsg,false);
    if(isLoad)
    {
      int index = msg.get()->SourceID-1;
      _otherUavsState.at(index).id = msg.get()->SourceID;
      _otherUavsState.at(index).pos_x = uavInfoMsg.PosX;
      _otherUavsState.at(index).pos_y = uavInfoMsg.PosY;
      _otherUavsState.at(index).pos_z = uavInfoMsg.PosZ;
      _otherUavsState.at(index).vel_x = uavInfoMsg.LVelX;
      _otherUavsState.at(index).vel_y = uavInfoMsg.LVelY;
      _otherUavsState.at(index).vel_z = uavInfoMsg.LVelZ;
      _isLinkUavs.at(index) = true;
      _updatedUavsId.insert(msg.get()->SourceID);
    }

    if(_isFirstSend)
    {
      if(_updatedUavsId.size()==_uavNum-1)
      {
        dt_message_package::uavs_pose_vel uavsPosVelMsg;
        uavsPosVelMsg.header.frame_id = "UAV";
        uavsPosVelMsg.header.stamp = ros::Time::now();
        geometry_msgs::Point pos;
        geometry_msgs::Vector3 vel;
        for(int i =0; i<_otherUavsState.size();++i)
        {
          uavsPosVelMsg.uav_id.push_back(_otherUavsState.at(i).id);
          pos.x = _otherUavsState.at(i).pos_x;
          pos.y = _otherUavsState.at(i).pos_y;
          pos.z = _otherUavsState.at(i).pos_z;
          vel.x = _otherUavsState.at(i).vel_x;
          vel.y = _otherUavsState.at(i).vel_y;
          vel.z = _otherUavsState.at(i).vel_z;
          uavsPosVelMsg.position.push_back(pos);
          uavsPosVelMsg.velocity.push_back(vel);
        }
        _other_uavs_state_pub.publish(uavsPosVelMsg);
        _isFirstSend = false;
        _updatedUavsId.erase(_updatedUavsId.begin(),_updatedUavsId.end());
        _updatedUavsId.clear();
      }
    }
    else
    {
      if(_isAllUpdate)
      {
        if(_updatedUavsId.size()==_uavNum-1)
        {
          dt_message_package::uavs_pose_vel uavsPosVelMsg;
          uavsPosVelMsg.header.frame_id = "UAV";
          uavsPosVelMsg.header.stamp = ros::Time::now();
          geometry_msgs::Point pos;
          geometry_msgs::Vector3 vel;
          for(int i =0; i<_otherUavsState.size();++i)
          {
            uavsPosVelMsg.uav_id.push_back(_otherUavsState.at(i).id);
            pos.x = _otherUavsState.at(i).pos_x;
            pos.y = _otherUavsState.at(i).pos_y;
            pos.z = _otherUavsState.at(i).pos_z;
            vel.x = _otherUavsState.at(i).vel_x;
            vel.y = _otherUavsState.at(i).vel_y;
            vel.z = _otherUavsState.at(i).vel_z;
            uavsPosVelMsg.position.push_back(pos);
            uavsPosVelMsg.velocity.push_back(vel);
          }
          _other_uavs_state_pub.publish(uavsPosVelMsg);
          _updatedUavsId.erase(_updatedUavsId.begin(),_updatedUavsId.end());
          _updatedUavsId.clear();
        }
      }
      else
      {
        dt_message_package::uavs_pose_vel uavsPosVelMsg;
        uavsPosVelMsg.header.frame_id = "UAV";
        uavsPosVelMsg.header.stamp = ros::Time::now();
        geometry_msgs::Point pos;
        geometry_msgs::Vector3 vel;
        for(int i =0; i<_otherUavsState.size();++i)
        {
          uavsPosVelMsg.uav_id.push_back(_otherUavsState.at(i).id);
          pos.x = _otherUavsState.at(i).pos_x;
          pos.y = _otherUavsState.at(i).pos_y;
          pos.z = _otherUavsState.at(i).pos_z;
          vel.x = _otherUavsState.at(i).vel_x;
          vel.y = _otherUavsState.at(i).vel_y;
          vel.z = _otherUavsState.at(i).vel_z;
          uavsPosVelMsg.position.push_back(pos);
          uavsPosVelMsg.velocity.push_back(vel);
        }
        _other_uavs_state_pub.publish(uavsPosVelMsg);
      }
    }
  }
    break;
  case HeartMsgID:
  {
  }
    break;
  case TargetObjectPoseMessageID:
  {
    TargetObjPoseMessage poseMsg;
    bool isLoad = x2struct::X::loadjson(msg.get()->MessageData,poseMsg,false);
    if(isLoad)
    {
     geometry_msgs::PoseStamped objPosMsg;
     objPosMsg.header.frame_id = "target_object";
     objPosMsg.header.stamp = ros::Time::now();
     objPosMsg.pose.position.x = poseMsg.posX;
     objPosMsg.pose.position.y = poseMsg.posY;
     objPosMsg.pose.position.z = poseMsg.posZ;
     objPosMsg.pose.orientation.x = poseMsg.rotX;
     objPosMsg.pose.orientation.y = poseMsg.rotY;
     objPosMsg.pose.orientation.z = poseMsg.rotZ;
     objPosMsg.pose.orientation.w = poseMsg.rotW;
     _target_obj_pos_pub.publish(objPosMsg);
    }
  }
    break;
  default:
    break;
  }
}














