#include "../include/dt_target_object/dt_target_object.h"

dt_target_object::dt_target_object()
{
  ros::NodeHandle n("~");
  if(!n.getParam("SourceID",_sourceID))
  {
    _sourceID = 0;
    std::cout<<"dt_target_object--SourceID No Configure"<<std::endl;
  }
  std::cout<<"dt_target_object--SourceID: "<<_sourceID<<std::endl;

  if(!n.getParam("TargetID",_targetID))
  {
    _targetID = 100;
    std::cout<<"dt_target_object--TargetID No Configure"<<std::endl;
  }
  std::cout<<"dt_target_object--TargetID: "<<_targetID<<std::endl;

  if(!n.getParam("DtObjectID",_dtObjectID))
  {
    _dtObjectID = 100;
    std::cout<<"dt_target_object--DtObjectID No Configure"<<_dtObjectID<<std::endl;
  }
  std::cout<<"dt_target_object--DtObjectID: "<<_dtObjectID<<std::endl;

  std::string CloudMsgSubTopic = "/R_UAV_0/Message_from_Cloud";
  if(!n.getParam("CloudMessageSubTopic",CloudMsgSubTopic))
  {
    std::cout<<"dt_target_object--CloudMessageSubTopic No Configure"<<std::endl;
  }
  std::cout<<"dt_target_object--CloudMessageSubTopic: "<<CloudMsgSubTopic<<std::endl;

  std::string LocalPosMsgSubTopic = "/mavros/local_position/pose";
  if(!n.getParam("LocalPositionMessageSubTopic",LocalPosMsgSubTopic))
  {
    std::cout<<"dt_target_object--LocalPositionMessageSubTopic No Configure"<<std::endl;
  }
  std::cout<<"dt_target_object--LocalPositionMessageSubTopic: "<<LocalPosMsgSubTopic<<std::endl;

  if(!n.getParam("MessagePubFrequency",_msgPubHz))
  {
    _msgPubHz = 10;
    std::cout<<"dt_target_object--MessagePubFrequency No Configure"<<std::endl;
  }
  std::cout<<"dt_target_object--MessagePubFrequency: "<<_msgPubHz<<std::endl;

  if(!n.getParam("StartPositionX",_startPosX))
  {
    _startPosX = 0;
    std::cout<<"dt_target_object--StartPositionX No Configure"<<std::endl;
  }
  std::cout<<"dt_target_object--StartPositionX: "<<_startPosX<<std::endl;

  if(!n.getParam("StartPositionY",_startPosY))
  {
    _startPosY = 0;
    std::cout<<"dt_target_object--StartPositionY No Configure"<<std::endl;
  }
  std::cout<<"dt_target_object--StartPositionY: "<<_startPosY<<std::endl;

  if(!n.getParam("StartPositionZ",_startPosZ))
  {
    _startPosZ = 0;
    std::cout<<"dt_target_object--StartPositionZ No Configure"<<std::endl;
  }
  std::cout<<"dt_target_object--StartPositionZ: "<<_startPosZ<<std::endl;

  std::string TargetVelPubTopic = "/R_UAV_0/Target/Velocity";
  if(!n.getParam("TargetVelocityMessagePubTopic",TargetVelPubTopic))
  {
    std::cout<<"dt_target_object--TargetVelocityMessagePubTopic No Configure"<<std::endl;
  }
  std::cout<<"dt_target_object--TargetVelocityMessagePubTopic: "<<TargetVelPubTopic<<std::endl;

  std::string CloudMsgPubTopic = "/R_UAV_0/Message_to_Cloud";
  if(!n.getParam("CloudMessagePubTopic",CloudMsgPubTopic))
  {
    std::cout<<"dt_target_object--CloudMessagePubTopic No Configure"<<std::endl;
  }
  std::cout<<"dt_target_object--CloudMessagePubTopic: "<<CloudMsgPubTopic<<std::endl;

  std::string UavsPosVelSubTopic = "/uav1/other_uav/state";
  if(!n.getParam("UavsPosVelSubTopic",UavsPosVelSubTopic))
  {
    std::cout<<"dt_target_object--UavsPosVelSubTopic No Configure"<<std::endl;
  }
  std::cout<<"dt_target_object--UavsPosVelSubTopic: "<<UavsPosVelSubTopic<<std::endl;

  _plotPointNum = 20;
  n.getParam("PlotPointNum",_plotPointNum);

  _objectNum = 7;
  n.getParam("ObjectNum",_objectNum);

  _plotHz = 10;
  n.getParam("PlotHz",_plotHz);

  _isShowTargetObje = false;
  n.getParam("IsShowTargetObje",_isShowTargetObje);

  _allUavState.resize(_objectNum);
  for(int i=0;i<_objectNum;++i){
    _allUavState.at(i).xs.resize(_plotPointNum);
    _allUavState.at(i).ys.resize(_plotPointNum);
    _allUavState.at(i).zs.resize(_plotPointNum);
    _allUavState.at(i).vxs.resize(_plotPointNum);
    _allUavState.at(i).vys.resize(_plotPointNum);
    _allUavState.at(i).vzs.resize(_plotPointNum);
  }
  _getDataNum = 0;
  _isPlot = false;



  _localPoseSub = n.subscribe(LocalPosMsgSubTopic,1,&dt_target_object::localPosSubCb,this);
  _cloudMsgSub = n.subscribe(CloudMsgSubTopic,1,&dt_target_object::cloudMsgCb,this);

  _uavsPoseVelMsgSub = n.subscribe(UavsPosVelSubTopic,1,&dt_target_object::uavsPoseVelCb,this);

  _cloudMsgPub = n.advertise<dt_message_package::CloudMessage>(CloudMsgPubTopic,1);
  _targetVelPub = n.advertise<geometry_msgs::TwistStamped>(TargetVelPubTopic,1);

  int flag_thread = pthread_create(&_runThread,NULL,&dt_target_object::run,this);
  if (flag_thread < 0) {
    ROS_ERROR("dt_target_object--pthread_create ros_process_thread failed: %d\n", flag_thread);
  }

  flag_thread = pthread_create(&_runPlotThread,NULL,&dt_target_object::runPlot,this);
  if (flag_thread < 0) {
    ROS_ERROR("dt_target_object--pthread_create ros_process_thread failed: %d\n", flag_thread);
  }

}

void dt_target_object::uavsPoseVelCb(const dt_message_package::uavs_pose_velConstPtr &msg)
{

 // if(msg.get()->position.size()==_objectNum){
    if(_getDataNum<_plotPointNum&&!_isPlot) _getDataNum++;
    else _isPlot = true;
    {
      std::lock_guard<mutex> guard(_plotHzM);
      for(int i=0;i<msg.get()->position.size();i++){
        _allUavState.at(i).xs.push_back(msg.get()->position.at(i).x);
        _allUavState.at(i).ys.push_back(msg.get()->position.at(i).y);
        _allUavState.at(i).zs.push_back(msg.get()->position.at(i).z);
        _allUavState.at(i).vxs.push_back(msg.get()->velocity.at(i).x);
        _allUavState.at(i).vys.push_back(msg.get()->velocity.at(i).y);
        _allUavState.at(i).vzs.push_back(msg.get()->velocity.at(i).z);
      }
    }
  }
//  else{
 //   ROS_INFO("The number of uavs_pose_vel message != the config object num");
  //}

//}
void *dt_target_object::runPlot(void *args){
  dt_target_object* ptr = (dt_target_object*)(args);
  ros::Rate rate(ptr->_plotHz);
  while(ros::ok()){
    if(ptr->_isPlot){
      plt::clf();
      for(int i=0;i<ptr->_allUavState.size();++i){
        vector<float> x,y/*,z*/;
        x.resize(ptr->_allUavState.at(i).xs.size());
        y.resize(ptr->_allUavState.at(i).ys.size());
        //z.resize(_allUavState.at(i).zs.size());
        for(int j=0;j<ptr->_allUavState.at(i).xs.size();++j){
          x.at(j) = ptr->_allUavState.at(i).xs.at(j);
          y.at(j) = ptr->_allUavState.at(i).ys.at(j);
          //z.at(j) = _allUavState.at(i).zs.at(j);
        }
        if(ptr->_isShowTargetObje){
        if(i==ptr->_allUavState.size()-1) plt::named_plot("Target",x,y);
        else {
          std::string name = "UAV"+std::to_string(i+1);
          plt::named_plot(name,x,y);
        }
        }
        else{
          std::string name = "UAV"+std::to_string(i+1);
          plt::named_plot(name,x,y);
        }
      }
      plt::xlim(-50, 50);
      // Add graph title
      plt::title("All Object Position Figure");
      // Enable legend.
      plt::legend();
      plt::pause(1.0/ptr->_plotHz);
      //rate.sleep();
    }
  }
}

void *dt_target_object::run(void *args)
{
  dt_target_object* ptr = (dt_target_object*)(args);
  ros::Rate rate(ptr->_msgPubHz);
  dt_message_package::CloudMessage pubMsg;
  pubMsg.SourceID = ptr->_sourceID;
  pubMsg.TargetID = ptr->_targetID;
  pubMsg.MessageID =TargetObjectPoseMessageID;
  TargetObjPoseMessage msg;
  AllUavStateMessage allUavStateMsg;
  int size = 0;
  if(ptr->_isShowTargetObje) size= ptr->_objectNum;
  else size = ptr->_objectNum+1;
  allUavStateMsg.xs.resize(size);
  allUavStateMsg.ys.resize(size);
  allUavStateMsg.zs.resize(size);
  allUavStateMsg.vxs.resize(size);
  allUavStateMsg.vys.resize(size);
  allUavStateMsg.vzs.resize(size);
  while(ros::ok())
  {
    {
      std::lock_guard<mutex> guard(ptr->m);
      msg.posX = ptr->_localPose.pose.position.x + ptr->_startPosX;
      msg.posY = ptr->_localPose.pose.position.y + ptr->_startPosY;
      msg.posZ = ptr->_localPose.pose.position.z + ptr->_startPosZ;

      msg.rotX = ptr->_localPose.pose.orientation.x;
      msg.rotY = ptr->_localPose.pose.orientation.y;
      msg.rotZ = ptr->_localPose.pose.orientation.z;
      msg.rotW = ptr->_localPose.pose.orientation.w;
    }
    pubMsg.TimeStamp = ros::Time::now().toNSec();
    pubMsg.MessageData = x2struct::X::tojson(msg);
    ptr->_cloudMsgPub.publish(pubMsg);
    for(int i=0;i<ptr->_objectNum;++i){
      allUavStateMsg.xs.at(i) = ptr->_allUavState.at(i).xs.at(ptr->_plotPointNum-1);
      allUavStateMsg.ys.at(i) = ptr->_allUavState.at(i).ys.at(ptr->_plotPointNum-1);
      allUavStateMsg.zs.at(i) = ptr->_allUavState.at(i).zs.at(ptr->_plotPointNum-1);
      allUavStateMsg.vxs.at(i) = ptr->_allUavState.at(i).vxs.at(ptr->_plotPointNum-1);
      allUavStateMsg.vys.at(i) = ptr->_allUavState.at(i).vys.at(ptr->_plotPointNum-1);
      allUavStateMsg.vzs.at(i) = ptr->_allUavState.at(i).vzs.at(ptr->_plotPointNum-1);
    }
    if(!ptr->_isShowTargetObje){
      allUavStateMsg.xs.at(ptr->_objectNum) = ptr->_localPose.pose.position.x;
      allUavStateMsg.ys.at(ptr->_objectNum) = ptr->_localPose.pose.position.y;
      allUavStateMsg.zs.at(ptr->_objectNum) = ptr->_localPose.pose.position.z;
    }
    pubMsg.MessageID = AllUavStateMessageID;
    pubMsg.MessageData = x2struct::X::tojson(allUavStateMsg);
    ptr->_cloudMsgPub.publish(pubMsg);
    rate.sleep();
  }
  pthread_join(ptr->_runThread,NULL);
}
void dt_target_object::cloudMsgCb(const dt_message_package::CloudMessageConstPtr &msg)
{
  switch (msg.get()->MessageID) {
  case UavControlID:
  {
    UavControl uavControlMsg;
    bool isLoad = x2struct::X::loadjson(msg.get()->MessageData,uavControlMsg,false);
    if(isLoad)
    {
      if(uavControlMsg.Mode==1)
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
        _targetVelPub.publish(targetVelMsg);
        //std::cout<<"dddddd"<<std::endl;
      }
    }
    else
      ROS_INFO("Unpack UavControl Message Fail!!!");
  }
    break;
  default:
    break;
  }
}

void dt_target_object::localPosSubCb(const geometry_msgs::PoseStampedConstPtr &msg)
{
  std::lock_guard<mutex> guard(m);
  _localPose = *msg;
  if(_isShowTargetObje){
    _allUavState.at(_objectNum-1).xs.push_back(msg.get()->pose.position.x);
    _allUavState.at(_objectNum-1).ys.push_back(msg.get()->pose.position.y);
    _allUavState.at(_objectNum-1).zs.push_back(msg.get()->pose.position.z);
  }
}

