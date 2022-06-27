#include "../include/uwb_message_wrapper/uwb_message_wrapper.h"

string uwb_message_wrapper::testPack(PosVelStu& data)
{
  return packData2(data);

}
std::pair<PosVelStu,bool> uwb_message_wrapper::testUnpack(string data)
{
  return unpackData2(data,_lastPosVel);
}



string uwb_message_wrapper::packData(PosVelStu data)
{
  string ret = "{";
  ret += std::to_string(data.px);
  ret += ",";
  ret += std::to_string(data.py);
  ret += ",";
  ret += std::to_string(data.pz);
  ret += ",";
  ret += std::to_string(data.vx);
  ret += ",";
  ret += std::to_string(data.vy);
  ret += ",";
  ret += std::to_string(data.vz);
  ret += "}";
  return ret;
}

string uwb_message_wrapper::removeNoiseData(string data)
{
  //vaild data: 0-9 and A -F ===> ASCII: 48-57 and 65-70
  int leftIndex = 0;
  int rightIndex = 0;
  while(rightIndex<data.size()){
    if((data.at(rightIndex)>=48&&data.at(rightIndex)<=57)||(data.at(rightIndex)>=65&&data.at(rightIndex)<=70)){
      data.at(leftIndex) = data.at(rightIndex);
      leftIndex++;
    }
    rightIndex++;
  }
  return data.substr(0,leftIndex);
}

/*
 * return pair<data,flag> flag == true: this packet is unpack.
*/
std::pair<PosVelStu,bool> uwb_message_wrapper::unpackData2(string &msg,PosVelStu &lastData){
  std::pair<PosVelStu,bool> ret;
  //remove noise data
  //std::cout<<"RawMsg"<<msg<<std::endl;
  string validMsg = removeNoiseData(msg);
  //std::cout<<"validMsg"<<validMsg<<std::endl;
  //
  //Extract data
  if(validMsg.size()>=40){
    string subStr = validMsg.substr(2,36);
    int errorFlag = validMsg.at(38)=='0'?1:0;
    //std::cout<<"errorFlag: "<<errorFlag<<std::endl;
    int oneNum = 0;
    for(int i=2;i<39;++i){
      oneNum+=oneSize(validMsg.at(i));
    }
    int trueValue= oneNum%2==0?1:0;
    //std::cout<<"trueValue: "<<trueValue<<std::endl;
    if(errorFlag==trueValue){
      string str = validMsg.substr(2,6);
      std::pair<float,bool> data = string2float(str);
      ret.first.px = data.second?data.first:lastData.px+lastData.vx*0.03;

      str = validMsg.substr(8,6);
      data = string2float(str);
      ret.first.py = data.second?data.first:lastData.py+lastData.vy*0.03;

      str = validMsg.substr(14,6);
      data = string2float(str);
      ret.first.pz = data.second?data.first:lastData.pz+lastData.vz*0.03;

      str = validMsg.substr(20,6);
      data = string2float(str);
      ret.first.vx = data.second?data.first:0;

      str = validMsg.substr(26,6);
      data = string2float(str);
      ret.first.vy = data.second?data.first:0;

      str = validMsg.substr(32,6);
      data = string2float(str);
      ret.first.vz = data.second?data.first:0;

      lastData.px = ret.first.px;
      lastData.py = ret.first.py;
      lastData.pz = ret.first.pz;
      lastData.vx = ret.first.vx;
      lastData.vy = ret.first.vy;
      lastData.vz = ret.first.vz;
      ret.second = true;
      return ret;
    }
    else {
      ret.second = false;
      return ret;
    }
  }
  else{
    //find a valid data;
    ret.second = false;
    return ret;
  }
}

int uwb_message_wrapper::oneSize(char data){
  switch (data) {
  case '0': return 0;
    break;
  case '1': return 1;
    break;
  case '2': return 1;
    break;
  case '3': return 2;
    break;
  case '4': return 1;
    break;
  case '5': return 2;
    break;
  case '6': return 2;
    break;
  case '7': return 3;
    break;
  case '8': return 1;
    break;
  case '9': return 2;
    break;
  case 'A': return 2;
    break;
  case 'B': return 3;
    break;
  case 'C': return 2;
    break;
  case 'D': return 3;
    break;
  case 'E': return 3;
    break;
  case 'F': return 4;
  default: return 0;
    break;
  }
}

/*
 * input: the state of UAV
 * output: the pack data: string ==" ": fail
*/

string uwb_message_wrapper::packData2(PosVelStu &data){
  string ret;
  ret.resize(40);
  ret.at(0) = 'F';
  ret.at(1) = 'E';
  ret.at(39) = 'F';
  // ret.at(38) = 'E';
  int i = 2;
  string dataStr = float2string(data.px)+float2string(data.py)+float2string(data.pz)+float2string(data.vx)+float2string(data.vy)+float2string(data.vz);
  if(dataStr.size()!=36){
    std::cout<<"The data pack fail"<<std::endl;
    return " ";
  }
  else{
    int oneNum = 0;
    for(auto val:dataStr){
      ret.at(i) = val;
      i++;
      oneNum+=oneSize(val);
    }
    // std::cout<<oneNum<<std::endl;
    if(oneNum%2) ret.at(38) = '0';
    else ret.at(38) = '1';
    //std::cout<<ret.at(38)<<std::endl;

    return ret;
  }
}

int uwb_message_wrapper::hex2number(char data){
  switch (data) {
  case '0': return 0;
    break;
  case '1': return 1;
    break;
  case '2': return 2;
    break;
  case '3': return 3;
    break;
  case '4': return 4;
    break;
  case '5': return 5;
    break;
  case '6': return 6;
    break;
  case '7': return 7;
    break;
  case '8': return 8;
    break;
  case '9': return 9;
    break;
  case 'A': return 10;
    break;
  case 'B': return 11;
    break;
  case 'C': return 12;
    break;
  case 'D': return 13;
    break;
  case 'E': return 14;
    break;
  case 'F': return 15;
    break;
  default:  return -1;//fail
    break;
  }
}

char uwb_message_wrapper::number2hex(int data){
  switch (data) {
  case 0: return '0';
    break;
  case 1: return '1';
    break;
  case 2: return '2';
    break;
  case 3: return '3';
    break;
  case 4: return '4';
    break;
  case 5: return '5';
    break;
  case 6: return '6';
    break;
  case 7: return '7';
    break;
  case 8: return '8';
    break;
  case 9: return '9';
    break;
  case 10: return 'A';
    break;
  case 11: return 'B';
    break;
  case 12: return 'C';
    break;
  case 13: return 'D';
    break;
  case 14: return 'E';
    break;
  case 15: return 'F';
    break;
  default: return ' ';
    break;
  }
}

/*
  input: data && bit(the result size)
  output: string type(size == bit)
  if(output==" ") this data transformation fail
 */
string uwb_message_wrapper::dec2hex(int data,int bit){
  vector<int> subData;
  while(data/16){
    subData.push_back(data%16);
    data /=16;
  }
  subData.push_back(data);
  //for(int i=0;i<subData.size();++i) std::cout<<subData.at(i)<<std::endl;
  if(subData.size()>bit) return " ";
  else{
    string ret;
    ret.resize(bit);
    for(auto &var:ret) var = '0';
    int retPtr = bit-1;
    int dataPtr = 0;
    while(dataPtr<subData.size()){
      ret.at(retPtr) = number2hex(subData.at(dataPtr));
      retPtr--;
      dataPtr++;
    }
    return ret;
  }
}

std::pair<float,bool> uwb_message_wrapper::string2float(string data){
  std::pair<float,bool> ret;
  ret.second = false;
  ret.first = 0;
  if(data.size()!=6){return ret;}
  else{
    if(data.at(0)!='0'&&data.at(0)!='1') return ret;
    int flag = data.at(0)=='0'?-1:1;
    int integer = 0;
    for(int i=1;i<=3;++i){
      if(hex2number(data.at(i))!=-1){
        integer+=pow(16,3-i)*hex2number(data.at(i));
      }else{return ret;}
    }

    int decimal = 0;
    for(int i=4;i<=5;++i){
      if(hex2number(data.at(i))!=-1){
        decimal+=pow(16,5-i)*hex2number(data.at(i));
      }else{return ret;}
    }

    float decimalData = decimal*0.01;
    ret.first = flag*(integer+decimalData);
    ret.second = true;
    return ret;
  }
}

string uwb_message_wrapper::float2string(float data){
  int flag = data>0?1:0;//the sign bit
  data = fabs(data);
  int integer = (int)data;
  int decimal = (int)((data-integer)*100);
  if(integer>4095) return " ";
  string ret;
  ret.resize(6);
  ret.at(0) = number2hex(flag);
  string dataStr = dec2hex(integer,3)+dec2hex(decimal,2);
  if(dataStr.size()!=5) {
    std::cout<<"The data from dec to hex fail, the size != 5"<<std::endl;
    return " ";
  }
  else{
    int i= 1;
    for(auto val:dataStr){
      ret.at(i) = val;
      i++;
    }
    return ret;
  }
}

std::pair<PosVelStu,bool> uwb_message_wrapper::unpackData(string msg)
{

}

std::pair<PosVelStu,bool> uwb_message_wrapper::getRecvData(string msg)
{
  std::pair<PosVelStu,bool> ret;
  ret.second = false;
  int dotSize = 5;
  int indexSize = 6;
  vector<int> dotPos;
  vector<int> indexPos;
  for(int i=0;i<msg.size();++i){
    if(msg.at(i)==',') {dotSize--;dotPos.push_back(i);}
    if(msg.at(i)==':') {indexSize--;indexPos.push_back(i+1);}
  }
  if(dotSize==0&&indexSize==0){
    ret.second = true;
    dotPos.push_back(msg.size()-1);
    ret.first.px = std::stof(msg.substr(indexPos.at(0),dotPos.at(0)-indexPos.at(0)));
    ret.first.py = std::stof(msg.substr(indexPos.at(1),dotPos.at(1)-indexPos.at(1)));
    ret.first.pz = std::stof(msg.substr(indexPos.at(2),dotPos.at(2)-indexPos.at(2)));
    ret.first.vx = std::stof(msg.substr(indexPos.at(3),dotPos.at(3)-indexPos.at(3)));
    ret.first.vy = std::stof(msg.substr(indexPos.at(4),dotPos.at(4)-indexPos.at(4)));
    ret.first.vz = std::stof(msg.substr(indexPos.at(5),dotPos.at(5)-indexPos.at(5)));
  }
  return ret;
}

bool uwb_message_wrapper::IsJsonData(string data)
{
  if(data[0] != '{')
    return false;
  int num = 1;
  int dotNum = 12;
  for(int i=1;i<data.length();++i)
  {
    if(data[i] == '"') dotNum--;

    if(data[i] == '{')
    {
      ++num;
      // cout<<"true json "<<i<<data[i]<<endl;
    }
    else if(data[i] =='}')
    {
      --num;
      //cout<<"true json "<<i<<data[i]<<endl;
    }
    if(num == 0&&data[data.length()-1] =='}') break;
  }
  if(dotNum==0&&num == 0&&data[data.length()-1] =='}')
  {
    // cout<<"true json"<<"  sucess"<<endl;
    return true;
  }
  else
    return false;
  //cout<<"true json"<<"  fali"<<endl;

}
string uwb_message_wrapper::CheckRecvData(string msg)
{
  string ret;
  if(msg[0] != '{')
  {
    ret = "";
  }
  else
  {
    int num = 0;
    for(int i=0;i<msg.length();i++)
    {
      if(msg[i] =='}')
      {
        num++;
      }
      if(num == 1)
      {
        ret = msg.substr(0,i+1);
        break;
      }
    }
  }
  return ret;
}

uwb_message_wrapper::uwb_message_wrapper()
{
  ros::NodeHandle n("~");
  std::string UwbMsgSubTopic = "/uwb/msg_from_other";
  n.getParam("UwbMsgSubTopic",UwbMsgSubTopic);

  std::string UwbMsgPubTopic = "/uwb/msg_to_other";
  n.getParam("UwbMsgPubTopic",UwbMsgPubTopic);

  std::string UavLocalPosSubTopic = "/mavros/local_position/pose";
  n.getParam("UavLocalPosSubTopic",UavLocalPosSubTopic);

  std::string UavVelSubTopic = "/mavros/local_position/velocity";
  n.getParam("UavVelSubTopic",UavVelSubTopic);

  _uavNum = 6;
  n.getParam("UavNum",_uavNum);

  _msgToOtherHz = 30.0;
  n.getParam("MsgToOtherHz",_msgToOtherHz);

  _uavId = 1;
  n.getParam("UavId",_uavId);

  _uavsPosVelPubHz = 30.0;
  n.getParam("UavsPosVelPubHz",_uavsPosVelPubHz);

  std::string UavsPosVelPubTopic = "/other_uav/state";
  n.getParam("UavsPosVelPubTopic",UavsPosVelPubTopic);

  std::string NetworkStatePubTopic = "/network/state";
  n.getParam("NetworkStatePubTopic",NetworkStatePubTopic);

  _startPositionX = 0;
  n.getParam("StartPositionX",_startPositionX);

  _startPositionY = 0;
  n.getParam("StartPositionY",_startPositionY);

  _startPositionZ = 0;
  n.getParam("StartPositionZ",_startPositionZ);

  std::string TargetObjectPosPubTopic = "/target/object/pose";
  n.getParam("TargetObjectPosPubTopic",TargetObjectPosPubTopic);

  _isCar = false;
  n.getParam("IsCar",_isCar);

  _posVels.resize(_uavNum+1);
  _lastPosVels.resize(_uavNum+1);

  for(int i=0;i<_uavNum+1;++i){
    _lastPosVels.at(i).px = -12.5+i*5;
    _lastPosVels.at(i).py = -60;
    _lastPosVels.at(i).pz = 0;
  }


  _uwbMsgSub = n.subscribe(UwbMsgSubTopic,1,&uwb_message_wrapper::uwbMsgSubCb,this);
  _localPosMsgSub = n.subscribe(UavLocalPosSubTopic,1,&uwb_message_wrapper::localPosMsgSubCb,this);
  _velMsgSub = n.subscribe(UavVelSubTopic,1,&uwb_message_wrapper::velMsgSubCb,this);

  _uwbMsgPub = n.advertise<std_msgs::String>(UwbMsgPubTopic,5);
  _uavsPosVelMsgPub = n.advertise<dt_message_package::uavs_pose_vel>(UavsPosVelPubTopic,5);
  _networkStateMsgPub = n.advertise<std_msgs::Bool>(NetworkStatePubTopic,5);
  _targetObjPosMsgPub = n.advertise<geometry_msgs::PoseStamped>(TargetObjectPosPubTopic,5);


  _msgToOtherTimer   = n.createTimer(ros::Duration(1/_msgToOtherHz), &uwb_message_wrapper::msgToOtherTimerCb, this);
  _uavsPosVelPubTimer = n.createTimer(ros::Duration(1/_uavsPosVelPubHz),&uwb_message_wrapper::uavsPosVelPubTimerCb,this);

}

void uwb_message_wrapper::uwbMsgSubCb(const nlink_parser::LinktrackNodeframe0ConstPtr &msg)
{
  std_msgs::Bool stateMsg;
  stateMsg.data = true;
  _networkStateMsgPub.publish(stateMsg);
  for(int i=0;i<msg.get()->nodes.size();++i)
  {
    std::string msgData = msg.get()->nodes.at(i).data;
    int id = msg.get()->nodes.at(i).id;
    std::pair<PosVelStu,bool> posvelMsg = unpackData2(msgData,_lastPosVels.at(id-1));
    if(posvelMsg.second){
      _posVels.at(id-1).px = posvelMsg.first.px;
      _posVels.at(id-1).py = posvelMsg.first.py;
      _posVels.at(id-1).pz = posvelMsg.first.pz;
      _posVels.at(id-1).vx = posvelMsg.first.vx;
      _posVels.at(id-1).vy = posvelMsg.first.vy;
      _posVels.at(id-1).vz = posvelMsg.first.vz;
    }
    /*
    if(IsJsonData(msgData))
    {
      msgData = CheckRecvData(msgData);
      if(msgData!=""){
        std::pair<PosVelStu,bool> posvelMsg = getRecvData(msgData);
        if(posvelMsg.second){
          _posVels.at(id-1).px = posvelMsg.first.px;
          _posVels.at(id-1).py = posvelMsg.first.py;
          _posVels.at(id-1).pz = posvelMsg.first.pz;
          _posVels.at(id-1).vx = posvelMsg.first.vx;
          _posVels.at(id-1).vy = posvelMsg.first.vy;
          _posVels.at(id-1).vz = posvelMsg.first.vz;
        }
      }else{std::cout<<"msgData=null"<<std::endl;}
    }else{std::cout<<"is not Json Data"<<std::endl;}
    */
  }
}

void uwb_message_wrapper::localPosMsgSubCb(const geometry_msgs::PoseStampedConstPtr &msg)
{
  _posVels.at(_uavId-1).px = msg.get()->pose.position.x+_startPositionX;
  _posVels.at(_uavId-1).py = msg.get()->pose.position.y+_startPositionY;
  _posVels.at(_uavId-1).pz = msg.get()->pose.position.z+_startPositionZ;
}

void uwb_message_wrapper::velMsgSubCb(const geometry_msgs::TwistStampedConstPtr &msg)
{
  _posVels.at(_uavId-1).vx = msg.get()->twist.linear.x;
  _posVels.at(_uavId-1).vy = msg.get()->twist.linear.y;
  _posVels.at(_uavId-1).vz = msg.get()->twist.linear.z;
}

void uwb_message_wrapper::msgToOtherTimerCb(const ros::TimerEvent &event)
{
  /*
  UavPosVelMessage msg;
  msg.px = _posVels.at(_uavId-1).px+_startPositionX;
  msg.py = _posVels.at(_uavId-1).py+_startPositionY;
  msg.pz = _posVels.at(_uavId-1).pz+_startPositionZ;
  msg.vx = _posVels.at(_uavId-1).vx;
  msg.vy = _posVels.at(_uavId-1).vy;
  msg.vz = _posVels.at(_uavId-1).vz;
  std::string msgData = x2struct::X::tojson(msg);
  std_msgs::String pubMsg;
  pubMsg.data = msgData;
  _uwbMsgPub.publish(pubMsg);
  */
  std::string msgData = packData2(_posVels.at(_uavId-1));
  if(msgData!=" "){
    std_msgs::String pubMsg;
    pubMsg.data = msgData;
    _uwbMsgPub.publish(pubMsg);
  }
}

void uwb_message_wrapper::uavsPosVelPubTimerCb(const ros::TimerEvent &event)
{
  dt_message_package::uavs_pose_vel msg;
  msg.header.frame_id ="uavs";
  msg.header.stamp = ros::Time::now();
  int num = 0;
  if(_isCar) num = _uavNum+1;
  else num = _uavNum;
  msg.uav_id.resize(num);
  msg.position.resize(num);
  msg.velocity.resize(num);

  for(int i = 0;i<num;++i)
  {
    msg.uav_id.at(i) = i+1;
    msg.position.at(i).x = _posVels.at(i).px;
    msg.position.at(i).y = _posVels.at(i).py;
    msg.position.at(i).z = _posVels.at(i).pz;
    msg.velocity.at(i).x = _posVels.at(i).vx;
    msg.velocity.at(i).y = _posVels.at(i).vy;
    msg.velocity.at(i).z = _posVels.at(i).vz;
    
  }
  _uavsPosVelMsgPub.publish(msg);

  geometry_msgs::PoseStamped targetObjPosMsg;
  targetObjPosMsg.pose.position.x = _posVels.at(_uavNum).px;
  targetObjPosMsg.pose.position.y = _posVels.at(_uavNum).py;
  targetObjPosMsg.pose.position.z = _posVels.at(_uavNum).pz;
  targetObjPosMsg.header.frame_id = "target_object";
  targetObjPosMsg.header.stamp = ros::Time::now();
  _targetObjPosMsgPub.publish(targetObjPosMsg);
}









