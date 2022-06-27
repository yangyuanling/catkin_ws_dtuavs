#include "../include/px4_control_wrapper/px4_velocity_rc_control_wrapper.h"

void px4_velocity_rc_control_wrapper::data_valid_sub_cb(const std_msgs::BoolConstPtr &msg)
{
  _data_valid = msg.get()->data;
}

void px4_velocity_rc_control_wrapper::tgt_vel_msg_sub_cb(const geometry_msgs::TwistStampedConstPtr &msg)
{
  if(!_is_land_flag&&_is_start_other_control_flag)
  {
    if(_is_check_data_valid)
    {
      if(_data_valid)
      {
        _ref_velocity_x = msg.get()->twist.linear.x;
        _ref_velocity_y = msg.get()->twist.linear.y;
        _ref_velocity_z = msg.get()->twist.linear.z;
      }
      else
      {
        _ref_velocity_x = 0;
        _ref_velocity_y = 0;
        _ref_velocity_z = 0;
      }
    }
    else
    {
      _ref_velocity_x = msg.get()->twist.linear.x;
      _ref_velocity_y = msg.get()->twist.linear.y;
      _ref_velocity_z = msg.get()->twist.linear.z;
    }
  }
  else
  {
    _ref_velocity_x = 0;
    _ref_velocity_y = 0;
    _ref_velocity_z = -0.6;
  }
}

void px4_velocity_rc_control_wrapper::px4_rc_msg_sub_cb(const mavros_msgs::RCInConstPtr &msg)
{
  if(msg.get()->channels.at(_rc_start_ch)>1800&&msg.get()->channels.at(_rc_start_ch)<2000&&!_is_land_flag)
  {
    _is_start_other_control_flag = true;
    if(!_is_set_start)
    {
      std::cout<<"start_uav"<<std::endl;
      _is_set_start = true;
    }
    _reset_start_pos = true;
  }
  else
  {
    _is_start_other_control_flag = false;
  }
  if(msg.get()->channels.at(_rc_land_ch)>1800&&msg.get()->channels.at(_rc_land_ch)<2000)
  {
    _is_land_flag = true;
    _ref_velocity_x = 0;
    _ref_velocity_y = 0;
    _ref_velocity_z = -0.6;
    _is_set_start = false;
  }
  else
  {
    _is_land_flag = false;
  }

}

void *px4_velocity_rc_control_wrapper::offboard_run(void *args)
{
  px4_velocity_rc_control_wrapper* control_wrapper = (px4_velocity_rc_control_wrapper*)(args);
  ros::Rate rate(control_wrapper->_run_frequen_hz);
  while(ros::ok())
  {
    control_wrapper->_px4_ref_vel.linear.x = control_wrapper->_ref_velocity_x;
    control_wrapper->_px4_ref_vel.linear.y = control_wrapper->_ref_velocity_y;
    control_wrapper->_px4_ref_vel.linear.z = control_wrapper->_ref_velocity_z;
    control_wrapper->_px4_ref_vel_msg_pub.publish(control_wrapper->_px4_ref_vel);
    rate.sleep();

  }
  pthread_join(control_wrapper->_offboard_control_run,NULL);
}


void px4_velocity_rc_control_wrapper::uav_pos_msg_sub_cb(const geometry_msgs::PoseStampedConstPtr &msg)
{
  if(_is_local_pos)
  {
    if(_reset_start_pos&&!_initStartPos)
    {
      _start_pos_x = msg.get()->pose.position.x;
      _start_pos_y = msg.get()->pose.position.y;
      _start_pos_z = msg.get()->pose.position.z;
      _initStartPos = true;
    }
    else if(_reset_start_pos&&_initStartPos)
    {
      geometry_msgs::PoseStamped local_pos;
      local_pos.header = msg.get()->header;
      local_pos.pose.orientation = msg.get()->pose.orientation;
      local_pos.pose.position.x = msg.get()->pose.position.x - _start_pos_x;
      local_pos.pose.position.y = msg.get()->pose.position.y - _start_pos_y;
      local_pos.pose.position.z = msg.get()->pose.position.z;//height stable
      _uav_local_msg_pub.publish(local_pos);
    }
    else
    {
      ROS_INFO("Wait reset start position.....");
    }
  }
  else
  {
    _uav_local_msg_pub.publish(msg);
  }
}

px4_velocity_rc_control_wrapper::px4_velocity_rc_control_wrapper()
{
  ros::NodeHandle n("~");
  n.getParam("px4_ref_vel_msg_pub_topic",_px4_ref_vel_msg_pub_topic);
  n.getParam("tgt_vel_msg_sub_topic",_tgt_vel_msg_sub_topic);
  n.getParam("px4_rc_msg_sub_topic",_px4_rc_msg_sub_topic);
  n.getParam("run_frequen_hz",_run_frequen_hz);
  n.getParam("rc_start_ch",_rc_start_ch);
  n.getParam("rc_land_ch",_rc_land_ch);
  n.getParam("is_check_data_valid",_is_check_data_valid);
  n.getParam("data_valid_sub_topic",_data_valid_sub_topic);
  std::cout<<"Is check data valid: "<<_is_check_data_valid<<std::endl;
  if(!n.getParam("is_local_pos",_is_local_pos))
  {
    _is_local_pos = false;
  }

  std::string uav_pos_sub_topic = "/mavros/local_position/pose";
  std::string uav_local_pos_pub_topic = "/uav/local_position/local_pose";

  n.getParam("uav_pos_sub_topic",uav_pos_sub_topic);
  n.getParam("uav_local_pos_pub_topic",uav_local_pos_pub_topic);

  _uav_pos_msg_sub = n.subscribe(uav_pos_sub_topic,1,&px4_velocity_rc_control_wrapper::uav_pos_msg_sub_cb,this);

  _tgt_vel_msg_sub = n.subscribe(_tgt_vel_msg_sub_topic,1,&px4_velocity_rc_control_wrapper::tgt_vel_msg_sub_cb,this);
  _px4_rc_msg_sub = n.subscribe(_px4_rc_msg_sub_topic,1,&px4_velocity_rc_control_wrapper::px4_rc_msg_sub_cb,this);
  _data_valid_sub = n.subscribe(_data_valid_sub_topic,1,&px4_velocity_rc_control_wrapper::data_valid_sub_cb,this);
  _px4_ref_vel_msg_pub = n.advertise<geometry_msgs::Twist>(_px4_ref_vel_msg_pub_topic,1);
  _uav_local_msg_pub = n.advertise<geometry_msgs::PoseStamped>(uav_local_pos_pub_topic,1);

  _px4_ref_vel.linear.x = 0;
  _px4_ref_vel.linear.y = 0;
  _px4_ref_vel.linear.z = 0;

  _ref_velocity_x = 0;
  _ref_velocity_y = 0;
  _ref_velocity_z = 0;

  _data_valid = false;


  _start_pos_x = 0;
  _start_pos_y = 0;
  _start_pos_z = 0;

  _is_start_other_control_flag = false;
  _is_land_flag = false;
  _is_set_start = false;
  _is_local_pos = false;
  _initStartPos = false;
  _reset_start_pos = false;

  int flag_thread = pthread_create(&_offboard_control_run,NULL,&px4_velocity_rc_control_wrapper::offboard_run,this);
  if (flag_thread < 0)
  {
    ROS_ERROR("pthread_create ros_process_thread failed: %d\n", flag_thread);
  }
}
