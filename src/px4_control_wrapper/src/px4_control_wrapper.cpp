#include "../include/px4_control_wrapper/px4_control_wrapper.h"

void px4_control_wrapper::data_valid_sub_cb(const std_msgs::BoolConstPtr &msg)
{
  _data_valid = msg.get()->data;
}

void px4_control_wrapper::tgt_vel_msg_sub_cb(const geometry_msgs::TwistStampedConstPtr &msg)
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

void px4_control_wrapper::px4_rc_msg_sub_cb(const mavros_msgs::RCInConstPtr &msg)
{
  if(_is_rc_control)
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
}

void *px4_control_wrapper::offboard_run(void *args)
{
  px4_control_wrapper* control_wrapper = (px4_control_wrapper*)(args);
  ros::Rate rate(control_wrapper->_run_frequen_hz);
  while(ros::ok())
  {
    if(!control_wrapper->_is_vr_control)
    {
      if(control_wrapper->_control_mode == 0)
      {
        //position control
        control_wrapper->_px4_ref_pos.pose.position.x = control_wrapper->_ref_pos_x;
        control_wrapper->_px4_ref_pos.pose.position.y = control_wrapper->_ref_pos_y;
        control_wrapper->_px4_ref_pos.pose.position.z = control_wrapper->_ref_pos_z;
        control_wrapper->_px4_ref_pos.pose.orientation.x = control_wrapper->_ref_rot_x;
        control_wrapper->_px4_ref_pos.pose.orientation.y = control_wrapper->_ref_rot_y;
        control_wrapper->_px4_ref_pos.pose.orientation.z = control_wrapper->_ref_rot_z;
        control_wrapper->_px4_ref_pos.pose.orientation.w = control_wrapper->_ref_rot_w;
        control_wrapper->_px4_ref_pos_msg_pub.publish(control_wrapper->_px4_ref_pos);
      }
      else if(control_wrapper->_control_mode == 1)
      {
        //velocity control
        control_wrapper->_px4_ref_vel.linear.x = control_wrapper->_ref_velocity_x;
        control_wrapper->_px4_ref_vel.linear.y = control_wrapper->_ref_velocity_y;
        control_wrapper->_px4_ref_vel.linear.z = control_wrapper->_ref_velocity_z;
        control_wrapper->_px4_ref_vel_msg_pub.publish(control_wrapper->_px4_ref_vel);
      }
    }
    rate.sleep();
  }
  pthread_join(control_wrapper->_offboard_control_run,NULL);
}


void px4_control_wrapper::uav_pos_msg_sub_cb(const geometry_msgs::PoseStampedConstPtr &msg)
{
  _current_height = msg.get()->pose.position.z;
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
      // ROS_INFO("Wait reset start position.....");
    }
  }
  else
  {
    _uav_local_msg_pub.publish(msg);
  }
}

void px4_control_wrapper::tgt_pos_msg_sub_cb(const geometry_msgs::PoseStampedConstPtr &msg)
{
  _ref_pos_x = msg.get()->pose.position.x;
  _ref_pos_y = msg.get()->pose.position.y;
  _ref_pos_z = msg.get()->pose.position.z;

  if(_is_set_rotation)
  {
    _ref_rot_x = msg.get()->pose.orientation.x;
    _ref_rot_y = msg.get()->pose.orientation.y;
    _ref_rot_z = msg.get()->pose.orientation.z;
    _ref_rot_w = msg.get()->pose.orientation.w;
  }
}

void *px4_control_wrapper::auto_run(void *args)
{
  px4_control_wrapper* control_wrapper = (px4_control_wrapper*)(args);
  ros::Rate rate(control_wrapper->_run_frequen_hz);
  while(ros::ok()&&control_wrapper->_is_vr_control)
  {
    if(control_wrapper->_control_mode == 0)
    {
      //position control
      control_wrapper->_px4_ref_pos.pose.position.x = control_wrapper->_ref_pos_x;
      control_wrapper->_px4_ref_pos.pose.position.y = control_wrapper->_ref_pos_y;
      control_wrapper->_px4_ref_pos.pose.position.z = control_wrapper->_ref_pos_z;
      control_wrapper->_px4_ref_pos.pose.orientation.x = control_wrapper->_ref_rot_x;
      control_wrapper->_px4_ref_pos.pose.orientation.y = control_wrapper->_ref_rot_y;
      control_wrapper->_px4_ref_pos.pose.orientation.z = control_wrapper->_ref_rot_z;
      control_wrapper->_px4_ref_pos.pose.orientation.w = control_wrapper->_ref_rot_w;
      control_wrapper->_px4_ref_pos_msg_pub.publish(control_wrapper->_px4_ref_pos);
    }
    else if(control_wrapper->_control_mode == 1)
    {
      //velocity control
      control_wrapper->_px4_ref_vel.linear.x = control_wrapper->_ref_velocity_x;
      control_wrapper->_px4_ref_vel.linear.y = control_wrapper->_ref_velocity_y;
      control_wrapper->_px4_ref_vel.linear.z = control_wrapper->_ref_velocity_z;
      control_wrapper->_px4_ref_vel_msg_pub.publish(control_wrapper->_px4_ref_vel);
    }
    rate.sleep();
  }
  pthread_join(control_wrapper->_offboard_control_run,NULL);
  control_wrapper->_is_start_vr_control = false;
  control_wrapper->_ref_velocity_x = 0;
  control_wrapper->_ref_velocity_y = 0;
  control_wrapper->_ref_velocity_z = 0;
}


void px4_control_wrapper::vr_control_msg_sub_cb(const std_msgs::BoolConstPtr &msg)
{
  _is_vr_control = msg.get()->data;
  if(_is_vr_control&&!_is_start_vr_control)
  {
    _is_start_vr_control = true;
    int flag_thread = pthread_create(&_auto_run,NULL,&px4_control_wrapper::auto_run,this);
    if (flag_thread < 0)
    {
      ROS_ERROR("pthread_create ros_process_thread failed: %d\n", flag_thread);
    }
  }
}

void px4_control_wrapper::arm_msg_sub_cb(const std_msgs::BoolConstPtr &msg)
{
  if(_is_vr_control)
  {
    if(!_px4_current_state.armed == msg.get()->data)
    {
      _px4_arming.request.value = msg.get()->data;

      if( _px4_arming_client.call(_px4_arming) &&
          _px4_arming.response.success)
      {
        ROS_INFO("Carry out the arm command sucessfully!!!");
      }
    }
  }
}

void px4_control_wrapper::flight_mode_msg_sub_cb(const std_msgs::Int8ConstPtr &msg)
{
  /*
  * 0: the manual mode
  * 1: the stabilizing mode
  * 2: the altitude mode
  * 3: the position mode
  * 4: the offboard mode
  * 5: the return mode
  */

  if(msg.get()->data == 4)
  {
    _px4_set_mode.request.custom_mode = "OFFBOARD";
    if(_px4_current_state.mode != "OFFBOARD")
    {
      if(_px4_set_mode_client.call(_px4_set_mode) &&
         _px4_set_mode.response.mode_sent)
      {
        ROS_INFO("Offboard enabled");
      }
    }
  }
}


void px4_control_wrapper::px4_state_msg_sub_cb(const mavros_msgs::StateConstPtr &msg)
{
  _px4_current_state = *msg;
}
px4_control_wrapper::px4_control_wrapper()
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
  if(!n.getParam("is_rc_control",_is_rc_control))
  {
    _is_rc_control = true;
  }
  std::string uav_pos_sub_topic = "/mavros/local_position/pose";
  std::string uav_local_pos_pub_topic = "/uav/local_position/local_pose";

  n.getParam("uav_pos_sub_topic",uav_pos_sub_topic);
  n.getParam("uav_local_pos_pub_topic",uav_local_pos_pub_topic);

  std::string tgt_pos_msg_sub_topic = "/other/target/position";
  n.getParam("tgt_pos_msg_sub_topic",tgt_pos_msg_sub_topic);

  std::string px4_ref_pos_msg_pub_topic = "/mavros/setpoint_position/pose";
  n.getParam("px4_ref_pos_msg_pub_topic",px4_ref_pos_msg_pub_topic);

  if(!n.getParam("is_set_rotation",_is_set_rotation))
    _is_set_rotation = false;

  if(!n.getParam("control_mode",_control_mode))
    _control_mode = 0;

  std::string vr_control_sub_topic = "/R_UAV_0/VR/Control";
  n.getParam("vr_control_sub_topic",vr_control_sub_topic);

  std::string arm_sub_topic = "/R_UAV_0/Set/Arm";
  n.getParam("arm_msg_sub_topic",arm_sub_topic);

  std::string fligth_mode_msg_pub_topic = "/R_UAV_0/Set/FMode";
  n.getParam("fligth_mode_msg_pub_topic",fligth_mode_msg_pub_topic);

  std::string px4_state_msg_sub_topic = "/mavros/state";
  n.getParam("px4_state_msg_sub_topic",px4_state_msg_sub_topic);
  std::string px4_arming_client_topic = "/mavros/cmd/arming";
  n.getParam("px4_arming_client_topic",px4_arming_client_topic);
  std::string px4_set_mode_client_topic = "/mavros/set_mode";
  n.getParam("px4_set_mode_client_topic",px4_set_mode_client_topic);

  _px4_state_msg_sub = n.subscribe(px4_state_msg_sub_topic,1,&px4_control_wrapper::px4_state_msg_sub_cb,this);
  _px4_arming_client = n.serviceClient<mavros_msgs::CommandBool>(px4_arming_client_topic);
  _px4_set_mode_client = n.serviceClient<mavros_msgs::SetMode>(px4_set_mode_client_topic);


  _uav_pos_msg_sub = n.subscribe(uav_pos_sub_topic,1,&px4_control_wrapper::uav_pos_msg_sub_cb,this);

  _tgt_vel_msg_sub = n.subscribe(_tgt_vel_msg_sub_topic,1,&px4_control_wrapper::tgt_vel_msg_sub_cb,this);
  _tgt_pos_msg_sub = n.subscribe(tgt_pos_msg_sub_topic,1,&px4_control_wrapper::tgt_pos_msg_sub_cb,this);

  _px4_rc_msg_sub = n.subscribe(_px4_rc_msg_sub_topic,1,&px4_control_wrapper::px4_rc_msg_sub_cb,this);
  _data_valid_sub = n.subscribe(_data_valid_sub_topic,1,&px4_control_wrapper::data_valid_sub_cb,this);

  _vr_control_sub = n.subscribe(vr_control_sub_topic,1,&px4_control_wrapper::vr_control_msg_sub_cb,this);

  _arm_com_sub = n.subscribe(arm_sub_topic,1,&px4_control_wrapper::arm_msg_sub_cb,this);

  _flight_mode_msg_sub = n.subscribe(fligth_mode_msg_pub_topic,1,&px4_control_wrapper::flight_mode_msg_sub_cb,this);

  _px4_ref_vel_msg_pub = n.advertise<geometry_msgs::Twist>(_px4_ref_vel_msg_pub_topic,1);
  _px4_ref_pos_msg_pub = n.advertise<geometry_msgs::PoseStamped>(px4_ref_pos_msg_pub_topic,1);

  _uav_local_msg_pub = n.advertise<geometry_msgs::PoseStamped>(uav_local_pos_pub_topic,1);

  _px4_ref_vel.linear.x = 0;
  _px4_ref_vel.linear.y = 0;
  _px4_ref_vel.linear.z = 0;

  _px4_ref_pos.pose.orientation.x = 0;
  _px4_ref_pos.pose.orientation.y = 0;
  _px4_ref_pos.pose.orientation.z = 0;
  _px4_ref_pos.pose.orientation.w = 1;
  _px4_ref_pos.pose.position.x = 0;
  _px4_ref_pos.pose.position.y = 0;
  _px4_ref_pos.pose.position.z = -50;

  _ref_velocity_x = 0;
  _ref_velocity_y = 0;
  _ref_velocity_z = 0;

  _ref_pos_x = 0;
  _ref_pos_y = 0;
  _ref_pos_z = -50;

  _ref_rot_x = 0;
  _ref_rot_y = 0;
  _ref_rot_z = 0;
  _ref_rot_w = 1;

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
  _is_start_vr_control = false;
  _is_vr_control = false;

  int flag_thread = pthread_create(&_offboard_control_run,NULL,&px4_control_wrapper::offboard_run,this);
  if (flag_thread < 0)
  {
    ROS_ERROR("pthread_create ros_process_thread failed: %d\n", flag_thread);
  }
}
