#include "../include/px4_simulation_wrapper/unity_velocity_control_wrapper.h"

unity_velocity_control_wrapper::unity_velocity_control_wrapper()
{
  ros::NodeHandle n("~");
  std::string px4_ref_vel_msg_pub_topic = "/mavros/setpoint_velocity/cmd_unstamp";
  n.getParam("px4_ref_vel_msg_pub_topic",px4_ref_vel_msg_pub_topic);
  std::string tgt_vel_msg_sub_topic = "/other/target/velocity";
  n.getParam("tgt_vel_msg_sub_topic",tgt_vel_msg_sub_topic);
  _cmd_reset_duration = 1.0;
  n.getParam("cmd_reset_time",_cmd_reset_duration);
  n.getParam("run_frequen_hz",_run_frequen_hz);
  std::string network_data_case_pub_topic = "/network/data/get";
  n.getParam("network_data_case_pub_topic",network_data_case_pub_topic);
  _network_data_case_sub = n.subscribe(network_data_case_pub_topic,1,&unity_velocity_control_wrapper::network_data_get_cb,this);

  _is_get_data = false;
  _px4_ref_vel_msg_pub = n.advertise<geometry_msgs::Twist>(px4_ref_vel_msg_pub_topic,1);
   _tgt_vel_msg_sub = n.subscribe(tgt_vel_msg_sub_topic,1,&unity_velocity_control_wrapper::tgt_vel_msg_sub_cb,this);
  _cmd_reset_timer   = n.createTimer(ros::Duration(_cmd_reset_duration), &unity_velocity_control_wrapper::cmd_reset_time_cb, this);

  int flag_thread = pthread_create(&_offboard_control_run,NULL,&unity_velocity_control_wrapper::offboard_run,this);
  if (flag_thread < 0)
  {
    ROS_ERROR("pthread_create ros_process_thread failed: %d\n", flag_thread);
  }
}

void unity_velocity_control_wrapper::tgt_vel_msg_sub_cb(const geometry_msgs::TwistStampedConstPtr &msg)
{
  if(_is_get_data)
  {
    _ref_velocity_x = msg.get()->twist.linear.x;
    _ref_velocity_y = msg.get()->twist.linear.y;
    _ref_velocity_z = msg.get()->twist.linear.z;

  }

}

void unity_velocity_control_wrapper::network_data_get_cb(const std_msgs::BoolConstPtr &msg)
{
   _is_get_data = true;
}

void unity_velocity_control_wrapper::cmd_reset_time_cb(const ros::TimerEvent& event)
{
  {
    if(!_is_get_data)
    {
      _ref_velocity_x = 0;
      _ref_velocity_y = 0;
      _ref_velocity_z = 0;
    }
    else _is_get_data = false;
  }

}

void *unity_velocity_control_wrapper::offboard_run(void *args)
{
  unity_velocity_control_wrapper* control_wrapper = (unity_velocity_control_wrapper*)(args);
  ros::Rate rate(20);
  geometry_msgs::Twist target_vel_msg;
  while(ros::ok())
  {
    target_vel_msg.linear.x = control_wrapper->_ref_velocity_x;
    target_vel_msg.linear.y = control_wrapper->_ref_velocity_y;
    target_vel_msg.linear.z = control_wrapper->_ref_velocity_z;
    control_wrapper->_px4_ref_vel_msg_pub.publish(target_vel_msg);
    rate.sleep();
  }
  pthread_join(control_wrapper->_offboard_control_run,NULL);
}
