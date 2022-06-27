/************************lsk 1481054609@qq.com********************************/

#include <geometry_msgs/TwistStamped.h>
#include "ros/ros.h"
#include "iostream"
#include "geometry_msgs/PoseStamped.h"
#include "dt_message_package/uavs_pose_vel.h"
#include"dt_message_package/uav_target.h"
#include "multi_collab/swarm_crossing.h"
#include "multi_collab/swarm_hunting.h"
#include "std_msgs/Bool.h"
#include "iostream"

class multi_collab
{
public:
  multi_collab(char* argv[]) //////////////////////////////////////////
  {
    ros::NodeHandle private_nh_("~");

    //loading  global parameters
    ros::NodeHandle nh_;

    private_nh_.param("UAVS_NUM", uavs_num, 6);
    ROS_INFO("Multi UAVS NUM:%d",uavs_num);
    private_nh_.param<bool>("control_mode", control_mode, true);
    private_nh_.param("source_of_control_",  source_of_ctrl_, 0);
    private_nh_.param<double>("main_loop_duration_", main_loop_duration_, 0.2);
    std::string tgt_object_pos_sub_topic = "/target_obj/pose";//
    private_nh_.param<std::string>("tgt_object_pos_sub_topic",tgt_object_pos_sub_topic,"/target_obj/pose");//
    _target_obj_pos_sub = n_.subscribe(tgt_object_pos_sub_topic,1,&multi_collab::target_obj_pose_sub_cb,this);//

    std::string network_data_case_pub_topic = "/network/data/get";
    private_nh_.param<std::string>("network_data_case_pub_topic",network_data_case_pub_topic,"/network/data/get");



    _network_data_case_pub = n_.advertise<std_msgs::Bool>(network_data_case_pub_topic,1);
\

    if (source_of_ctrl_==0)
    {
      std::string uavs_pos_vel_topic = "uavs/sensor/pose_vel";
      private_nh_.param<std::string>("uavs_pos_vel_topic", uavs_pos_vel_topic, "uavs/sensor/pose_vel");
      _uavs_pos_vel_sub = n_.subscribe(uavs_pos_vel_topic, 1, &multi_collab::data_valid_sub_cb, this ); // subscribe  the states of the all uavs
    }
    else
    {
      std::string uav_target_topic ;
      private_nh_.param<std::string>("uav_target_topic", uav_target_topic, "uav/target/pose_vel");
      _game_joy_ctrl_sub = n_.subscribe(uav_target_topic, 1, &multi_collab::game_joy_ctrl_sub_cb, this ); // subscribe  the states of the all uavs
    }


    if(control_mode)
    {
      std::string tgt_vel_msg_sub_topic = "/R_UAV_1/Target/Velocity";
      private_nh_.param<std::string>("tgt_vel_msg_pub_topic", tgt_vel_msg_sub_topic, "/R_UAV_1/Target/Velocity");
      _targ_vel_pub = n_.advertise< geometry_msgs::TwistStamped > (tgt_vel_msg_sub_topic, 1);
    }
    else
    {
      std::string tgt_pose_msg_sub_topic = "/R_UAV_1/Target/Position";
      private_nh_.param<std::string>("tgt_pose_msg_pub_topic", tgt_pose_msg_sub_topic, "/R_UAV_1/Target/Position");
      _targ_pose_pub = n_.advertise< geometry_msgs::PoseStamped> (tgt_pose_msg_sub_topic, 1);
    }



    des_vel_x=0;
    des_vel_y=0;
    des_vel_z=0;
    des_pose_x=0;
    des_pose_y=0;
    des_pose_z=0;

    this->uav_id = (int)(atof(argv[1]) + 0.1);                ///////////////////////////////////////////////////
    printf("\nUAV ID:%d\n",this->uav_id);
    this->my_model.set_number_total(uavs_num);  ///////////////////////////////////////////////////
    double data[28];
    for (int i = 0; i < 28; i++)
    {
      data[i] = atof(argv[2+i]);
    }
    this->my_model.set_parameters(data);
    this->flag_crossing_former = false;
    this->trigger_swarm = false;
    
    double data_hunting[14];
    data_hunting[0] = atof(argv[2]); //r_com
    data_hunting[1] = atof(argv[3]); //r_agent
    data_hunting[2] = atof(argv[4]); //v_max
    data_hunting[3] = atof(argv[5]); //v_z_max
    data_hunting[4] = atof(argv[6]); //height
    data_hunting[5] = atof(argv[7]); //p_h

    for (int i = 0; i < 8; i++)
    {
      data_hunting[6+i] = atof(argv[30 +i]);
    }
    this->my_model_hunting.set_parameters(data_hunting);

    isGetData = false;
    isGetTargetObjPos = false;
    main_loop_timer_   = n_.createTimer(ros::Duration(main_loop_duration_), &multi_collab::mainLoopOnlineOffline, this);

  }

  void target_obj_pose_sub_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
    isGetTargetObjPos = true;
    target_obj_pos = *msg;
  }

  void mainLoopOnlineOffline(const ros::TimerEvent& event)
  {
    // std::cout<<"isGetData:"<<isGetData<<std::endl;
    if(isGetData)
      multi_uavs_coll();
  }

  ~multi_collab()
  {

  }
  void data_valid_sub_cb(const dt_message_package::uavs_pose_vel &msg)
  {
    _data_valid= msg;
    isGetData = true;
   // std::cout<<"ddddd"<<std::endl;
    std_msgs::Bool dataGetMsg;
    dataGetMsg.data = true;
    _network_data_case_pub.publish(dataGetMsg);
  }

  void game_joy_ctrl_sub_cb(const dt_message_package::uav_target &msg)
  {
    _data_joy_ctrl =msg;
    game_joy_control();
  }

  void game_joy_control()
  {
    tgt_vel_.twist.linear.x=_data_joy_ctrl.velocity.x;
    tgt_vel_.twist.linear.y=_data_joy_ctrl.velocity.y;
    tgt_vel_.twist.linear.z=_data_joy_ctrl.velocity.z;
    tgt_vel_.header.stamp =  ros::Time::now();
    _targ_vel_pub.publish(tgt_vel_);
  }

  void multi_uavs_coll()
  {
    /********************所有无人机的位置速度信息都在这个*************************/
    /*使用方式：这里是拿出ID号为0的无人机位置、速度信息

    pose_x=_data_valid.position[0].x;
    pose_y=_data_valid.position[0].y ;
    pose_z=_data_valid.position[0].z;

    vel_x=_data_valid.velocity[0].x;
    vel_y=_data_valid.velocity[0].y ;
    vel_z=_data_valid.velocity[0].z;
    */


    /*
    des_vel_x=_data_valid.velocity[3].x;
    des_vel_y=_data_valid.velocity[3].y;
    des_vel_z=_data_valid.velocity[3].z;
*/
    /*************往这里填写多机协同的代码**************/
    // printf("1111111111\n");
    geometry_msgs::Vector3 v_desired;
    if (!this->trigger_swarm)
    {
        v_desired = my_model.generateDesiredVelocity(this->uav_id,_data_valid.uav_id,_data_valid.position,_data_valid.velocity);
        if (this->flag_crossing_former != this->my_model.flag_finish_crossing)
        {
            this->trigger_swarm = true;
            std::cout<<"Start hunting!"<<std::endl;
        }
        // std::cout<<"?????"<<std::endl;
    }
    else
    {
      if(isGetTargetObjPos)
        v_desired = my_model_hunting.generateDesiredVelocity(this->uav_id,_data_valid.uav_id,_data_valid.position,_data_valid.velocity,
            this->target_obj_pos.pose.position.x,this->target_obj_pos.pose.position.y);
    }
    /*************解算出期望的速度值在这里进行赋值**************************/
    
    des_vel_x=v_desired.x;
    des_vel_y=v_desired.y;
    des_vel_z=v_desired.z;
    

    if(control_mode)
    {
      tgt_vel_.twist.linear.x=des_vel_x;
      tgt_vel_.twist.linear.y=des_vel_y;
      tgt_vel_.twist.linear.z=des_vel_z;
      tgt_vel_.header.stamp =  ros::Time::now();
      _targ_vel_pub.publish(tgt_vel_);
    }
    else
    {
      tgt_pose_.pose.position.x=des_pose_x;
      tgt_pose_.pose.position.y=des_pose_y;
      tgt_pose_.pose.position.z=des_pose_z;
      tgt_pose_.header.stamp =  ros::Time::now();
      _targ_pose_pub.publish(tgt_pose_);
    }
  }

private:  
  ros::NodeHandle n_;
  ros::Subscriber _uavs_pos_vel_sub;
  ros::Subscriber _game_joy_ctrl_sub;
  ros::Subscriber _target_obj_pos_sub;//==============The position subscriber of target object=================
  ros::Publisher _targ_vel_pub;
  ros::Publisher _targ_pose_pub;

  ros::Publisher _network_data_case_pub;

  bool control_mode;
  bool isGetData;
  bool isGetTargetObjPos;
  double main_loop_duration_;
  ros::Timer main_loop_timer_;

  geometry_msgs::PoseStamped target_obj_pos;//============The position of target object=======================

  geometry_msgs::PoseStamped tgt_pose_;
  geometry_msgs::TwistStamped tgt_vel_;
  int uavs_num;
  int source_of_ctrl_;
  dt_message_package::uavs_pose_vel _data_valid;
  dt_message_package::uav_target _data_joy_ctrl;
  double des_vel_x,des_vel_y,des_vel_z;
  double des_pose_x,des_pose_y,des_pose_z;

  int uav_id;                     ///////////////////////////////////////////////////
  swarm_crossing::model my_model; ///////////////////////////////////////////////////
  swarm_hunting::model my_model_hunting; ///////////////////////////////////////////////////
  bool trigger_swarm;
  bool flag_crossing_former;
};

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "muti_collab");
  //   int id = atoi(argv[1]);
  multi_collab multi_collab_(argv);
  dt_message_package::uavs_pose_vel test;
  test.position.resize(10);
  ROS_INFO("%d",test.position.size());
  ROS_INFO("%d",test.velocity.size());
  
  //   std::cout << id <<std::endl;

  //   double temp = 0.0;
  //   for (int i = 0; i < 29; i++)
  //   {
  //       temp = atof(argv[i+1]);
  //       std::cout << temp <<std::endl;
  //   }
  
  ros::spin();
  return 0;
}
