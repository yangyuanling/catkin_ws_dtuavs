#include"../include/video_transmission/video_transmission.h"
using namespace std;
CameraManager::CameraManager(ros::NodeHandle nh,std::string name,int hz,std::string path):
  nh_(nh),device_name_(name),hz_(hz),path_(path)
{
  delay_ = 1000.0/hz_;
  ros::NodeHandle nh_private;
  cout<<"delay: "<<delay_<<endl;
  capture_ = cv::VideoCapture(0);

  if(!capture_.isOpened())
    return;
  cv::Mat frame;
  capture_ >> frame;
  size_[0] = frame.size[0];
  size_[1] = frame.size[1];
  size_[2] = frame.size[2];
  string camera_topic_name = "/camera";
  string video_cloud_topic_name= "/to_cloud";
  string camera_compressed_name = "/camera/compressed";
  string apply_cam_name = "/apply/camera";
  string save_cam_name = "/save/image";
  ros::param::get("~/msg_to_cloud_name",video_cloud_topic_name);
  ros::param::get("~/camera_topic_name",camera_topic_name);
  ros::param::get("~/camera_compressed_name",camera_compressed_name);
  ros::param::get("~/cloud_msg_target_id",cloud_msg_target_id);
  ros::param::get("~/cloud_msg_source_id",cloud_msg_source_id);
  ros::param::get("~/camera_to_cloud_hz", camera_to_cloud_hz);
  ros::param::get("~/apply_cam_sub_name",apply_cam_name);
  ros::param::get("~/save_cam_sub_name",save_cam_name);
  send_time_s = 1.0/camera_to_cloud_hz;
  video_cloud_pub = nh_private.advertise<dt_message_package::CloudMessage>(video_cloud_topic_name,5);
  compressed_image_sub = nh_private.subscribe(camera_compressed_name,2,&CameraManager::compressedImageSub,this);
  apply_cam_sub = nh_private.subscribe(apply_cam_name,1,&CameraManager::applyCamSub,this);
  image_pub = nh_.advertise(camera_topic_name,5);
  save_image_srv =nh_private.advertiseService(save_cam_name,&CameraManager::saveImageServerCb,this);
  struct timeval tv;
  gettimeofday(&tv, NULL);
  last_send_time = tv.tv_sec;
  is_send = false;
  is_save = false;
}

bool CameraManager::saveImageServerCb(dt_message_package::save_image::Request &req, dt_message_package::save_image::Response &res)
{
   is_save = req.command;
   res.is_finish = true;
}

void CameraManager::applyCamSub(const std_msgs::BoolConstPtr &msg)
{
  is_send = msg.get()->data;
}

void CameraManager::compressedImageSub(const sensor_msgs::CompressedImageConstPtr &msg)
{
  if(is_send)
  {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    long rawtime_ms = tv.tv_sec * 1000 + tv.tv_usec / 1000;
    double current_send_time = tv.tv_sec;
    if(current_send_time-last_send_time>=send_time_s)
    {
      DTUAV::CompressedImageMessage img_msg;
      img_msg.format = msg.get()->format;
      img_msg.data.resize(msg.get()->data.size());
      for(int i=0;i<msg.get()->data.size();i++)
      {
        img_msg.data.at(i) = msg.get()->data.at(i);
      }
      dt_message_package::CloudMessage cloudMsg;
      cloudMsg.SourceID = cloud_msg_source_id;
      cloudMsg.TargetID = cloud_msg_target_id;
      cloudMsg.MessageID = CompressedImageMessageID;
      cloudMsg.MessageData = x2struct::X::tojson(img_msg);
      cloudMsg.TimeStamp = rawtime_ms;
      video_cloud_pub.publish(cloudMsg);
    }
    last_send_time = current_send_time;
  }
}

cv::Mat* CameraManager::read_image(bool save)
{
  cv::Mat frame,gray;
  capture_ >> frame;
  ros::Time timestamp = ros::Time::now();
  std::string time_second = std::to_string(timestamp.toSec()*1e9);
  std::string image_name(time_second);
  image_name.erase(19);
  image_name.append(".png");
  if(frame.empty())
  {
    //std::cout << "frequency is too high"<<std::endl;
    return nullptr;
  }
  cv::cvtColor(frame, gray, CV_BGR2GRAY);
  RGB_ = frame;
  GRAY_ = gray;
  if(save)
  {
    save_image(image_name, gray);
  }
  return &RGB_;
}


void CameraManager::spin(bool ros_send ,bool save,bool visualization)
{    
  sensor_msgs::ImagePtr msg;
  while(ros::ok())
  {
    cv::Mat* imagePtr = read_image(is_save);
    if(visualization) imshow("image_gray",*imagePtr);
    if(imagePtr&&ros_send)
    {
      msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", *imagePtr).toImageMsg();
      image_pub.publish(msg);
    }
    cv::waitKey(delay_);
    ros::spinOnce();
  }
}

