#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <string>
#include <iostream>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CompressedImage.h>
#include <std_msgs/Header.h>
#include <sys/stat.h>
#include "sys/time.h"
#include <dt_common/define_common.h>
#include <dt_message_package/CloudMessage.h>
#include <std_msgs/Bool.h>
#include <dt_message_package/save_image.h>
#include "ros/service.h"
class CameraManager
{
public:
    CameraManager(ros::NodeHandle nh,std::string name,int hz,std::string path="./");
    cv::Mat* read_image(bool save);
    const cv::Mat& getRGB() const{return RGB_;}
    const cv::Mat& getGRY() const{return GRAY_;}
    const double get_delay() const{return delay_;}
bool save_image(std::string name,cv::Mat& image)
    {
         //std::cout << path_+name << std::endl;
         return cv::imwrite(path_+name,image);
    }
  void spin(bool ros_send = true ,bool save = false,bool visualization = false);
  void compressedImageSub(const sensor_msgs::CompressedImageConstPtr& msg);
  void applyCamSub(const std_msgs::BoolConstPtr& msg);
  bool saveImageServerCb(dt_message_package::save_image::Request& req,dt_message_package::save_image::Response& res);

private:
 image_transport::ImageTransport nh_;

 image_transport::Publisher image_pub;
 ros::Publisher video_cloud_pub;
 ros::Subscriber compressed_image_sub;
 ros::Subscriber apply_cam_sub;

 ros::ServiceServer save_image_srv;
 int cloud_msg_source_id;
 int cloud_msg_target_id;
 float camera_to_cloud_hz;
 float send_time_s;
 double last_send_time;
 bool is_send;
 bool is_save;
   //for visualization
    cv::Mat GRAY_;
    cv::Mat RGB_;
   cv::VideoCapture capture_;
   int size_[3];
   int hz_;
    double delay_;
    std::string device_name_;
    std::string path_;
};

