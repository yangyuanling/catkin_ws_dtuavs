#include "ros/ros.h"
#include "../include/uwb_wrapper/uwb_wrapper.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "uwb_wrapper_main");
  ros::NodeHandle nh;
  uwb_wrapper uwb;

  ros::spin();
  return 0;
}
