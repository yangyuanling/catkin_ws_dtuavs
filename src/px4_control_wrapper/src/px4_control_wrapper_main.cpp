#include "../include/px4_control_wrapper/px4_control_wrapper.h"

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "px4_control_wrapper");
  px4_control_wrapper control_wrapper;
  ros::spin();
  return 0;
}
