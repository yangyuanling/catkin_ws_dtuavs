#include "../include/px4_simulation_wrapper/px4_get_data_set_wrapper.h"

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "px4_get_data_set_wrapper");
  px4_get_data_set_wrapper get_data_wrapper;
  ros::spin();
  return 0;
}
