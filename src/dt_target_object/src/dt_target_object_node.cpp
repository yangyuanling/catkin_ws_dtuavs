#include "../include/dt_target_object/dt_target_object.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dt_target_object_node");
  dt_target_object dt_target_object_node;
  ros::spin();
}
