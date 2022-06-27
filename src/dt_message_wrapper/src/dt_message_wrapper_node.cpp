#include "../include/dt_message_wrapper/dt_message_wrapper.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dt_message_wrapper_node");
  dt_message_wrapper node;
  ros::MultiThreadedSpinner spinner(2); // Use 2 threads
  spinner.spin(); // spin
}
