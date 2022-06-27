#include "../include/dt_simulation_wrapper/dt_simulation_wrapper.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dt_simulation_wrapper_node");
  dt_simulation_wrapper node;
  ros::MultiThreadedSpinner spinner(2); // Use 2 threads
  spinner.spin(); // spin

}
