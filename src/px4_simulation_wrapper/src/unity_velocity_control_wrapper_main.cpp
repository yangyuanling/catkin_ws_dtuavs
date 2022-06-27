#include "../include/px4_simulation_wrapper/unity_velocity_control_wrapper.h"

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "px4_velocity_control_wrapper");
  unity_velocity_control_wrapper control_wrapper;
  ros::MultiThreadedSpinner spinner(1); // Use 2 threads
  spinner.spin(); // spin
  return 0;
}
