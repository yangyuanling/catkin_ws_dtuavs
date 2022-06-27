#include "../include/uwb_message_wrapper/uwb_message_wrapper.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "uwb_message_wrapper_main");
  ros::NodeHandle nh;
  uwb_message_wrapper  node;
  ros::MultiThreadedSpinner spinner(2); // Use 2 threads
  //std::cout<<std::stof("2d0.d03ddd")<<std::endl;
  PosVelStu stateData;
  stateData.px = -153.43;
  stateData.py = 34.444;
  stateData.pz = 123.4445;
  stateData.vx = 23.54222;
  stateData.vy = 1.234;
  stateData.vz = -0.334;
  //std::cout<<node.testPack(stateData)<<std::endl;

  //string data = node.testPack(stateData);
/*
  string data ="((((((((((>>>>>>FE00992A1,,,,0222C1///dddaaaa07B2C1017361001170000211FSSSS{}}}}}}}}}}}}}}}}}}dasjdSSSDDDDDdfsd";

  std::pair<PosVelStu,bool> unpackData = node.testUnpack(data);
  std::cout<<unpackData.second<<std::endl;
  std::cout<<unpackData.first.px<<std::endl;
  std::cout<<unpackData.first.py<<std::endl;
  std::cout<<unpackData.first.pz<<std::endl;
  std::cout<<unpackData.first.vx<<std::endl;
  std::cout<<unpackData.first.vy<<std::endl;
  std::cout<<unpackData.first.vz<<std::endl;

*/
  spinner.spin(); // spin

  ROS_INFO("Hello world!");
}
