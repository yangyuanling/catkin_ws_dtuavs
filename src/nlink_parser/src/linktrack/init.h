#ifndef LINKTRACKINIT_H
#define LINKTRACKINIT_H

#include <ros/ros.h>
#include <serial/serial.h>

#include <unordered_map>

#include "nlink_unpack/nlink_utils.h"
#include "protocol_extracter/nprotocol_extracter.h"

class NProtocolExtracter;
namespace linktrack
{
  class Init
  {
  public:
    explicit Init(NProtocolExtracter *protocol_extraction,
                  serial::Serial *serial);

  private:
    void initDataTransmission();
    void initAnchorFrame0(NProtocolExtracter *protocol_extraction);
    void initTagFrame0(NProtocolExtracter *protocol_extraction);
    void initNodeFrame0(NProtocolExtracter *protocol_extraction);
    void initNodeFrame1(NProtocolExtracter *protocol_extraction);
    void initNodeFrame2(NProtocolExtracter *protocol_extraction);
    void initNodeFrame3(NProtocolExtracter *protocol_extraction);
    void initNodeFrame5(NProtocolExtracter *protocol_extraction);
    void initNodeFrame6(NProtocolExtracter *protocol_extraction);

    std::unordered_map<NProtocolBase *, ros::Publisher> publishers_;

    ros::Publisher uwb0_data_pub;//
    ros::NodeHandle nh_;
    ros::Subscriber dt_sub_;
    std::string RecvDataMsgTopic;
    std::string SendDataMsgTopic;
    std::string NlinkLinktrackNodeFrameTopic;
    std::string NlinkLinktrackNodeframe0Topic;
  };
} // namespace linktrack

#endif // LINKTRACKINIT_H
