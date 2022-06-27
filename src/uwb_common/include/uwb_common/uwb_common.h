#ifndef UWB_COMMON_H
#define UWB_COMMON_H
#include "iostream"
#include "x2struct/x2struct.hpp"
class uwb_common
{
public:
    uwb_common();
};

struct UavPosVelMessage
{
  float px;
  float py;
  float pz;
  float vx;
  float vy;
  float vz;
  XTOSTRUCT(O(px,py,pz,vx,vy,vz));
};

struct UwbPoseMessage
{
    float p_x;
    float p_y;
    float p_z;
    float p_inc_x;
    float p_inc_y;
    float p_inc_z;
    XTOSTRUCT(O(p_x,p_y,p_z,p_inc_x,p_inc_y,p_inc_z));
};

struct LightFlowMessage
{
    float p_inc_x;
    float p_inc_y;
    float p_inc_z;
    XTOSTRUCT(O(p_inc_x,p_inc_y,p_inc_z));
};

struct LightFlowStateEstMessage
{
    float p_inc_x;
    float p_inc_y;
    float p_inc_z;
    float e_x;
    float e_y;
    XTOSTRUCT(O(p_inc_x,p_inc_y,p_inc_z,e_x,e_y));
};



#endif // UWB_COMMON_H
