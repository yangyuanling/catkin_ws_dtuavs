#ifndef _SWARM_HUNTING_
#define _SWARM_HUNTING_
#include"geometry_msgs/Point.h"
#include"geometry_msgs/Vector3.h"
#include <vector>

namespace swarm_hunting{


// #define NUMBER_MAX 10

struct vehicle_property
{
  double r_com;
  double r_agent;
  double v_max;
  double v_z_max;
};

//
struct hunting_parameters
{
  double height;
  double p_h;
  double delta;
  double ind_scale;
  double repulse_scale;
  double obs_influence_range;
};

class Point2D
{
public:
  double x;
  double y;
  Point2D();
  Point2D(double x,double y);
  ~Point2D();
  void operator =(const swarm_hunting::Point2D&);
  friend swarm_hunting::Point2D operator +(const swarm_hunting::Point2D&,const swarm_hunting::Point2D&);
  friend swarm_hunting::Point2D operator -(const swarm_hunting::Point2D&,const swarm_hunting::Point2D&);
  friend swarm_hunting::Point2D operator *(double,const swarm_hunting::Point2D&);
  friend swarm_hunting::Point2D operator *(const swarm_hunting::Point2D&,double);
  friend swarm_hunting::Point2D operator /(const swarm_hunting::Point2D&, double);
};

swarm_hunting::Point2D operator +(const swarm_hunting::Point2D&,const swarm_hunting::Point2D&);
swarm_hunting::Point2D operator -(const swarm_hunting::Point2D&,const swarm_hunting::Point2D&);
swarm_hunting::Point2D operator *(double,const swarm_hunting::Point2D&);
swarm_hunting::Point2D operator *(const swarm_hunting::Point2D&,double);
swarm_hunting::Point2D operator /(const swarm_hunting::Point2D&, double);
class model
{
private:
  hunting_parameters params; //参数
  vehicle_property propers;   //
  int number_total;

  double rec_left_down[2]; //Left-down point of rectangle
  double rec_right_up[2]; //Right-up point of rectangle

public:
  model();
  ~model();
  geometry_msgs::Vector3 generateDesiredVelocity(int id ,const std::vector<int>uav_id,
                                                 const std::vector<geometry_msgs::Point> &position,
                                                 const std::vector<geometry_msgs::Vector3> &velocity, double target_x, double target_y);

  void set_number_total(int number_total);
  void set_parameters(double*data);

};

}

#endif
