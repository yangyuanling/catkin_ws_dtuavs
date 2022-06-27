#ifndef _SWARM_CROSSING_
#define _SWARM_CROSSING_
#include"geometry_msgs/Point.h"
#include"geometry_msgs/Vector3.h"
#include <vector>
#include"sensor_msgs/LaserScan.h"

namespace swarm_crossing{

#define NUMBER_MAX 10

struct vehicle_property
{
  double r_com;
  double r_agent;
  double v_max;
  double v_z_max;
};

//详细说明见cpp文件中model
struct crossing_parameters
{
  double height; ////////////////////定高飞行//////////////////////
  double p_h;   ////////////////////高度p控制//////////////////////

  double y_finish; /////////////////任务区停止位置//////////////////
  double dy_finish; /////////////////任务区停止位置增量//////////////////
  // double p_xy;      /////////////////停止悬停时的p控制///////////////////
  //===========自推进项或对齐项===========
  double v_flock;
  //速度差修正项
  double p_diff;
  //速度摩擦项
  double C_frict;
  double v_frict;
  double r_frict_0;
  double a_frict;
  double p_frict;
  //===========排斥项===========
  double r_rep;
  double p_rep;
  //===========吸引项===========
  double r_att1;
  double r_att2;
  double p_att;
  int k;
  //Obstacle avoidance
  double r_wall;
  double r_adjust;
  double C_wall;
};

class map2D
{
private:
  int number_b_0;                  //边界直线的数量
  int number_b;                    //边界直线的数量 + 离散圆的直线数量
  double **boundary_start_points;  //描述边界直线的起点
  double **boundary_end_points;    //描述边界直线的终点

  int number_c;                    // Number of a circular obstacles
  double* circle_radius;           // Radius of a circular obstacles
  double** circle_center_point;    // Center of a circular obstacles
  double circle_angle_number;   // Angular <resolution> of circle discretization

public:
  sensor_msgs::LaserScan laser_data;
  std::vector<bool> ranges_binary;
  std::vector<float> angles;
  int num_ranges;
public:
  map2D(/* args */);
  ~map2D();
  void set_map(int number_b_0,int number_c,double**boundary_start_points,double**boundary_end_points,double*circle_radius,double**circle_center,int circle_angle_number);
  void set_laser_parameters(double angle_min, double angle_max, int angle_n, double range_min, double range_max);
  void set_laser_range(double range_min,double range_max);
  void print_map_data();
  //heading为传感器朝向(全局)
  void get_projection_field(const double posx, const double posy, double heading);

  bool linspacePolar(double p0x, double p0y, double p1x, double p1y,double heading);

  int get_index_in_ranges(double angle);
};



class model
{
private:
  crossing_parameters params; //参数
  vehicle_property propers;   //

  int flag_align; //VIST为速度平均，FRICT为速度摩擦
  int flag_att;   //K_NEAREST_NEIGHBOR为k近邻，FIX_NEIGHBOR为固定拓扑
  enum ALIGN_TYPE
  {
    VIST = 0,
    DIFFERENT = 1,
    FRICT = 2
  };
  enum ATT_TYPE
  {
    K_NEAREST_NEIGHBOR = 0,
    FIX_NEIGHBOR = 1
  };

  int number_total;
  bool connection_matrix[NUMBER_MAX][NUMBER_MAX]; //这里的索引是对应着实际id的
  bool informed_array[NUMBER_MAX];
  double informed_direction;

  map2D myMap;
  ros::Time last_request;

public:
  model();
  ~model();
  geometry_msgs::Vector3 generateDesiredVelocity(int id ,const std::vector<int>uav_id,
                                                 const std::vector<geometry_msgs::Point> &position,
                                                 const std::vector<geometry_msgs::Vector3> &velocity);

  void set_number_total(int number_total);
  void set_parameters(double*data);
  void set_connection_matrix();
  double DFunction(const double r, const double a, const double p);
  bool flag_finish_crossing;
};

double clamp_min_max(double input,double min_d,double max_d);
double wrap_to_pi(double input);
void swap(double &a, double &b);
bool roots(double &x0, double &x1, double a, double b, double c);
bool intersection_line_with_circle(double &p0x, double &p0y, double &p1x, double &p1y,
                                   double slope, double intercpt, double centerx, double centery, double radius);
bool intersection_line_segment_with_circle(double &p0x, double &p0y, double &p1x, double &p1y,
                                           double p0x_in, double p0y_in, double p1x_in, double p1y_in,
                                           double centerx, double centery, double radius);
void cart2pol(double &alpha, double&rho, double x, double y);


}

#endif
