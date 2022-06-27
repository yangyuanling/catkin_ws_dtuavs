#include"multi_collab/swarm_hunting.h"

using namespace swarm_hunting;
const double pi = 3.141592653589793;

double norm(swarm_hunting::Point2D p)
{
    return sqrt(p.x*p.x + p.y*p.y);
}


swarm_hunting::Point2D::Point2D(){
    this->x = 0.0;
    this->y = 0.0;
}
swarm_hunting::Point2D::Point2D(double x,double y)
{
    this->x = x;
    this->y = y;
}
swarm_hunting::Point2D::~Point2D(){
}
void swarm_hunting::Point2D::operator =(const swarm_hunting::Point2D& input)
{
    this->x = input.x;
    this->y = input.y;
    
}
swarm_hunting::Point2D swarm_hunting::operator +(const swarm_hunting::Point2D&p1,const swarm_hunting::Point2D&p2)
{
    swarm_hunting::Point2D result = p1;
    result.x += p2.x;
    result.y += p2.y;
    return result;
}
swarm_hunting::Point2D swarm_hunting::operator -(const swarm_hunting::Point2D&p1,const swarm_hunting::Point2D&p2)
{
    swarm_hunting::Point2D result = p1;
    result.x -= p2.x;
    result.y -= p2.y;
    return result;
}
swarm_hunting::Point2D swarm_hunting::operator *(double a,const swarm_hunting::Point2D&p)
{
    swarm_hunting::Point2D result = p;
    result.x *= a;
    result.y *= a;
    return result;
}
swarm_hunting::Point2D swarm_hunting::operator *(const swarm_hunting::Point2D&p,double a)
{
    swarm_hunting::Point2D result = p;
    result.x *= a;
    result.y *= a;
    return result;
}
swarm_hunting::Point2D swarm_hunting::operator /(const swarm_hunting::Point2D&p, double a)
{
    swarm_hunting::Point2D result = p;
    result.x /= a;
    result.y /= a;
    return result;
}
swarm_hunting::Point2D _attract(swarm_hunting::Point2D x)
{
    double x_norm = norm(x);
    swarm_hunting::Point2D result(0.0,0.0);
    if (x_norm != 0)
    {
        result = -1 * x / x_norm;
    }
     return result;
}
swarm_hunting::Point2D _repulse(swarm_hunting::Point2D x,double obs_influence_range,double repulse_scale)
{
    double x_norm = norm(x);
    swarm_hunting::Point2D result(0.0,0.0);
    if (x_norm < obs_influence_range)
    {
        if (x_norm !=0)
        {
            result = repulse_scale * x / norm(x) * (obs_influence_range / norm(x) - 1);
        }
    }
    return result;
}
swarm_hunting::Point2D _individual(swarm_hunting::Point2D x,double ind_scale,double delta)
{
    double x_norm = norm(x);
    swarm_hunting::Point2D result(0.0,0.0);
    if (x_norm != 0)
    {
        result = -ind_scale * x / norm(x) * (0.5 - delta / norm(x));
    }
    return result;
}
int argmin(double*input,int len)
{
    double val_min = INFINITY;
    int ind_min = -1;
    for (int i = 0; i < len; i++)
    {
        if (input[i]<val_min)
        {
            val_min = input[i];
            ind_min = i;
        }
    }
}
swarm_hunting::Point2D mean(swarm_hunting::Point2D * ps,int len)
{
    swarm_hunting::Point2D result(0.0,0.0);
    for (int i = 0; i < len; i++)
    {
        result = result + ps[i];
    }
    result = result/(double)len;
    return result;
}
swarm_hunting::Point2D odefcn(double target_x,double target_y,double self_x,double self_y,double*friend_x,double*friend_y,int num_friend,
    double rec_left_down[2],double rec_right_up[2],
    double obs_influence_range,double repulse_scale,
    double ind_scale,double delta)
{
    swarm_hunting::Point2D v(0.0,0.0);
    swarm_hunting::Point2D distance(self_x - target_x, self_y - target_y);
    swarm_hunting::Point2D v_attrack = _attract(distance);

    double distance_from_boundary[4]{self_x - rec_left_down[0], rec_right_up[0] - self_x, self_y - rec_left_down[1], rec_right_up[1] - self_y};
    int ind_min = argmin(distance_from_boundary,4);
    double obstacle_x = 0.0, obstacle_y = 0.0;
    switch (ind_min)
    {
    case 0:
        obstacle_x = rec_left_down[0];
        obstacle_y = self_y;
        break;
    
    case 1:
        obstacle_x = rec_right_up[0];
        obstacle_y = self_y;
        break;
    
    case 2:
        obstacle_x = self_x;
        obstacle_y = rec_left_down[1];
        break;
    
    case 3:
        obstacle_x = self_x;
        obstacle_y = rec_right_up[1];
        break;
    default:
        break;
    }
    distance = swarm_hunting::Point2D(self_x - obstacle_x, self_y - obstacle_y);
    swarm_hunting::Point2D v_repulse = _repulse(distance,obs_influence_range,repulse_scale);

    swarm_hunting::Point2D *v_individual = new swarm_hunting::Point2D[num_friend];

    for (int i = 0; i < num_friend; i++)
    {
        distance = swarm_hunting::Point2D(self_x - friend_x[i], self_y - friend_y[i]);
        v_individual[i] = _individual(distance,ind_scale, delta);
    }
    swarm_hunting::Point2D v_individual_mean = mean(v_individual,num_friend);

    v = v_attrack + v_repulse + v_individual_mean;
    v = v / norm(v);

    delete[] v_individual;
    v_individual = nullptr;
    return v;
}
swarm_hunting::model::model()
{
    this->number_total = 6;      //默然是6，需要在multi_collab.cpp赋值，仅影响固定拓扑吸引的邻接矩阵定义

    double parameters_all[14]{100.0, 3.0, 1.2, 1.0, 2, 0.2, 5, 5, 2.0, 10.0, -39.9914821124361, 3.83304940374788, 33.603066439523, 50.3407155025554};
    // this->set_parameters(parameters_all);
    this->propers.r_com   = parameters_all[0]; // 最大通讯距离，也是对齐作用的最大范围
    this->propers.r_agent = parameters_all[1]; // 个体最大半径，影响自适应任务方向
    this->propers.v_max   = parameters_all[2]; // 最大水平速度
    this->propers.v_z_max = parameters_all[3]; // 最大高度速度

    this->params.height     = parameters_all[4];   //
    this->params.p_h        = parameters_all[5];
    this->params.delta      = parameters_all[6];     
    this->params.ind_scale  = parameters_all[7]; 
    this->params.repulse_scale       = parameters_all[8]; 
    this->params.obs_influence_range = parameters_all[9];

    this->rec_left_down[0] = parameters_all[10];
    this->rec_left_down[1] = parameters_all[11];
    this->rec_right_up[0] = parameters_all[12];
    this->rec_right_up[1] = parameters_all[13];

    // //==========================下面无需更改==========================
    // this->rec_left_down[0] = -39.9914821124361;
    // this->rec_left_down[1] = 3.83304940374788;
    // this->rec_right_up[0] = 33.603066439523;
    // this->rec_right_up[1] = 50.3407155025554;
}



swarm_hunting::model::~model()
{
}

void swarm_hunting::model::set_parameters(double*data)
{
    if (data == nullptr)
    {
        std::cout<<"data error"<<std::endl;
    }
    
    this->propers.r_com   = data[0]; // 最大通讯距离，也是对齐作用的最大范围
    this->propers.r_agent = data[1]; // 个体最大半径，影响自适应任务方向
    this->propers.v_max   = data[2]; // 最大水平速度
    this->propers.v_z_max = data[3]; // 最大高度速度

    this->params.height     = data[4];   //
    this->params.p_h        = data[5];
    this->params.delta      = data[6];     
    this->params.ind_scale  = data[7]; 
    this->params.repulse_scale       = data[8]; 
    this->params.obs_influence_range = data[9];

    this->rec_left_down[0] = data[10];
    this->rec_left_down[1] = data[11];
    this->rec_right_up[0] = data[12];
    this->rec_right_up[1] = data[13];
    std::cout<<"Hunting parameters:"<<std::endl;
    std::cout<<"r_com:\t"<<this->propers.r_com<<std::endl;
    std::cout<<"r_agent:\t"<<this->propers.r_agent<<std::endl;
    std::cout<<"v_max:\t"<<this->propers.v_max<<std::endl;
    std::cout<<"v_z_max:\t"<<this->propers.v_z_max<<std::endl;
    std::cout<<"height:\t"<<this->params.height<<std::endl;
    std::cout<<"p_h:\t"<<this->params.p_h<<std::endl;
    std::cout<<"delta:\t"<<this->params.delta<<std::endl;
    std::cout<<"ind_scale:\t"<<this->params.ind_scale<<std::endl;
    std::cout<<"repulse_scale:\t"<<this->params.repulse_scale<<std::endl;
    std::cout<<"obs_influence_range:\t"<<this->params.obs_influence_range<<std::endl;
    std::cout<<"rec_left_down_x:\t"<<this->rec_left_down[0]<<std::endl;
    std::cout<<"rec_left_down_y:\t"<<this->rec_left_down[1]<<std::endl;
    std::cout<<"rec_right_up_x:\t"<<this->rec_right_up[0]<<std::endl;
    std::cout<<"rec_right_up_y:\t"<<this->rec_right_up[1]<<std::endl;
}

geometry_msgs::Vector3 swarm_hunting::model::generateDesiredVelocity(int id ,const std::vector<int>uav_id,
    const std::vector<geometry_msgs::Point> &position, 
    const std::vector<geometry_msgs::Vector3> &velocity, double target_x, double target_y)
{
    geometry_msgs::Vector3 v_desired;
    //获得id在uav_id中的索引
    int ind_id = id;
    bool if_exist = false;
    for (int i = 0; i < uav_id.size(); i++)
    {
        if (uav_id[i] == id)
        {
            ind_id = i;
            if_exist = true;
            break;
        }
    }
    if (!if_exist)
    {
        return v_desired;
    }
    //
    double *friend_x = nullptr, *friend_y =  nullptr;
    int num_friend = 0;
    if (uav_id.size() > 1)
    {
        friend_x = new double[uav_id.size()-1];
        friend_y = new double[uav_id.size()-1];
        for (int i = 0; i < uav_id.size(); i++)
        {
            if (i != ind_id && !(position[i].x==0&&position[i].y==0&&position[i].z==0))
            {
                friend_x[num_friend] = position[i].x;
                friend_y[num_friend] = position[i].y;
                num_friend++;
            }   
        }
    }
    //======================================Combine======================================
    swarm_hunting::Point2D v2d = odefcn(target_x, target_y, position[ind_id].x, position[ind_id].y, friend_x, friend_y, num_friend,
     this->rec_left_down, this->rec_right_up,
     this->params.obs_influence_range, this->params.repulse_scale,
     this->params.ind_scale,this->params.delta);
    
    v_desired.x = v2d.x;
    v_desired.y = v2d.y;
    v_desired.z = this->params.p_h * (this->params.height - position[ind_id].z);
    // Clamp desired velocity
    double v_norm = sqrt(v_desired.x*v_desired.x + v_desired.y*v_desired.y);
    if (v_norm > this->propers.v_max)
    {
        v_desired.x = v_desired.x/v_norm * this->propers.v_max;
        v_desired.y = v_desired.y/v_norm * this->propers.v_max;
    }
    v_norm = abs(v_desired.z);
    if (v_norm > this->propers.v_z_max)
    {
        v_desired.z = v_desired.z/v_norm * this->propers.v_z_max;
    }
    // std::cout<<v_desired.x<<","<<v_desired.y<<","<<v_desired.z<<std::endl;q


    delete[] friend_x;
    delete[] friend_y;
    friend_x = nullptr;
    friend_y = nullptr;
    return v_desired;
}


void swarm_hunting::model::set_number_total(int number_total){
    this->number_total = number_total;
}
