#include"multi_collab/swarm_crossing.h"
#include<fstream>

const double pi = 3.141592653589793;

swarm_crossing::model::model()
{
    this->number_total = 6;      //默然是6，需要在multi_collab.cpp赋值，仅影响固定拓扑吸引的邻接矩阵定义

    this->propers.r_com = 100.0; // 最大通讯距离，也是对齐作用的最大范围
    this->propers.r_agent = 3.0; // 个体最大半径，影响自适应任务方向
    this->propers.v_max   = 1.2; // 最大水平速度
    this->propers.v_z_max = 1.0; // 最大高度速度


    this->params.height = 2;     ////////////////////定高飞行//////////////////////
    this->params.p_h = 0.2;      ////////////////////高度p控制//////////////////////
    this->params.y_finish = 60; /////////////////任务区停止位置//////////////////
    this->params.dy_finish = 3; /////////////////任务区停止位置增量//////////////////
    // this->params.p_xy = 1;      /////////////////停止悬停时的p控制///////////////////
    //===========自推进项或对齐项===========
    this->flag_align = ALIGN_TYPE::VIST;            //VIST为速度平均，FRICT为速度摩擦，DIFFERENT为速度差修正
    this->params.v_flock = 1.0;  // 维持速度大小
        //===========速度差修正项，this->flag_align = ALIGN_TYPE::DIFFERENT起作用===========
    this->params.p_diff = 1.0;
        //===========速度摩擦项，this->flag_align = ALIGN_TYPE::FRICT起作用，由于参数较多，暂且不用===========
    this->params.v_frict = this->params.v_flock/8;   // 
    this->params.r_frict_0 = 10;
    this->params.a_frict = 0.02;
    this->params.p_frict = 0.5;
    this->params.C_frict = 0.1;
    //===========排斥项===========
    this->params.r_rep = 10.0;   // 排斥半径
    this->params.p_rep = 1.0;    // 排斥线性增益
    //===========吸引项===========
    this->flag_att = ATT_TYPE::K_NEAREST_NEIGHBOR;  //K_NEAREST_NEIGHBOR为k近邻，FIX_NEIGHBOR为固定拓扑
    this->params.r_att1 = this->params.r_rep + 2.0;  // 吸引范围下限
    this->params.r_att2 = this->params.r_rep + 20.0; // 吸引范围上限
    this->params.p_att = 1.0;                        // 吸引线性增益
    this->params.k = 2;                              // 最近邻吸引数量，当this->flag_att = ATT_TYPE::K_NEAREST_NEIGHBOR时起作用
    this->set_connection_matrix();                   // 固定拓扑吸引邻接矩阵，采用根据总数量的自动首尾相连，当this->flag_att = ATT_TYPE::FIX_NEIGHBOR时起作用
    //===========避障项===========
    this->params.C_wall = 1.0;     // 线性增益
    this->params.r_wall = 6.0;     // 避障距离
    //===========任务项===========
    this->params.r_adjust = 6.0;    //自适应的感知障碍距离
    this->informed_array[0] = true;  //身份标志，true为informed
    this->informed_array[4] = true;
    this->informed_direction = pi/2; //任务方向，与x轴的夹角，rad


    //===========其他===========
    this->last_request = ros::Time::now(); //触发时计时用
    this->flag_finish_crossing = false;    //集群是否到达任务区的标志

    //========================map=========================
    //double x0[2]{0,0};

    double boundary_start_points_in[2][7]{
		{-20.5706984667802,-13.160136286201,-22.6149914821124,16.9931856899489,14.1822827938671,7.79386712095402,10.8603066439523},
		{-72.3168654173765,-37.8194207836457,-9.71039182282794,-72.8279386712096,-42.4190800681431,-20.954003407155,-2.0442930153322}
	};
	double boundary_end_points_in[2][7]{
		{-13.160136286201,-22.6149914821124,-9.58262350936968,14.1822827938671,7.79386712095402,10.8603066439523,19.2930153321976},
		{-37.8194207836457,-9.71039182282794,12.7768313458262,-42.4190800681431,-20.954003407155,-2.0442930153322,13.0323679727428}
	};
	double circle_radius_in[4]{ 0.29,0.29,0.29,0.29 };
	double circle_center_point_in[2][4]{
		{-1.14991482112436,-9.83816013628621,1.40545144804091,0.894378194207832},
		{-47.5298126064736,-17.8875638841567,-11.4991482112436,9.96592844974447}
	};
    // double boundary_start_points_in[2][34]{
	// 	{-19.2176870748299,-17.8571428571429,-18.5374149659864,-23.9795918367347,-28.7414965986395,-30.1020408163265,-30.7823129251701,-30.4421768707483,
    //     -27.0408163265306,-21.2585034013606,-13.0952380952381,-11.0544217687075,-10.374149659864,-15.8163265306122,-19.2176870748299,19.8979591836735,
    //     19.8979591836735,18.8775510204081,16.156462585034,13.7755102040816,12.0748299319728,10.3741496598639,8.67346938775511,8.67346938775511,4.93197278911565,
    //     4.93197278911565,6.97278911564625,16.4965986394558,21.2585034013605,22.6190476190476,23.2993197278911,23.2993197278911,18.1972789115646,15.1360544217687},
	// 	{-72.4489795918367,-60.2040816326531,-46.2585034013606,-33.3333333333334,-26.1904761904762,-22.108843537415,-9.18367346938778,1.36054421768704,
    //     11.9047619047619,21.7687074829932,33.3333333333333,38.4353741496598,48.2993197278911,67.0068027210884,69.3877551020408,-71.7687074829932,
    //     -66.3265306122449,-59.8639455782313,-51.3605442176871,-46.2585034013606,-40.1360544217687,-33.3333333333334,-27.8911564625851,-27.2108843537415,
    //     -13.265306122449,-4.42176870748303,4.76190476190473,20.4081632653061,33.6734693877551,40.8163265306122,44.2176870748299,52.0408163265306,62.9251700680272,68.0272108843537}
	// };
	// double boundary_end_points_in[2][34]{
	// 	{-17.8571428571429,-18.5374149659864,-23.9795918367347,-28.7414965986395,-30.1020408163265,-30.7823129251701,-30.4421768707483,-27.0408163265306,
    //     -21.2585034013606,-13.0952380952381,-11.0544217687075,-10.374149659864,-15.8163265306122,-19.2176870748299,-22.2789115646259,19.8979591836735,
    //     18.8775510204081,16.156462585034,13.7755102040816,12.0748299319728,10.3741496598639,8.67346938775511,8.67346938775511,4.93197278911565,4.93197278911565,
    //     6.97278911564625,16.4965986394558,21.2585034013605,22.6190476190476,23.2993197278911,23.2993197278911,18.1972789115646,15.1360544217687,11.3945578231292},
	// 	{-60.2040816326531,-46.2585034013606,-33.3333333333334,-26.1904761904762,-22.108843537415,-9.18367346938778,1.36054421768704,11.9047619047619,
    //     21.7687074829932,33.3333333333333,38.4353741496598,48.2993197278911,67.0068027210884,69.3877551020408,71.4285714285714,-66.3265306122449,-59.8639455782313,
    //     -51.3605442176871,-46.2585034013606,-40.1360544217687,-33.3333333333334,-27.8911564625851,-27.2108843537415,-13.265306122449,-4.42176870748303,4.76190476190473,
    //     20.4081632653061,33.6734693877551,40.8163265306122,44.2176870748299,52.0408163265306,62.9251700680272,68.0272108843537,73.8095238095238}
	// };
	// double circle_radius_in[4]{ 0.29,0.29,0.29,0.29 };
	// double circle_center_point_in[2][4]{
	// 	{-0.0694,-0.0694,-26.2225,3.2517},
	// 	{-59.9861,-9.0636,-8.7869,43.7961}
	// };

    int circle_angle_number = 20; //将圆离散为线段的数量
    //==========================下面无需更改==========================
    int number_c = sizeof(circle_radius_in) / sizeof(double);
	int number_b_0 = sizeof(boundary_start_points_in) / sizeof(double) / 2;
    double**boundary_start_points;
    double**boundary_end_points;
    double*circle_radius;
    double**circle_center_point;
    //
	circle_radius = new double[number_c];
	circle_center_point = new double*[2];
	circle_center_point[0] = new double[number_c];
	circle_center_point[1] = new double[number_c];
	for (int i = 0; i < number_c; i++)
	{
		circle_radius[i] = circle_radius_in[i];
		circle_center_point[0][i] = circle_center_point_in[0][i];
		circle_center_point[1][i] = circle_center_point_in[1][i];
	}
    //
	boundary_start_points = new double*[2];
	boundary_start_points[0] = new double[number_b_0];
	boundary_start_points[1] = new double[number_b_0];
	boundary_end_points = new double*[2];
	boundary_end_points[0] = new double[number_b_0];
	boundary_end_points[1] = new double[number_b_0];
    for (int i = 0; i < 2; i++)
	{
		for (int j = 0; j < number_b_0; j++)
		{
			boundary_start_points[i][j] = boundary_start_points_in[i][j];
			boundary_end_points[i][j] = boundary_end_points_in[i][j];
		}
	}

    this->myMap.set_map(number_b_0,number_c,boundary_start_points,boundary_end_points,circle_radius,circle_center_point,circle_angle_number);

    delete[] boundary_start_points[0];
    delete[] boundary_start_points[1];
    delete[] boundary_start_points;
    boundary_start_points = nullptr;
    delete[] boundary_end_points[0];
    delete[] boundary_end_points[1];
    delete[] boundary_end_points;
    boundary_end_points = nullptr;
    delete[] circle_radius;
    delete[] circle_center_point[0];
    delete[] circle_center_point[1];
    delete[] circle_center_point;
    circle_radius = nullptr;
    circle_center_point = nullptr;
    // this->myMap.print_map_data();
}



swarm_crossing::model::~model()
{
}

void swarm_crossing::model::set_parameters(double*data)
{
    if (data == nullptr)
    {
        std::cout<<"data error"<<std::endl;
    }
    
    this->propers.r_com   = data[0]; // 最大通讯距离，也是对齐作用的最大范围
    this->propers.r_agent = data[1]; // 个体最大半径，影响自适应任务方向
    this->propers.v_max   = data[2]; // 最大水平速度
    this->propers.v_z_max = data[3]; // 最大高度速度


    this->params.height    = data[4];     ////////////////////定高飞行//////////////////////
    this->params.p_h       = data[5];      ////////////////////高度p控制//////////////////////
    this->params.y_finish  = data[6]; /////////////////任务区停止位置//////////////////
    this->params.dy_finish = data[7]; /////////////////任务区停止位置增量//////////////////
    // this->params.p_xy = 1;      /////////////////停止悬停时的p控制///////////////////
    //===========自推进项或对齐项===========
    this->flag_align       = (int)(data[8]+0.1);            //VIST为速度平均，FRICT为速度摩擦，DIFFERENT为速度差修正
    this->params.v_flock   = data[9];  // 维持速度大小
        //===========速度差修正项，this->flag_align = ALIGN_TYPE::DIFFERENT起作用===========
    this->params.p_diff    = data[10];
        //===========速度摩擦项，this->flag_align = ALIGN_TYPE::FRICT起作用，由于参数较多，暂且不用===========
    // this->params.v_frict = this->params.v_flock/8;   // 
    // this->params.r_frict_0 = 10;
    // this->params.a_frict = 0.02;
    // this->params.p_frict = 0.5;
    // this->params.C_frict = 0.1;
    //===========排斥项===========
    this->params.r_rep = data[11];   // 排斥半径
    this->params.p_rep = data[12];    // 排斥线性增益
    //===========吸引项===========
    this->flag_att      = (int)(data[13]+0.1);  //K_NEAREST_NEIGHBOR为k近邻，FIX_NEIGHBOR为固定拓扑
    this->params.r_att1 = data[14];  // 吸引范围下限
    this->params.r_att2 = data[15]; // 吸引范围上限
    this->params.p_att  = data[16];                        // 吸引线性增益
    this->params.k      = data[17];                              // 最近邻吸引数量，当this->flag_att = ATT_TYPE::K_NEAREST_NEIGHBOR时起作用
    // this->set_connection_matrix();                   // 固定拓扑吸引邻接矩阵，采用根据总数量的自动首尾相连，当this->flag_att = ATT_TYPE::FIX_NEIGHBOR时起作用
    //===========避障项===========
    this->params.C_wall = data[18];     // 线性增益
    this->params.r_wall = data[19];     // 避障距离
    //===========任务项===========
    this->params.r_adjust = data[20];    //自适应的感知障碍距离
    this->informed_direction = data[21]; //任务方向，与x轴的夹角，rad
    for (int i = 0; i < this->number_total; i++)
    {
       this->informed_array[i] = (bool)(int)(data[22+i]+0.1); 
    }
    std::cout<<"Crossing parameters:"<<std::endl;
    std::cout<<"r_com:\t"<<this->propers.r_com<<std::endl;
    std::cout<<"r_agent:\t"<<this->propers.r_agent<<std::endl;
    std::cout<<"v_max:\t"<<this->propers.v_max<<std::endl;
    std::cout<<"v_z_max:\t"<<this->propers.v_z_max<<std::endl;
    std::cout<<"height:\t"<<this->params.height<<std::endl;
    std::cout<<"p_h:\t"<<this->params.p_h<<std::endl;
    std::cout<<"y_finish:\t"<<this->params.y_finish<<std::endl;
    std::cout<<"dy_finish:\t"<<this->params.dy_finish<<std::endl;
    std::cout<<"flag_align:\t"<<this->flag_align<<std::endl;
    std::cout<<"v_flock:\t"<<this->params.v_flock<<std::endl;
    std::cout<<"p_diff:\t"<<this->params.p_diff<<std::endl;
    std::cout<<"r_rep:\t"<<this->params.r_rep<<std::endl;
    std::cout<<"p_rep:\t"<<this->params.p_rep<<std::endl;
    std::cout<<"flag_att:\t"<<this->flag_att<<std::endl;
    std::cout<<"r_att1:\t"<<this->params.r_att1<<std::endl;
    std::cout<<"r_att2:\t"<<this->params.r_att2<<std::endl;
    std::cout<<"p_att:\t"<<this->params.p_att<<std::endl;
    std::cout<<"k:\t"<<this->params.k<<std::endl;
    std::cout<<"C_wall:\t"<<this->params.C_wall<<std::endl;
    std::cout<<"r_wall:\t"<<this->params.r_wall<<std::endl;
    std::cout<<"r_adjust:\t"<<this->params.r_adjust<<std::endl;
    std::cout<<"informed_direction:\t"<<this->informed_direction<<std::endl;
    for (int i = 0; i < this->number_total; i++)
    {
        std::cout<<"informed_"<<i<<":\t"<<this->informed_array[i]<<std::endl;
    }
    
    // this->informed_array[22] = true;  //身份标志，true为informed
    // this->informed_array[23] = true;
}

geometry_msgs::Vector3 swarm_crossing::model::generateDesiredVelocity(int id ,const std::vector<int>uav_id,
    const std::vector<geometry_msgs::Point> &position, 
    const std::vector<geometry_msgs::Vector3> &velocity)
{
    //获得id在uav_id中的索引
    int ind_id = id;
    for (int i = 0; i < uav_id.size(); i++)
    {
        if (uav_id[i] == id)
        {
            ind_id = i;
        }
        
    }
    
    geometry_msgs::Vector3 v_desired;

    int number = position.size();
    double v_flock_id[2]{0.0, 0.0}; //自推动项或ALIGN_TYPE::VIST时的速度平均项
    double v_frict_id[2]{0.0, 0.0};
    double v_rep_id[2]{0.0, 0.0};
    double v_att_id[2]{0.0, 0.0};
    double v_wall_id[2]{0.0, 0.0};

    double v_target_global[2]{cos(this->informed_direction),sin(this->informed_direction)};
    double v_target_id[2] {v_target_global[0],v_target_global[1]};
    // 自推动项
    v_flock_id[0] = velocity[ind_id].x;
    v_flock_id[1] = velocity[ind_id].y;
    double v_flock_id_2Dnorm = sqrt(v_flock_id[0]*v_flock_id[0] + v_flock_id[1]*v_flock_id[1]);
    if (v_flock_id_2Dnorm == 0)
    {
        v_flock_id[0] = (double)rand() /(double)RAND_MAX;
        v_flock_id[1] = (double)rand() /(double)RAND_MAX;
        v_flock_id_2Dnorm = sqrt(v_flock_id[0]*v_flock_id[0] + v_flock_id[1]*v_flock_id[1]);
    }
    // v_flock_id[0] = v_flock_id[0]/v_flock_id_2D * this->params.v_flock;
    // v_flock_id[1] = v_flock_id[1]/v_flock_id_2D * this->params.v_flock;
    v_flock_id[0] = v_flock_id[0]/v_flock_id_2Dnorm;
    v_flock_id[1] = v_flock_id[1]/v_flock_id_2Dnorm;
    
    

    double posij[2]{0.0, 0.0};
    double posijNorm = 0.0;
    double velj[2]{0.0,0.0};
    double veljNorm = 0.0;
    double velij[2]{0.0,0.0};
    double velijNorm = 0.0;
    int N_align_vist = 1;

    // 计算吸引项的相关变量初始化
    double **posij_att = new double*[this->params.k];
    double *posijNorm_att = new double[this->params.k];
    int k_att = 0;
    for (int i = 0; i < this->params.k; i++)
    {
        posij_att[i] = new double[2];
        posijNorm_att[i] = 10000000000000;
    }
    // 遍历所有个体
    for (int j = 0; j < number; j++)
    {
        int id_j = uav_id[j];
        if (j != ind_id)
        {
            if (position[j].x==0&&position[j].y==0&&position[j].z==0)
            {
                continue;
            }
            
            posij[0] = position[ind_id].x - position[j].x; //由邻居指向自身的相对位置矢量
            posij[1] = position[ind_id].y - position[j].y;
            posijNorm = sqrt(posij[0]*posij[0] + posij[1]*posij[1]);
            //======================================Alignment term======================================
            if (posijNorm < this->propers.r_com)
            {
                velj[0] = velocity[j].x;
                velj[1] = velocity[j].y;
                switch (this->flag_align)
                {
                case ALIGN_TYPE::VIST:
                    N_align_vist++;
                    veljNorm = sqrt(velj[0]*velj[0] + velj[1]*velj[1]);
                    if (veljNorm != 0)
                    {
                        v_flock_id[0] += velj[0]/veljNorm;
                        v_flock_id[1] += velj[1]/veljNorm;
                    }
                    break;
                case ALIGN_TYPE::FRICT:
                {
                    double vij_frictMax = 
                        this->DFunction(posijNorm-this->params.r_frict_0,this->params.a_frict,this->params.p_frict);
                    if (vij_frictMax < this->params.v_frict)
                    {
                        vij_frictMax = this->params.v_frict;
                    }
                    velij[0] = velocity[ind_id].x - velj[0];
                    velij[1] = velocity[ind_id].y - velj[1];
                    velijNorm = sqrt(velij[0]*velij[0] + velij[1]*velij[1]);
                    if (velijNorm > vij_frictMax)
                    {
                        v_frict_id[0] += (this->params.C_frict * (vij_frictMax - velijNorm) * velij[0]/velijNorm);
                        v_frict_id[1] += (this->params.C_frict * (vij_frictMax - velijNorm) * velij[1]/velijNorm);
                    }
                }
                    break;
                case ALIGN_TYPE::DIFFERENT:
                    if (posijNorm !=0 )
                    {
                        velij[0] = velocity[ind_id].x - velj[0];
                        velij[1] = velocity[ind_id].y - velj[1];
                        v_frict_id[0] -= (velij[0]/posijNorm/posijNorm * this->params.p_diff);
                        v_frict_id[1] -= (velij[1]/posijNorm/posijNorm * this->params.p_diff);
                    }
                    break;
                default:
                    break;
                }
                //========================获得k近邻，用于吸引====================
                switch (this->flag_att)
                {
                case ATT_TYPE::K_NEAREST_NEIGHBOR:
                    k_att++;
                    for (int m = 0; m < this->params.k; m++)
                    {
                        if (posijNorm_att[m] > posijNorm)
                        {
                            posijNorm_att[m] = posijNorm;
                            posij_att[m][0] = posij[0];
                            posij_att[m][1] = posij[1];
                            break;
                        }
                    }
                    break;
                case ATT_TYPE::FIX_NEIGHBOR:
                    {
                        if (this->connection_matrix[id][id_j])
                        {
                            
                        }
                        
                    }
                    break;
                default:
                    break;
                }
            }
            //======================================Repulsion term======================================
            if (posijNorm < this->params.r_rep)
            {
                v_rep_id[0] = v_rep_id[0] + this->params.p_rep * (this->params.r_rep - posijNorm) * posij[0]/posijNorm;
                v_rep_id[1] = v_rep_id[1] + this->params.p_rep * (this->params.r_rep - posijNorm) * posij[1]/posijNorm;  
            }
            //======================================Attraction term======================================
            if (posijNorm < this->params.r_att2 && posijNorm > this->params.r_att1)
            {
                switch (this->flag_att)
                {
                case ATT_TYPE::K_NEAREST_NEIGHBOR:
                    break;
                case ATT_TYPE::FIX_NEIGHBOR:
                    {
                        if (this->connection_matrix[id][id_j])
                        {
                            v_att_id[0] = v_att_id[0] + this->params.p_att * (this->params.r_att1 - posijNorm) * posij[0]/posijNorm;
                            v_att_id[1] = v_att_id[1] + this->params.p_att * (this->params.r_att1 - posijNorm) * posij[1]/posijNorm; 
                        }
                        
                    }
                    break;
                default:
                    break;
                }
            }
            //============================================================================
        }
    }
    //================================Reprocess alignment term==================================
    switch (this->flag_align)
    {
    case ALIGN_TYPE::VIST:
        if (N_align_vist>1){
            v_flock_id_2Dnorm = sqrt(v_flock_id[0]*v_flock_id[0] + v_flock_id[1]*v_flock_id[1]);
            v_flock_id[0] = v_flock_id[0]/v_flock_id_2Dnorm;
            v_flock_id[1] = v_flock_id[1]/v_flock_id_2Dnorm;
        }
        break;
    default:
        break;
    }
    v_flock_id[0] = v_flock_id[0] * this->params.v_flock;
    v_flock_id[1] = v_flock_id[1] * this->params.v_flock;

    //================================Reprocess attraction term================================
    switch (this->flag_att)
    {
        case ATT_TYPE::K_NEAREST_NEIGHBOR:
        {
            int kk = this->params.k;
            if (this->params.k > k_att)
                kk = k_att;
            for (int i = 0; i < kk; i++)
            {
                if (posijNorm_att[i] < this->params.r_att2 && posijNorm_att[i] > this->params.r_att1)
                {
                    v_att_id[0] = v_att_id[0] + this->params.p_att * (this->params.r_att1 - posijNorm_att[i]) * posij_att[i][0]/posijNorm_att[i];
                    v_att_id[1] = v_att_id[1] + this->params.p_att * (this->params.r_att1 - posijNorm_att[i]) * posij_att[i][1]/posijNorm_att[i]; 
                }
            }   
        }
        break;
        case ATT_TYPE::FIX_NEIGHBOR:
            break;
        default:
            break;
    }

    //======================================Obstacle avoidance======================================
    double vel_id[2]{velocity[ind_id].x,velocity[ind_id].y};
    double vel_norm = sqrt(vel_id[0]*vel_id[0] + vel_id[1]*vel_id[1]);
    double heading = 0.0;
    if (vel_norm != 0)
    {
        heading = atan2(vel_id[1],vel_id[0]);
    }
    // this->myMap.get_projection_field(position[ind_id].x,position[ind_id].y,heading);
    this->myMap.set_laser_range(0,this->params.r_wall);
    //printf("%lf,%lf,\n",position[ind_id].x,position[ind_id].y);
    this->myMap.get_projection_field(position[ind_id].x,position[ind_id].y,0); //由于是全局运算，直接按朝向为0结算全局避障速度
    this->myMap.ranges_binary;
    
    for (int i = 0;i < this->myMap.ranges_binary.size();i++)
    {
        /* code */
        v_wall_id[0] -= cos(this->myMap.angles[i])*this->myMap.ranges_binary[i];
        v_wall_id[1] -= sin(this->myMap.angles[i])*this->myMap.ranges_binary[i];
    }
    v_wall_id[0] *= (this->params.C_wall*this->myMap.laser_data.angle_increment);
    v_wall_id[1] *= (this->params.C_wall*this->myMap.laser_data.angle_increment);
    //=====================================Adjust direction===============================================
    this->myMap.set_laser_range(0,this->params.r_adjust);
    this->myMap.get_projection_field(position[ind_id].x,position[ind_id].y,0); //由于是全局运算，直接按朝向为0结算全局informed速度
    std::vector<float> V;
    V.resize(this->myMap.num_ranges);
    // double angle_agent_half = atan(this->propers.r_agent/this->params.r_adjust);
    // double angle_agent_half = atan(this->params.r_wall*1.2/this->params.r_adjust);
    double angle_agent_half = 0.0;
    if (this->propers.r_agent > this->params.r_adjust)
    {
        angle_agent_half = pi/2;
    }
    else
    {
        angle_agent_half = asin(this->propers.r_agent/this->params.r_adjust);
    }

    int n_agent_half = (int)ceil(angle_agent_half/this->myMap.laser_data.angle_increment);
    int n_agent = n_agent_half*2+1;
    //Cross-correlation
    for (int i = 0; i < this->myMap.num_ranges; i++)
    {
        int ind_j = 0;
        if (i < n_agent_half)
        {
            for (int j = 0; j < n_agent; j++)
            {
                ind_j =  i - n_agent_half + j;
                ind_j = (ind_j < 0)?(this->myMap.num_ranges + ind_j):ind_j;
                V[i] += (float)this->myMap.ranges_binary[ind_j];
            }
        }
        else if(i > this->myMap.num_ranges - n_agent_half - 1)
        {
            for (int j = 0; j < n_agent; j++)
            {
                ind_j =  i - n_agent_half + j;
                ind_j = (ind_j >= this->myMap.num_ranges)?(ind_j - this->myMap.num_ranges):ind_j;
                V[i] += (float)this->myMap.ranges_binary[ind_j];
            }
        }
        else
        {
            for (int j = 0; j < n_agent; j++)
            {
                ind_j =  i - n_agent_half + j;
                V[i] += (float)this->myMap.ranges_binary[ind_j];
            }
        }
        // printf("%f,",V[i]);
    }
    // printf("\n");
    // for (int i = 0; i < this->myMap.num_ranges; i++)
    // {
    //     std::cout<<this->myMap.ranges_binary[i]<<",";
    // }
    // std::cout<<std::endl;
    // for (int i = 0; i < this->myMap.num_ranges; i++)
    // {
    //     std::cout<<V[i]<<",";
    // }
    // std::cout<<std::endl;
    
    // Find minimum and the direction of minimum is closer to velocity.
    
    if (this->informed_array[id])
    {
        //任务方向对应索引
        int ind_target = (int)((this->informed_direction - this->myMap.laser_data.angle_min)/this->myMap.laser_data.angle_increment);
        //速度方向对应索引
        int ind_heading = (int)((heading - this->myMap.laser_data.angle_min)/this->myMap.laser_data.angle_increment);
        // int ind_heading = 0;
        //索引数量的一半
        int angle_number_half = (int)(this->myMap.num_ranges/2);
        int ind_min_target = 0, d_ind = 0;
        float val_min = INFINITY;
        //找出任务方向左右两边的最小值与对应索引
        int ind_min_left_target = 0, delta_ind_left_to_target = angle_number_half;
        int ind_min_right_target = 0, delta_ind_right_to_target = angle_number_half;
        float val_left_min = INFINITY, val_right_min = INFINITY;
        
        //遍历V
        for (int i = 0; i < this->myMap.num_ranges; i++)
        {
            //判断当前索引在任务方向左边或者右边
            int temp = i - ind_target + angle_number_half;
            if (temp < 0)
            {
                temp = this->myMap.num_ranges + temp;
            }
            else if(temp > this->myMap.num_ranges)
            {
                temp = temp - this->myMap.num_ranges;
            }
            if (temp < angle_number_half)
            {
                //左边
                if (V[i] < val_left_min)
                {
                    val_left_min = V[i];
                    ind_min_left_target = i;
                    delta_ind_left_to_target = angle_number_half - temp;
                }
                else if (fabs(V[i] - val_left_min) < 0.00001)
                {
                    int temp1 = angle_number_half - temp;
                    if (temp1 < delta_ind_left_to_target)
                    {
                        val_left_min = V[i];
                        ind_min_left_target = i;
                        delta_ind_left_to_target =temp1;
                    }
                }  
            }
            else
            {
                //右边
                if (V[i] < val_right_min)
                {
                    val_right_min = V[i];
                    ind_min_right_target = i;
                    delta_ind_right_to_target = temp - angle_number_half;
                }
                else if (fabs(V[i] - val_right_min) < 0.00001)
                {
                    int temp1 = temp - angle_number_half;
                    if (temp1 < delta_ind_right_to_target)
                    {
                        val_right_min = V[i];
                        ind_min_right_target = i;
                        delta_ind_right_to_target =temp1;
                    }
                }  
            }
        }
        if (fabs(val_left_min - val_right_min) < 0.01)
        {
            int delta_left = fabs(ind_min_left_target - ind_heading);
            int delta_right = fabs(ind_min_right_target - ind_heading);
            delta_left = (delta_left > angle_number_half)?(this->myMap.num_ranges - delta_left):delta_left;
            delta_right = (delta_right > angle_number_half)?(this->myMap.num_ranges - delta_right):delta_right;
            ind_min_target = (delta_left < delta_right)? ind_min_left_target:ind_min_right_target;
        }
        else if (val_left_min < val_right_min)
        {
            ind_min_target = ind_min_left_target;
        }
        else
        {
            ind_min_target = ind_min_right_target;
        }
        double direction_modify = (ind_min_target * this->myMap.laser_data.angle_increment) + this->myMap.laser_data.angle_min;
        // printf("%d,%d,%d,%d,%d,%lf\n",ind_target,ind_heading,ind_min_left_target,ind_min_right_target,ind_min_target,direction_modify);     
        v_target_id[0] = cos(direction_modify);
        v_target_id[1] = sin(direction_modify);
    }
    if (this->informed_array[id])
    {
        v_flock_id[0] = v_target_id[0]*this->params.v_flock;
        v_flock_id[1] = v_target_id[1]*this->params.v_flock;
    }
    //==================finish=====================

    int count_finish = -2;
    for (int i = 0; i < number; i++)
    {
        if (position[i].y > this->params.y_finish - this->params.dy_finish)
        {
            count_finish++;
        }
    }
    count_finish = abs(count_finish);
    if (position[ind_id].y > this->params.y_finish - this->params.dy_finish * count_finish/4.0)
    {
        v_flock_id[0] = 0.0;
        v_flock_id[1] = 0.0;
        v_frict_id[0] = 0.0;
        v_frict_id[1] = 0.0;
    }
    if (count_finish + 2 == this->number_total)
    {
        this->flag_finish_crossing = true;
        std::cout<<"Finished crossing"<<std::endl;
    }
    
    //======================================Combine======================================
    
    
    v_desired.x = v_flock_id[0] + v_frict_id[0] + v_rep_id[0] + v_att_id[0] + v_wall_id[0];
    v_desired.y = v_flock_id[1] + v_frict_id[1] + v_rep_id[1] + v_att_id[1] + v_wall_id[1];
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
    //=================================================================================
    for (int i = 0; i < this->params.k; i++)
    {
        delete[] posij_att[i];
    }
    delete[] posij_att;
    delete[] posijNorm_att;

    // if (ros::Time::now() - this->last_request > ros::Duration(1.0) && this->informed_array[id])
    // {    
    //     // std::cout<<"V_binary:"<<std::endl;
    //     // for (int i = 0; i < this->myMap.num_ranges; i++)
    //     // {
    //     //     std::cout<<this->myMap.ranges_binary[i]<<",";
    //     // }
    //     // std::cout<<std::endl;
    //     // std::cout<<"V_cross:"<<std::endl;
    //     // for (int i = 0; i < this->myMap.num_ranges; i++)
    //     // {
    //     //     std::cout<<V[i]<<",";
    //     // }
    //     // std::cout<<std::endl;
        
    //     printf("id:%d, v_d:%lf,%lf,%lf. v_flock:%lf,%lf. v_frict:%lf,%lf, v_rep:%lf,%lf. v_att:%lf,%lf. v_wall:%lf,%lf\n",
    //         id, v_desired.x, v_desired.y, v_desired.z, v_flock_id[0], v_flock_id[1], v_frict_id[0], v_frict_id[1], v_rep_id[0], v_rep_id[1], v_att_id[0], v_att_id[1], v_wall_id[0], v_wall_id[1]);
    //     last_request = ros::Time::now();
    // }
    
    
    return v_desired;
}

double swarm_crossing::model::DFunction(const double r, const double a, const double p)
{
	//make sure p>0
	double output = 0.0f;
	if (r > 0.0f)
	{
		double rp = r * p;
		if (rp < a/p)
		{
			output = rp;
		}
		else
		{
			output = sqrt(2 * a*r - a * a / p * p);
		}
	}
	return output;
}
void swarm_crossing::model::set_number_total(int number_total){
    this->number_total = number_total;
    this->set_connection_matrix();
}


void swarm_crossing::model::set_connection_matrix(){
    for (int i = 0; i < NUMBER_MAX; i++)
    {
        for (int j = 0; j < NUMBER_MAX; j++)
        {
            connection_matrix[i][j] = false;
        }
        
    }
    if (this->number_total>1)
    {

        this->connection_matrix[0][1] = true;
        this->connection_matrix[0][this->number_total-1] = true;
        for (int i = 1; i < number_total-1; i++)
        {
            this->connection_matrix[i][i-1] = true;
            this->connection_matrix[i][i+1] = true;
        }
        this->connection_matrix[this->number_total-1][0] = true;
        this->connection_matrix[this->number_total-1][this->number_total-2] = true;
    }
    
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

swarm_crossing::map2D::map2D(/* args */)
{

	this->boundary_end_points = nullptr;
    this->boundary_start_points = nullptr;
    this->circle_center_point = nullptr;
    this->circle_radius = nullptr;

	this->set_laser_parameters(-3.14, 3.14, 359, 0.2, 6);

    // this->get_projection_field( -28.904975278597746,-8.966852571955727,pi/2*0);
    // for (size_t i = 0; i < this->ranges_binary.size(); i++)
    // {
    //     printf("%f\n",this->laser_data.ranges[i]);
    //     // std::cout<<this->ranges_binary[i]<<std::endl;
    // }
    

}

void swarm_crossing::map2D::print_map_data()
{
    printf("The start points of boundary(include dicrete circle):\n");
    for (int i = 0; i < this->number_b; i++)
	{
		printf("%lf,%lf\n", this->boundary_start_points[0][i], this->boundary_start_points[1][i]);
	}
	printf("\n");
    printf("The end points of boundary(include dicrete circle):\n");
	for (int i = 0; i < this->number_b; i++)
	{
		printf("%lf,%lf\n", this->boundary_end_points[0][i], this->boundary_end_points[1][i]);
	}
	printf("\n");
    printf("The radius and center point of Circles:\n");
	for (int i = 0; i < this->number_c; i++)
	{
	    printf("%lf,%lf,%lf\n",this->circle_radius[i],this->circle_center_point[0][i],this->circle_center_point[1][i]);
	}
}

void swarm_crossing::map2D::set_map(int number_b_0,int number_c,double**boundary_start_points,double**boundary_end_points,double*circle_radius,double**circle_center,int circle_angle_number)
{
    if (this->boundary_start_points != nullptr)
    {
        delete[] this->boundary_start_points[0];
        delete[] this->boundary_start_points[1];
        delete[] this->boundary_start_points;
        this->boundary_start_points = nullptr;
        delete[] this->boundary_end_points[0];
        delete[] this->boundary_end_points[1];
        delete[] this->boundary_end_points;
        this->boundary_end_points = nullptr;
        delete[] this->circle_radius;
        delete[] this->circle_center_point[0];
        delete[] this->circle_center_point[1];
        delete[] this->circle_center_point;
        this->circle_radius = nullptr;
        this->circle_center_point = nullptr;
    }
	this->number_c = number_c;
	this->circle_angle_number = circle_angle_number;
    this->number_b_0 = number_b_0;
	this->number_b = this->number_b_0 + this->circle_angle_number*this->number_c;
    //
	this->circle_radius = new double[this->number_c];
	this->circle_center_point = new double*[2];
	this->circle_center_point[0] = new double[this->number_c];
	this->circle_center_point[1] = new double[this->number_c];
	for (int i = 0; i < this->number_c; i++)
	{
		this->circle_radius[i] = circle_radius[i];
		this->circle_center_point[0][i] = circle_center[0][i];
		this->circle_center_point[1][i] = circle_center[1][i];
	}
    //
	this->boundary_start_points = new double*[2];
	this->boundary_start_points[0] = new double[this->number_b];
	this->boundary_start_points[1] = new double[this->number_b];
	this->boundary_end_points = new double*[2];
	this->boundary_end_points[0] = new double[this->number_b];
	this->boundary_end_points[1] = new double[this->number_b];
    //
    for (int i = 0; i < 2; i++)
	{
		for (int j = 0; j < this->number_b_0; j++)
		{
			this->boundary_start_points[i][j] = boundary_start_points[i][j];
			this->boundary_end_points[i][j] = boundary_end_points[i][j];
		}
	}
	int count = this->number_b_0;
	for (int i = 0; i < this->number_c; i++)
	{
		for (int j = 0; j < this->circle_angle_number; j++)
		{
			this->boundary_start_points[0][count] = this->circle_radius[i] * cos(2 * pi / this->circle_angle_number*j) + this->circle_center_point[0][i];
			this->boundary_start_points[1][count] = this->circle_radius[i] * sin(2 * pi / this->circle_angle_number*j) + this->circle_center_point[1][i];

			this->boundary_end_points[0][count] = this->circle_radius[i] * cos(2 * pi / this->circle_angle_number*(j+1)) + this->circle_center_point[0][i];
			this->boundary_end_points[1][count] = this->circle_radius[i] * sin(2 * pi / this->circle_angle_number*(j+1)) + this->circle_center_point[1][i];
			count++;
		}
	}
    
}
swarm_crossing::map2D::~map2D()
{
    if (this->boundary_start_points[0] != nullptr)
    {
        delete[] this->boundary_start_points[0];
        delete[] this->boundary_start_points[1];
        delete[] this->boundary_start_points;
        this->boundary_start_points = nullptr;
    }
    if (this->boundary_end_points[0] != nullptr)
    {
        delete[] this->boundary_end_points[0];
        delete[] this->boundary_end_points[1];
        delete[] this->boundary_end_points;
        this->boundary_end_points = nullptr;
    }
    if (this->circle_radius != nullptr)
    {
        delete[] this->circle_radius;
        delete[] this->circle_center_point[0];
        delete[] this->circle_center_point[1];
        delete[] this->circle_center_point;
        this->circle_radius = nullptr;
        this->circle_center_point = nullptr;
    }
}


void swarm_crossing::map2D::get_projection_field(const double posx,const double posy, double heading)
{
	for (size_t i = 0; i < this->laser_data.ranges.size(); i++)
	{
		this->laser_data.ranges[i] = INFINITY;
		this->ranges_binary[i] = false;
	}
	//边界直线的投影
	double sp[2]{ 0,0 }, ep[2]{ 0,0 };
	double sp_[2]{ 0,0 }, ep_[2]{ 0,0 };

	for (int i = 0; i < this->number_b; i++)
	{
		sp[0] = this->boundary_start_points[0][i];
		sp[1] = this->boundary_start_points[1][i];
		ep[0] = this->boundary_end_points[0][i];
		ep[1] = this->boundary_end_points[1][i];
		bool re1 = intersection_line_segment_with_circle(sp_[0], sp_[1], ep_[0], ep_[1],
			sp[0], sp[1], ep[0], ep[1],
			posx, posy, this->laser_data.range_max);
		if (re1)
		{
			sp_[0] -= posx;
			sp_[1] -= posy;
			ep_[0] -= posx;
			ep_[1] -= posy;
			// std::vector<double> rhos;
			// std::vector<double> angles;
			bool re1 = linspacePolar(sp_[0], sp_[1], ep_[0], ep_[1],heading);
            if (re1)
            {
                printf("%lf,%lf,%lf,%lf\n",sp[0], sp[1], ep[0], ep[1]);
            }
		}
		else{
			continue;
		}
	}
	this->laser_data.header.stamp = ros::Time::now();
	// this->laser_data.ranges[this->num_ranges - 1] = this->laser_data.ranges[0];
	// this->ranges_binary[this->num_ranges - 1] = this->ranges_binary[0];

}

void swarm_crossing::map2D::set_laser_parameters
(double angle_min, double angle_max, int angle_n, double range_min, double range_max)
{
    angle_max = wrap_to_pi(angle_max);
    angle_min = wrap_to_pi(angle_min);
    if (angle_min > angle_max || range_min > range_max)
    {
        return;
    }
    
	this->laser_data.angle_increment = (angle_max - angle_min) / angle_n;
	this->laser_data.angle_min = angle_min;
	this->laser_data.angle_max = angle_max;
	this->laser_data.range_min = range_min;
	this->laser_data.range_max = range_max;
	this->laser_data.ranges.resize(angle_n+1);
	this->laser_data.intensities.resize(angle_n+1);
	this->ranges_binary.resize(angle_n + 1);
    this->angles.resize(angle_n + 1);
	for (size_t i = 0; i < this->laser_data.ranges.size(); i++)
	{
		this->laser_data.ranges[i] = INFINITY;
		this->ranges_binary[i] = false;
        this->angles[i] = this->laser_data.angle_min + this->laser_data.angle_increment*i;
	}
	this->num_ranges = angle_n + 1;
}
void swarm_crossing::map2D::set_laser_range(double range_min,double range_max)
{
	this->laser_data.range_min = range_min;
	this->laser_data.range_max = range_max;
}

int swarm_crossing::map2D::get_index_in_ranges(double angle)
{
    angle = wrap_to_pi(angle);
    return (int)((angle - this->laser_data.angle_min)/this->laser_data.angle_increment);
}
bool swarm_crossing::map2D::linspacePolar(double p0x, double p0y, double p1x, double p1y,double heading)
{
    bool re = false;
	if ((p0x == 0 && p0y == 0) || (p0x == 0 && p0y == 0))
	{
	}
	else
	{
        double p0x_ = p0x, p0y_ = p0y, p1x_ = p1x, p1y_ = p1y;
        // rotate
        double chead = cos(heading), shead = sin(heading); 
        p0x =  p0x_*chead + p0y_*shead;
        p0y = -p0x_*shead + p0y_*chead;
        p1x =  p1x_*chead + p1y_*shead;
        p1y = -p1x_*shead + p1y_*chead;
        //
		double alpha0 = 0.0, alpha1 = 0.0, rho0 = 0.0, rho1 = 0.0;
		cart2pol(alpha0, rho0, p0x, p0y);
		cart2pol(alpha1, rho1, p1x, p1y);
        if (alpha0 > alpha1) //sort
        {
            swap(alpha0,alpha1);
            swap(rho0,rho1);
        }

        int ind_0 = (int)((alpha0 - this->laser_data.angle_min) / this->laser_data.angle_increment);
        double alpha0_modify = (double)ind_0 * this->laser_data.angle_increment + this->laser_data.angle_min;
        alpha0_modify = wrap_to_pi(alpha0_modify);

		double dalpha_ = alpha1 - alpha0;

		double beta = atan((p1y - p0y) / (p1x - p0x));
		double c1 = rho0 * sin(beta - alpha0);
		if (dalpha_ > pi)
		{
			//line across pi/-pi boundary

			double dalpha = 6.283185307179586 - dalpha_;
			int num = (int)ceil(dalpha / this->laser_data.angle_increment) + 2;
			// rhos.resize(num);
			// angles.resize(num);
			double outTheta = alpha0_modify;
            int count = 0;
			double outRho = 0.0;
			for (int i = 0; i < num; i++)
			{
                outTheta = alpha0_modify - this->laser_data.angle_increment * i + 0.0001;
                if (outTheta < -3.141592653589793)
                {
                    outTheta = 6.283185307179586 + outTheta;
                }
                if (outTheta < 0)
                {
                    count = i;
                    if (outTheta < this->laser_data.angle_min)continue;   
                }
                else
                {
                    if (outTheta > this->laser_data.angle_max)
                    {
                        count = i;
                        continue;
                    }
                    else
                    {
                        outTheta = this->laser_data.angle_max - this->laser_data.angle_increment * (i - count - 1);
                    }
                }
				
                // Out of line segment
                if ((outTheta > alpha0 && outTheta<0) || (outTheta < alpha1 && outTheta>0))
                {
                    // rhos[i] = INFINITY;
                    continue;
                }

				outRho = c1 / sin(beta - outTheta);
                // output
                ind_0 = (int)((outTheta - this->laser_data.angle_min) / this->laser_data.angle_increment+0.0001); //
                outRho = (this->laser_data.range_min > outRho)?this->laser_data.range_min:outRho;
                this->laser_data.ranges[ind_0] = (this->laser_data.ranges[ind_0]>outRho)?outRho:this->laser_data.ranges[ind_0];
                this->ranges_binary[ind_0] = true;
			}
		}
		else
		{
			double dalpha = dalpha_;
			int num = (int)ceil(dalpha / this->laser_data.angle_increment) + 2;
			double outTheta = 0.0;
			double outRho = 0.0;
			for (int i = 0; i < num; i++)
			{
				outTheta = alpha0_modify + this->laser_data.angle_increment * i + 0.0001;
                // Out of line segment
                if (outTheta < alpha0 || outTheta > alpha1 || outTheta < this->laser_data.angle_min)
                {
                    continue;
                }
                if (outTheta > this->laser_data.angle_max)
                {
                    if (abs(outTheta - this->laser_data.angle_max) > this->laser_data.angle_increment)
                    {
                        continue;
                    }
                }
                
                
                
				outRho = c1 / sin(beta - outTheta);            
                // output
                ind_0 = (int)((outTheta - this->laser_data.angle_min) / this->laser_data.angle_increment+0.0001);
                outRho = (this->laser_data.range_min > outRho)?this->laser_data.range_min:outRho;
                this->laser_data.ranges[ind_0] = (this->laser_data.ranges[ind_0]>outRho)?outRho:this->laser_data.ranges[ind_0];
                this->ranges_binary[ind_0] = true;
                // if (abs(outRho - 5.42207)<0.001)
                // {
                //     printf("%d,%lf,%lf,%lf,%lf,%lf,%d\n",ind_0,outRho,outTheta,beta,rho0,alpha0,num);
                //     printf("%lf,%lf,%lf,%lf\n",p0x,p0y,p1x,p1y);
                //     return true;
                // }
                // if (i == 109)
                // {
                    
                // }
                
			}
            
		}
	}
    return re;
}

double swarm_crossing::clamp_min_max(double input,double min_d,double max_d)
{
    if (input<min_d)
    {
        input = min_d;
    }
    else if(input>max_d)
    {
        input = max_d;
    }
    return input;
}
double swarm_crossing::wrap_to_pi(double input)
{
    if (input<-3.141592653589793)
    {
        input = input + 6.283185307179586;
    }
    else if(input>3.141592653589793)
    {
        input = input - 6.283185307179586;
    }
    return input;
}

void swarm_crossing::swap(double &a, double &b) {
	double temp = a;
	a = b;
	b = temp;
}
//Find roots
bool swarm_crossing::roots(double &x0, double &x1, double a, double b, double c)
{
	double temp = b * b - 4 * a*c;
	if (temp > 0)
	{
		x0 = (-b - sqrt(temp)) / 2.0 / a;
		x1 = (-b + sqrt(temp)) / 2.0 / a;
		return true;
	}
	else
	{
		return false;
	}
}

//  Find the intersection of line and circle and p0x < p1x or p0y < p1y when slope is INFINITY
bool swarm_crossing::intersection_line_with_circle(double &p0x, double &p0y, double &p1x, double &p1y,
	double slope, double intercpt, double centerx, double centery, double radius)
{
	if (slope != INFINITY)
	{
		double a = 1 + slope * slope;
		double b = 2 * (slope*(intercpt - centery) - centerx);
		double c = centery * centery + centerx * centerx + intercpt * intercpt - 2 * centery*intercpt - radius * radius;
		if (roots(p0x, p1x, a, b, c))
		{
			p0y = intercpt + slope * p0x;
			p1y = intercpt + slope * p1x;
			return true;
		}
		else {
			return false;
		}
	}
	// vertical slope case
	else if (abs(centerx - intercpt) > radius) //They don't intercept
	{
		return false;
	}
	else
	{
		p0x = intercpt;
		p1x = intercpt;
		double temp = sqrt(radius*radius - (intercpt - centerx)*(intercpt - centerx));
		p0y = centery - temp;
		p1y = centery + temp;
		return true;
	}

}

// Find the intersection of line segment and circle. It will outputs points within the circle.
bool swarm_crossing::intersection_line_segment_with_circle(double &p0x, double &p0y, double &p1x, double &p1y,
	double p0x_in, double p0y_in, double p1x_in, double p1y_in,
	double centerx, double centery, double radius)
{
	//Get slope and sort the two point according to x
	double slope = 0.0;
	double intercpt = 0.0;
	if (p0x_in == p1x_in) {
		// vertical slope case
		slope = INFINITY;
		intercpt = p0x_in;
		if (p0y_in > p1y_in)
			swap(p0y_in, p1y_in);
	}
	else {
		slope = (p1y_in - p0y_in) / (p1x_in - p0x_in);
		intercpt = -slope * p0x_in + p0y_in;
		if (p0x_in > p1x_in)
		{
			swap(p0x_in, p1x_in);
			swap(p0y_in, p1y_in);
		}
	}

	//
	if (intersection_line_with_circle(p0x, p0y, p1x, p1y, slope, intercpt, centerx, centery, radius))
	{
		double t0, t1, t2, t3;
		t0 = p0x_in - centerx; t1 = (p0y_in - centery);
		t2 = p1x_in - centerx; t3 = (p1y_in - centery);
		double dis_0 = sqrt(t0*t0 + t1 * t1);
		double dis_1 = sqrt(t2*t2 + t3 * t3);
		if (dis_0 <= radius && dis_1 <= radius)
		{
			// The two points are all inside the circle.
			p0x = p0x_in; p0y = p0y_in; p1x = p1x_in; p1y = p1y_in;
			return true;
		}
		else if (dis_0 > radius && dis_1 > radius)
		{
			// The two points are all outside the circle.
			if (p0x_in == p1x_in)
			{
				// vertical slope case
				if (p0y_in <= centery && p1y_in >= centery)
				{
					//  Two outer points on different sides 
					return true;
				}
				else
				{
					//  Two outer points on same sides. The line segment and circle don't intercept
					return false;
				}
			}
			else
			{
                double point_vertical_x = (centerx+slope*centery-slope*intercpt)/(slope*slope+1); // x of perpendicular foot 
				if (p0x_in <= point_vertical_x && p1x_in >= point_vertical_x)
				{
					//  Two outer points on different sides 
					return true;
				}
				else
				{
					//  Two outer points on same sides. The line segment and circle don't intercept
					return false;
				}
			}
		}
		else
		{
			//One of the two points is outside the circle and the other is inside the circle
			if (dis_0 < radius)
			{
				//The first point is inside the circle
				if (p0x_in == p1x_in)
				{
					// vertical slope case
					p0y = p0y_in;
				}
				else
				{
					p0x = p0x_in;
					p0y = p0y_in;
				}
			}
			else
			{
				//The second point is inside the circle
				if (p0x_in == p1x_in)
				{
					// vertical slope case
					p1y = p1y_in;
				}
				else
				{
					p1x = p1x_in;
					p1y = p1y_in;
				}
			}
			return true;
		}
	}
	else {
		//The line and circle don't intercept
		return false;
	}

}

void swarm_crossing::cart2pol(double &alpha, double&rho, double x, double y)
{
	alpha = atan2(y, x);
	rho = sqrt(x*x + y * y);
}
