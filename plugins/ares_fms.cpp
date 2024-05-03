#include "ares_fms.h"

ARES_FMS::ARES_FMS() : ac("move_base", true), rate(100)
{
    setlocale(LC_ALL,"");
    ROS_INFO_STREAM("\033[1;32m----> Move_Base Navigation started.\033[0m");
    ros::service::waitForService("/move_base/make_plan");
    ROS_INFO("\033[1;32m----> move_base init OK \033[0m");

    ac.waitForServer(ros::Duration(60));
    ROS_INFO("Connected to move base server");
    suplly_17mm_pub = nh.advertise<std_msgs::Int8>("/suplly_17mm",10);
    base_yaw_pub = nh.advertise<std_msgs::Float64>("/base_yaw_vel",10);

    Base_vel_sub_ = nh.subscribe("/base_vel", 1, &ARES_FMS::cmdCallback, this);
    CopyCar_vel_sub_ = nh.subscribe("cmd_vel_copy", 1, &ARES_FMS::cmd_cpoyCallBack, this);
    global_plan_sub_ = nh.subscribe("pid_position_follow/path", 1, &ARES_FMS::callback_global_plan, this);
    localCostMap_sub_ = nh.subscribe("/move_base/local_costmap/costmap_updates",1 ,&ARES_FMS::CostMapCallback, this);
    odom_subscriber_ = nh.subscribe("/Odometry", 10, &ARES_FMS::Odom_Callback, this);
    // STM_Date_sub_ = nh.subscribe("/STM_Date", 10, &ARES_FMS::STM_Date_callback, this);
    STM_Date_sub_ = nh.subscribe("/STM_Date_virtual", 10, &ARES_FMS::STM_Date_callback, this);
    Param_Init();

    Update_FMS = std::thread(&ARES_FMS::thread_updateFMS, this);
    Update_TF = std::thread(&ARES_FMS::thread_update_TF, this);
    FMS_Control = std::thread(&ARES_FMS::thread_FMS_Control, this);
    Update_pursuit = std::thread(&ARES_FMS::thread_Updatepursuit, this);
}

void ARES_FMS::Param_Init()
{
    self_tfBuffer_.reset(new tf2_ros::Buffer());
    self_tfListener_.reset(new tf2_ros::TransformListener(*self_tfBuffer_));
    self_tfBuffer_->setUsingDedicatedThread(true);
    nh.param<float>("tole_dis_random", tole_dis_random, 0.5);
    nh.param<float>("enemy2self_max_dis", enemy2self_max_dis, 2.5);
    nh.param<float>("enemy2self_min_dis", enemy2self_min_dis, 1.5);
    nh.param<int>("slef_scan_area", slef_scan_area, 60);
    nh.param<int>("serial_debug_enable", serial_debug_enable, 0);
    nh.param<int>("Self_HP_thre_low", Self_HP_thre_low, 150);
    nh.param<int>("Self_Outpost_HP_thre_low", Self_Outpost_HP_thre_low, 200);
    nh.param<int>("remain_gold_thre_low", remain_gold_thre_low, 300);
    nh.param<int>("allowance_17mm_thre_low", allowance_17mm_thre_low, 30);
    nh.param<float>("nav_timeout", nav_timeout, 12.0);
    nh.param<float>("enemy_Outpost_time_out", enemy_Outpost_time_out, 120.0);
    nh.param<float>("suplly_time_out", suplly_time_out, 10.0);
    nh.param<float>("base_vyaw", base_vyaw, 2.5);
    nh.param<int>("assault_flag", assault_flag, 0);
}

ARES_FMS::~ARES_FMS() 
{
}

void ARES_FMS::thread_updateFMS()
{
    printf("\033[1;32m ------>准备完成 等待比赛开始 \033[0m\n");
    while(1)
    {
        if (serial_debug_enable)
        {
            // printf("%d", Self_Robot_info.self_color_);
            printf("Self --> Color=%2d, Robot_HP=%4d, Base_HP=%4d, Outpost_HP=%4d\n",
                   Self_Robot_info.self_color_, Self_Robot_info.Robot_HP_, building_info.self_Base_HP, building_info.self_Outpost_HP);
            printf("Enemy--> Color=%2d, Robot_HP=%4d, Base_HP=%4d, Outpost_HP=%4d, Dist=%.2f, Yaw=%.2f, ID=%d Find_flag=%d\n",
                   1 - Self_Robot_info.self_color_, Hit_Robot_info.Robot_HP_, building_info.self_Base_HP, building_info.self_Outpost_HP, Hit_Robot_info.enemy_dist_, Hit_Robot_info.yaw_, Hit_Robot_info.armor_id, Hit_Robot_info.enemy_find_flag);
        }
    }
}

void ARES_FMS::thread_FMS_Control()
{
    step = 1;
    // std::chrono::duration<double> nav_timeout = 10;
    printf("\033[1;32m ----------->waiting for game \033[0m\n");
    printf("game_state: %d\n", Game_state);
    // go_point(7,nav_timeout);
    // go_point(8,nav_timeout);
    // go_point(9,nav_timeout);

    while (1)
    {
        // if (step)
        // {
        //     go_point(7, nav_timeout);
        //     go_point(8, nav_timeout);
        //     go_point(9, nav_timeout);
        //     step = 0;
        // }
        
        if (this->Game_state == 4)
        {
            switch (step)
            {
            case 0: // 回巡逻区
            {
                printf("\033[1;32m ----------->返回巡逻区 \033[0m\n");
                float arrive_base_flag = 0;
                while ( arrive_base_flag == 0 )
                {
                    printf("\033[1;32m ----------->返回中 \033[0m\n");
                    current_point--;
                    go_point(current_point, nav_timeout);
                    printf("cur_point %d\n", current_point);
                    if (current_point == 0)
                        arrive_base_flag = 1;
                }
                printf("\033[1;32m ----------->成功返回 \033[0m\n");
                if (building_info.self_Outpost_HP == 0)
                {
                    std_msgs::Float64 base_yaw;
                    t = t + 0.2;
                    base_yaw.data = base_vyaw + 0.4*sin(t);
                    // printf("base_vyaw: %f, t:%f, sin:%f \n", base_yaw.data, t, sin(t) );
                    base_yaw_pub.publish(base_yaw);
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                    if (t >= 1000)
                        t = 0;
                }
                if (Self_Robot_info.Robot_HP_ <= Self_HP_thre_low || (projectile_allowance_17mm <=allowance_17mm_thre_low && remain_gold >= remain_gold_thre_low))
                    {
                        printf("\033[1;32m 血量过少/弹量过少，返回补血区 \033[0m\n");
                        step = 8;
                    }
                else
                    step = 5;
            }
            break;
            case 1:
            {
                if (plan_flag == 1)  //只防守基地
                {
                    printf("\033[1;32m 只防守基地 \033[0m\n");
                    step = 3;
                }
                else if (plan_flag == 2) //去己方前哨站
                {
                    step = 2;
                    printf("\033[1;32m 防守己方前哨站 \033[0m\n");
                }
                else if (plan_flag == 3) //到敌方前哨站
                {
                    step = 4;
                    printf("\033[1;32m 进攻敌方前哨站 \033[0m\n");
                }
                else if (plan_flag == 4)
                {
                    step = 12;
                    printf("\033[1;32m 到飞坡区进攻敌方前哨站 \033[0m\n");
                }
            }
            break;
            case 2: // 到己方前哨站
            {
                printf("\033[1;32m ----------->去己方前哨站 \033[0m\n");
                go_point(1, nav_timeout);
                go_point(2, nav_timeout);
                step = 5;
            }
            break;
            case 3: // 不出去
            {
                printf("\033[1;32m ----------->只防守基地 \033[0m\n");
                go_point(0, nav_timeout);
                step = 5;
            }
            break;
            case 4: // 到敌方前哨站
            {
                printf("\033[1;32m ----------->到敌方前哨站 \033[0m\n");
                go_point(1, nav_timeout);
                go_point(2, nav_timeout);
                go_point(3, nav_timeout);
                go_point(4, nav_timeout);
                step = 9;
            }
            break;
            case 5: // 巡逻
            {
                printf("\033[1;32m ----------->巡逻 \033[0m\n");
                // task_random();

                if ( (current_point > 0 && (building_info.self_Outpost_HP <= Self_Outpost_HP_thre_low || Self_Robot_info.Robot_HP_ <= Self_HP_thre_low))
                      || (projectile_allowance_17mm <= allowance_17mm_thre_low  && current_point > 0 )  ) // 在外面 或者 弹量过少
                    {
                        printf("\033[1;32m 血量过少/弹量过少 \033[0m\n");
                        step = 0;
                    }
                if (current_point == 0 && (Self_Robot_info.Robot_HP_ <= Self_HP_thre_low ||( projectile_allowance_17mm <= allowance_17mm_thre_low  && remain_gold >= remain_gold_thre_low )))
                  {  
                    printf("\033[1;32m 血量过少/弹量过少，返回补血区 \033[0m\n");
                    step = 8;
                  }
                if (current_point == 0 && building_info.self_Outpost_HP == 0)
                {
                    // printf("\033[1;32m 无敌解除，开始自转 \033[0m\n");
                    std_msgs::Float64 base_yaw;
                    t1 = t1 + 0.2;
                    base_yaw.data = base_vyaw + 0.4*sin(t1);
                    // printf("base_vyaw: %f, t:%f, sin:%f \n", base_yaw.data, t1, sin(t1) );
                    base_yaw_pub.publish(base_yaw);
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                    if (t1 >= 1000)
                        t1 = 0;
                    //旋转偏离，重新导航
                    if ( (pow(Self_Robot_info.position_x_ - tarPoints[0].x, 2) + pow(Self_Robot_info.position_y_ - tarPoints[0].y, 2)) > 1.5)
                        {
                            printf("\033[1;32m 位置偏离 \033[0m\n");
                            go_point(0, nav_timeout);
                        }
                }
                if (Hit_Robot_info.enemy_find_flag == 1)
                {
                    printf("\033[1;32m 发现敌人 \033[0m\n");
                    step = 6;
                }
            }
            break;
            case 6: // 进攻
            {
                printf("\033[1;32m ----------->进攻 \033[0m\n");
                /*追击   */
                // pursuit_enemy();
                sleep(1);
                if (Hit_Robot_info.enemy_find_flag == 0)
                {
                    sleep(1);
                    if (Hit_Robot_info.enemy_find_flag == 0)
                        step = 7;
                }
            }
            break;
            case 7: // 目标丢失
            {
                printf("\033[1;32m ----------->目标丢失 返回初始点 \033[0m\n");
                // go_point(current_point, nav_timeout);
                step = 5;
            }
            break;
            case 8: // 回补血点
            {
                printf("\033[1;32m ----------->回补血点 \033[0m\n");
                goto_point(tarPoints[7].x, tarPoints[7].y, tole_dis_random, nav_timeout);

                auto last_time = std::chrono::high_resolution_clock::now();
                float time_out_flag = 0;
                while (Self_Robot_info.Robot_HP_ <= 350 && time_out_flag == 0) 
                {
                    auto current_time = std::chrono::high_resolution_clock::now();
                    std::chrono::duration<double> pass_time = current_time - last_time;
                    if (pass_time.count() >= suplly_time_out)
                    {
                        printf("\033[1;31m ----------->time out,suplly_time_out:%f \033[0m\n", suplly_time_out);
                        time_out_flag = 1;
                    }
                    // printf("\033[1;32m ----------->waiting for add HP \033[0m\n");
                }
                if (projectile_allowance_17mm <= allowance_17mm_thre_low && remain_gold >= remain_gold_thre_low)               
                {
                    auto last_time = std::chrono::high_resolution_clock::now();
                    float time_out_flag = 0;
                    while (projectile_allowance_17mm <= allowance_17mm_thre_low && time_out_flag == 0) 
                    {
                        auto current_time = std::chrono::high_resolution_clock::now();
                        
                        std::chrono::duration<double> pass_time = current_time - last_time;
                        printf("%.2f\n", pass_time.count());
                        if (pass_time.count() >= suplly_time_out)
                        {
                            time_out_flag = 1;
                            printf("\033[1;31m ----------->time out, suplly_time_out:%f \033[0m\n", suplly_time_out);
                        }
                        // printf("\033[1;32m ----------->waiting for add 17mm \033[0m\n");
                        std_msgs::Int8 allow_17mm;
                        if ((remain_gold - remain_gold_thre_low) <= 70 && (remain_gold - remain_gold_thre_low) <= 35)
                            allow_17mm.data = 1; // 50
                        if ((remain_gold - remain_gold_thre_low) >= 70 && (remain_gold - remain_gold_thre_low) <= 135)
                            allow_17mm.data = 2; // 100
                        if ((remain_gold - remain_gold_thre_low) >= 135 && (remain_gold - remain_gold_thre_low) <= 175)
                            allow_17mm.data = 3; // 150
                        if ((remain_gold - remain_gold_thre_low) >= 175 && (remain_gold - remain_gold_thre_low) <= 225)
                            allow_17mm.data = 4; // 200
                        suplly_17mm_pub.publish(allow_17mm);
                    }
                std_msgs::Int8 allow_17mm_non;
                allow_17mm_non.data = 0;
                suplly_17mm_pub.publish(allow_17mm_non);
                sleep(1);
                }
                step = 3;
            }
            break;
            case 9: // 打击前哨战
            {
                auto last_time = std::chrono::high_resolution_clock::now();
                uint8_t time_out_flag = 0;
                uint8_t self_Outpost_HP_low = 0;
                while (building_info.enemy_Outpost_HP > 0 && building_info.self_Outpost_HP >= Self_Outpost_HP_thre_low && time_out_flag == 0 && self_Outpost_HP_low ==0 )
                {
                    printf("打击\n");
                    auto current_time = std::chrono::high_resolution_clock::now();
                    std::chrono::duration<double> pass_time = current_time - last_time;
                    if (pass_time.count() >= enemy_Outpost_time_out)     
                    {              
                        time_out_flag ==1;  
                        printf("\033[1;31m ----------->time out, enemy_Outpost_time_out:%f \033[0m\n", enemy_Outpost_time_out);
                    }                
                    if (building_info.self_Outpost_HP <= Self_Outpost_HP_thre_low)
                        self_Outpost_HP_low = 1;
                }
                if (time_out_flag || self_Outpost_HP_low)  
                {
                    step = 10;
                    printf("\033[1;32m ----------->打击超时/己方前哨站血量过低 \033[0m\n");
                }
                if (building_info.enemy_Outpost_HP == 0 && Game_remain_time >= 300 && building_info.self_Outpost_HP >= 1200)
                {
                    if (assault_flag && plan_flag ==3)
                        step = 11;
                    else
                        step = 10;
                }
                else if (building_info.enemy_Outpost_HP == 0 && building_info.self_Outpost_HP <= 1200)
                    step = 10;
                else if (building_info.self_Outpost_HP == 0 && plan_flag == 3) // 回基地
                    step = 0;
                else if (building_info.self_Outpost_HP == 0 && plan_flag == 4) // 回基地
                    step = 10;
            }
            break;

            case 10: // 打击超时 回己方前哨站
            {
                if (plan_flag == 3)
                {
                    printf("\033[1;32m ----------->回己方前哨站 \033[0m\n");
                    go_point(3, nav_timeout);
                    go_point(2, nav_timeout);
                    step = 5;
                }
                else if (plan_flag == 4)
                {
                    printf("\033[1;32m ----------->返回基地 \033[0m\n");
                    go_point(9, nav_timeout);
                    go_point(8, nav_timeout);
                    go_point(0, nav_timeout);
                    step = 5;
                }
            }
            break;

            case 11: // 继续前进，进攻基地
            {
                printf("\033[1;32m ----------->继续前进，进攻基地 \033[0m\n");
                go_point(5, nav_timeout);
                go_point(6, nav_timeout);

                if (building_info.self_Outpost_HP <= Self_Outpost_HP_thre_low) // 回基地
                    step = 0;
                else
                    step = 5;
            }
            break;

            case 12: 
            {
                go_point(8, nav_timeout);
                go_point(9, nav_timeout);
                go_point(10, nav_timeout);
                step = 9;
            }
            break;

            default:
                break;
            }
        }
    }
}

void ARES_FMS::thread_update_TF()
{
    ros::NodeHandle nh_tf;
    ros::Rate rate1(100);
    

    while(ros::ok())
    {
        geometry_msgs::TransformStamped self_transformStamped;
        try
        {
            ros::Time now = ros::Time::now();
            self_transformStamped = self_tfBuffer_->lookupTransform("map", "base_footprint", now, ros::Duration(1));
        }
        catch(tf::TransformException &ex)
        {
            // ROS_WARN("DIDNT GET TRANSFORM ");
            // ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
        }

        Self_Robot_info.position_x_ = self_transformStamped.transform.translation.x;
        Self_Robot_info.position_y_ = self_transformStamped.transform.translation.y;

        if (Hit_Robot_info.enemy_find_flag)
        {
// #ifdef serial_enable
            Hit_Robot_info.position_x_ = Self_Robot_info.position_x_ + Hit_Robot_info.enemy_dist_ * cos(Hit_Robot_info.yaw_);
            Hit_Robot_info.position_y_ = Self_Robot_info.position_y_ + Hit_Robot_info.enemy_dist_ * sin(Hit_Robot_info.yaw_);
// #endif
            // Maker.mark_point(Hit_Robot_info.position_x_, Hit_Robot_info.position_y_, 1, 0, 0);
            // tf::Transform enemy_transform;
            // enemy_transform.setOrigin(tf::Vector3(Hit_Robot_info.position_x_, Hit_Robot_info.position_y_, 0.0));
            // enemy_transform.setRotation(tf::Quaternion(0, 0, 0, 1)); // 设置坐标变换的旋转部分
            // enemy_tf.sendTransform(tf::StampedTransform(enemy_transform,ros::Time::now(),"/base_footprint", "/enemy_footprint"));
        }
        rate1.sleep();
    }
}

void ARES_FMS::thread_Updatepursuit()
{
    // std::vector<std::vector<float>> matrix_weigh = obstancle_count();
    // std::vector<std::vector<float>> matrix_cost = weight_count(matrix_weigh);
    // printf("---------------cost mat \n");
    // printf("%.2f  %.2f  %.2f \n", matrix_cost[2][2], matrix_cost[2][1], matrix_cost[2][0]);
    // printf("%.2f  %.2f  %.2f \n", matrix_cost[1][2], matrix_cost[1][1], matrix_cost[1][0]);
    // printf("%.2f  %.2f  %.2f \n", matrix_cost[0][2], matrix_cost[0][1], matrix_cost[0][0]);
    // printf("--------------- \n");
    // printf("------>enemy_yaw: %.2f,  enemy_dist: %.2f\n", enemy_yaw_, enemy_dist_);
    // best_pursuit_dir = Find_best_dir(Hit_Robot_info.yaw_, Hit_Robot_info.enemy_dist_, matrix_cost);
    // printf
    // if (Hit_Robot_info.enemy_dist_ <= enemy2self_min_dis || Hit_Robot_info.enemy_dist_ >= enemy2self_max_dis)
    // {
        // if (best_pursuit_dir == 0)
        //     printf("best_dir : UP_LEFT \n");
        // else if (best_pursuit_dir == 1)
        //     printf("best_dir : UP \n");
        // else if (best_pursuit_dir == 2)
        //     printf("best_dir : UP_RIGHT \n");
        // else if (best_pursuit_dir == 3)
        //     printf("best_dir : LEFT \n");
        // else if (best_pursuit_dir == 5)
        //     printf("best_dir : RIGHT \n");
        // else if (best_pursuit_dir == 6)
        //     printf("best_dir : DOWN_LEFT \n");
        // else if (best_pursuit_dir == 7)
        //     printf("best_dir : DOWN \n");
        // else if (best_pursuit_dir == 8)
        //     printf("best_dir : DOWN_RIGHT \n");
    // }
}

void ARES_FMS::cmdCallback(const geometry_msgs::Twist msg)
{
    PIDNav_arrive_flag = msg.linear.z;
}

void ARES_FMS::cmd_cpoyCallBack(const geometry_msgs::Twist msg)
{
    //test
#ifndef serial_enable
    // Hit_Robot_info.position_x_ += msg.linear.x ;
    // Hit_Robot_info.position_y_ += msg.linear.y;
    // Hit_Robot_info.enemy_dist_ = sqrt(pow(Self_Robot_info.position_x_ - Hit_Robot_info.position_x_, 2) + pow(Self_Robot_info.position_y_ - Hit_Robot_info.position_y_, 2));
    // Hit_Robot_info.yaw_ = atan((Hit_Robot_info.position_y_ - Self_Robot_info.position_y_) / (Hit_Robot_info.position_x_ - Self_Robot_info.position_x_));
    // Hit_Robot_info.enemy_find_flag = 1;
    // Maker.mark_point(Hit_Robot_info.position_x_, Hit_Robot_info.position_y_, 1, 0, 0);
    
#endif 
}

void ARES_FMS::callback_global_plan(const nav_msgs::Path::ConstPtr& msg)
{
    if (msg->poses.empty())
        global_plan_state = 0;  //没有消息无法回调
    else
        global_plan_state = 1;
}

void ARES_FMS::CostMapCallback(const map_msgs::OccupancyGridUpdate::ConstPtr& msg)
{
    // Costmap_width_map_ = msg->width;
    // Costmap_height_map_ = msg->height;
    
    // if (!CostMap_init_flag)
    // {
    //     CostMap_data_.resize(Costmap_height_map_, std::vector<int>(Costmap_width_map_, 0));
    //     this->CostMap_init_flag = 1;
    // }

    // for (int h = 0; h < (int)(Costmap_height_map_); ++h) {
    //     for (int w = 0; w < (int)(Costmap_width_map_); ++w) {
    //         // 计算数组下标
    //         int index = h * Costmap_width_map_ + w;
    //         // 获取地图数据并存储到数组中
    //         if (msg->data[index]>=0)
    //             CostMap_data_[h][w] = msg->data[index];
    //         else
    //             CostMap_data_[h][w] = 1;
    //     }
    // }

    // std::vector<std::vector<float>> matrix_weigh = obstancle_count();
    // std::vector<std::vector<float>> matrix_cost = weight_count(matrix_weigh);
    // best_pursuit_dir = Find_best_dir(Hit_Robot_info.yaw_, Hit_Robot_info.enemy_dist_, matrix_cost);
    // if (Hit_Robot_info.enemy_dist_ <= enemy2self_min_dis || Hit_Robot_info.enemy_dist_ >= enemy2self_max_dis)
    // {
        // if (best_pursuit_dir == 0)
        //     printf("best_dir : UP_LEFT \n");
        // else if (best_pursuit_dir == 1)
        //     printf("best_dir : UP \n");
        // else if (best_pursuit_dir == 2)
        //     printf("best_dir : UP_RIGHT \n");
        // else if (best_pursuit_dir == 3)
        //     printf("best_dir : LEFT \n");
        // else if (best_pursuit_dir == 5)
        //     printf("best_dir : RIGHT \n");
        // else if (best_pursuit_dir == 6)
        //     printf("best_dir : DOWN_LEFT \n");
        // else if (best_pursuit_dir == 7)
        //     printf("best_dir : DOWN \n");
        // else if (best_pursuit_dir == 8)
        //     printf("best_dir : DOWN_RIGHT \n");
    // }
    // for (int h = 0; h < (int)(Costmap_height_map_); ++h) {
    //     for (int w = 0; w < (int)(Costmap_width_map_); ++w) {
    //         // 获取地图数据并存储到数组中
    //         CostMap_data_[h][w] = 0;
    //     }
    // }
}

void ARES_FMS::Odom_Callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    float yaww = msg->pose.pose.orientation.w;
    float yawx = msg->pose.pose.orientation.x;
    float yawy = msg->pose.pose.orientation.y;
    float yawz = msg->pose.pose.orientation.z;
    Self_Robot_info.yaw_rela = atan2(2 * (yaww * yawz + yawx * yawy), 1 - 2 * (yawy * yawy + yawz * yawz));
}

void ARES_FMS::STM_Date_callback(const rm_robotmsg::Stmdate &msg )
{
    Robot_HP.Blue1_HP = msg.Blue1_HP;
    Robot_HP.Blue2_HP = msg.Blue2_HP;
    Robot_HP.Blue3_HP = msg.Blue3_HP;
    Robot_HP.Blue4_HP = msg.Blue4_HP;
    Robot_HP.Blue5_HP = msg.Blue5_HP;
    Robot_HP.Blue7_HP = msg.Blue7_HP;
    Robot_HP.Blue_Base_HP = msg.Blue_Base_HP;
    Robot_HP.Blue_Outpost_HP = msg.Blue_Outpost_HP;
    

    Robot_HP.Red1_HP = msg.Red1_HP;
    Robot_HP.Red2_HP = msg.Red2_HP;
    Robot_HP.Red3_HP = msg.Red3_HP;
    Robot_HP.Red4_HP = msg.Red4_HP;
    Robot_HP.Red5_HP = msg.Red5_HP;
    Robot_HP.Red7_HP = msg.Red7_HP;
    Robot_HP.Red_Base_HP = msg.Red_Base_HP;
    Robot_HP.Red_Outpost_HP = msg.Red_Outpost_HP;

    Hit_Robot_info.armor_id = msg.armor_id;
    Self_Robot_info.self_color_ = msg.self_color;
    
    Hit_Robot_info.enemy_find_flag = msg.find_flag;
    if (Hit_Robot_info.armor_id)
    {
        Hit_Robot_info.yaw_ = Self_Robot_info.yaw_rela + msg.yaw;

        Hit_Robot_info.armor_id = msg.armor_id;
        Hit_Robot_info.enemy_dist_ = msg.enemy_dist;
        /* 判断赋值血量 */
        if (Self_Robot_info.self_color_ == 1) // 0:蓝
        {
            if (Hit_Robot_info.armor_id == 1)
            {
                Hit_Robot_info.Robot_HP_ = Robot_HP.Blue1_HP;
            }
            else if (Hit_Robot_info.armor_id == 2)
                Hit_Robot_info.Robot_HP_ = Robot_HP.Blue2_HP;
            else if (Hit_Robot_info.armor_id == 3)
                Hit_Robot_info.Robot_HP_ = Robot_HP.Blue3_HP;
            else if (Hit_Robot_info.armor_id == 4)
                Hit_Robot_info.Robot_HP_ = Robot_HP.Blue4_HP;
            else if (Hit_Robot_info.armor_id == 5)
                Hit_Robot_info.Robot_HP_ = Robot_HP.Blue5_HP;
            else if (Hit_Robot_info.armor_id == 6)
                Hit_Robot_info.Robot_HP_ = Robot_HP.Blue7_HP;
        }
        else
        {
            if (Hit_Robot_info.armor_id == 1)
                Hit_Robot_info.Robot_HP_ = Robot_HP.Red1_HP;
            else if (Hit_Robot_info.armor_id == 2)
                Hit_Robot_info.Robot_HP_ = Robot_HP.Red2_HP;
            else if (Hit_Robot_info.armor_id == 3)
                Hit_Robot_info.Robot_HP_ = Robot_HP.Red3_HP;
            else if (Hit_Robot_info.armor_id == 4)
                Hit_Robot_info.Robot_HP_ = Robot_HP.Red4_HP;
            else if (Hit_Robot_info.armor_id == 5)
                Hit_Robot_info.Robot_HP_ = Robot_HP.Red5_HP;
            else if (Hit_Robot_info.armor_id == 6)
                Hit_Robot_info.Robot_HP_ = Robot_HP.Red7_HP;
        }
    }

    /* 赋值血量 */
    Game_state = msg.Game_state;
    Game_remain_time = msg.Game_remain_time;
    projectile_allowance_17mm = msg.projectile_allowance_17mm;
    remain_gold = msg.remain_gold;
    // 接收机器人信息
    if (Self_Robot_info.self_color_ == 1)
    {
        Self_Robot_info.Robot_HP_ = Robot_HP.Red7_HP;
        building_info.self_Base_HP = Robot_HP.Red_Base_HP;
        building_info.self_Outpost_HP = Robot_HP.Red_Outpost_HP;
        building_info.enemy_Base_HP = Robot_HP.Blue_Base_HP;
        building_info.enemy_Outpost_HP = Robot_HP.Blue_Outpost_HP;
    }
    else
    {
        Self_Robot_info.Robot_HP_ = Robot_HP.Blue7_HP;
        building_info.self_Base_HP = Robot_HP.Blue_Base_HP;
        building_info.self_Outpost_HP = Robot_HP.Blue_Outpost_HP;
        building_info.enemy_Base_HP = Robot_HP.Red_Base_HP;
        building_info.enemy_Outpost_HP = Robot_HP.Red_Outpost_HP;
    }
    plan_flag = msg.plan;
}

bool ARES_FMS::Nav_to(float x, float y, float w)
{
    bool plan_flag = 0;
    move_base_msgs::MoveBaseGoal goal;
    uint8_t plan_count = 0;
    float goal_x = x;
    float goal_y = y;

    while (plan_count < 20 && plan_flag == 0)
    {
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = goal_x;
        goal.target_pose.pose.position.y = goal_y;
        goal.target_pose.pose.orientation.w = w;
        ac.sendGoal(goal);

        ros::Duration delay_duration(0.2);
        delay_duration.sleep();

        if (global_plan_state == 1)
        {
            plan_flag = 1;
            // printf("\033[33m 规划完成  \033[33m\n");
        }
        else
        {
            // 无法到达 目标点 从新规划
            printf(" \033[31m 无法到达 重新规划  \033[31m\n");
            goal_x = random_point(x, tole_dis_random);
            goal_y = random_point(y, tole_dis_random);
            ac.cancelGoal();
            plan_count++;
            plan_flag = 0;
        }
    }
    if (plan_count < 20)
        return 1;
    else
        return 0;
}

bool ARES_FMS::reach_point()
{
    if (PIDNav_arrive_flag == 1)
    {
        ac.cancelGoal();
        return true;
    }
    else
        return false;
}

bool ARES_FMS::goto_point(float goal_point_x, float goal_point_y, float tole_dis, double nav_timeout)
{
    Maker.mark_point(goal_point_x, goal_point_y , 1, 0, 0); //目标数组里的元素
    // double nav_start_time = ros::Time::now().toSec();
    auto nav_start_time = std::chrono::high_resolution_clock::now();
    // printf("\033[3m ------------> goal.x: %f, goal.y: %f \033[0m\n", goal_point_x, goal_point_y);
    bool plan_result = Nav_to(goal_point_x, goal_point_y, 1.0);
    printf("\033[1;32m naving \033[0m\n");
    // printf("\033[1;32m nav---->导航中 \033[0m\n");
    uint8_t FMS_state_last = this->FMS_state;
    while (!reach_point() && ros::ok())
    {
        // float nav_current_time = ros::Time::now().toSec();
        auto nav_current_time = std::chrono::high_resolution_clock::now();
        
        std::chrono::duration<double> pass_time = nav_current_time - nav_start_time;
        // printf("nav_time:%f\n", pass_time.count());
        if (pass_time.count() > nav_timeout)
        {
            printf("\033[1;31m      ----->导航超时, nav_timeout:%f\033[0m\n", nav_timeout);
            
            ac.cancelGoal();
            return 0;
        }
        if (plan_result == 0)
        {
            printf("\033[1;31m      ----->导航无法到达 \033[0m\n");
            ac.cancelGoal();
            return 0;
        }
        if (FMS_state != FMS_state_last)
        {
            printf("\033[1;33m      ------>状态更新，导航取消 \033[0m\n");
            ac.cancelGoal();
            return 0;
        }
        rate.sleep();
    }

    if (reach_point())
    {
        // printf("\033[1;32m      ------>到达目标  导航结束\033[0m\n");
        return 1;
    }
    else
    {
        printf("\033[1;32m      ------>导航失败 \033[0m\n");
        return 0;
    }
}

float ARES_FMS::random_point(float point, float raid)
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution <float> dist(-raid, raid); // 生成随机整数
    float random_number = dist(gen);
    float random_raid = point + random_number;
    return random_raid;
}

double ARES_FMS::dist_of_two_point(Point2D point1, Point2D point2)
{
    double dist = 0;
    dist = sqrt(pow((point2.x - point1.x), 2) + pow((point2.y - point1.y), 2));
    return dist;
}

void ARES_FMS::pursuit_enemy()
{
    printf("enemy_dist_:%f,  yaw_: %f\n", Hit_Robot_info.enemy_dist_, Hit_Robot_info.yaw_);
    if (Hit_Robot_info.enemy_dist_ <= enemy2self_min_dis) // 后退
    {
        float diff_dis = std::abs(Hit_Robot_info.enemy_dist_ - enemy2self_min_dis);
        geometry_msgs::Point back_point;
        if (diff_dis > 0.1)
        {
            printf("\033[1;32m ---->敌人过近 \033[0m\n");
            back_point.x = Self_Robot_info.position_x_ + diff_dis * point_symbol_x[best_pursuit_dir];
            back_point.y = Self_Robot_info.position_y_ + diff_dis * point_symbol_y[best_pursuit_dir];

            Maker.mark_point(back_point.x, back_point.y, 1, 0, 0);
            goto_point(back_point.x, back_point.y, tole_dis_random, nav_timeout);
            flag_count = 0;
        }
    }
    else if (Hit_Robot_info.enemy_dist_ >= enemy2self_max_dis) // 追击
    {
        float diff_dis = std::abs(Hit_Robot_info.enemy_dist_ - enemy2self_max_dis);
        geometry_msgs::Point up_point;
        if (diff_dis > 0.1)
        {
            printf("\033[1;32m ---->敌人过远 \033[0m\n");
            up_point.x = Self_Robot_info.position_x_ + diff_dis * point_symbol_x[best_pursuit_dir];
            up_point.y = Self_Robot_info.position_y_ + diff_dis * point_symbol_y[best_pursuit_dir];

            Maker.mark_point(up_point.x, up_point.y, 1, 0, 0);
            goto_point(up_point.x, up_point.y, tole_dis_random, nav_timeout);
            flag_count = 0;
        }
    }
    else // 原地不动
    {
        printf("\033[1;32m ---->距离合适 \033[0m\n");
    }
}

std::vector<std::vector<float>> ARES_FMS::obstancle_count()
{
    std::vector<std::vector<float>> Block_Obstacle_Weights;
    std::vector<std::vector<int>> Block_Obstacle_Sum;
    uint32_t Block_Obstacle_All = 0;


    geometry_msgs::Point point_enemy, point_self, point_self_in_tf;
    point_enemy.x = (int)((Hit_Robot_info.position_x_ - Obstacle_Weights.GetMap_Origin().x) / 0.05);
    point_enemy.y = (int)((Hit_Robot_info.position_y_ - Obstacle_Weights.GetMap_Origin().y) / 0.05);

    point_self.x = (int)((Self_Robot_info.position_x_ - Obstacle_Weights.GetMap_Origin().x) / 0.05);
    point_self.y = (int)((Self_Robot_info.position_y_ - Obstacle_Weights.GetMap_Origin().y) / 0.05);

    point_self_in_tf.x = Self_Robot_info.position_x_;
    point_self_in_tf.y = Self_Robot_info.position_y_;
    Maker.mark_rectangle(point_self_in_tf, slef_scan_area * 0.05);
    
    geometry_msgs::Point Map_Size;
    Map_Size.x = Obstacle_Weights.Get_Map_size().x;
    Map_Size.y = Obstacle_Weights.Get_Map_size().y;

    std::vector<std::vector<int>> Map_data_;
    Map_data_.resize(Map_Size.y, std::vector<int>(Map_Size.x, 0));
    Map_data_ = Obstacle_Weights.Get_Map_Date();

    //*************加入局部代价地图
    for (int h = 0; h < (int)(Costmap_height_map_); ++h) {
        for (int w = 0; w < (int)(Costmap_width_map_); ++w) {
            // 计算数组下标
            int index = h * Costmap_width_map_ + w;
            // 获取地图数据并存储到数组中
            Map_data_[point_self.y - Costmap_height_map_ + h*2][point_self.x - Costmap_width_map_ + w*2] = Map_data_[point_self.y - Costmap_height_map_ + h*2][point_self.x - Costmap_width_map_ + w*2] + CostMap_data_[h][w];
        }
    }

    Block_Obstacle_Sum.resize(4, std::vector<int>(4, 0));
    Block_Obstacle_Weights.resize(block_part_num, std::vector<float>(block_part_num, 0.0));
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            int min_x = std::fmax(0, point_self.x + (i - 2) * slef_scan_area / 4);
            int max_x = std::fmin(Map_Size.x - 1, point_self.x + (i - 1) * slef_scan_area / 4);
            int min_y = std::fmax(0, point_self.y + (j - 2) * slef_scan_area / 4);
            int max_y = std::fmin(Map_Size.y - 1, point_self.y + (j - 1) * slef_scan_area / 4);
            for (int x = min_x; x <= max_x; ++x)
            {
                for (int y = min_y; y <= max_y; ++y)
                {
                    Block_Obstacle_Sum[i][j] += Map_data_[y][x];
                    Block_Obstacle_All = Block_Obstacle_All + Map_data_[y][x];
                }
            }
        }
    }

    for (size_t i = 0; i < Block_Obstacle_Sum.size(); ++i)
    {
        for (size_t j = 0; j < Block_Obstacle_Sum[i].size(); ++j)
        {
            if (Block_Obstacle_All != 0)
            {
                Block_Obstacle_Weights[i][j] = (float)Block_Obstacle_Sum[i][j] / Block_Obstacle_All;
                // std::cout << Block_Obstacle_Sum[i][j] << std::endl;
            }
        }
    }
    //*************减去代价地图
    for (int h = 0; h < (int)(Costmap_height_map_); ++h) {
        for (int w = 0; w < (int)(Costmap_width_map_); ++w) {
            // 计算数组下标
            int index = h * Costmap_width_map_ + w;
            // 获取地图数据并存储到数组中
            Map_data_[point_self.y - Costmap_height_map_ + h*2][point_self.x - Costmap_width_map_ + w*2] =Map_data_[point_self.y - Costmap_height_map_ + h*2][point_self.x - Costmap_width_map_ + w*2] - CostMap_data_[h][w];
        }
    }
    // std::cout << "----------obstancle array: " << std::endl;
    // printf("%.2f  %.2f  %.2f  %.2f\n", Block_Obstacle_Weights[3][3], Block_Obstacle_Weights[3][2], Block_Obstacle_Weights[3][1], Block_Obstacle_Weights[3][0]);
    // printf("%.2f  %.2f  %.2f  %.2f\n", Block_Obstacle_Weights[2][3], Block_Obstacle_Weights[2][2], Block_Obstacle_Weights[2][1], Block_Obstacle_Weights[2][0]);
    // printf("%.2f  %.2f  %.2f  %.2f\n", Block_Obstacle_Weights[1][3], Block_Obstacle_Weights[1][2], Block_Obstacle_Weights[1][1], Block_Obstacle_Weights[1][0]);
    // printf("%.2f  %.2f  %.2f  %.2f\n", Block_Obstacle_Weights[0][3], Block_Obstacle_Weights[0][2], Block_Obstacle_Weights[0][1], Block_Obstacle_Weights[0][0]);
    // std::cout << "----------obstancle array: " << std::endl;

    for (size_t i = 0; i < Block_Obstacle_Sum.size(); ++i)
    {
        for (size_t j = 0; j < Block_Obstacle_Sum[i].size(); ++j)
        {
            Block_Obstacle_Sum[i][j] = 0;
            // Block_Obstacle_Weights[i][j] = 0.0;
        }
    }
    return Block_Obstacle_Weights;
}

std::vector<std::vector<float>> ARES_FMS::weight_count(const std::vector<std::vector<float>> & map)
{
    std::vector<std::vector<float>> costMatrix;
    int mapRows = map.size();
    int mapCols = map[0].size();
    costMatrix.resize(mapCols-1, std::vector<float>(mapRows-1, 0));

    // 在地图上滑动代价矩阵的窗口
    for (int i = 0; i < mapRows - 1; ++i) {
        for (int j = 0; j < mapCols - 1; ++j) {
            // 计算当前窗口中的代价总和
            float totalCost = 0;
            for (int m = 0; m < 2; ++m) {
                for (int n = 0; n < 2; ++n) {
                    totalCost += map[i + m][j + n];
                }
            }
            costMatrix[i][j] = totalCost;
        }
    }
    return costMatrix;
}

uint8_t ARES_FMS::Find_best_dir(float angle, float enemy2self_distance, std::vector<std::vector<float>> matrix_cost_)
{
    /* x y
        ++(1)   +0(2)     +-(3)
        0+(4)   self(0)   0-(5)
        -+(6)   -0(7)     --(8)
     */
    uint8_t point_center_x = 1, point_center_y = 1;
    uint8_t dir = 0;
    float atan1_2 = 0.464, aran2 = 1.107;
    if (enemy2self_distance >= enemy2self_max_dis)
    {
        if (angle > atan1_2 && angle < aran2)
            dir = ARES_FMS::UP_LEFT;
        else if (angle > -atan1_2 && angle < atan1_2)
            dir = ARES_FMS::UP;
        else if (angle > -aran2 && angle < -atan1_2)
            dir = ARES_FMS::UP_RIGHT;
        else if (angle > aran2 && angle < M_PI / 2 + atan1_2)
            dir = ARES_FMS::LEFT;
        else if (angle > -M_PI / 2 - atan1_2 && angle < -aran2)
            dir = ARES_FMS::RIGHT;
        else if (angle > M_PI / 2 + atan1_2 && angle < M_PI / 2 + aran2)
            dir = ARES_FMS::DOWN_LEFT;
        else if (angle > M_PI / 2 + aran2 || angle < -M_PI / 2 - aran2)
            dir = ARES_FMS::DOWN;
        else if (angle > -M_PI / 2 - aran2 && angle < -M_PI / 2 - atan1_2)
            dir = ARES_FMS::DOWN_RIGHT;
    }
    else if (enemy2self_distance <= enemy2self_min_dis)
    {
        if (angle > atan1_2 && angle < aran2)
            dir = ARES_FMS::DOWN_RIGHT;
        else if (angle > -atan1_2 && angle < atan1_2)
            dir = ARES_FMS::DOWN;
        else if (angle > -aran2 && angle < -atan1_2)
            dir = ARES_FMS::DOWN_LEFT;
        else if (angle > aran2 && angle < M_PI / 2 + atan1_2)
            dir = ARES_FMS::RIGHT;
        else if (angle > -M_PI / 2 - atan1_2 && angle < -aran2)
            dir = ARES_FMS::LEFT;
        else if (angle > M_PI / 2 + atan1_2 && angle < M_PI / 2 + aran2)
            dir = ARES_FMS::UP_RIGHT;
        else if (angle > M_PI / 2 + aran2 || angle < -M_PI / 2 - aran2)
            dir = ARES_FMS::UP;
        else if (angle > -M_PI / 2 - aran2 && angle < -M_PI / 2 - atan1_2)
            dir = ARES_FMS::UP_LEFT;
    }

    switch (dir)
    {
    case DOWN_LEFT:
    {
        if (matrix_cost_[0][2] == 0)
            dir = ARES_FMS::DOWN_LEFT;
        else
        {
            if (matrix_cost_[0][1] < matrix_cost_[0][2] && matrix_cost_[0][1] < matrix_cost_[1][2])
                dir = ARES_FMS::DOWN;
            if (matrix_cost_[1][2] < matrix_cost_[0][2] && matrix_cost_[1][2] < matrix_cost_[0][1])
                dir = ARES_FMS::LEFT;
            if (matrix_cost_[0][2] < matrix_cost_[0][1] && matrix_cost_[0][2] < matrix_cost_[1][2])
                dir = ARES_FMS::DOWN_LEFT;
        }
        break;
    }
    case LEFT:
    {
        if (matrix_cost_[1][2] == 0)
            dir = ARES_FMS::LEFT;
        else
        {
            if (matrix_cost_[1][2] < matrix_cost_[0][2] && matrix_cost_[1][2] < matrix_cost_[2][2])
                dir = ARES_FMS::LEFT;
            if (matrix_cost_[0][2] < matrix_cost_[1][2] && matrix_cost_[0][2] < matrix_cost_[2][2])
                dir = ARES_FMS::DOWN_LEFT;
            if (matrix_cost_[2][2] < matrix_cost_[1][2] && matrix_cost_[2][2] < matrix_cost_[0][2])
                dir = ARES_FMS::UP_LEFT;
        }
        break;
    }
    case UP_LEFT:
    {
        if (matrix_cost_[2][2] == 0)
            dir = ARES_FMS::UP_LEFT;
        else
        {
            if (matrix_cost_[2][2] < matrix_cost_[2][1] && matrix_cost_[2][2] < matrix_cost_[1][2])
                dir = ARES_FMS::UP_LEFT;
            if (matrix_cost_[1][2] < matrix_cost_[2][1] && matrix_cost_[1][2] < matrix_cost_[2][2])
                dir = ARES_FMS::LEFT;
            if (matrix_cost_[2][1] < matrix_cost_[2][2] && matrix_cost_[2][1] < matrix_cost_[1][2])
                dir = ARES_FMS::UP;
        }
        break;
    }
    case DOWN:
    {
        if (matrix_cost_[0][1] == 0)
            dir = ARES_FMS::DOWN;
        else
        {
            if (matrix_cost_[0][1] < matrix_cost_[0][2] && matrix_cost_[0][1] < matrix_cost_[0][0])
                dir = ARES_FMS::DOWN;
            if (matrix_cost_[0][2] < matrix_cost_[0][1] && matrix_cost_[0][2] < matrix_cost_[0][0])
                dir = ARES_FMS::DOWN_LEFT;
            if (matrix_cost_[0][0] < matrix_cost_[0][1] && matrix_cost_[0][0] < matrix_cost_[0][2])
                dir = ARES_FMS::DOWN_RIGHT;
        }
        break;
    }
    case UP:
    {
        if (matrix_cost_[2][1] == 0)
            dir = ARES_FMS::UP;
        else
        {
            if (matrix_cost_[2][2] < matrix_cost_[2][1] && matrix_cost_[2][2] < matrix_cost_[2][0])
                dir = ARES_FMS::UP_LEFT;
            if (matrix_cost_[2][1] < matrix_cost_[2][2] && matrix_cost_[2][1] < matrix_cost_[2][0])
                dir = ARES_FMS::UP;
            if (matrix_cost_[2][0] < matrix_cost_[2][2] && matrix_cost_[2][0] < matrix_cost_[2][1])
                dir = ARES_FMS::UP_RIGHT;
        }

        break;
    }
    case DOWN_RIGHT:
    {
        if (matrix_cost_[0][0] == 0)
            dir = ARES_FMS::DOWN_RIGHT;
        else
        {
            if (matrix_cost_[0][0] < matrix_cost_[0][1] && matrix_cost_[0][0] < matrix_cost_[1][0])
                dir = ARES_FMS::DOWN_RIGHT;
            if (matrix_cost_[0][1] < matrix_cost_[0][0] && matrix_cost_[0][1] < matrix_cost_[1][0])
                dir = ARES_FMS::DOWN;
            if (matrix_cost_[1][0] < matrix_cost_[0][0] && matrix_cost_[1][0] < matrix_cost_[0][1])
                dir = ARES_FMS::RIGHT;
        }
        break;
    }
    case RIGHT:
    {
        if (matrix_cost_[1][0] == 0)
            dir = ARES_FMS::RIGHT;
        else
        {
            if (matrix_cost_[0][0] < matrix_cost_[1][0] && matrix_cost_[0][0] < matrix_cost_[2][0])
                dir = ARES_FMS::DOWN_RIGHT;
            if (matrix_cost_[1][0] < matrix_cost_[0][0] && matrix_cost_[1][0] < matrix_cost_[2][0])
                dir = ARES_FMS::RIGHT;
            if (matrix_cost_[2][0] < matrix_cost_[0][0] && matrix_cost_[2][0] < matrix_cost_[1][0])
                dir = ARES_FMS::UP_RIGHT;
        }
        break;
    }
    case UP_RIGHT:
    {
        if (matrix_cost_[2][0] == 0)
            dir = ARES_FMS::UP_RIGHT;
        else
        {
            if (matrix_cost_[2][0] < matrix_cost_[2][1] && matrix_cost_[2][0] < matrix_cost_[1][0])
                dir = ARES_FMS::UP_RIGHT;
            if (matrix_cost_[1][0] < matrix_cost_[2][0] && matrix_cost_[1][0] < matrix_cost_[2][1])
                dir = ARES_FMS::RIGHT;
            if (matrix_cost_[2][1] < matrix_cost_[1][0] && matrix_cost_[2][1] < matrix_cost_[2][0])
                dir = ARES_FMS::UP;
        }
        break;
    }
    default:
    {
        printf("dir wrong");
        break;
    }
    }
    return dir;
}

void ARES_FMS::task_random()
{
    printf("\033[1;32m -----------开始巡逻------------- \033[0m\n");
    float current_pose_x = Self_Robot_info.position_x_;
    float current_pose_y = Self_Robot_info.position_y_;
    // ros::Rate rate(100);
    // while(ros::ok() && FMS_state ==1)
    // {
        float random_pose_x = random_point(current_pose_x, 0.6);
        float random_pose_y = random_point(current_pose_y, 0.6);
        if (sqrt(pow((current_pose_x - random_pose_x), 2)), pow((current_pose_y - random_pose_y), 2) > 0.5)
        {
            goto_point(random_pose_x, random_pose_y, tole_dis_random, nav_timeout);
        }
        // ros::Duration(5.0).sleep();
    // }
    printf("\033[1;32m -----------退出巡逻------------- \033[0m\n");
    goto_point(current_pose_x, current_pose_y, tole_dis_random, nav_timeout);
}


void ARES_FMS::go_point(uint8_t point,double nav_timeout)
{
    current_point = point;
    goto_point(tarPoints[point].x, tarPoints[point].y, tole_dis_random, nav_timeout);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ares_fms");
    ARES_FMS ares_fms;
    ros::spin();
    return 0;
}