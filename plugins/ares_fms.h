#include <ros/ros.h>
#include <string>
#include <math.h>
#include <mutex>
#include <shared_mutex>
#include <iostream>
#include <sstream>
#include <random>
#include <ros/time.h>
#include <chrono>
#include <thread>

#include "rm_robotmsg/RobotControl.h"
#include "rm_robotmsg/Stmdate.h"
#include "rm_robotmsg/RobotState.h"
#include <move_base_msgs/MoveBaseGoal.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include "std_msgs/Float64.h"
#include "std_msgs/Int8.h"
#include <nav_msgs/Odometry.h>
#include "../../config.h"
#include "tf/transform_datatypes.h"
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <map_msgs/OccupancyGridUpdate.h>
#include "obstacle_weights.h"
#include "maker.h"

#include <Eigen/Dense>

using namespace std;

class ARES_FMS
{
private:
    obstacle_weights Obstacle_Weights;
    maker Maker;
    ros::NodeHandle nh;

    ros::Publisher suplly_17mm_pub;
    ros::Publisher base_yaw_pub;
    // ros::Publisher marker_line_pub_;

    ros::Subscriber Base_vel_sub_;
    ros::Subscriber CopyCar_vel_sub_;
    ros::Subscriber STMDate_sub_;
    ros::Subscriber global_plan_sub_;
    ros::Subscriber localCostMap_sub_;
    ros::Subscriber odom_subscriber_;
    ros::Subscriber STM_Date_sub_;

    rm_robotmsg::RobotState Robotdate_msg_;
    rm_robotmsg::RobotControl robotcontrol_msg;
    rm_robotmsg::Stmdate STM_date;

    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac;
    ros::Rate rate;

    move_base_msgs::MoveBaseGoal goal;

    std::thread Update_FMS;
    std::thread Update_TF;
    std::thread FMS_Control;
    std::thread Update_pursuit;

    // visualization_msgs::MarkerArray marker_array;

    int global_plan_state = 0;
    float tole_dis_random = 0.5;
    int FMS_state = 2;

    std::unique_ptr<tf2_ros::Buffer> self_tfBuffer_;
    std::unique_ptr<tf2_ros::TransformListener> self_tfListener_;
    tf::TransformBroadcaster enemy_tf;

    float self_position_x = 0.0;
    float self_position_y = 0.0;
    uint8_t PIDNav_arrive_flag = 0;

    int slef_scan_area = 100; // 自身扫描像素范围
    uint8_t block_part_num = 4;  //unuse
    float obstancle_weight_thre = 0.15;
    float enemy2self_max_dis = 3.0;
    float enemy2self_min_dis = 2.5;
    int flag_count = 0;
    std::vector<std::vector<int>> CostMap_data_;
    int CostMap_init_flag = 0;
    float Costmap_width_map_ = 0.0;
    float Costmap_height_map_ = 0.0;

    enum ex_Direction
    {
        UP_LEFT = 0, //左上
        UP,
        UP_RIGHT,
        LEFT,    //左
        CENTER,
        RIGHT,    //右
        DOWN_LEFT,
        DOWN,
        DOWN_RIGHT,
    };

    int point_symbol_x[9] = {
        1, 1, 1,
        0, 0, 0,
        -1, -1, -1,
    };
    int point_symbol_y[9] = {
        1, 0, -1,
        1, 0, -1,
        1, 0, -1,
    };
    uint8_t best_pursuit_dir = -1;
    //决策数据
    uint8_t current_point = 0;
    uint8_t step = 1;
    int Self_HP_thre_low = 150;
    int Self_Outpost_HP_thre_low = 200;
    int remain_gold_thre_low = 200;
    int allowance_17mm_thre_low = 50;
    float nav_timeout = 0.0;
    float enemy_Outpost_time_out = 0.0;
    float suplly_time_out = 0.0;
    float base_vyaw = 0.0;
    int assault_flag = 0;
    float t =0.0;
    float t1 =0.0;

    /* 裁判系统数据 */
    int plan_flag = 0;
    int serial_debug_enable = 0;
    volatile int Game_state = 0;  //0:比赛未开始 1：准备 2：15s自检 3：5s倒计时 4：比赛中 5：比赛结束
    volatile uint16_t Game_remain_time = 0;
    volatile uint16_t projectile_allowance_17mm = 0;  //允许发弹量
    volatile uint16_t remain_gold = 0;
    struct robot_HP
    {
        uint16_t Red1_HP;
        uint16_t Red2_HP;
        uint16_t Red3_HP;
        uint16_t Red4_HP;
        uint16_t Red5_HP;
        uint16_t Red7_HP;
        uint16_t Red_Base_HP;
        uint16_t Red_Outpost_HP;

        uint16_t Blue1_HP;
        uint16_t Blue2_HP;
        uint16_t Blue3_HP;
        uint16_t Blue4_HP;
        uint16_t Blue5_HP;
        uint16_t Blue7_HP;
        uint16_t Blue_Base_HP;
        uint16_t Blue_Outpost_HP;
    };
    robot_HP Robot_HP;

    struct Robot_info
    {
        uint8_t armor_id = 0;
        uint16_t Robot_HP_ = 0;
        float position_x_ = 0;
        float position_y_ = 0;
        float  enemy_dist_ = 0; //enemy
        uint8_t enemy_find_flag; //enemy
        float yaw_ = 0;  
        float yaw_rela = 0;
        uint8_t self_color_ = 0;  //self
    };
    Robot_info Hit_Robot_info, Self_Robot_info;
    struct Building_info
    {
        uint16_t self_Base_HP;
        uint16_t self_Outpost_HP;
        uint16_t enemy_Base_HP;
        uint16_t enemy_Outpost_HP;
    };
    Building_info building_info;

public:
    ARES_FMS();
    ~ARES_FMS();
    void Param_Init();

    void thread_updateFMS();
    void thread_update_TF();
    void thread_FMS_Control();
    void thread_Updatepursuit();

    void task_random();
    void go_point(uint8_t point, double nav_time_out);

    void cmdCallback(const geometry_msgs::Twist msg);
    void cmd_cpoyCallBack(const geometry_msgs::Twist msg);
    void callback_global_plan(const nav_msgs::Path::ConstPtr& msg);
    void CostMapCallback(const map_msgs::OccupancyGridUpdate::ConstPtr& msg);
    void Odom_Callback(const nav_msgs::Odometry::ConstPtr &msg);
    void STM_Date_callback(const rm_robotmsg::Stmdate &msg );

    bool Nav_to(float x, float y, float w);
    float random_point(float point, float raid);
    bool goto_point(float goal_point_x, float goal_point_y, float tole_dis, double nav_timeout);
    bool reach_point();
    
    double dist_of_two_point(Point2D point1, Point2D  point2);
    void pursuit_enemy();
    std::vector<std::vector<float>> obstancle_count();
    std::vector<std::vector<float>> weight_count(const std::vector<std::vector<float>> & map);
    uint8_t Find_best_dir(float angle, float enemy2self_distance,std::vector<std::vector<float>> matrix_cost_);
};