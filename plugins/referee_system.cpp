#include "ros/ros.h"
#include "rm_robotmsg/Stmdate.h"
#include <iostream>
#include <fstream>
#include <time.h>
#include <sstream>
#include <string>
#include <dynamic_reconfigure/server.h>
#include  "/home/ares_yt/ares/devel/include/rm_decision/refereeConfig.h"

int Blue1_HP, Blue2_HP, Blue3_HP, Blue4_HP, Blue5_HP, Blue7_HP, Blue_Base_HP, Blue_Outpost_HP;
int Red1_HP, Red2_HP, Red3_HP, Red4_HP, Red5_HP, Red7_HP, Red_Base_HP, Red_Outpost_HP, Game_remain_time, remain_gold, projectile_allowance_17mm;
int plan, armor_id, self_color, Game_state, find_flag, serial_open;
float enemy_dist, yaw;

rm_robotmsg::Stmdate STM_date;

void callback(referee::refereeConfig &msg,uint32_t level)
{
    serial_open = msg.serial_open;
    if (serial_open)
    {
        STM_date.Blue1_HP = msg.Blue1_HP;
        STM_date.Blue1_HP = msg.Blue1_HP;
        STM_date.Blue2_HP = msg.Blue2_HP;
        STM_date.Blue3_HP = msg.Blue3_HP;
        STM_date.Blue4_HP = msg.Blue4_HP;
        STM_date.Blue5_HP = msg.Blue5_HP;
        STM_date.Blue7_HP = msg.Blue7_HP;
        STM_date.Blue_Base_HP = msg.Blue_Base_HP;
        STM_date.Blue_Outpost_HP = msg.Blue_Outpost_HP;

        STM_date.Red1_HP = msg.Red1_HP;
        STM_date.Red2_HP = msg.Red2_HP;
        STM_date.Red3_HP = msg.Red3_HP;
        STM_date.Red4_HP = msg.Red4_HP;
        STM_date.Red5_HP = msg.Red5_HP;
        STM_date.Red7_HP = msg.Red7_HP;
        STM_date.Red_Base_HP = msg.Red_Base_HP;
        STM_date.Red_Outpost_HP = msg.Red_Outpost_HP;

        STM_date.armor_id = msg.armor_id;
        STM_date.Game_remain_time = msg.Game_remain_time;
        STM_date.remain_gold = msg.remain_gold;
        STM_date.projectile_allowance_17mm = msg.projectile_allowance_17mm;
        STM_date.enemy_dist = msg.enemy_dist;
        STM_date.yaw = msg.yaw;
        STM_date.plan = msg.plan;
        STM_date.self_color = msg.self_color;
        STM_date.Game_state = msg.Game_state;
        STM_date.find_flag = msg.find_flag;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "referee__");
    ros::NodeHandle nh;
    ros::Publisher STM_pub = nh.advertise<rm_robotmsg::Stmdate>("/STM_Date_virtual", 10);


    dynamic_reconfigure::Server<referee::refereeConfig> server;
    dynamic_reconfigure::Server<referee::refereeConfig>::CallbackType ca = boost::bind(&callback,_1,_2);
    server.setCallback(ca);

    nh.param<int>("Blue1_HP", Blue1_HP, 400);
    nh.param<int>("Blue2_HP", Blue2_HP, 100);
    nh.param<int>("Blue3_HP", Blue3_HP, 100);
    nh.param<int>("Blue4_HP", Blue4_HP, 100);
    nh.param<int>("Blue5_HP", Blue5_HP, 100);
    nh.param<int>("Blue7_HP", Blue7_HP, 400);
    nh.param<int>("Blue_Base_HP", Blue_Base_HP, 1500);
    nh.param<int>("Blue_Outpost_HP", Blue_Outpost_HP, 1500);

    nh.param<int>("Red1_HP", Red1_HP, 400);
    nh.param<int>("Red2_HP", Red2_HP, 440);
    nh.param<int>("Red3_HP", Red3_HP, 400);
    nh.param<int>("Red4_HP", Red4_HP, 400);
    nh.param<int>("Red5_HP", Red5_HP, 400);
    nh.param<int>("Red7_HP", Red7_HP, 400);
    nh.param<int>("Red_Base_HP", Red_Base_HP, 1500);
    nh.param<int>("Red_Outpost_HP", Red_Outpost_HP, 1500);

    nh.param<int>("plan", plan, 3);
    nh.param<int>("armor_id", armor_id, 0);
    nh.param<int>("self_color", self_color, 0);
    nh.param<float>("enemy_dist", enemy_dist, 0);
    nh.param<float>("yaw", yaw, 0);
    nh.param<int>("Game_state", Game_state, 4);
    nh.param<int>("Game_remain_time", Game_remain_time, 500);
    nh.param<int>("remain_gold", remain_gold, 100);
    nh.param<int>("projectile_allowance_17mm", projectile_allowance_17mm, 400);
    nh.param<int>("find_flag", find_flag, 0);
    ros::Rate rate(100);
    while (ros::ok())
    {
        // printf("Blue1_HP: %d\n", Blue1_HP);
        // STM_date.Blue1_HP = (uint16_t)Blue1_HP;
        // STM_date.Blue2_HP = (uint16_t)Blue2_HP;
        // STM_date.Blue3_HP = (uint16_t)Blue3_HP;
        // STM_date.Blue4_HP = (uint16_t)Blue4_HP;
        // STM_date.Blue5_HP = (uint16_t)Blue5_HP;
        // STM_date.Blue7_HP = (uint16_t)Blue7_HP;
        // STM_date.Blue_Base_HP = (uint16_t)Blue_Base_HP;
        // STM_date.Blue_Outpost_HP = (uint16_t)Blue_Outpost_HP;

        // STM_date.Red1_HP = (uint16_t)Red1_HP;
        // STM_date.Red2_HP = (uint16_t)Red2_HP;
        // STM_date.Red3_HP = (uint16_t)Red3_HP;
        // STM_date.Red4_HP = (uint16_t)Red4_HP;
        // STM_date.Red5_HP = (uint16_t)Red5_HP;
        // STM_date.Red7_HP = (uint16_t)Red7_HP;
        // STM_date.Red_Base_HP = (uint16_t)Red_Base_HP;
        // STM_date.Red_Outpost_HP = (uint16_t)Red_Outpost_HP;

        // STM_date.plan = (uint8_t)plan;
        // STM_date.armor_id =(uint8_t) armor_id;
        // STM_date.self_color = (uint8_t)self_color;
        // STM_date.enemy_dist = (double)enemy_dist;
        // STM_date.yaw = (double)yaw;
        // STM_date.Game_state = (uint8_t)Game_state;
        // STM_date.Game_remain_time = (uint16_t)Game_remain_time;
        // STM_date.remain_gold = (uint16_t)Game_remain_time;
        // STM_date.projectile_allowance_17mm = (uint16_t)projectile_allowance_17mm;
        // STM_date.find_flag =(uint8_t) find_flag;
        STM_pub.publish(STM_date);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}