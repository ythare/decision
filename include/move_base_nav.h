#ifndef __RM_DECISION_H
#define __RM_DECISION_H

#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <time.h>
#include <sstream>
#include <string>
#include <math.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include <ros/time.h>
#include <boost/asio.hpp>
#include "tf/transform_datatypes.h"
#include <nav_msgs/Odometry.h>
#include "../../config.h"
#include "rm_robotmsg/RobotState.h"
#include "rm_robotmsg/RobotControl.h"
#include "rm_robotmsg/Stmdate.h"
#include "../../serialport/include/serialport.hpp"



class rm_nav
{
private:
    ros::NodeHandle nh;

    ros::Subscriber goal_subscriber_;
    ros::Subscriber mvStatus_subscriber_;
    ros::Subscriber odom_subscriber_;
    ros::Subscriber TargetLocation_subscriber_;


    ros::Publisher goal_publisher_;
    ros::Publisher RobotState_publisher_;

    // geometry_msgs::PoseStamped goal;
    move_base_msgs::MoveBaseGoal goal;

    rm_robotmsg::RobotState robotdate_msg;
    rm_robotmsg::RobotControl robotcontrol_msg;

 // typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac;

    bool goal_received = false;

    enum move_base_state_
    {
        PENDING = 0, // 目标已被接受，但还没有开始执行。
        ACTIVE,      // 目标正在执行中。
        PREEMPTED,   // 目标被取消（例如，通过调用 cancelGoal()）。
        SUCCEEDED,   // 目标已成功完成。
        ABORTED,     // 目标由于某种原因而无法完成。
        REJECTED,    // 目标被拒绝，可能是由于目标无法实现。
        PREEMPTING,  // 目标正在被取消。
        RECALLING,   // 目标已经被取消，但是底层动作服务器正在执行撤销操作。
        RECALLED,    // 目标已被取消，并且底层动作服务器已经成功执行撤销操作。
        LOST,        // 与目标关联的动作服务器已经丢失，这可能是由于与服务器的连接丢失引起的。
    } move_base_state;

    uint8_t MoveBase_Status = 10;
    int8_t nav_goal = 0;
    int8_t nav_goal_last = 0;
    int8_t Tar_Turn_angle = 0;
    bool goal_pub_flag = true;
    bool reset_arriveINFO_flag = false;



public:
    // MoveBaseClient ac;
    rm_nav();
    ~rm_nav();

    void rviz_goal_callback(geometry_msgs::PoseStamped goal);
    void Odom_Callback(const nav_msgs::Odometry::ConstPtr &msg);
    void State_Callback(const actionlib_msgs::GoalStatusArray::ConstPtr &status_msg);
    void GetLocation_callback(const rm_robotmsg::Stmdate &msg);
    void decision();

   
    // MoveBaseClient ac = MoveBaseClient("move_base", true);
};

#endif