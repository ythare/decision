#include "../include/move_base_nav.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

using namespace std;

// typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
rm_nav::rm_nav() : ac("move_base", true)
{
    ROS_INFO_STREAM("\033[1;32m----> Move_Base Navigation started.\033[0m");
    ros::service::waitForService("/move_base/make_plan");
    ROS_INFO("\033[1;32m----> move_base init OK \033[0m");

    // MoveBaseClient ac("move_base", true);
    ac.waitForServer(ros::Duration(60));
    ROS_INFO("Connected to move base server");

    goal_publisher_ = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
    RobotState_publisher_ = nh.advertise<rm_robotmsg::RobotState>("/Robotstate", 1);

    goal_subscriber_ = nh.subscribe("/move_base_simple/goal", 10, &rm_nav::rviz_goal_callback, this);
    mvStatus_subscriber_ = nh.subscribe("/move_base/status", 10, &rm_nav::State_Callback, this);
    odom_subscriber_ = nh.subscribe("/Odometry", 10, &rm_nav::Odom_Callback, this);
    TargetLocation_subscriber_ = nh.subscribe("/STM_Date", 10, &rm_nav::GetLocation_callback, this);

    robotdate_msg.vw = 0, robotdate_msg.vx = 0, robotdate_msg.vy = 0,
    robotdate_msg.w_now = 0, robotdate_msg.x_now = 0, robotdate_msg.w_now = 0;
    robotdate_msg.arrive_flag_last = 0, robotdate_msg.arrive_flag_last = 0;
}

rm_nav::~rm_nav()
{
    ROS_INFO("Destroying Movebase Navigation");
}

void rm_nav::rviz_goal_callback(geometry_msgs::PoseStamped goal)
{
    // ROS_INFO("\n\n\n rviz_goal: goal_x=%f, goal_y=%f\n\n", goal.pose.position.x, goal.pose.position.y);
    goal_received = true;
    this ->reset_arriveINFO_flag = true;
}

void rm_nav::State_Callback(const actionlib_msgs::GoalStatusArray::ConstPtr &status_msg)
{
    for (const auto &status : status_msg->status_list)
    {
        // 可以访问 status 对象的各种信息，例如状态、目标 ID 等
        MoveBase_Status = status.status;
        // ROS_INFO("Status: %d",status.status);
    }
}

void rm_nav::Odom_Callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    robotdate_msg.x_now = msg->pose.pose.position.x;
    robotdate_msg.y_now = msg->pose.pose.position.y;

    float yaww = msg->pose.pose.orientation.w;
    float yawx = msg->pose.pose.orientation.x;
    float yawy = msg->pose.pose.orientation.y;
    float yawz = msg->pose.pose.orientation.z;

    robotdate_msg.w_now = atan2(2 * (yaww * yawz + yawx * yawy), 1 - 2 * (yawy * yawy + yawz * yawz));

}

void rm_nav::GetLocation_callback(const rm_robotmsg::Stmdate &msg )
{
    // this->nav_goal = msg.goal;

    if (this->nav_goal_last != this->nav_goal)
    {
        this->goal_pub_flag = true;
        
        printf("\033[1;3;32m ------->goal: %d \033[0m\n ", this->nav_goal);
        this ->Tar_Turn_angle = Tar_angle_mat[this->nav_goal_last][this->nav_goal]-robotdate_msg.w_now;
    }

    this->nav_goal_last = this->nav_goal;

    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    
    if (this->nav_goal > 0)
    {
        goal.target_pose.pose.position.x = tarPoints[this->nav_goal].x;
        goal.target_pose.pose.position.y = tarPoints[this->nav_goal].y;
        goal.target_pose.pose.orientation.w = tarPoints[this->nav_goal].w;
    }
    else
    {
        // goal.target_pose.pose.position.x = std::numeric_limits<double>::quiet_NaN();
        // goal.target_pose.pose.position.y = std::numeric_limits<double>::quiet_NaN();
    }

    // printf("robot_x: %f, robot_y: %f , goal_x: %f, goal_y: %f \n", robotdate_msg.x_now,robotdate_msg.y_now,goal.pose.position.x,goal.pose.position.y);
    if ( ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED && this ->reset_arriveINFO_flag == true)
    {
        ac.cancelGoal();
        this ->reset_arriveINFO_flag = false;
        
        // goal.pose.position.x = std::numeric_limits<double>::quiet_NaN();
        // goal.pose.position.y = std::numeric_limits<double>::quiet_NaN();
        // goal_publisher_.publish(goal); // 取消目标
   
        ROS_INFO("\033[1;32m----> Arrived OK \033[0m");
        robotdate_msg.arrive_flag_last = robotdate_msg.arrive_flag;
        robotdate_msg.arrive_flag = 1 + robotdate_msg.arrive_flag;
        
        RobotState_publisher_.publish(robotdate_msg);
    }
    else
    {
        if (this->goal_pub_flag)
        {
            ac.sendGoal(goal);
            // goal_publisher_.publish(goal);
            this->goal_pub_flag = false;
            this ->reset_arriveINFO_flag = true;
        }
        bool finished_within_time = ac.waitForResult(ros::Duration(20));
        if ( !finished_within_time)
        {
            ac.cancelGoal();
            // goal.pose.position.x = std::numeric_limits<double>::quiet_NaN();
            // goal.pose.position.y = std::numeric_limits<double>::quiet_NaN();
            // goal_publisher_.publish(goal); // 取消目标
            ROS_INFO("UNable arrive goal");
        }
    }
    
}

void rm_nav::decision()
{
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_base_navigation"); // 节点的名字

    rm_nav rm_navigation;

    ros::MultiThreadedSpinner spinner(3); // Use 3 threads
    spinner.spin();                       // spin() will not return until the node has been shutdown
    return 0;
}