#include <ros/time.h>
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <math.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

// tf2
#include <tf/transform_listener.h>
#include <tf2/utils.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/transform_broadcaster.h"


#include <nav_msgs/OccupancyGrid.h>
#include "nav_msgs/Path.h"

#include <thread>

class obstacle_weights
{
private:
    ros::NodeHandle nh;

    ros::Publisher pointcloud_pub_;
    ros::Subscriber scan_sub_;
    ros::Subscriber globalMap_sub_;
    // ros::Subscriber localCostMap_sub_;

    typedef pcl::PointXYZ PointT;
    typedef pcl::PointCloud<PointT> PointCloudT;
    PointCloudT::Ptr pointcloud_scan_; //pointcloud scan msg
    double Scan_Range_Max = 4.00;  //最大雷达数据距离
    double Scan_Range_Min = 0.30;  //最小雷达数据距离

    std::unique_ptr<tf2_ros::Buffer> tfBuffer_;
    std::unique_ptr<tf2_ros::TransformListener> tfListener_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    int scan_initialized_ = 0;

    Eigen::Isometry3d map_to_base_ = Eigen::Isometry3d::Identity();     //map到base的欧式变换矩阵4x4
    Eigen::Isometry3d map_to_lidar_ = Eigen::Isometry3d::Identity();     //map到laser的欧式变换矩阵4x4

public:
    int map_initialized_ = 0;
    std::vector<std::vector<int>> map_data_;
    int width_map = 0;
    int height_map = 0;
    geometry_msgs::Point map_origin;
    float robot_pose_x = 0.0;
    float robot_pose_y = 0.0;

    obstacle_weights();
    ~obstacle_weights();

    void ScanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg);
    void ScanToPointCloudOnMap(const sensor_msgs::LaserScan::ConstPtr &scan_msg, PointCloudT::Ptr &cloud_msg);

    // void CostMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void MapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    geometry_msgs::Point GetMap_Origin();
    geometry_msgs::Point Get_Map_size();
    std::vector<std::vector<int>> Get_Map_Date();
    Eigen::Matrix3d obstacle_weights_global(geometry_msgs::Point point1, geometry_msgs::Point point2, int part_num);

    bool GetTransform(Eigen::Isometry3d &trans , const std::string parent_frame, const std::string child_frame, const ros::Time stamp);
};