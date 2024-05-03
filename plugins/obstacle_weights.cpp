#include "obstacle_weights.h"

obstacle_weights::obstacle_weights() 
{
    scan_sub_ = nh.subscribe("/scan_filtered",1, &obstacle_weights::ScanCallback, this);
    pointcloud_pub_ = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("scan_point_cloud", 1);
    
    globalMap_sub_ = nh.subscribe("map",1,&obstacle_weights::MapCallback, this);


    pointcloud_scan_ = boost::shared_ptr<PointCloudT>(new PointCloudT());
    tfBuffer_.reset(new tf2_ros::Buffer());
    tfListener_.reset(new tf2_ros::TransformListener(*tfBuffer_));

    Scan_Range_Max = 4.0;
    Scan_Range_Min = 0.3;
}

obstacle_weights::~obstacle_weights()
{
}

void obstacle_weights::ScanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
    // if (!scan_initialized_)
    // {
        // map_to_base_ = Eigen::Isometry3d::Identity();
        // if (!GetTransform(map_to_base_, "map", "base_footprint", scan_msg->header.stamp))
        // {
            // ROS_WARN("Did not get base pose at now");
            // return;
        // }
        // map_to_lidar_ = map_to_base_;
        // scan_initialized_ = 1;
    // }
    // ScanToPointCloudOnMap(scan_msg, pointcloud_scan_);

    // std::map<float, int> angle_count_map; // 角度及对应的点数
    
    // for (const pcl::PointXYZ& point : pointcloud_scan_->points) {
    //     // 获取点的扫描角度
    //     float azimuth = atan2(point.y, point.x) * 180.0 / M_PI;

    //     // 统计角度出现的次数
    //     angle_count_map[azimuth]++;
    // }
    // std::cout << "Scan angle statistics:" << std::endl;
    // for (const auto& pair : angle_count_map) {
    //     std::cout << "Angle: " << pair.first << ", Point count: " << pair.second << std::endl;
    // }

    // pointcloud_pub_.publish(pointcloud_scan_);
}


void obstacle_weights::ScanToPointCloudOnMap(const sensor_msgs::LaserScan::ConstPtr &scan_msg, PointCloudT::Ptr &cloud_msg)
{
    // std::cout << "start Scan to pointcloud......" << std::endl;
    // 计算雷达数据点长度
    unsigned int point_len = 0;
    for (unsigned int i = 0; i < scan_msg->ranges.size(); ++i)
    {
        // 获取scan的第i个点的距离值
        float range = scan_msg->ranges[i];
        // std::cout << "range = " << range << std::endl; 

        // 将 inf 与 nan 点 设置为无效点
        if (!std::isfinite(range))
        {
            continue;
        }
        // 有些雷达驱动会将无效点设置成 range_max+1
        // 所以要根据雷达的range_min与range_max进行有效值的判断
        if ((range > scan_msg->range_min) && (range < scan_msg->range_max) 
            && (range > Scan_Range_Min) && (range < Scan_Range_Max))
        {
            point_len += 1;

        }
    }
    // 对容器进行初始化
    unsigned int point_num = 0;
    cloud_msg->points.resize(point_len);

    // tran_start_time_ = std::chrono::steady_clock::now();
    for (unsigned int i = 0; i < scan_msg->ranges.size(); ++i)
    {
        // 获取scan的第i个点的距离值
        float range = scan_msg->ranges[i];
        // std::cout << "range = " << range << std::endl; 
        // 将 inf 与 nan 点 设置为无效点
        if (!std::isfinite(range))
        {
            continue;
        }
        // 有些雷达驱动会将无效点设置成 range_max+1
        // 所以要根据雷达的range_min与range_max进行有效值的判断
        if (range > scan_msg->range_min && range < scan_msg->range_max 
            && range > Scan_Range_Min && range < Scan_Range_Max)
        {
            // 首先声明一个 cloud_msg第i个点的 引用
            PointT &point_tmp = cloud_msg->points[point_num];

            // 获取第i个点对应的角度
            float angle = scan_msg->angle_min + i * scan_msg->angle_increment;
            // 获取第i个点在笛卡尔坐标系下的坐标
            // point_tmp.x = range * cos(angle);
            // point_tmp.y = range * sin(angle);
            // point_tmp.z = 0.0;

            Eigen::Vector3d point_vector(range * cos(angle), range * sin(angle), 0.0);
            point_vector = map_to_lidar_ * point_vector;     //进行坐标变换

            // std::cout << "point_vector after tranform = " << point_vector.transpose() << std::endl; 
            // std::cout << "point_vector(0) = " << point_vector(0) << std::endl; 

            // 获取map坐标系下在笛卡尔坐标系下的坐标
            point_tmp.x = point_vector(0);
            point_tmp.y = point_vector(1);
            point_tmp.z = 0.0;

            point_num += 1;

        }
    }
    // tran_end_time_ = std::chrono::steady_clock::now();
    // tran_time_used_ = std::chrono::duration_cast<std::chrono::duration<double>>(tran_end_time_ - tran_start_time_);
    // std::cout << "雷达数据转换后半处理用时: " << tran_time_used_.count() << " 秒。" << std::endl;

    // cloud_msg->width = scan_msg->ranges.size();
    cloud_msg->width = point_len;
    cloud_msg->height = 1;
    cloud_msg->is_dense = false; // contains nans

    std_msgs::Header header;
    header.stamp = scan_msg->header.stamp;
    // header.frame_id = lidar_frame_;
    header.frame_id = "map";
    cloud_msg->header = pcl_conversions::toPCL(header);


}

bool obstacle_weights::GetTransform(Eigen::Isometry3d &trans , const std::string parent_frame, const std::string child_frame, const ros::Time stamp)
{
    uint8_t gotTransform = 0;
    trans = Eigen::Isometry3d::Identity();     //map到base的欧式变换矩阵4x4
    geometry_msgs::TransformStamped transformStamped;
    
    ros::Time now = ros::Time::now();
    try
    {
        gotTransform = 1;
        // transformStamped = tfBuffer_.lookupTransform(parent_frame, child_frame, stamp,ros::Duration(1.0));
        transformStamped = tfBuffer_->lookupTransform(parent_frame, child_frame, now, ros::Duration(1));
        std::cout << "input rostime of Transform:" << stamp <<std::endl;
        std::cout << "output rostime of Transform:" << transformStamped.header.stamp <<std::endl;
    }
    catch (tf2::TransformException &ex)
    {
        gotTransform = 0;
        ROS_WARN("DIDNT GET TRANSFORM %s %s IN B", parent_frame.c_str(), child_frame.c_str());
        ROS_WARN("%s", ex.what());
        ros::Duration(1.0).sleep();
        return 0;
    }

    tf2::Quaternion quat(
                transformStamped.transform.rotation.x,
                transformStamped.transform.rotation.y,
                transformStamped.transform.rotation.z,
                transformStamped.transform.rotation.w);

    double roll, pitch, yaw;   //定义存储r\p\y的容器
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换
    std::cout << "trans roll, pitch, yaw =  " << roll << "  " << pitch << "  " << yaw << std::endl;

    Eigen::Matrix3d point_rotation;
    point_rotation = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());

    // std::cout << "point_rotation =  \n" << point_rotation <<std::endl;

    trans.rotate(point_rotation);
    trans.pretranslate(Eigen::Vector3d(transformStamped.transform.translation.x,
                                transformStamped.transform.translation.y,
                                transformStamped.transform.translation.z));


    return gotTransform;
}

void obstacle_weights::MapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    // printf("\033[1;32m ---->Map Callback \033[0m\n");
    width_map = msg->info.width;
    height_map = msg->info.height;
    float resolution = msg->info.resolution;
    map_origin = msg->info.origin.position;
    // printf("map_origin: %f, %f\n", map_origin.x, map_origin.y);
    // printf("width_map: %d, height_map %d\n", width_map, height_map);
    
    if (!map_initialized_)
    {
        map_data_.resize(height_map, std::vector<int>(width_map, 0));
        this->map_initialized_ = 1;
    }

    for (int h = 0; h < height_map; ++h) {
        for (int w = 0; w < width_map; ++w) {
            // 计算数组下标
            int index = h * width_map + w;
            // 获取地图数据并存储到数组中
            map_data_[h][w] = msg->data[index];
        }
    }
    // printf("arr26_1: %d, arr11_0: %d, %d", map_data_[26][1], map_data_[27][1], map_data_[28][1]);
    // printf("\033[1;32m ---->Map Callback over \033[0m\n");
}

geometry_msgs::Point obstacle_weights::GetMap_Origin()
{
    geometry_msgs::Point Map_origin_;
    Map_origin_.x = map_origin.x;
    Map_origin_.y = map_origin.y;
    Map_origin_.z = 0;
    return Map_origin_;

}

geometry_msgs::Point obstacle_weights::Get_Map_size()
{
    geometry_msgs::Point Map;
    Map.x = width_map;
    Map.y = height_map;
    Map.z = 0;
    return Map;
}

std::vector<std::vector<int>> obstacle_weights::Get_Map_Date()
{
    std::vector<std::vector<int>> Map_Date;
    Map_Date.resize(height_map, std::vector<int>(width_map, 0));
    Map_Date = map_data_;
    return Map_Date;
}

Eigen::Matrix3d obstacle_weights::obstacle_weights_global(geometry_msgs::Point point1, geometry_msgs::Point point2, int part_num)
{
    printf("%f, %f, %f, %f \n",point1.x, point1.y,point2.x, point2.y);
    Eigen::Matrix3d array;
    float square_h = std::abs(point1.x - point2.x);
    float square_w = std::abs(point1.y - point2.y);
    float sum_date = 0.0;
    printf("square_h: %f, square_w: %f", square_h, square_w);

    float base_point_x = std::min(point1.x, point2.x);
    float base_point_y = std::min(point1.y, point2.y);

    for (int i = 0; i < part_num; i++)
    {
        for (int j = 0; j < part_num; j++)
        {
            int min_x = std::fmax(0, base_point_x + i * square_h / part_num);
            int max_x = std::fmin(width_map - 1, base_point_x + (i + 1) * square_h / part_num);
            int min_y = std::fmax(0, base_point_y + j * square_h / part_num);
            int max_y = std::fmin(height_map - 1, base_point_y + (j + 1) * square_h / part_num);

            for (int x = min_x; x <= max_x; ++x)
            {
                for (int y = min_y; y <= max_y; ++y)
                {
                    if (map_data_[y][x] > 0)
                    {
                        array.coeffRef(i, j) += map_data_[y][x];
                        sum_date += map_data_[y][x];
                    }
                }
            }
        }
        // array /= sum_date;

        std::cout <<"obstancle array: " <<array << std::endl;
        return array;
    }
}

