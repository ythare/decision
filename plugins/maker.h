#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <ros/time.h>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>

class maker
{
private:
    ros::NodeHandle nh;
    ros::Publisher marker_pub_;
    ros::Publisher marker_rectangle_pub_;
    ros::Publisher marker_line_pub_;

    int marker_count = 0;
    visualization_msgs::MarkerArray marker_array;
public:
    maker()
    {
        marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("maker/visualization_marker", 1);
        marker_rectangle_pub_ = nh.advertise<visualization_msgs::Marker>("maker/rectangle", 1);
        marker_line_pub_ = nh.advertise<visualization_msgs::Marker>("maker/line", 1);
    }
    ~maker()
    {
    }

    void mark_point(float x, float y, uint8_t r, uint8_t g, uint8_t b)
    {
        if (marker_array.markers.size() > 5)
        {
            marker_array.markers.erase(marker_array.markers.begin());
            marker_array.markers[0].color.a = 0.0;
            // marker_array.markers[1].color.a = 0.0;
            // marker_array.markers[0].color.a = 0.0;
            // marker_array.markers[1].color.a = 0.0;
        }

        visualization_msgs::Marker marker;
        // 设置消息的类型
        marker.type = visualization_msgs::Marker::SPHERE;
        // 设置消息的位置
        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = 0.0;
        // 设置消息的大小
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
        // 设置消息的颜色
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = 1.0;
        // 设置消息的时间戳
        marker.header.stamp = ros::Time::now();
        // 设置消息的坐标系
        marker.header.frame_id = "map";
        // 设置消息的唯一标识符
        marker.id = marker_count;

        // 发布 Marker 消息到 MarkerArray
        marker_array.markers.push_back(marker);
        marker_pub_.publish(marker_array);

        marker_count++;
    }

    void mark_rectangle(geometry_msgs::Point center, float lenth)
    {
        // 创建一个Marker消息
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "rectangle";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.1; // 线宽
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        // 定义矩形的四个顶点
        geometry_msgs::Point p1, p2, p3, p4;
        p1.x = center.x - lenth / 2;
        p1.y = center.y - lenth / 2;
        p1.z = 0.0;
        p2.x = center.x - lenth / 2;
        p2.y = center.y + lenth / 2;
        p2.z = 0.0;
        p3.x = center.x + lenth / 2;
        p3.y = center.y + lenth / 2;
        p3.z = 0.0;
        p4.x = center.x + lenth / 2;
        p4.y = center.y - lenth / 2;
        p4.z = 0.0;

        // 将矩形的四个顶点添加到Marker消息中
        marker.points.push_back(p1);
        marker.points.push_back(p2);
        marker.points.push_back(p3);
        marker.points.push_back(p4);
        marker.points.push_back(p1); // 连接回第一个顶点，形成闭合的矩形
        ///////////////////////

        visualization_msgs::Marker marker1;
        marker1.header.frame_id = "map";
        marker1.header.stamp = ros::Time::now();
        marker1.ns = "rectangle";
        marker1.id = 1;
        marker1.type = visualization_msgs::Marker::LINE_STRIP;
        marker1.action = visualization_msgs::Marker::ADD;
        marker1.pose.position.x = 0;
        marker1.pose.position.y = 0;
        marker1.pose.position.z = 0;
        marker1.pose.orientation.x = 0.0;
        marker1.pose.orientation.y = 0.0;
        marker1.pose.orientation.z = 0.0;
        marker1.pose.orientation.w = 1.0;
        marker1.scale.x = 0.1; // 线宽
        marker1.color.r = 1.0;
        marker1.color.g = 0.0;
        marker1.color.b = 0.0;
        marker1.color.a = 1.0;

        // 定义矩形的四个顶点
        geometry_msgs::Point p1_e, p2_e, p3_e, p4_e;
        p1_e.x = center.x - lenth / 4;
        p1_e.y = center.y - lenth / 4;
        p1_e.z = 0.0;
        p2_e.x = center.x - lenth / 4;
        p2_e.y = center.y + lenth / 4;
        p2_e.z = 0.0;
        p3_e.x = center.x + lenth / 4;
        p3_e.y = center.y + lenth / 4;
        p3_e.z = 0.0;
        p4_e.x = center.x + lenth / 4;
        p4_e.y = center.y - lenth / 4;
        p4_e.z = 0.0;

        // 将矩形的四个顶点添加到Marker消息中
        marker1.points.push_back(p1_e);
        marker1.points.push_back(p2_e);
        marker1.points.push_back(p3_e);
        marker1.points.push_back(p4_e);
        marker1.points.push_back(p1_e); // 连接回第一个顶点，形成闭合的矩形
        ///////////////////////

        visualization_msgs::Marker line_marker;
        line_marker.header.frame_id = "map";
        line_marker.header.stamp = ros::Time::now();
        line_marker.ns = "line";
        line_marker.id = 2; // 设置不同的ID
        line_marker.type = visualization_msgs::Marker::LINE_STRIP;
        line_marker.action = visualization_msgs::Marker::ADD;
        line_marker.pose.orientation.w = 1.0;
        line_marker.scale.x = 0.1;
        line_marker.color.r = 1.0;
        line_marker.color.g = 0.0;
        line_marker.color.b = 0.0;
        line_marker.color.a = 1.0;

        geometry_msgs::Point p1_, p2_;
        p1_.x = center.x - lenth / 2;
        p1_.y = center.y;

        p2_.x = center.x + lenth / 2;
        p2_.y = center.y;

        line_marker.points.push_back(p1_);
        line_marker.points.push_back(p2_);

        ///////////////////////////
        visualization_msgs::Marker line_marker1;
        line_marker1.header.frame_id = "map";
        line_marker1.header.stamp = ros::Time::now();
        line_marker1.ns = "line";
        line_marker1.id = 3; // 设置不同的ID
        line_marker1.type = visualization_msgs::Marker::LINE_STRIP;
        line_marker1.action = visualization_msgs::Marker::ADD;
        line_marker1.pose.orientation.w = 1.0;
        line_marker1.scale.x = 0.1;
        line_marker1.color.r = 1.0;
        line_marker1.color.g = 0.0;
        line_marker1.color.b = 0.0;
        line_marker1.color.a = 1.0;

        geometry_msgs::Point p3_, p4_;
        p3_.x = center.x;
        p3_.y = center.y - lenth / 2;

        p4_.x = center.x;
        p4_.y = center.y + lenth / 2;

        line_marker1.points.push_back(p3_);
        line_marker1.points.push_back(p4_);

        marker_rectangle_pub_.publish(marker);
        marker_rectangle_pub_.publish(marker1);
        marker_rectangle_pub_.publish(line_marker);
        marker_rectangle_pub_.publish(line_marker1);
    }
    void mark_line(geometry_msgs::Point p1, geometry_msgs::Point p2)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "line";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.1; // 线宽
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        marker.points.push_back(p1);
        marker.points.push_back(p2);
        marker_line_pub_.publish(marker);
    }
};