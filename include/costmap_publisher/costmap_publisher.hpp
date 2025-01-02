#ifndef SQUARE_MAP_CREATOR_HPP_
#define SQUARE_MAP_CREATOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <chrono>


using namespace std::chrono_literals;
using std::placeholders::_1;

namespace costmap_publisher
{

    class CostMapPublisher : public rclcpp::Node
    {
        public:
        explicit CostMapPublisher(const rclcpp::NodeOptions& option= rclcpp::NodeOptions());

        void timer_callback();
        void object_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg);
        void line_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg);

        private:
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr object_sub_;
        rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr line_sub_;
        rclcpp::TimerBase::SharedPtr timer_;
        std::unique_ptr<tf2_ros::Buffer> buf_;
        std::unique_ptr<tf2_ros::TransformListener> listener_;
        nav_msgs::msg::OccupancyGrid map_;
        geometry_msgs::msg::PoseArray::SharedPtr prev_object;
        visualization_msgs::msg::MarkerArray::SharedPtr prev_line;
        geometry_msgs::msg::TransformStamped t;
        double size_x, size_y;
        int data_size_;
    };

    int position2Index(const uint32_t& width, const double x, const double y);

    tf2::Vector3 robot2world(const double &x, const double &y, const geometry_msgs::msg::Quaternion& rotation);
}

#endif