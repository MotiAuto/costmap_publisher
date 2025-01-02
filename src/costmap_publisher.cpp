#include "costmap_publisher/costmap_publisher.hpp"

namespace costmap_publisher
{
    CostMapPublisher::CostMapPublisher(const rclcpp::NodeOptions& option):Node("CostMapPublisher", option)
    {
        pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", rclcpp::SystemDefaultsQoS());

        timer_ = this->create_wall_timer(1ms, std::bind(&CostMapPublisher::timer_callback, this));

        buf_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        listener_ = std::make_unique<tf2_ros::TransformListener>(*buf_);

        object_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/object",
            rclcpp::SystemDefaultsQoS(),
            std::bind(&CostMapPublisher::object_callback, this, _1)
        );

        line_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
            "/line",
            rclcpp::SystemDefaultsQoS(),
            std::bind(&CostMapPublisher::line_callback, this, _1)
        );

        this->declare_parameter("max_range_x", 5.0);
        this->get_parameter("max_range_x", size_x);
        this->declare_parameter("max_range_y", 5.0);
        this->get_parameter("max_range_y", size_y);

        map_.header.frame_id = "map";
        map_.info.resolution = 0.1;
        map_.info.width = size_x / map_.info.resolution;
        map_.info.height = size_y / map_.info.resolution;
        map_.data.resize(map_.info.width * map_.info.height);
        data_size_ = map_.info.width * map_.info.height;

        for(int y = 0; y < map_.info.height; y++)
        {
            for(int x = 0; x < map_.info.width; x++)
            {
                const auto idx = position2Index(map_.info.width, x, y);

                map_.data[idx] = 0;
            }
        }

        prev_object = nullptr;
        prev_line = nullptr;

        RCLCPP_INFO(this->get_logger(), "Start CostMapPublisher");
    }

    void CostMapPublisher::timer_callback()
    {
        try
        {
            t = buf_->lookupTransform("map", "base_link", tf2::TimePointZero);
            map_.info.origin.position.x = t.transform.translation.x - size_x / 2.0;
            map_.info.origin.position.y = t.transform.translation.y - size_y / 2.0;

            pub_->publish(map_);
        }
        catch(const tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not listen transform %s", ex.what());
        }
        
    }

    void CostMapPublisher::object_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        if(prev_object != nullptr)
        {
            for(const auto& obj : prev_object->poses)
            {
                const auto fixed_pose = robot2world(obj.position.x, obj.position.y, t.transform.rotation);
                const auto x = (fixed_pose.x() + size_x / 2.0) / map_.info.resolution;
                const auto y = (fixed_pose.y() + size_y / 2.0) / map_.info.resolution;

                for(int y_ = y-5; y_ < y+5; y_++)
                {
                    for(int x_ = x-5; x_ < x+5; x_++)
                    {
                        const auto idx = position2Index(map_.info.width, x_, y_);

                        if(idx < data_size_)
                        {
                            map_.data[idx] = 0;
                        }
                    }
                }
            }
        }

        for(const auto& obj : msg->poses)
        {
            const auto fixed_pose = robot2world(obj.position.x, obj.position.y, t.transform.rotation);
            const auto x = (fixed_pose.x() + size_x / 2.0) / map_.info.resolution;
            const auto y = (fixed_pose.y() + size_y / 2.0) / map_.info.resolution;

            for(int y_ = y-1; y_ < y+1; y_++)
            {
                for(int x_ = x-1; x_ < x+1; x_++)
                {
                    const auto idx = position2Index(map_.info.width, x_, y_);

                    if(idx < data_size_)
                    {
                        map_.data[idx] = 100;
                    }
                }
            }
        }

        prev_object = msg;
    }

    void CostMapPublisher::line_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
    {
        // if(prev_line != nullptr)
        // {

        // }

        // for(const auto& line : msg->markers)
        // {
        //     const auto start_p = line.points[0];
        //     const auto end_p = line.points[1];


        // }
    }

    int position2Index(const uint32_t& width, const double x, const double y)
    {
        const auto y_ = static_cast<int>(y);
        const auto x_ = static_cast<int>(x);
        return width * y_ + x_;
    }

    tf2::Vector3 robot2world(const double &x, const double &y, const geometry_msgs::msg::Quaternion& rotation)
    {
        const tf2::Vector3 v(x, y, 0.0);
        const tf2::Quaternion q(rotation.x, rotation.y, rotation.z, rotation.w);

        return tf2::quatRotate(q, v);
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(costmap_publisher::CostMapPublisher)