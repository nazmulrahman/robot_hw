#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

using std::placeholders::_1;

class RTKHeadingPublisher : public rclcpp::Node
{
  public:
    RTKHeadingPublisher() : Node("combined_odom_publisher_node")
    {
        rtk_fix_subscription = this->create_subscription<geometry_msgs::msg::Point>("/rtk_fix", 10, std::bind(&RTKHeadingPublisher::rtk_fix_callback, this, _1));
        odom_subscription = this->create_subscription<nav_msgs::msg::Odometry>("/odom_encoder", 10, std::bind(&RTKHeadingPublisher::odom_callback, this, _1));
        heading_publisher = this->create_publisher<std_msgs::msg::Float64>("/rtk_heading", 10);
    }

  private:
    void rtk_fix_callback(const geometry_msgs::msg::Point::SharedPtr msg)
    {
        double x = msg->x;
        double y = msg->y;

        // Only calculate heading if robot is moving straight and fast enough
        if (lin_vel > linear_vel_threshold && std::abs(ang_vel) < angular_vel_threshold)
        {
            if (has_prev_position)
            {
                double dx = x - prev_x;
                double dy = y - prev_y;

                if (std::hypot(dx, dy) > distance_threshold)
                {
                    double heading_rad = std::atan2(dy, dx);
                    double heading_deg = heading_rad * 180.0 / M_PI;

                    std_msgs::msg::Float64 heading_msg;
                    heading_msg.data = heading_deg;
                    heading_publisher->publish(heading_msg);

                    RCLCPP_INFO(this->get_logger(), "Published heading: %.2f degrees", heading_deg);
                }
            }

            prev_x = x;
            prev_y = y;
            has_prev_position = true;
        }

    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        lin_vel = msg->twist.twist.linear.x;
        ang_vel = msg->twist.twist.angular.z;
    }
    

    // Variables to store velocity
    double lin_vel = 0.0, ang_vel = 0.0;

    // Previous RTK position
    double prev_x = 0.0, prev_y = 0.0;
    bool has_prev_position = false;

    // Thresholds
    const double linear_vel_threshold = 0.2;  // m/s, adjust based on your robot
    const double angular_vel_threshold = 0.1; // rad/s, near zero means straight
    const double distance_threshold = 0.05;   // meters, to avoid noise


    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr rtk_fix_subscription;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr heading_publisher;
    
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RTKHeadingPublisher>());
  rclcpp::shutdown();
  return 0;
}