#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <tf2/LinearMath/Quaternion.h>

using std::placeholders::_1;

class OdometryPublisher : public rclcpp::Node
{
  public:
    OdometryPublisher() : Node("odom_publisher_node")
    {
        // Declare parameters with default values
        this->declare_parameter<double>("wheel_radius_multiplier", 1.0); // ticks
        this->declare_parameter<double>("wheel_separation_multiplier", 1.0); // ticks
        this->declare_parameter<double>("wheel_radius", 0.065);  // meters
        this->declare_parameter<double>("wheel_separation", 0.36); // meters
        this->declare_parameter<int>("ticks_per_revolution", 160); // ticks

        this->declare_parameter<std::string>("odom_topic", "odom");
        this->declare_parameter<std::string>("frame_id", "odom");
        this->declare_parameter<std::string>("child_frame_id", "base_footprint");
        this->declare_parameter<bool>("publish_tf", true);
        this->declare_parameter<bool>("print_odom", false);
        

        // Retrieve parameters
        wheel_radius_multiplier_ = this->get_parameter("wheel_radius_multiplier").as_double();
        wheel_separation_multiplier_ = this->get_parameter("wheel_separation_multiplier").as_double();
        wheel_radius_ = (this->get_parameter("wheel_radius").as_double()) * wheel_radius_multiplier_;
        wheel_separation_ = (this->get_parameter("wheel_separation").as_double()) * wheel_separation_multiplier_;
        ticks_per_revolution_ = this->get_parameter("ticks_per_revolution").as_int();

        odom_topic_ = this->get_parameter("odom_topic").as_string();
        frame_id_ = this->get_parameter("frame_id").as_string();
        child_frame_id_ = this->get_parameter("child_frame_id").as_string();
        publish_tf_ = this->get_parameter("publish_tf").as_bool();
        print_odom_ = this->get_parameter("print_odom").as_bool();

        //Print Parameters
        RCLCPP_INFO(this->get_logger(), "wheel_radius_multiplier: %f", wheel_radius_multiplier_);
        RCLCPP_INFO(this->get_logger(), "wheel_separation_multiplier: %f", wheel_separation_multiplier_);
        RCLCPP_INFO(this->get_logger(), "wheel_radius: %f -> %f", this->get_parameter("wheel_radius").as_double(), wheel_radius_);
        RCLCPP_INFO(this->get_logger(), "wheel_separation: %f -> %f", this->get_parameter("wheel_separation").as_double(), wheel_separation_);


        ticks_per_meter = ticks_per_revolution_ / ((wheel_radius_ * 2 * 3.1415));

        ticks_subscription = this->create_subscription<geometry_msgs::msg::Vector3>("ticks", 10, std::bind(&OdometryPublisher::ticks_callback, this, _1));
        odom_publisher = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        prev_time = this->now();

    }

  private:
    void ticks_callback(const geometry_msgs::msg::Vector3::SharedPtr msg)
    {
        current_time = this->now();
        time_diff = (current_time - prev_time).seconds();
        if (time_diff >= 0.05 && init)
        {
            prev_time = current_time;

            double d_left_ticks = msg->x - prev_left_ticks;
            double d_right_ticks = msg->y - prev_right_ticks;

            double left_dist_delta = d_left_ticks/ticks_per_meter;
            double right_dist_delta = d_right_ticks/ticks_per_meter;

            double d_dist = (left_dist_delta + right_dist_delta)/2;
            double d_theta = (right_dist_delta - left_dist_delta)/wheel_separation_;

            theta+=d_theta;
            theta = atan2(sin(theta), cos(theta));
            x_pos += d_dist * cos(theta);
            y_pos += d_dist * sin(theta);
            
            double v_x = d_dist / time_diff;
            double v_theta = d_theta / time_diff;

            ConstructAndPublish(x_pos, y_pos, theta, v_x, v_theta);

        }
        prev_left_ticks = msg->x;
        prev_right_ticks = msg->y;
        init=true;

      
    }
    void ConstructAndPublish(double xpos, double ypos, double th, double lin_vel, double ang_vel)
    {
        geometry_msgs::msg::Quaternion odom_quat = yawToQuaternion(th);

        odom_msg.header.stamp=current_time;
        odom_msg.header.frame_id= frame_id_;
        odom_msg.child_frame_id= child_frame_id_;

        odom_msg.pose.pose.position.x=xpos;
        odom_msg.pose.pose.position.y=ypos;
        odom_msg.pose.pose.orientation=odom_quat;
        
        odom_msg.twist.twist.linear.x=lin_vel;
        odom_msg.twist.twist.angular.z=ang_vel;
        odom_publisher->publish(odom_msg);
        

        if(publish_tf_)
        {
          geometry_msgs::msg::TransformStamped odom_tf;
          odom_tf.header.stamp = current_time;
          odom_tf.header.frame_id = frame_id_;
          odom_tf.child_frame_id = child_frame_id_;

          odom_tf.transform.translation.x = xpos;
          odom_tf.transform.translation.y = ypos;
          odom_tf.transform.translation.z = 0.0;
          odom_tf.transform.rotation = odom_quat;
          tf_broadcaster_->sendTransform(odom_tf); 
        }

        if(print_odom_)
        {
          RCLCPP_INFO(this->get_logger(), "pose.x: : %f", xpos);
          RCLCPP_INFO(this->get_logger(), "pose.y: %f", ypos);
          RCLCPP_INFO(this->get_logger(), "linear.x: %f", lin_vel);
          RCLCPP_INFO(this->get_logger(), "angular.z: %f", ang_vel);
          RCLCPP_INFO(this->get_logger(), "----------------------------------------------------");
        }

        
    }


    geometry_msgs::msg::Quaternion yawToQuaternion(double yaw)
    {
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw); // Roll = 0, Pitch = 0, Yaw = theta

        geometry_msgs::msg::Quaternion quat_msg;
        quat_msg.x = q.x();
        quat_msg.y = q.y();
        quat_msg.z = q.z();
        quat_msg.w = q.w();

        return quat_msg;
    }

    // parameters
    double wheel_radius_ , wheel_radius_multiplier_ , wheel_separation_ , wheel_separation_multiplier_ , ticks_per_revolution_;
    std::string odom_topic_ , frame_id_ , child_frame_id_ ;
    bool print_odom_ , publish_tf_ ;

    bool init=false;
    double ticks_per_meter;
    double time_diff = 0.0;
    double x_pos = 0.0, y_pos = 0.0, theta = 0.0;
    double prev_left_ticks = 0.0, prev_right_ticks = 0.0;
    
    rclcpp::Time current_time, prev_time;
    nav_msgs::msg::Odometry odom_msg;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr ticks_subscription;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdometryPublisher>());
  rclcpp::shutdown();
  return 0;
}