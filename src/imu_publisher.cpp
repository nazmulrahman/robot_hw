#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "robot_hw/arduino_comms.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <tf2/LinearMath/Quaternion.h>

class ImuPublisher : public rclcpp::Node {
public:
    ImuPublisher()
        : Node("imu_publisher_node") {
        
        // Parameters
        this->declare_parameter<std::string>("serial_device", "/dev/ttyACM0");
        this->declare_parameter<int>("baud_rate", 115200);
        this->declare_parameter<int>("timeout_ms", 1000);
        this->declare_parameter<std::string>("imu_topic", "imu/data_raw");
        this->declare_parameter<std::string>("frame_id", "imu");
        this->declare_parameter<bool>("publish_tf", true);
        
        // Get parameters
        serial_device_ = this->get_parameter("serial_device").as_string();
        baud_rate_ = this->get_parameter("baud_rate").as_int();
        timeout_ms_ = this->get_parameter("timeout_ms").as_int();
        imu_topic_ = this->get_parameter("imu_topic").as_string();
        frame_id_ = this->get_parameter("frame_id").as_string();
        publish_tf_ = this->get_parameter("publish_tf").as_bool();
        
        // Initialize Arduino communication
        RCLCPP_INFO(this->get_logger(), "Connecting to Arduino...");
        arduino_.connect(serial_device_, baud_rate_, timeout_ms_);
        
        if (!arduino_.connected()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to connect to Arduino!");
            rclcpp::shutdown();
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Connected successfully!");

        arduino_.send_empty_msg();

        // Publisher
        imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);
        
        // Timer for periodic publishing
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33),  // Adjust as needed
            std::bind(&ImuPublisher::publish_imu, this)
        );

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    ~ImuPublisher() {
        RCLCPP_INFO(this->get_logger(), "Disconnecting from Arduino...");
        arduino_.disconnect();
        RCLCPP_INFO(this->get_logger(), "Disconnected successfully.");
    }

private:
    void publish_imu() {
        if (!arduino_.connected()) {
            RCLCPP_ERROR(this->get_logger(), "Lost connection to Arduino!");
            return;
        }

        float quat_w, quat_x, quat_y, quat_z, ang_x, ang_y ,ang_z, lin_x, lin_y, lin_z;

        arduino_.read_imu_values(quat_w, quat_x, quat_y, quat_z, ang_x, ang_y ,ang_z, lin_x, lin_y, lin_z); // Call function without checking return value

        current_time = this->now();
        auto msg = sensor_msgs::msg::Imu();
        
        msg.header.stamp=current_time;
        msg.header.frame_id= frame_id_;
        msg.orientation.w = quat_w;
        msg.orientation.x = quat_x;
        msg.orientation.y = quat_y;
        msg.orientation.z = quat_z;
        msg.angular_velocity.x = ang_x;
        msg.angular_velocity.y = ang_y;
        msg.angular_velocity.z = ang_z;
        msg.linear_acceleration.x = lin_x;
        msg.linear_acceleration.y = lin_y;
        msg.linear_acceleration.z = lin_z;

        imu_publisher_->publish(msg);

        if(publish_tf_)
        {
          geometry_msgs::msg::TransformStamped imu_tf;
          imu_tf.header.stamp = current_time;
          imu_tf.header.frame_id = "base_link";
          imu_tf.child_frame_id = frame_id_;

          imu_tf.transform.translation.z = 0.0;
          imu_tf.transform.rotation = msg.orientation;
          tf_broadcaster_->sendTransform(imu_tf); 
        }
    }

    std::string serial_device_ , imu_topic_ , frame_id_;
    int baud_rate_;
    int timeout_ms_;
    bool publish_tf_;

    robot_hw::ArduinoComms arduino_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Time current_time;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImuPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
