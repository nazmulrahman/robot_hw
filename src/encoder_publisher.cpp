#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "robot_hw/arduino_comms.hpp"

class EncoderPublisher : public rclcpp::Node {
public:
    EncoderPublisher()
        : Node("encoder_publisher") {
        
        // Parameters
        this->declare_parameter<std::string>("serial_device", "/dev/ttyACM1");
        this->declare_parameter<int>("baud_rate", 57600);
        this->declare_parameter<int>("timeout_ms", 1000);
        
        // Get parameters
        serial_device_ = this->get_parameter("serial_device").as_string();
        baud_rate_ = this->get_parameter("baud_rate").as_int();
        timeout_ms_ = this->get_parameter("timeout_ms").as_int();
        
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
        ticks_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3>("ticks", 10);
        
        // Timer for periodic publishing
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33),  // Adjust as needed
            std::bind(&EncoderPublisher::publish_ticks, this)
        );
    }

    ~EncoderPublisher() {
        RCLCPP_INFO(this->get_logger(), "Disconnecting from Arduino...");
        arduino_.disconnect();
        RCLCPP_INFO(this->get_logger(), "Disconnected successfully.");
    }

private:
    void publish_ticks() {
        if (!arduino_.connected()) {
            RCLCPP_ERROR(this->get_logger(), "Lost connection to Arduino!");
            return;
        }

        int enc_1 = 0, enc_2 = 0;
        arduino_.read_encoder_values(enc_1, enc_2); // Call function without checking return value


        auto msg = geometry_msgs::msg::Vector3();
        msg.x = static_cast<double>(enc_1); // Left encoder ticks
        msg.y = static_cast<double>(enc_2); // Right encoder ticks
        msg.z = 0.0; // Not used

        ticks_publisher_->publish(msg);
    }

    std::string serial_device_;
    int baud_rate_;
    int timeout_ms_;

    robot_hw::ArduinoComms arduino_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr ticks_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EncoderPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
