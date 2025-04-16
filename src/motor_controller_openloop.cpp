#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "robot_hw/arduino_comms.hpp"

using std::placeholders::_1;

class MotorControllerOpenloop : public rclcpp::Node {
public:
    MotorControllerOpenloop()
        : Node("motor_controller_openloop") {
        
        // Parameters
        this->declare_parameter<std::string>("serial_device", "/dev/ttyACM0");
        this->declare_parameter<int>("baud_rate", 57600);
        this->declare_parameter<int>("timeout_ms", 1000);
        this->declare_parameter<double>("wheel_separation", 0.36);
        this->declare_parameter<double>("wheel_radius", 0.065);
        
        // Get parameters
        serial_device_ = this->get_parameter("serial_device").as_string();
        baud_rate_ = this->get_parameter("baud_rate").as_int();
        timeout_ms_ = this->get_parameter("timeout_ms").as_int();
        wheel_separation_ = this->get_parameter("wheel_separation").as_double();
        wheel_radius_ = this->get_parameter("wheel_radius").as_double();


        ms_to_rpm = (1/(wheel_radius_ * 2 * 3.1415))*60;
        
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

        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&MotorControllerOpenloop::cmd_vel_callback, this, _1));

    }

    ~MotorControllerOpenloop() {
        RCLCPP_INFO(this->get_logger(), "Disconnecting from Arduino...");
        arduino_.disconnect();
        RCLCPP_INFO(this->get_logger(), "Disconnected successfully.");
    }

private:

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        float lin_vel = msg->linear.x;
        float ang_vel = msg->angular.z;

        float left_rpm = 0.0;
        float right_rpm = 0.0;

        // Calculate left and right RPMs based on the linear and angular velocities
        left_rpm = (lin_vel - (ang_vel * wheel_separation_ / 2)) * ms_to_rpm;
        right_rpm = (lin_vel + (ang_vel * wheel_separation_ / 2)) * ms_to_rpm;

        int left_pwm = rpm_to_pwm(left_rpm);
        int right_pwm = rpm_to_pwm(right_rpm);

        RCLCPP_INFO(this->get_logger(), "LEFTPWM: %d", left_pwm);
        RCLCPP_INFO(this->get_logger(), "RIGHTPWM: %d", right_pwm);


        if (!arduino_.connected()) {
            RCLCPP_ERROR(this->get_logger(), "Lost connection to Arduino!");
            return;
        }

        arduino_.set_motor_values_ol(left_pwm, right_pwm); // Call function without checking return value

      
    }

    int rpm_to_pwm(float rpm)
    {
        float abs_rpm = abs(rpm);  //Accounting for negative rpm

        int pwm = 0.0164*abs_rpm*abs_rpm + 2.2207 * abs_rpm + 9.6945;
        if(rpm < 0)
            pwm*=-1;  //If negative speed is required

        if (pwm > 255)
            pwm = 255;
        if (pwm < -255)
            pwm = -255;

        return pwm;
    }


    std::string serial_device_;
    int baud_rate_;
    int timeout_ms_;
    float wheel_radius_;
    float wheel_separation_;

    robot_hw::ArduinoComms arduino_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    float ms_to_rpm;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotorControllerOpenloop>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
