#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "robot_hw/arduino_comms.hpp"

using std::placeholders::_1;


class EMAFilter {
public:
    EMAFilter(double alpha = 0.2) : alpha_(alpha), initialized_(false), prev_(0.0) {}

    double filter(double value) {
        if (!initialized_) {
            prev_ = value;
            initialized_ = true;
        } else {
            prev_ = alpha_ * value + (1.0 - alpha_) * prev_;
        }
        return prev_;
    }

private:
    double alpha_;
    double prev_;
    bool initialized_;
};


class MotorControllerPID : public rclcpp::Node {
public:
    MotorControllerPID()
        : Node("motor_controller_pid"), prev_time_(this->now()) {
        
        // Parameters
        this->declare_parameter<std::string>("serial_device", "/dev/ttyACM0");
        this->declare_parameter<int>("baud_rate", 57600);
        this->declare_parameter<int>("timeout_ms", 1000);
        this->declare_parameter<double>("wheel_separation", 0.36);
        this->declare_parameter<double>("wheel_radius", 0.065);
        this->declare_parameter<double>("Kp", 0.8);
        this->declare_parameter<double>("Ki", 2.0);
        this->declare_parameter<double>("Kd", 0.05);
        this->declare_parameter<int>("pid_control_interval", 100);
        this->declare_parameter<int>("ticks_per_revolution", 240);
        
        // Get parameters
        serial_device_ = this->get_parameter("serial_device").as_string();
        baud_rate_ = this->get_parameter("baud_rate").as_int();
        timeout_ms_ = this->get_parameter("timeout_ms").as_int();
        wheel_separation_ = this->get_parameter("wheel_separation").as_double();
        wheel_radius_ = this->get_parameter("wheel_radius").as_double();
        Kp_ = this->get_parameter("Kp").as_double();
        Ki_ = this->get_parameter("Ki").as_double();
        Kd_ = this->get_parameter("Kd").as_double();
        pid_control_interval_ = this->get_parameter("pid_control_interval").as_int();
        ticks_per_revolution_ = this->get_parameter("ticks_per_revolution").as_int();
        
        ms_to_rpm_ = (1/(wheel_radius_ * 2 * 3.1415)) * 60;
        
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

        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&MotorControllerPID::cmd_vel_callback, this, _1));
        ticks_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            "ticks", 10, std::bind(&MotorControllerPID::ticks_callback, this, _1));
    }

    ~MotorControllerPID() {
        RCLCPP_INFO(this->get_logger(), "Disconnecting from Arduino...");
        arduino_.disconnect();
        RCLCPP_INFO(this->get_logger(), "Disconnected successfully.");
    }

private:
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        target_left_rpm_ = (msg->linear.x - (msg->angular.z * wheel_separation_ / 2)) * ms_to_rpm_;
        target_right_rpm_ = (msg->linear.x + (msg->angular.z * wheel_separation_ / 2)) * ms_to_rpm_;
    }

    void ticks_callback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
        rclcpp::Time now = this->now();
        double dt = (now - prev_time_).seconds();
        prev_time_ = now;
        
        if (dt == 0) return;

        if (dt <= 0.1)
        {
            double d_left_ticks = msg->x - prev_left_ticks;
            double d_right_ticks = msg->y - prev_right_ticks;

            prev_left_ticks = msg->x;
            prev_right_ticks = msg->y;

            // Convert ticks to RPM
            double actual_left_rpm = (d_left_ticks / dt) * 60.0 / ticks_per_revolution_;
            double actual_right_rpm = (d_right_ticks / dt) * 60.0 / ticks_per_revolution_;

            double filtered_left_rpm = left_rpm_filter_.filter(actual_left_rpm);
            double filtered_right_rpm = right_rpm_filter_.filter(actual_right_rpm);

            // Compute PID control
            int left_pwm = compute_pid(target_left_rpm_, filtered_left_rpm, left_error_, left_integral_, left_prev_error_, dt);
            int right_pwm = compute_pid(target_right_rpm_, filtered_right_rpm, right_error_, right_integral_, right_prev_error_, dt);

            RCLCPP_INFO(this->get_logger(), "Target Right RPM: %f", target_right_rpm_);
            RCLCPP_INFO(this->get_logger(), "Actual Right RPM: %f", filtered_right_rpm);
            RCLCPP_INFO(this->get_logger(), "Right PWM: %d", right_pwm);
            RCLCPP_INFO(this->get_logger(), "--------------------------------");

            RCLCPP_INFO(this->get_logger(), "Target Left RPM: %f", target_left_rpm_);
            RCLCPP_INFO(this->get_logger(), "Actual Left RPM: %f", filtered_left_rpm);
            RCLCPP_INFO(this->get_logger(), "Left PWM: %d", left_pwm);
            RCLCPP_INFO(this->get_logger(), "--------------------------------");
            
            // Send commands
            if (!arduino_.connected()) {
                RCLCPP_ERROR(this->get_logger(), "Lost connection to Arduino!");
                return;
            }
            arduino_.set_motor_values_ol(left_pwm, right_pwm);
        }

        
    }

    int compute_pid(double target_rpm, double actual_rpm, double &error, double &integral, double &prev_error, double dt) {
        error = target_rpm - actual_rpm;
        integral += error * dt;
        double derivative = (error - prev_error) / dt;
        prev_error = error;

        int pwm = Kp_ * error + Ki_ * integral + Kd_ * derivative;

        if (target_rpm>0 && pwm<0)
            pwm =0;
        if (target_rpm<0 && pwm>0)
            pwm=0;
        if (target_rpm == 0)
            pwm =0;

        if (pwm > 255)
            pwm = 255;
        if (pwm < -255)
            pwm = -255;
        
        return pwm;
    }

    std::string serial_device_;
    int baud_rate_, timeout_ms_ , ticks_per_revolution_;
    double wheel_radius_, wheel_separation_;
    double Kp_, Ki_, Kd_;
    int pid_control_interval_;
    double ms_to_rpm_;
    double prev_rpm_ = 0.0;
    
    double target_left_rpm_ = 0.0, target_right_rpm_ = 0.0;
    double left_error_ = 0.0, right_error_ = 0.0;
    double left_integral_ = 0.0, right_integral_ = 0.0;
    double left_prev_error_ = 0.0, right_prev_error_ = 0.0;

    double prev_left_ticks = 0.0, prev_right_ticks = 0.0;
    double rpm_;
    
    rclcpp::Time prev_time_;
    robot_hw::ArduinoComms arduino_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr ticks_sub_;

    EMAFilter left_rpm_filter_;
    EMAFilter right_rpm_filter_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotorControllerPID>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}