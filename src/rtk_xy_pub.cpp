#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "ublox_msgs/msg/nav_status.hpp"
#include "ublox_msgs/msg/nav_pvt.hpp"
#include "geometry_msgs/msg/point.hpp"

#include <GeographicLib/Geodesic.hpp>
#include <GeographicLib/Constants.hpp>

using namespace GeographicLib;

using std::placeholders::_1;



class RTKXYPub : public rclcpp::Node
{
  public:
    RTKXYPub() : Node("rtk_xy_publisher_node")
    {
        nav_pvt_sub_ = this->create_subscription<ublox_msgs::msg::NavPVT>("/ublox/navpvt", 20, std::bind(&RTKXYPub::nav_pvt_callback, this, _1));
        nav_status_sub_ = this->create_subscription<ublox_msgs::msg::NavSTATUS>("/ublox/navstatus", 20, std::bind(&RTKXYPub::nav_status_callback, this, _1));
        rtk_fix_pub_ = this->create_publisher<geometry_msgs::msg::Point>("/rtk_fix", 20);

        lat_org = 23.76030833; 
        long_org = 90.35303250;

        rtk_status = 0;
    }

  private:
    void nav_pvt_callback(const ublox_msgs::msg::NavPVT::SharedPtr msg)
    {
        geometry_msgs::msg::Point rtk_msg;
        double x,y;
        calcGoal(lat_org, long_org, msg->lat * 1e-7 , msg->lon * 1e-7, x, y);

        // RCLCPP_INFO(this->get_logger(), "LatOrg: %f, LonOrg: %f, LatGoal: %f, LonGoal: %f", lat_org, long_org, msg->lat * 1e-7 , msg->lon * 1e-7);
        // RCLCPP_INFO(this->get_logger(), "X: %f, Y: %f", x,y);

        rtk_msg.x = x;
        rtk_msg.y = y;
        rtk_msg.z = 0;

        rtk_fix_pub_->publish(rtk_msg);

      
    }

    void nav_status_callback(const ublox_msgs::msg::NavSTATUS::SharedPtr msg)
    {
        rtk_status = msg->fix_stat;

      
    }

    void calcGoal(double latOrg, double lonOrg, double latGoal, double lonGoal, double &x_, double &y_)
    {
        const Geodesic& geod = Geodesic::WGS84();
        double s12, azi1;
        geod.Inverse(latOrg,lonOrg,latGoal,lonGoal, s12, azi1);
        double hypotenuse = s12;
        double azimuth = azi1 * 0.0174533;
        
        x_ = -1*cos(azimuth) * hypotenuse;
        y_ = sin(azimuth) * hypotenuse;

        RCLCPP_INFO(this->get_logger(), "latOrg: %f, lonOrg: %f, latGoal: %f, lonGoal: %f", latOrg, lonOrg, latGoal, lonGoal);
        RCLCPP_INFO(this->get_logger(), "s12: %f, azi1: %f", s12, azi1);

        // RCLCPP_INFO(this->get_logger(), "X: %f, Y: %f", s12,azi1);

    }


    double lat_org;
    double long_org;

    int rtk_status;
    
    rclcpp::Subscription<ublox_msgs::msg::NavPVT>::SharedPtr nav_pvt_sub_;
    rclcpp::Subscription<ublox_msgs::msg::NavSTATUS>::SharedPtr nav_status_sub_;

    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr rtk_fix_pub_;

    
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RTKXYPub>());
  rclcpp::shutdown();
  return 0;
}