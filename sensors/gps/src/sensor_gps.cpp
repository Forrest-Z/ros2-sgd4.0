#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
using std::placeholders::_1;

using namespace std::chrono_literals;

class GPS_to_Point : public rclcpp::Node
{
public:
  GPS_to_Point()
  : Node("gps_to_point")
  {
    auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
    subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "gps", default_qos, std::bind(&GPS_to_Point::topic_callback, this, _1));

    publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>("gpspos", default_qos);    
  }

private:
  void topic_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg_) const
  {    
    auto pnt = geometry_msgs::msg::Point();
    pnt.x = msg_->latitude;
    pnt.y = msg_->longitude;
    pnt.z = msg_->altitude;

    auto pntst = geometry_msgs::msg::PointStamped();
    pntst.point = pnt;
    pntst.header = msg_->header;

    publisher_->publish(pntst);
  }
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr publisher_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GPS_to_Point>());
  rclcpp::shutdown();
  return 0;
}
