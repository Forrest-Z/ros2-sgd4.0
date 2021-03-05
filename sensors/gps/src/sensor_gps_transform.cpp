#include <functional>
#include <memory>
#include <cfloat>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
using std::placeholders::_1;

#define DEFAULT_orig_lat 10.021944
#define DEFAULT_orig_lon 53.555833
#define DEFAULT_coeff_a -110623.2362476
#define DEFAULT_coeff_b -109632.4399804

using namespace std;
using namespace std::chrono_literals;

class GPS_Transform : public rclcpp::Node
{
public:
  GPS_Transform() : Node("gps_transform")
  {
    this->declare_parameter("orig_lat", DEFAULT_orig_lat);
    this->declare_parameter("orig_lon", DEFAULT_orig_lon);
    this->declare_parameter("coeff_a", DEFAULT_coeff_a);
    this->declare_parameter("coeff_b", DEFAULT_coeff_b);

    this->get_parameter("orig_lat", orig_lat);
    this->get_parameter("orig_lon", orig_lon);
    this->get_parameter("coeff_a", coeff_a);
    this->get_parameter("coeff_b", coeff_b);

    RCLCPP_INFO(get_logger(), "Declared parameter %f, %f, %f, %f", orig_lat, orig_lon, coeff_a, coeff_b);

    auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
    subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "gps", default_qos, std::bind(&GPS_Transform::OnMsgReceived, this, _1));

    publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>("gpspos", default_qos);    
  }

private:
  double orig_lat;
  double orig_lon;
  double coeff_a;
  double coeff_b;

  void OnMsgReceived(const sensor_msgs::msg::NavSatFix::SharedPtr msg_) const
  {    
    auto pnt = geometry_msgs::msg::Point();
    pnt.y = (msg_->latitude - orig_lat) * coeff_a;
    pnt.x = (msg_->longitude - orig_lon) * coeff_b;
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
  rclcpp::spin(std::make_shared<GPS_Transform>());
  rclcpp::shutdown();
  return 0;
}
