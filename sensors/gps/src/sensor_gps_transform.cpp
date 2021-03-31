#include <functional>
#include <memory>
#include <cfloat>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "sgd_util/sgd_util.hpp"
using std::placeholders::_1;

using namespace std;
using namespace std::chrono_literals;

class GPS_Transform : public rclcpp::Node
{
public:
  GPS_Transform() : Node("gps_transform")
  {

    auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
    subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "gps", default_qos, std::bind(&GPS_Transform::OnMsgReceived, this, _1));

    publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>("gpspos", default_qos);    
  }

private:

  void OnMsgReceived(const sensor_msgs::msg::NavSatFix::SharedPtr msg_) const
  {    
    auto pnt = geometry_msgs::msg::Point();
    auto xy = sgd_util::WGS84_to_local(msg_->latitude, msg_->longitude);
    pnt.x = xy.first;
    pnt.y = xy.second;
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
