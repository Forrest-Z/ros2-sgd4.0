
#include "gps/navilock_ublox6_gps.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace sgd_sensors
{

Navilock_UBlox6_GPS::Navilock_UBlox6_GPS()
    :Node("navilock_ublox6_gps")
{
  // Declare and read parameters from launch file
  this->declare_parameter("port", "/dev/novalue");
  this->declare_parameter("xml_file", "/home/ipp/dev_ws/src/ros2-sgd4.0/sensors/gps/data/nmea.xml");

  std::string serial_topic = this->get_parameter("port").as_string();
  serial_topic = "serial_" + serial_topic.substr(serial_topic.find_last_of("/")+1);

  std::string map_file_ = this->get_parameter("xml_file").as_string();
  nmea_parser_ = std::shared_ptr<Nmea_Parser>(new Nmea_Parser(map_file_));
  
  // Create publisher and subscriber
  RCLCPP_INFO(this->get_logger(), "Publishing GPS messages on topic /gps");
  publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("gps", default_qos);
  subscriber_ = this->create_subscription<sgd_msgs::msg::Serial>(
    serial_topic, default_qos, std::bind(&Navilock_UBlox6_GPS::read_msg, this, _1));
}

Navilock_UBlox6_GPS::~Navilock_UBlox6_GPS()
{
  RCLCPP_INFO(this->get_logger(), "Destroying");
}

void
Navilock_UBlox6_GPS::read_msg(const sgd_msgs::msg::Serial::SharedPtr msg) {
  std::string line = msg->msg;
  nmea_parser_->parse_line(line);
  
  if (nmea_parser_->msg_complete())
  {
    // Alle Daten sind da und können gepublished werden.
    sensor_msgs::msg::NavSatFix nsf;
    nsf.latitude = nmea_parser_->latitude();
    nsf.longitude = nmea_parser_->longitude();
    nsf.header.stamp.sec = nmea_parser_->time();
    
    nsf.status.status = nmea_parser_->fix() > 1 ? sensor_msgs::msg::NavSatStatus::STATUS_FIX
                : sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
    
    publisher_->publish(nsf);

    // Clear old message 
    nmea_parser_->clear();
  }
}

}   // namespace sgd_sensors

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sgd_sensors::Navilock_UBlox6_GPS>());

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "GPS node startup complete");

  rclcpp::shutdown();
  return 0;
}
