
#include "gps/navilock_ublox6_gps.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace sgd_sensors
{

Navilock_UBlox6_GPS::Navilock_UBlox6_GPS()
    :Node("navilock_ublox6_gps")
{

  std::string map_file_ = "/home/pascal/dev_ws/src/ros2-sgd4.0/sensors/gps/data/nmea.xml";
  nmea_parser_ = std::shared_ptr<Nmea_Parser>(new Nmea_Parser(map_file_));
  
  RCLCPP_INFO(this->get_logger(), "Publishing GPS messages on topic /gps");
  publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("gps", default_qos);
  timer_ = this->create_wall_timer(100ms, std::bind(&Navilock_UBlox6_GPS::read_msg, this));
}

Navilock_UBlox6_GPS::~Navilock_UBlox6_GPS()
{
  RCLCPP_DEBUG(this->get_logger(), "Destroying");
}

void
Navilock_UBlox6_GPS::read_msg() {
  std::string path = fs::temp_directory_path().string() + "/serial";
  long last_time = 0;
  try
  {
    for (const auto & entry : fs::directory_iterator(path))
    {
      if (entry.path() == last_file_ || entry.path().extension() == ".lck")
      {
        continue;
      }

      std::string fname = entry.path().filename();
      std::string time = fname.substr(0,fname.find("."));

      try
      {
          long ft = std::stol(time);
          if (ft > last_time)
          {
              last_time = ft;
              last_file_ = entry.path();
          }
      }
      catch(const std::invalid_argument& e)
      {
          RCLCPP_WARN(get_logger(), "Could not parse string %s",time);
      }
    }
  } catch (const fs::filesystem_error& e)
  {
    RCLCPP_WARN(get_logger(), "Could not read any gps data.");
    return;
  }
  
  if (last_time <= 0)
  {
    return;
  }

  nmea_parser_->clear();
  std::string line;
  std::ifstream myfile(last_file_);

  if (myfile.is_open())
  {
    while (getline(myfile, line))
    {
      nmea_parser_->parse_line(line);
    }
    myfile.close();
  }
  // Alle Daten sind da und können gepublished werden.
  sensor_msgs::msg::NavSatFix nsf;
  nsf.latitude = nmea_parser_->latitude();
  nsf.longitude = nmea_parser_->longitude();
  nsf.header.stamp.sec = nmea_parser_->time();
  
  nsf.status.status = nmea_parser_->fix() > 1 ? sensor_msgs::msg::NavSatStatus::STATUS_FIX
              : sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
  
  publisher_->publish(nsf);
}

}   // namespace sgd_sensors

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sgd_sensors::Navilock_UBlox6_GPS>());
  rclcpp::shutdown();
  return 0;
}
