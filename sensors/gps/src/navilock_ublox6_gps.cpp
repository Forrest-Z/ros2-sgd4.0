


#include "gps/navilock_ublox6_gps.hpp"

#include <functional>
#include <memory>
#include <stdio.h>
#include <string>
#include <iostream>
#include <fcntl.h>
#include <cerrno>
#include <cstring>
#include <termios.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
//#include "sensor_msgs/msg/nav_sat_fix.hpp"
//#include "geometry_msgs/msg/point_stamped.hpp"
//#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

//namespace sensor_gps
//{

Navilock_UBlox6_GPS::Navilock_UBlox6_GPS():
        Node("navilock_ublox6_gps"),baud_rate_(9600),port_("/dev/ttyACM0")
{
    init();
    
    RCLCPP_DEBUG(this->get_logger(), "Publishing GPS messages on topic /gpspos");
    auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
  
    publisher_ = this->create_publisher<std_msgs::msg::String>("gpspos", default_qos);
    RCLCPP_DEBUG(this->get_logger(), "creating timer");
    timer_ = this->create_wall_timer(1000ms, std::bind(&Navilock_UBlox6_GPS::readLine, this));
}

Navilock_UBlox6_GPS::~Navilock_UBlox6_GPS()
{
  RCLCPP_INFO(this->get_logger(), "Destroying");
  close(serial_port_);
}

void
Navilock_UBlox6_GPS::readLine() {
  char read_buf [256];
  std::memset(&read_buf, '\0', sizeof(read_buf));
  int num_bytes = read(serial_port_, &read_buf, sizeof(read_buf));

  if (num_bytes < 0) {
    RCLCPP_ERROR(this->get_logger(), "Error reading %s", strerror(errno));
    return;
  }

  int retries = 0;
  while(num_bytes < 1 && retries < 5) {
    num_bytes = read(serial_port_, &read_buf, sizeof(read_buf));

    retries++;
  }

  if (retries >= 5)
  {
    RCLCPP_ERROR(this->get_logger(), "Could not receive valid input data.");
    return;
  }
  
  RCLCPP_DEBUG(this->get_logger(), "Read %i bytes. Received message: %s", num_bytes, read_buf);

  auto msg_ = std_msgs::msg::String();
  std::string s(read_buf);
  msg_.data = s;
  publisher_->publish(msg_);
}

int
Navilock_UBlox6_GPS::init() {
  serial_port_ = open(port_, O_RDWR);
    isPortOpen_ = serial_port_ > 0 ? false : true;

    if (tcgetattr(serial_port_, &tty_) != 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Error %i from togetattr %s", errno, strerror(errno));
        isPortOpen_ = false;
        return 1;       // TODO: Error handling
    }

    tty_.c_cflag &= ~PARENB;     // clear parity bit
    tty_.c_cflag &= ~CSTOPB;
    tty_.c_cflag &= ~CSIZE;
    //tty_.c_cflag != CS8;    // Compiler warning: statement has no effect
    tty_.c_cflag &= ~CRTSCTS;
    //tty_.c_cflag != (CREAD | CLOCAL);   // Compiler warning: statement has no effect

    tty_.c_lflag &= ~ICANON;
    tty_.c_lflag &= ~ECHO;
    tty_.c_lflag &= ~ECHOE;
    tty_.c_lflag &= ~ECHONL;
    tty_.c_lflag &= ~ISIG;

    tty_.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty_.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);

    tty_.c_oflag &= ~OPOST;
    tty_.c_oflag &= ~ONLCR;

    tty_.c_cc[VTIME] = 10;
    tty_.c_cc[VMIN] = 0;

    cfsetispeed(&tty_, B9600);      // TODO: use user defined baud-rate
    cfsetospeed(&tty_, B9600);

    return 0;
}

//}   // namespace sensor_gps

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Navilock_UBlox6_GPS>());
  rclcpp::shutdown();
  return 0;
}
