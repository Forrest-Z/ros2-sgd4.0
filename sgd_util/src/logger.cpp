/*
 *  Some text
 */

# include "sgd_util/logger.hpp"

namespace sgd_util
{

Logger::Logger() : rclcpp::Node("logger")
{
    RCLCPP_DEBUG(get_logger(), "Creating");
    // Add parameters

    imu_topic_ = declare_parameter<std::string>("imu_topic", "scan");
    gps_topic_ = declare_parameter<std::string>("gps_topic", "gps");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "odom");
    output_folder_ = declare_parameter<std::string>("output_folder", "log");

    time_at_start_ = round(now().nanoseconds() / 1.0E6);
    std::string time = std::to_string(time_at_start_); // time in millis

    scan_filename_ = output_folder_ + "/scan_" + time + ".log";
    //out_imu_.open(scan_filename_, std::ios::out | std::ios::trunc);
    //out_gps_.open(output_folder_ + "/gps_" + time + ".log", std::ios::out | std::ios::trunc);
    //out_odom_.open(output_folder_ + "/odom_" + time + ".log", std::ios::out | std::ios::trunc);

    // Initialize parameters, pub/sub, services, etc.
    init_pub_sub();
}

Logger::~Logger()
{
    // Destroy
}

void
Logger::init_pub_sub()
{
    sub_gps_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(gps_topic_, default_qos,
        std::bind(&Logger::gps_callback, this, std::placeholders::_1));

    // Publish odometry
    sub_imu_ = this->create_subscription<sensor_msgs::msg::LaserScan>(imu_topic_, default_qos,
        std::bind(&Logger::imu_callback, this, std::placeholders::_1));

    // Receive velocity command from local controller
    //sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(odom_topic_, default_qos,
    //    std::bind(&Logger::odom_callback, this, std::placeholders::_1));
}

void
Logger::imu_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg_)
{
    // linear acceleration, angular velocity, orientation
    out_imu_.open(scan_filename_, std::ios::out | std::ios::app);
    out_imu_ << time_to_string();

    for (float dist : msg_->ranges)
    {
        out_imu_ << "," << std::to_string(dist);
    }
    out_imu_ << "\n";
    out_imu_.close();
}

void
Logger::gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg_)
{
    // lat, lon
    out_gps_ << time_to_string();
    out_gps_ << std::to_string(msg_->latitude) << ",";
    out_gps_ << std::to_string(msg_->longitude) << "\n";
}

void
Logger::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg_)
{
    // lin velocity x and ang velocity z
    //out_odom_ << time_to_string();
    //out_odom_ << std::to_string(msg_->twist.twist.linear.x) << ",";
    //out_odom_ << std::to_string(msg_->twist.twist.angular.z) << "\n";
}

std::string
Logger::time_to_string()
{
  long t = round(now().nanoseconds() / 1.0E6) - time_at_start_;
  std::string ts = std::to_string(t);
  ts.append(",");
  return ts;
}

std::string
Logger::vec3_to_string(geometry_msgs::msg::Vector3 vec3)
{
  std::string s;
  s = std::to_string(vec3.x) + "," + std::to_string(vec3.y) + "," + std::to_string(vec3.z);
  return s;
}

}

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<sgd_util::Logger>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();

    return 0;
}
