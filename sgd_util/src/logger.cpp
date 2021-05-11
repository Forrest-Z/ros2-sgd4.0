/*
 *  Some text
 */

# include "sgd_util/logger.hpp"

namespace sgd_util
{

Logger::Logger() : nav2_util::LifecycleNode("logger", "", true)
{
    RCLCPP_DEBUG(get_logger(), "Creating");
    // Add parameters
    add_parameter("imu_topic", rclcpp::ParameterValue("imu"));
    add_parameter("gps_topic", rclcpp::ParameterValue("gps"));
    add_parameter("odom_topic", rclcpp::ParameterValue("odom"));
    add_parameter("output_folder", rclcpp::ParameterValue("log"));
}

Logger::~Logger()
{
    // Destroy
}

nav2_util::CallbackReturn
Logger::on_configure(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "Configuring");
    time_at_start_ = round(now().nanoseconds() / 1.0E6);
    std::string time = std::to_string(time_at_start_); // time in millis

    init_parameters();
    
    out_imu_.open(output_folder_ + "/imu_" + time + ".log", std::ios::out | std::ios::trunc);
    out_gps_.open(output_folder_ + "/gps_" + time + ".log", std::ios::out | std::ios::trunc);
    out_odom_.open(output_folder_ + "/odom_" + time + ".log", std::ios::out | std::ios::trunc);

    // Initialize parameters, pub/sub, services, etc.
    init_pub_sub();

    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
Logger::on_activate(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "Activating");

    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
Logger::on_deactivate(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "Deactivating");

    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
Logger::on_cleanup(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "Cleanup");
    
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
Logger::on_shutdown(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "Shutdown");

    out_imu_.close();
    out_gps_.close();
    out_odom_.close();

    return nav2_util::CallbackReturn::SUCCESS;
}

void
Logger::init_parameters()
{
    get_parameter("imu_topic", imu_topic_);
    get_parameter("odom_topic", odom_topic_);
    get_parameter("gps_topic", gps_topic_);
    get_parameter("output_folder", output_folder_);
}

void
Logger::init_pub_sub()
{
    sub_gps_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(gps_topic_, default_qos,
        std::bind(&Logger::gps_callback, this, std::placeholders::_1));

    // Publish odometry
    sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(imu_topic_, default_qos,
        std::bind(&Logger::imu_callback, this, std::placeholders::_1));

    // Receive velocity command from local controller
    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(odom_topic_, default_qos,
        std::bind(&Logger::odom_callback, this, std::placeholders::_1));
}

void
Logger::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg_)
{
    // linear acceleration, angular velocity, orientation
    out_imu_ << time_to_string();
    out_imu_ << vec3_to_string(msg_->linear_acceleration) << ",";
    out_imu_ << vec3_to_string(msg_->angular_velocity) << ",";
    out_imu_ << msg_->orientation.w << ",";
    out_imu_ << msg_->orientation.x << ",";
    out_imu_ << msg_->orientation.y << ",";
    out_imu_ << msg_->orientation.z << "\n";
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
    out_odom_ << time_to_string();
    out_odom_ << std::to_string(msg_->twist.twist.linear.x) << ",";
    out_odom_ << std::to_string(msg_->twist.twist.angular.z) << "\n";
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
