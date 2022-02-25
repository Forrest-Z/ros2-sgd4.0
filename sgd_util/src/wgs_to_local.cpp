#include "sgd_util/wgs_to_local.hpp"

namespace sgd_util
{

using namespace std::chrono_literals;
using std::placeholders::_1;

WGStoLocal::WGStoLocal() : rclcpp::Node("WGStoLocal")
{
    gps_topic_ = declare_parameter<std::string>("gps_topic", "gps");
    gps_local_topic_ = declare_parameter<std::string>("gps_local_topic", "gps_local");

    init_pub_sub();
    init_transforms();
}

WGStoLocal::~WGStoLocal() {}

void
WGStoLocal::init_transforms() {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        get_node_base_interface(),
        get_node_timers_interface());
    tf_buffer_->setCreateTimerInterface(timer_interface);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Wait for transform to be available
    std::string err;
    int retries = 0;
    while (rclcpp::ok() && !tf_buffer_->canTransform("earth", "map", tf2::TimePointZero, tf2::durationFromSec(0.1), &err)
        && retries < 10)
    {
        RCLCPP_INFO(this->get_logger(), "Timeout waiting for transform. Tf error: %s", err);
        err.clear();
        rclcpp::sleep_for(500000000ns);
        retries++;
    }

    if (retries > 9)
    {
        RCLCPP_ERROR(get_logger(), "Could not retrieve transform from earth to map.");
        return;
    }

    // transformation from earth -> map in WGS84 coordinates
    // according to REP-105 the x-axis points east (lon) and the y-axis north (lat)
    auto tf_ = tf_buffer_->lookupTransform("earth", "map", rclcpp::Time(0), rclcpp::Duration(5,0));
    map_origin.set_global_coordinates(tf_.transform.translation.y, tf_.transform.translation.x);
}

void
WGStoLocal::init_pub_sub() {
    sub_gps_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(gps_topic_, default_qos,
            std::bind(&WGStoLocal::on_gps_received, this, _1));

    pub_gps_local_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(gps_local_topic_, default_qos);
}

void
WGStoLocal::on_gps_received(const sensor_msgs::msg::NavSatFix::SharedPtr msg_)
{
    // create pose stamped msg
    geometry_msgs::msg::PoseWithCovarianceStamped pose;
    pose.header = msg_->header;

    // wgs84 -> map frame
    LatLon ll(msg_->latitude, msg_->longitude);
    auto xy = ll.to_local(map_origin);

    // publish local pose
    pose.pose.pose.position.x = xy.first;
    pose.pose.pose.position.y = xy.second;

    pub_gps_local_->publish(pose);
}
    
} // namespace sgd_util

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<sgd_util::WGStoLocal>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();

    return 0;
}