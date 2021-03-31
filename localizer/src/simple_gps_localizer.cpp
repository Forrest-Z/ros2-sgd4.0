
#include "localizer/simple_gps_localizer.hpp"

namespace sgd
{

using namespace std::chrono_literals;
using std::placeholders::_1;

Simple_Gps_Localizer::Simple_Gps_Localizer():
    nav2_util::LifecycleNode("simple_gps_localizer", "", true)
{   
    RCLCPP_INFO(get_logger(), "Creating");

    // Add parameters
    add_parameter("source_frame", rclcpp::ParameterValue("map"));
    add_parameter("target_frame", rclcpp::ParameterValue("odom"));
    add_parameter("gps_topic", rclcpp::ParameterValue("gps"));
    add_parameter("odom_topic", rclcpp::ParameterValue("odom"));

    is_odom_avail_ = false;

    // subscribe to gps topic
    // subscribe to odom topic

    // calculate map -> odom transformation

    // publish transform in global coordinates (-> global controller)
    // publish transform in local coordinates (temp.)

}

Simple_Gps_Localizer::~Simple_Gps_Localizer()
{
    // Destroy
}


nav2_util::CallbackReturn
Simple_Gps_Localizer::on_configure(const rclcpp_lifecycle::State & state)
{
    RCLCPP_INFO(get_logger(), "Configuring");

    // Initialize parameters, pub/sub, services, etc.
    init_paramters();
    init_pub_sub();
    init_transforms();

    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
Simple_Gps_Localizer::on_activate(const rclcpp_lifecycle::State & state)
{
    RCLCPP_INFO(get_logger(), "Activating");

    return wait_for_transform();
}

nav2_util::CallbackReturn
Simple_Gps_Localizer::on_deactivate(const rclcpp_lifecycle::State & state)
{
    RCLCPP_INFO(get_logger(), "Deactivating");
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
Simple_Gps_Localizer::on_cleanup(const rclcpp_lifecycle::State & state)
{
    RCLCPP_INFO(get_logger(), "Cleanup");

      // Transforms
    tf_broadcaster_.reset();
    tf_listener_.reset();
    tf_buffer_.reset();

    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
Simple_Gps_Localizer::on_shutdown(const rclcpp_lifecycle::State & state)
{
    RCLCPP_INFO(get_logger(), "Shutdown");
    return nav2_util::CallbackReturn::SUCCESS;
}

void
Simple_Gps_Localizer::gps_sub_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg_)
{
    RCLCPP_DEBUG(this->get_logger(), "GPS signal received. Lat: %.7f, Lon: %.7f", msg_->latitude, msg_->longitude);
    
    if (!is_odom_avail_)
    {
        RCLCPP_WARN(get_logger(), "Transform could not be calculated due to missing odom data.");
        return;
    }

    auto transform_tolerance_ = tf2::durationFromSec(1.0);
    tf2::TimePoint transform_expiration = tf2_ros::fromMsg(msg_->header.stamp) +
        transform_tolerance_;
    tf_map_odom_.header.stamp = tf2_ros::toMsg(transform_expiration);
    tf_map_odom_.header.frame_id = source_frame_;
    tf_map_odom_.child_frame_id = target_frame_;

    // Calculate transformation
    auto xy = sgd_util::WGS84_to_local(msg_->latitude, msg_->longitude);

    double dx = xy.first - (-cos(angle_odom_) * x_base_gps_ - sin(angle_odom_) * y_base_gps_ + x_odom_);
    double dy = xy.second - (-sin(angle_odom_) * x_base_gps_ + cos(angle_odom_) * y_base_gps_ + y_odom_);

    tf_map_odom_.transform.translation.x = dx;
    tf_map_odom_.transform.translation.y = dy;
    tf_map_odom_.transform.translation.z = 0;
    tf_map_odom_.transform.rotation = angleZ_to_Quaternion(0);
    tf_broadcaster_->sendTransform(tf_map_odom_);

}

void
Simple_Gps_Localizer::odom_sub_callback(const nav_msgs::msg::Odometry::SharedPtr msg_)
{
    x_odom_ = msg_->pose.pose.position.x;
    y_odom_ = msg_->pose.pose.position.y;

    tf2::Quaternion q;
    tf2::fromMsg(msg_->pose.pose.orientation ,q);
    auto t = tf2::Matrix3x3(q);
    tf2Scalar r,p,y;
    t.getRPY(r, p, y);

    angle_odom_ = y;
    is_odom_avail_ = true;
}

void
Simple_Gps_Localizer::init_paramters()
{
    get_parameter("target_frame", target_frame_);
    get_parameter("source_frame", source_frame_);
    get_parameter("gps_topic", gps_topic_);
    get_parameter("odom_topic", odom_topic_);
}

void
Simple_Gps_Localizer::init_pub_sub()
{
    RCLCPP_DEBUG(get_logger(), "Init publisher and subscriber");
    subscriber_gps_ = create_subscription<sensor_msgs::msg::NavSatFix>(
        gps_topic_, default_qos, std::bind(&Simple_Gps_Localizer::gps_sub_callback, this, _1));
    subscriber_odom_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, default_qos, std::bind(&Simple_Gps_Localizer::odom_sub_callback, this, _1));
}

void
Simple_Gps_Localizer::init_transforms()
{
    RCLCPP_DEBUG(get_logger(), "Init transforms");

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(rclcpp_node_->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        rclcpp_node_->get_node_base_interface(),
        rclcpp_node_->get_node_timers_interface());
    tf_buffer_->setCreateTimerInterface(timer_interface);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(rclcpp_node_);
}

nav2_util::CallbackReturn
Simple_Gps_Localizer::wait_for_transform()
{
    // Wait for transform to be available
    RCLCPP_DEBUG(get_logger(), "Wait for transform");

    std::string err;
    int retries = 0;
    while (rclcpp::ok() && !tf_buffer_->canTransform("gps_link", "base_footprint", tf2::TimePointZero, tf2::durationFromSec(0.1), &err)
        && retries < 10)
    {
        RCLCPP_INFO(this->get_logger(), "Timeout waiting for transform. Tf error: %s", err);
        err.clear();
        rclcpp::sleep_for(500000000ns);
        retries++;
    }

    try
    {
        geometry_msgs::msg::TransformStamped tf_base_gps_ = tf_buffer_->lookupTransform("gps_link", "base_footprint", rclcpp::Time(0), rclcpp::Duration(5,0));
        x_base_gps_ = tf_base_gps_.transform.translation.x;
        y_base_gps_ = tf_base_gps_.transform.translation.y;
        RCLCPP_INFO(this->get_logger(), "Transform base_footprint -> base_link z: %f", tf_base_gps_.transform.translation.z);
    } catch (tf2::TransformException &ex)
    {
        RCLCPP_WARN(this->get_logger(), "%s", ex.what());
        return nav2_util::CallbackReturn::FAILURE;
    }

    return nav2_util::CallbackReturn::SUCCESS;
}


}   // namespace nav_sgd

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<sgd::Simple_Gps_Localizer>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
}

