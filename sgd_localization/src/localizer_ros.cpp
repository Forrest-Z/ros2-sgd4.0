#include "sgd_localization/localizer_ros.hpp"

namespace sgd_localization
{

Localizer::Localizer() : rclcpp_lifecycle::LifecycleNode("localizer")
{
    // declare parameters
    declare_parameter("frequency", rclcpp::ParameterValue(10.0));
    declare_parameter("publish_tf", rclcpp::ParameterValue(true));

    declare_parameter("earth_frame", rclcpp::ParameterValue("earth"));
    declare_parameter("map_frame", rclcpp::ParameterValue("map"));
    declare_parameter("odom_frame", rclcpp::ParameterValue("odom"));
    declare_parameter("base_link_frame", rclcpp::ParameterValue("base_link"));
    declare_parameter("tf_base_frame", rclcpp::ParameterValue(""));

    declare_parameter("odom", rclcpp::ParameterValue(""));
    declare_parameter("imu", rclcpp::ParameterValue(""));
    declare_parameter("gps", rclcpp::ParameterValue(""));
    declare_parameter("uwb", rclcpp::ParameterValue(""));
}

Localizer::~Localizer() {}

CallbackReturn
Localizer::on_configure(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Configuring %s", this->get_name());

    // Initialize parameters, pub/sub, services, etc.
    auto callback_return = init_parameters();
    init_transforms();
    return callback_return;
}

CallbackReturn
Localizer::on_activate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Activating %s", this->get_name());

    // add and activate publisher/subscriber
    if (odom_.size() > 0)
    {
        std::function<void(std::shared_ptr<nav_msgs::msg::Odometry>)> fnc = std::bind(
            &Localizer::on_odom_received, this, std::placeholders::_1, odom_);
        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(odom_, default_qos, fnc);
    }

    if (imu_.size() > 0)
    {
        std::function<void(std::shared_ptr<sensor_msgs::msg::Imu>)> fnc = std::bind(
            &Localizer::on_imu_recevied, this, std::placeholders::_1, imu_);
        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(imu_, default_qos, fnc);
    }

    if (gps_.size() > 0)
    {
        std::function<void(std::shared_ptr<sensor_msgs::msg::NavSatFix>)> fnc = std::bind(
            &Localizer::on_navsatfix_received, this, std::placeholders::_1, gps_);
        sub_gps_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(gps_, default_qos, fnc);
    }

    if (uwb_.size() > 0)
    {
        std::function<void(std::shared_ptr<sensor_msgs::msg::NavSatFix>)> fnc = std::bind(
            &Localizer::on_navsatfix_received, this, std::placeholders::_1, uwb_);
        sub_uwb_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(uwb_, default_qos, fnc);
    }

    if (is_publish_tf_)
    {
        timer_tf_ = this->create_wall_timer(100ms, std::bind(&Localizer::publish_tf, this));
    }
    return CallbackReturn::SUCCESS;
}

CallbackReturn
Localizer::on_deactivate(const rclcpp_lifecycle::State &)
{
    return CallbackReturn::SUCCESS;
}

CallbackReturn
Localizer::on_cleanup(const rclcpp_lifecycle::State &)
{
    return CallbackReturn::SUCCESS;
}

CallbackReturn
Localizer::on_shutdown(const rclcpp_lifecycle::State &)
{
    return CallbackReturn::SUCCESS;
}

void
Localizer::on_odom_received(nav_msgs::msg::Odometry::SharedPtr msg_, std::string topic_name_)
{
    // get position and rotation from message
    last_timestamp_ = msg_->header.stamp;
    last_odom_ = *msg_;
}

void
Localizer::on_imu_recevied(sensor_msgs::msg::Imu::SharedPtr msg_, std::string topic_name_)
{
    // get rotation from message
    //last_timestamp_ = msg_->header.stamp;
    filter_imu_rot_.add_value(tf2::getYaw(msg_->orientation));
}

void
Localizer::on_navsatfix_received(sensor_msgs::msg::NavSatFix::SharedPtr msg_, std::string topic_name_)
{
    // get absolute position from message
    last_timestamp_ = msg_->header.stamp;
}

void
Localizer::publish_tf()
{
    // calculate odom -> base_link
    geometry_msgs::msg::TransformStamped t;
    tf2::TimePoint transform_expiration = tf2_ros::fromMsg(last_timestamp_) +
        transform_tolerance_;
    t.header.stamp = tf2_ros::toMsg(transform_expiration);

    // get rotation from imu filter
    auto rot_ = filter_imu_rot_.result();

    // get rotation from odom
    // set rotation offset for odom
    // odom is more robust, but imu is more accurate
    // if (imu.state > 1)   // check state of imu
    odom_offset_ = tf2::getYaw(last_odom_.pose.pose.orientation) - rot_;

    // get translation from odom message
    t.transform.translation.x = last_odom_.pose.pose.position.x;
    t.transform.translation.y = last_odom_.pose.pose.position.y;
    
    // set rotation -> from imu and odom
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, rot_);
    t.transform.rotation = tf2::toMsg(q);

    //t.header.stamp = last_msg_.header.stamp;
    t.header.frame_id = "odom";
    t.child_frame_id = "base_link";

    if (is_publish_tf_)
    {
        tf_broadcaster_->sendTransform(t);
    }
}

geometry_msgs::msg::TransformStamped
Localizer::get_transform(const std::string target_frame, const std::string source_frame)
{
    // Wait for transform to be available
    std::string err;
    int retries = 0;
    while (rclcpp::ok() && !tf_buffer_->canTransform(target_frame, source_frame, tf2::TimePointZero, tf2::durationFromSec(0.1), &err)
        && retries < 10)
    {
        RCLCPP_INFO(this->get_logger(), "Timeout waiting for transform. Tf error: %s", err);
        err.clear();
        rclcpp::sleep_for(500000000ns);
        retries++;
    }

    if (retries > 9)
    {
        RCLCPP_ERROR(get_logger(), "Could not retrieve transform from %s to %s.", target_frame.c_str(), source_frame.c_str());
        geometry_msgs::msg::TransformStamped ts;
        return ts;
    }

    // transformation from earth -> map in WGS84 coordinates
    // according to REP-105 the x-axis points east (lon) and the y-axis north (lat)
    return tf_buffer_->lookupTransform(target_frame, source_frame, rclcpp::Time(0), rclcpp::Duration(5,0));
}

void
Localizer::init_transforms()
{
    transform_tolerance_ = tf2::durationFromSec(1.0);
    // transformation from earth -> map in WGS84 coordinates
    // according to REP-105 the x-axis points east (lon) and the y-axis north (lat)
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        get_node_base_interface(),
        get_node_timers_interface());
    tf_buffer_->setCreateTimerInterface(timer_interface);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    if (is_publish_tf_)
    {
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    // get transform from earth to map (used for gps and uwb only)
    auto tf_earth_map_ = get_transform("earth", "map");
    map_origin.set_global_coordinates(tf_earth_map_.transform.translation.y, tf_earth_map_.transform.translation.x);
}

CallbackReturn
Localizer::init_parameters()
{
    // declare parameters
    get_parameter("frequency", frequency_);
    get_parameter("publish_tf", is_publish_tf_);

    get_parameter("earth_frame", earth_frame_);
    get_parameter("map_frame", map_frame_);
    get_parameter("odom_frame", odom_frame_);
    get_parameter("base_link_frame", base_link_frame_);
    get_parameter("tf_base_frame", tf_base_frame_);

    get_parameter("odom", odom_);
    get_parameter("imu", imu_);
    get_parameter("gps", gps_);
    get_parameter("uwb", uwb_);

    if (odom_.size() + imu_.size() + gps_.size() + uwb_.size() < 1)
    {
        RCLCPP_ERROR(get_logger(), "Could not initialize Localizer: No sensor topic specified.");
        return CallbackReturn::FAILURE;
    }

    return CallbackReturn::SUCCESS;
}

} // namespace sgd_localization

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<sgd_localization::Localizer>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
}
