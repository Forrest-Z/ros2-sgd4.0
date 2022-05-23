/*
 * This node takes a gps and imu signal as an input and sets the initial pose
 * for amcl algorithm.
 * 
 */

#include "sgd_localization/pose_estimator.hpp"

namespace sgd_localization
{

using namespace std::chrono_literals;
using std::placeholders::_1;

Pose_Estimator::Pose_Estimator()
    : rclcpp_lifecycle::LifecycleNode("pose_estimator")
{
    RCLCPP_DEBUG(get_logger(), "Creating node Pose_Estimator");

    // 


    // Add parameters
    declare_parameter("source_frame", rclcpp::ParameterValue("map"));
    declare_parameter("gps_topic", rclcpp::ParameterValue("gps"));
    declare_parameter("imu_topic", rclcpp::ParameterValue("imu"));
    declare_parameter("initial_pose_topic", rclcpp::ParameterValue("initialpose"));
}

Pose_Estimator::~Pose_Estimator() 
{
    // Destroy
}

CallbackReturn
Pose_Estimator::on_configure(const rclcpp_lifecycle::State & state __attribute__((unused)))
{
    RCLCPP_DEBUG(get_logger(), "Configuring");

    // Initialize parameters, pub/sub, services, etc.
    init_parameters();

    return CallbackReturn::SUCCESS;
}

CallbackReturn
Pose_Estimator::on_activate(const rclcpp_lifecycle::State & state __attribute__((unused)))
{
    RCLCPP_DEBUG(get_logger(), "Activating");

    imu_good = false;
    gps_good = false;
    initial_pose_set = false;
    init_pub_sub();
    pub_initial_pose_->on_activate();

    return CallbackReturn::SUCCESS;
}

CallbackReturn
Pose_Estimator::on_deactivate(const rclcpp_lifecycle::State & state __attribute__((unused)))
{
    RCLCPP_DEBUG(get_logger(), "Deactivating");
    pub_initial_pose_->on_deactivate();
    return CallbackReturn::SUCCESS;
}

CallbackReturn
Pose_Estimator::on_cleanup(const rclcpp_lifecycle::State & state __attribute__((unused)))
{
    RCLCPP_DEBUG(get_logger(), "Cleanup");
    pub_initial_pose_.reset();
    return CallbackReturn::SUCCESS;
}

CallbackReturn
Pose_Estimator::on_shutdown(const rclcpp_lifecycle::State & state __attribute__((unused)))
{
    RCLCPP_DEBUG(get_logger(), "Shutdown");
    return CallbackReturn::SUCCESS;
}

void
Pose_Estimator::on_gps_received(const sensor_msgs::msg::NavSatFix::SharedPtr msg_)
{
    // Calculate transformation in local coordinate system
    //RCLCPP_INFO(get_logger(), "GPS Position received");
    sgd_util::LatLon ll(msg_->latitude, msg_->longitude);
    auto xy = ll.to_local(map_origin);

    initial_pose.pose.pose.position.x = xy.first;
    initial_pose.pose.pose.position.y = xy.second;

    // TODO check covariance of gps signal decide whether the signal is good or bad

    if (imu_good && !initial_pose_set)
    {
        publish_initial_pose();
    }
    gps_good = true;
}

void
Pose_Estimator::on_imu_received(const sensor_msgs::msg::Imu::SharedPtr msg_)
{
    //RCLCPP_INFO(get_logger(), "IMU received");
    initial_pose.pose.pose.orientation = msg_->orientation;
    
    if (gps_good && !initial_pose_set)
    {
        publish_initial_pose();
    }
    imu_good = true;
}

void
Pose_Estimator::publish_initial_pose()
{
    RCLCPP_INFO(get_logger(), "Set initial pose.");

    initial_pose.header.frame_id = source_frame_;
    initial_pose.header.stamp = now();

    initial_pose_set = true;
    pub_initial_pose_->publish(initial_pose);

    // TODO: send signal to shutdown node
}

void
Pose_Estimator::init_parameters()
{
    get_parameter("source_frame", source_frame_);
    get_parameter("gps_topic", gps_topic_);
    get_parameter("imu_topic", imu_topic_);
    get_parameter("initial_pose_topic", initial_pose_topic_);
}

void
Pose_Estimator::init_pub_sub()
{ 
    subscriber_gps_ = create_subscription<sensor_msgs::msg::NavSatFix>(
        gps_topic_, default_qos, std::bind(&Pose_Estimator::on_gps_received, this, _1));
    subscriber_imu_ = create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, default_qos, std::bind(&Pose_Estimator::on_imu_received, this, _1));
    
    pub_initial_pose_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(initial_pose_topic_, default_qos);

    RCLCPP_DEBUG(get_logger(), "Initialised subscriber on topic %s and %s.",
            gps_topic_.c_str(), imu_topic_.c_str());
}

CallbackReturn
Pose_Estimator::init_transforms()
{
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        get_node_base_interface(),
        get_node_timers_interface());
    tf_buffer_->setCreateTimerInterface(timer_interface);
    //tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // get map origin
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
        return CallbackReturn::FAILURE;
    }

    // transformation from earth -> map in WGS84 coordinates
    // according to REP-105 the x-axis points east (lon) and the y-axis north (lat)
    auto tf_ = tf_buffer_->lookupTransform("earth", "map", rclcpp::Time(0), rclcpp::Duration(5,0));
    map_origin.set_global_coordinates(tf_.transform.translation.y, tf_.transform.translation.x);
}

}   // namespace sgd_localization

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<sgd_localization::Pose_Estimator>("pose_estimator");
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
}

