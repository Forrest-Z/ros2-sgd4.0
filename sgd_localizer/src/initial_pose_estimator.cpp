/*
 * This node takes a gps and imu signal as an input and sets the initial pose
 * for amcl algorithm.
 * 
 */

#include "sgd_localizer/initial_pose_estimator.hpp"

namespace sgd_localizer
{

using namespace std::chrono_literals;
using std::placeholders::_1;

Initial_Pose_Estimator::Initial_Pose_Estimator():
    nav2_util::LifecycleNode("initial_pose_estimator", "", true)
{   
    RCLCPP_DEBUG(get_logger(), "Creating");

    // Add parameters
    add_parameter("source_frame", rclcpp::ParameterValue("map"));
    add_parameter("gps_topic", rclcpp::ParameterValue("gps"));
    add_parameter("imu_topic", rclcpp::ParameterValue("imu"));
    add_parameter("initial_pose_topic", rclcpp::ParameterValue("initialpose"));
}

Initial_Pose_Estimator::~Initial_Pose_Estimator() 
{
    // Destroy
}


nav2_util::CallbackReturn
Initial_Pose_Estimator::on_configure(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "Configuring");

    // Initialize parameters, pub/sub, services, etc.
    init_parameters();
    init_pub_sub();

    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
Initial_Pose_Estimator::on_activate(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "Activating");

    imu_good = false;
    gps_good = false;
    initial_pose_set = false;
    pub_initial_pose_->on_activate();

    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
Initial_Pose_Estimator::on_deactivate(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "Deactivating");
    pub_initial_pose_->on_deactivate();
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
Initial_Pose_Estimator::on_cleanup(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "Cleanup");
    pub_initial_pose_.reset();
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
Initial_Pose_Estimator::on_shutdown(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "Shutdown");
    return nav2_util::CallbackReturn::SUCCESS;
}

void
Initial_Pose_Estimator::on_gps_received(const sensor_msgs::msg::NavSatFix::SharedPtr msg_)
{
    // Calculate transformation in local coordinate system
    //RCLCPP_INFO(get_logger(), "GPS Position received");
    auto xy = sgd_util::WGS84_to_local(msg_->latitude, msg_->longitude);
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
Initial_Pose_Estimator::on_imu_received(const sensor_msgs::msg::Imu::SharedPtr msg_)
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
Initial_Pose_Estimator::publish_initial_pose()
{
    RCLCPP_INFO(get_logger(), "Set initial pose.");

    initial_pose.header.frame_id = source_frame_;
    initial_pose.header.stamp = now();

    initial_pose_set = true;
    pub_initial_pose_->publish(initial_pose);

    // TODO: send signal to shutdown node
}

void
Initial_Pose_Estimator::init_parameters()
{
    get_parameter("source_frame", source_frame_);
    get_parameter("gps_topic", gps_topic_);
    get_parameter("imu_topic", imu_topic_);
    get_parameter("initial_pose_topic", initial_pose_topic_);
}

void
Initial_Pose_Estimator::init_pub_sub()
{ 
    subscriber_gps_ = create_subscription<sensor_msgs::msg::NavSatFix>(
        gps_topic_, default_qos, std::bind(&Initial_Pose_Estimator::on_gps_received, this, _1));
    subscriber_imu_ = create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, default_qos, std::bind(&Initial_Pose_Estimator::on_imu_received, this, _1));
    
    pub_initial_pose_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(initial_pose_topic_, default_qos);

    //timer_ = this->create_wall_timer(100ms, std::bind(&Simple_Gps_Localizer::kalman_filter, this));

    RCLCPP_DEBUG(get_logger(), "Initialised subscriber on topic %s and %s.",
            gps_topic_.c_str(), imu_topic_.c_str());
}

}   // namespace nav_sgd

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<sgd_localizer::Initial_Pose_Estimator>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
}

