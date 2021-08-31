
#include "localizer/simple_gps_localizer.hpp"

namespace sgd
{

using namespace std::chrono_literals;
using std::placeholders::_1;

Simple_Gps_Localizer::Simple_Gps_Localizer():
    nav2_util::LifecycleNode("simple_gps_localizer", "", true)
{   
    RCLCPP_DEBUG(get_logger(), "Creating");

    // Add parameters
    add_parameter("source_frame", rclcpp::ParameterValue("map"));
    add_parameter("target_frame", rclcpp::ParameterValue("odom"));
    add_parameter("gps_topic", rclcpp::ParameterValue("gps"));
    add_parameter("odom_topic", rclcpp::ParameterValue("odom"));
    add_parameter("imu_topic", rclcpp::ParameterValue("imu"));
    add_parameter("odom_thresh", rclcpp::ParameterValue(0.1));

    is_odom_avail_ = false;
}

Simple_Gps_Localizer::~Simple_Gps_Localizer() 
{
    // Destroy
}


nav2_util::CallbackReturn
Simple_Gps_Localizer::on_configure(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "Configuring");

    // Initialize parameters, pub/sub, services, etc.
    last_send_time_ = now().seconds();
    init_parameters();
    init_pub_sub();
    init_transforms();
    xyw = estimation();

    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
Simple_Gps_Localizer::on_activate(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "Activating");

    pub_position_->on_activate();

    return wait_for_transform();
}

nav2_util::CallbackReturn
Simple_Gps_Localizer::on_deactivate(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "Deactivating");
    pub_position_->on_deactivate();
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
Simple_Gps_Localizer::on_cleanup(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "Cleanup");

      // Transforms
    tf_broadcaster_.reset();
    tf_listener_.reset();
    tf_buffer_.reset();
    pub_position_.reset();

    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
Simple_Gps_Localizer::on_shutdown(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "Shutdown");
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
    tf2::TimePoint transform_expiration = tf2_ros::fromMsg(now()) +
        transform_tolerance_;
    tf_map_odom_.header.stamp = tf2_ros::toMsg(transform_expiration);
    tf_map_odom_.header.frame_id = source_frame_;
    tf_map_odom_.child_frame_id = target_frame_;

    // Calculate transformation
    auto xy = sgd_util::WGS84_to_local(msg_->latitude, msg_->longitude);
    gps_pos_.push_back(xy);
    if (gps_pos_.size() > 8) gps_pos_.pop_front();

    // check if robot is moving
    // if robot is not moving get average of last 2s
    double x_ = 0.0;
    double y_ = 0.0;
    if (is_robo_moving())
    {
        // gps is expected to be bad due to delay in signal
        x_ = xy.first;
        y_ = xy.second;
        gps_pos_.clear();
    }
    else
    {
        // get average
        // TODO filter bad values
        for (auto p : gps_pos_)
        {
            x_ += p.first / gps_pos_.size();
            y_ += p.second / gps_pos_.size();
        }
    }

    // TODO: use angle from gps data?
    double dx = x_ - (-cos(xyw.w) * x_base_gps_ - sin(xyw.w) * y_base_gps_ + xyw.x);
    double dy = y_ - (-sin(xyw.w) * x_base_gps_ + cos(xyw.w) * y_base_gps_ + xyw.y);
    //RCLCPP_INFO(get_logger(), "GPS delta x,y: %.4f, %.4f", dx, dy);

    // kein Signal ausgeben -> Signal für Kalman

    tf_map_odom_.transform.translation.x = dx;
    tf_map_odom_.transform.translation.y = dy;
    tf_map_odom_.transform.translation.z = 0.142176;
    tf_map_odom_.transform.rotation = angleZ_to_Quaternion(0);
    tf_broadcaster_->sendTransform(tf_map_odom_);
}

void
Simple_Gps_Localizer::odom_sub_callback(const nav_msgs::msg::Odometry::SharedPtr msg_)
{
    if (!is_odom_avail_)
    {
        odom_initial_ = std::make_pair(msg_->pose.pose.position.x, msg_->pose.pose.position.y);
    }

    x_odom_ = msg_->pose.pose.position.x - odom_initial_.first;
    y_odom_ = msg_->pose.pose.position.y - odom_initial_.second;
    xp_odom_ = msg_->twist.twist.linear.x;
    wp_odom_ = msg_->twist.twist.angular.z;

    tf2::Quaternion q;
    tf2::fromMsg(msg_->pose.pose.orientation ,q);
    auto t = tf2::Matrix3x3(q);
    tf2Scalar r,p,y;
    t.getRPY(r, p, y);

    angle_odom_ = y;
    is_odom_avail_ = true;

    geometry_msgs::msg::TransformStamped odom_tf;
    odom_tf.header.stamp = now();
    odom_tf.header.frame_id = "odom";
    odom_tf.child_frame_id = "base_link";

    odom_tf.transform.translation.x = x_odom_;
    odom_tf.transform.translation.y = y_odom_;
    odom_tf.transform.translation.z = 0.0;

    tf2::Quaternion q1;
    q1.setRPY(0, 0, xyw.w);
    odom_tf.transform.rotation = tf2::toMsg(q1);
    
    tf_broadcaster_->sendTransform(odom_tf);
}

void
Simple_Gps_Localizer::imu_sub_callback(const sensor_msgs::msg::Imu::SharedPtr msg_)
{
    // get imu data and calculate position and velocity
    xpp_imu_ = msg_->linear_acceleration.x;

    tf2::Quaternion q(
        msg_->orientation.x,
        msg_->orientation.y,
        msg_->orientation.z,
        msg_->orientation.w
    );
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, w_imu_);

    wp_imu_ = msg_->angular_velocity.z;

    if (!xyw.initial_pos_set)
    {
        xyw.w = w_imu_;
        xyw.initial_pos_set = true;
    }
}

void
Simple_Gps_Localizer::init_parameters()
{
    get_parameter("target_frame", target_frame_);
    get_parameter("source_frame", source_frame_);
    get_parameter("gps_topic", gps_topic_);
    get_parameter("odom_topic", odom_topic_);
    get_parameter("imu_topic", imu_topic_);
    get_parameter("odom_thresh", odom_thresh_);
}

void
Simple_Gps_Localizer::init_pub_sub()
{ 
    subscriber_gps_ = create_subscription<sensor_msgs::msg::NavSatFix>(
        gps_topic_, default_qos, std::bind(&Simple_Gps_Localizer::gps_sub_callback, this, _1));
    subscriber_odom_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, default_qos, std::bind(&Simple_Gps_Localizer::odom_sub_callback, this, _1));
    subscriber_imu_ = create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, default_qos, std::bind(&Simple_Gps_Localizer::imu_sub_callback, this, _1));
    
    pub_position_ = create_publisher<sgd_msgs::msg::SpeedAccel>("speed_accel", default_qos);

    timer_ = this->create_wall_timer(100ms, std::bind(&Simple_Gps_Localizer::kalman_filter, this));

    RCLCPP_DEBUG(get_logger(), "Initialised subscriber on topic %s and subscriber on topic %s.",
            gps_topic_.c_str(), odom_topic_.c_str());
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
    while (rclcpp::ok() && !tf_buffer_->canTransform("gps_link", "base_link", tf2::TimePointZero, tf2::durationFromSec(0.1), &err)
        && retries < 10)
    {
        RCLCPP_INFO(this->get_logger(), "Timeout waiting for transform. Tf error: %s", err);
        err.clear();
        rclcpp::sleep_for(500000000ns);
        retries++;
    }

    try
    {
        geometry_msgs::msg::TransformStamped tf_base_gps_ = tf_buffer_->lookupTransform("gps_link", "base_link", rclcpp::Time(0), rclcpp::Duration(5,0));
        x_base_gps_ = tf_base_gps_.transform.translation.x;
        y_base_gps_ = tf_base_gps_.transform.translation.y;
        RCLCPP_DEBUG(this->get_logger(), "Transform base_link -> base_link z: %f", tf_base_gps_.transform.translation.z);
    } catch (tf2::TransformException &ex)
    {
        RCLCPP_WARN(this->get_logger(), "%s", ex.what());
        return nav2_util::CallbackReturn::FAILURE;
    }

    return nav2_util::CallbackReturn::SUCCESS;
}

void
Simple_Gps_Localizer::kalman_filter()
{
    if (!xyw.initial_pos_set)
    {
        RCLCPP_WARN(get_logger(), "Could not calculate position due to missing odom or imu data.");
        return;     // IMU and odom data are required
    }

    double dt = now().seconds() - last_send_time_;   // time in s
    double dt2 = pow(dt,2) / 2.0;
    last_send_time_ = now().seconds();
    
    // Prediction
    xyw.w += dt*xyw.wp + dt2*xyw.wpp;
    xyw.wp += dt*xyw.wpp;

    double c = cos(xyw.w);
    double s = sin(xyw.w);

    xyw.x += dt*c*xyw.xp + dt2*c*xyw.xpp;
    xyw.y += dt*s*xyw.xp + dt2*s*xyw.xpp;
    xyw.xp += dt*c*xyw.xpp;
    xyw.yp += dt*s*xyw.xpp;
    xyw.xpp = c*xyw.xpp;
    xyw.ypp = s*xyw.xpp;

    // Correction
    double y[4] = {xp_odom_ - xyw.xp,
                   xpp_imu_ - xyw.xpp,
                   w_imu_ - xyw.w,
                   (0.75*wp_imu_ + 0.25*wp_odom_) - xyw.wp};
    
    xyw.x += Kx[0][0] * y[0] + Kx[0][1]*y[1];
    xyw.y += Kx[1][0] * y[0] + Kx[1][1]*y[1];
    xyw.xp += Kx[2][0] * y[0] + Kx[2][1]*y[1];
    xyw.yp += Kx[3][0] * y[0] + Kx[3][1]*y[1];
    xyw.xpp += Kx[4][0] * y[0] + Kx[4][1]*y[1];
    xyw.ypp += Kx[5][0] * y[0] + Kx[5][1]*y[1];

    xyw.w += Kw[0][0]*y[2] + Kw[0][1]*y[3];
    xyw.wp += Kw[1][0]*y[2] + Kw[1][1]*y[3];
    xyw.wpp += Kw[2][0]*y[2] + Kw[2][1]*y[3];

    // publish pose
    //RCLCPP_INFO(get_logger(), "Filter pose: %.4f, %.4f, %.4f", xyw.x, xyw.y, xyw.w);

    
    publish_position(xyw);
}

bool
Simple_Gps_Localizer::is_robo_moving()
{
    // odom > 0
    return (abs(xp_odom_) > odom_thresh_ ? true : false);
}

void
Simple_Gps_Localizer::publish_position(estimation xyz)
{
    sgd_msgs::msg::SpeedAccel message;
    message.header.stamp = now();
    message.header.frame_id = "base_footprint";

    // Position
    message.twist.linear.x = xyz.xp;
    message.twist.linear.y = xyz.yp;
    message.accel.linear.x = xyz.xpp;
    message.accel.linear.y = xyz.ypp;

    // Rotation
    message.twist.angular.z = xyz.wp;
    message.accel.angular.z = xyz.wpp;

    pub_position_->publish(message);
}

}   // namespace nav_sgd

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<sgd::Simple_Gps_Localizer>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
}

