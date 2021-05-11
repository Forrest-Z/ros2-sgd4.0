/*
 *  Some text
 */

# include "sgd_lc/wh_f-cruiser.hpp"

namespace sgd_lc
{

using namespace std::chrono_literals;

WH_Fcruiser::WH_Fcruiser() : nav2_util::LifecycleNode("wh_cruiser", "", true)
{
    RCLCPP_DEBUG(get_logger(), "Creating");
    // Add parameters
    add_parameter("port", rclcpp::ParameterValue("/dev/novalue"));
    add_parameter("msg_regex", rclcpp::ParameterValue("HS:(\\d+),Rm:(-?\\d+),Lm:(-?\\d+),V:(\\d+),T:(\\d+),S:(\\d?)"));
    add_parameter("motor_kp", rclcpp::ParameterValue(0.3));
    add_parameter("max_speed", rclcpp::ParameterValue(200.0));
}

WH_Fcruiser::~WH_Fcruiser()
{
    // Destroy
    sgd_msgs::msg::Serial msg;
    msg.header.stamp = now();
    msg.msg = "0,0";
    pub_motor_->publish(msg);
}

nav2_util::CallbackReturn
WH_Fcruiser::on_configure(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "Configuring");
    // Initialize parameters, pub/sub, services, etc.
    init_parameters();
    init_pub_sub();
    init_transforms();
    init_controller();

    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
WH_Fcruiser::on_activate(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "Activating");

    pub_odom_->on_activate();
    pub_motor_->on_activate();
    pub_battery_->on_activate();

    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
WH_Fcruiser::on_deactivate(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "Deactivating");

    pub_odom_->on_deactivate();
    pub_motor_->on_deactivate();
    pub_battery_->on_deactivate();

    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
WH_Fcruiser::on_cleanup(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "Cleanup");
    sgd_msgs::msg::Serial msg;
    msg.header.stamp = now();
    msg.msg = "0,0";
    pub_motor_->publish(msg);

    pub_odom_.reset();
    pub_motor_.reset();
    pub_battery_.reset();
    tf_broadcaster_.reset();

    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
WH_Fcruiser::on_shutdown(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "Shutdown");
    return nav2_util::CallbackReturn::SUCCESS;
}

void
WH_Fcruiser::init_parameters()
{
    get_parameter("port", port_);
    get_parameter("msg_regex", msg_regex_);
    get_parameter("motor_kp", motor_kp_);
    get_parameter("max_speed", max_speed_);
}

void
WH_Fcruiser::init_pub_sub()
{
    regex_ = std::regex(msg_regex_);
    std::string topic_pub = "write_" + port_.substr(port_.find_last_of("/")+1);
    std::string topic_sub = "serial_" + port_.substr(port_.find_last_of("/")+1);

    RCLCPP_DEBUG(get_logger(), "Initialised publisher on topic %s and subscriber on topic %s.",
            topic_pub.c_str(), topic_sub.c_str());

    // Communication with motorcontroller
    pub_motor_ = this->create_publisher<sgd_msgs::msg::Serial>(topic_pub,default_qos);
    sub_motor_ = this->create_subscription<sgd_msgs::msg::Serial>(topic_sub, default_qos,
        std::bind(&WH_Fcruiser::on_motor_received, this, std::placeholders::_1));

    // Publish odometry
    pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", default_qos);

    // Receive velocity command from local controller
    sub_data_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", default_qos,
        std::bind(&WH_Fcruiser::on_cmd_vel_received, this, std::placeholders::_1));

    // Publish battery state
    pub_battery_ = this->create_publisher<sensor_msgs::msg::BatteryState>("battery", default_qos);

    timer_ = this->create_wall_timer(100ms, std::bind(&WH_Fcruiser::publish_motordata, this));
}

void
WH_Fcruiser::init_transforms()
{
    // TODO get wheel separation etc from tf
    wheel_sep_ = 0.71;
    wheel_cir_ = 0.68;

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(rclcpp_node_);
}

void
WH_Fcruiser::init_controller()
{
    RCLCPP_DEBUG(get_logger(), "Initialise wheel controller with kp = %f and max speed = %f.",
            motor_kp_, max_speed_);
    wheel_r_controller_ = std::shared_ptr<PID_Controller>(new PID_Controller(motor_kp_));
    wheel_l_controller_ = std::shared_ptr<PID_Controller>(new PID_Controller(motor_kp_));
    wheel_r_controller_->set_max(max_speed_);
    wheel_r_controller_->set_min(-max_speed_);
    wheel_l_controller_->set_max(max_speed_);
    wheel_l_controller_->set_min(-max_speed_);
}

void
WH_Fcruiser::publish_motordata()
{   
    double set_r_speed = wheel_r_controller_->next(meas_r_);
    double set_l_speed = wheel_l_controller_->next(meas_l_);

    int steer = round(set_l_speed - set_r_speed);
    int speed = round((set_r_speed + set_l_speed) / 2);

    sgd_msgs::msg::Serial msg;
    msg.header.stamp = now();
    std::string m = std::to_string(speed) + "," + std::to_string(steer);
    msg.msg = m;
    pub_motor_->publish(msg);
}

void
WH_Fcruiser::publish_battery_state(double voltage)
{
    if (batt_volt_ < 0.01) batt_volt_ = voltage;

    batt_volt_ = (2 * batt_volt_ + voltage) / 3;

    sensor_msgs::msg::BatteryState msg;
    msg.header.stamp = now();
    msg.header.frame_id = "odom";

    if (batt_volt_ < 32)
    {
        RCLCPP_WARN(get_logger(), "Battery voltage low!");
        msg.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_DEAD;
    } else {
        msg.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
    }
    
    msg.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;
    msg.voltage = batt_volt_;
    pub_battery_->publish(msg);
}

void
WH_Fcruiser::on_motor_received(const sgd_msgs::msg::Serial::SharedPtr msg)
{
    // Parse motor data
    if (msg->msg.find("HS") == std::string::npos) {return;}     // message is not from motor

    std::smatch matches;
    std::regex_search(msg->msg, matches, regex_);
    
    int time, v;
    if (matches.size() > 4) // ist gut
    {
        time = std::stoi(matches[1]);       // in millis
        meas_r_ = std::stod(matches[2]);
        meas_l_ = std::stod(matches[3]);
        v = std::stoi(matches[4]);
    } else { return; }

    // -> Winkelgeschwindigkeit [rad/s]
    double w_r_ = (meas_r_ - sig(meas_r_) * 50) / (173 * wheel_cir_);
    double w_l_ = (meas_l_ - sig(meas_l_) * 50) / (175 * wheel_cir_);

    // Calculate pose
    rclcpp::Duration delta_t(now() - last_odom_msg_.header.stamp);

    double twist_ang_z = ((w_r_ - w_l_) * wheel_cir_) / wheel_sep_;
    pose_orie_z += twist_ang_z * delta_t.seconds();
    double speed_xy = ((w_r_ + w_l_) * wheel_cir_) / 2;

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = now();
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";
    // Twist relative to child_frame_id
    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    odom.twist.twist.angular.z = twist_ang_z;
    odom.twist.twist.linear.x = speed_xy;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.linear.z = 0.0;

    odom.pose.pose.position.x = last_odom_msg_.pose.pose.position.x + last_odom_msg_.twist.twist.linear.x * delta_t.seconds();
    odom.pose.pose.position.y = last_odom_msg_.pose.pose.position.y + last_odom_msg_.twist.twist.linear.y * delta_t.seconds();
    odom.pose.pose.position.z = 0.142176;
    tf2::Quaternion q;
    q.setRPY(0, 0, pose_orie_z);
    odom.pose.pose.orientation = tf2::toMsg(q);

    // TODO: covariance

    geometry_msgs::msg::TransformStamped odom_tf;
    odom_tf.header.stamp = now();
    odom_tf.header.frame_id = "odom";
    odom_tf.child_frame_id = "base_footprint";

    odom_tf.transform.translation.x = odom.pose.pose.position.x;
    odom_tf.transform.translation.y = odom.pose.pose.position.y;
    odom_tf.transform.translation.z = odom.pose.pose.position.z;
    odom_tf.transform.rotation = odom.pose.pose.orientation;

    pub_odom_->publish(odom);
    tf_broadcaster_->sendTransform(odom_tf);
    last_odom_msg_ = odom;

    publish_battery_state(v/100.0);
}

void
WH_Fcruiser::on_cmd_vel_received(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    // speed and turn speed to rotations
    double w_r = (msg->linear.x + (0.355 * msg->angular.z)) / 0.68;
    double w_l = (2 * msg->linear.x) / 0.68 - w_r;

    // set new reference to controller
    double r = 173 * 0.68 * w_r + sig(w_r) * 50;
    double l = 175 * 0.68 * w_l + sig(w_l) * 50;

    wheel_r_controller_->set_reference(r);
    wheel_l_controller_->set_reference(l);
}

}

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<sgd_lc::WH_Fcruiser>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();

    return 0;
}
