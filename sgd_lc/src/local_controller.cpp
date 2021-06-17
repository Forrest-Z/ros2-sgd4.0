
#include "sgd_lc/local_controller.hpp"

namespace nav_sgd
{

using std::placeholders::_1;
using namespace std::chrono_literals; 

Local_Controller::Local_Controller() 
        : nav2_util::LifecycleNode("local_controller", "", true)
{
    RCLCPP_DEBUG(get_logger(), "Creating");

    // Add parameters
    add_parameter("filter_i", rclcpp::ParameterValue(5));
    add_parameter("speed_kp", rclcpp::ParameterValue(0.05));
    add_parameter("turn_kp", rclcpp::ParameterValue(0.05));

    // Declare parameters from launch file
    this->declare_parameter("filter_i", 5);
    this->declare_parameter("speed_kp", 0.05);
    this->declare_parameter("turn_kp", 0.05);

    // Initialize poses (temporary) - x,y,w
    poses_.push_back(5.0);poses_.push_back(0.0);poses_.push_back(0.0);
    poses_.push_back(5.0);poses_.push_back(8.0);poses_.push_back(0.0);
    poses_.push_back(10.0);poses_.push_back(8.0);poses_.push_back(0.0);
    poses_.push_back(10.0);poses_.push_back(5.0);poses_.push_back(0.0);
    //poses_.push_back(0.0);poses_.push_back(0.0);poses_.push_back(0.0);
    ref_x_ = 0.0;
    ref_y_ = 0.0;
    ref_w_ = 0.0;
    curr_x_ = 0.0;
    curr_y_ = 0.0;
    curr_w_ = 0.0;

    filter_i_ = this->get_parameter("filter_i").as_int() - 1;   // 0 bis filter_i-1
    for (int i = 0; i < filter_i_; i++)
    {
        filter_speed.push_back(0.0);
        filter_steer.push_back(0.0);
    }

    speed_ = 0.0;

    speed_controller_ = std::shared_ptr<PID_Controller>(new PID_Controller(this->get_parameter("speed_kp").as_double()));
    turn_controller_ = std::shared_ptr<PID_Controller>(new PID_Controller(this->get_parameter("turn_kp").as_double()));
}

Local_Controller::~Local_Controller()
{
    // Destroy
}

nav2_util::CallbackReturn
Local_Controller::on_configure(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "Configure");

    // init
    init_parameters();
    init_pub_sub();
    init_transforms();

    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
Local_Controller::on_activate(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "Activate");

    speed_controller_ = std::shared_ptr<PID_Controller>(new PID_Controller(speed_kp_));
    turn_controller_ = std::shared_ptr<PID_Controller>(new PID_Controller(turn_kp_));

    return wait_for_transform();
}

nav2_util::CallbackReturn
Local_Controller::on_deactivate(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "Deactivate");
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
Local_Controller::on_cleanup(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "Cleanup");
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
Local_Controller::on_shutdown(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "Shutdown");
    return nav2_util::CallbackReturn::SUCCESS;
}

void
Local_Controller::init_transforms()
{
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(rclcpp_node_->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        rclcpp_node_->get_node_base_interface(),
        rclcpp_node_->get_node_timers_interface());
    tf_buffer_->setCreateTimerInterface(timer_interface);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

nav2_util::CallbackReturn
Local_Controller::wait_for_transform()
{
    // Wait for transform to be available
    RCLCPP_DEBUG(get_logger(), "Wait for transform");

    std::string err;
    int retries = 0;
    while (rclcpp::ok() && !tf_buffer_->canTransform("base_footprint", "map", tf2::TimePointZero, tf2::durationFromSec(0.1), &err)
        && retries < 10)
    {
        RCLCPP_INFO(this->get_logger(), "Timeout waiting for transform. Tf error: %s", err);
        err.clear();
        rclcpp::sleep_for(500000000ns);
        retries++;
    }
    return (retries > 9 ? nav2_util::CallbackReturn::FAILURE : nav2_util::CallbackReturn::SUCCESS);
}

void
Local_Controller::init_pub_sub()
{
    pub_motor_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("set_motor", default_qos); 
    sub_motor_ = this->create_subscription<geometry_msgs::msg::TwistStamped>("motordata", default_qos,
        std::bind(&Local_Controller::on_motor_received, this, _1));
    sub_touch_ = this->create_subscription<sgd_msgs::msg::Touch>("handle_touch", default_qos,
        std::bind(&Local_Controller::on_touch_received, this, std::placeholders::_1));
    sub_laser_ = this->create_subscription<sensor_msgs::msg::Range>("laser_1d", default_qos,
        std::bind(&Local_Controller::on_laser_received, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(100ms, std::bind(&Local_Controller::publish_motordata, this));
}

void
Local_Controller::init_parameters()
{
    get_parameter("filter_i", filter_i_);
    get_parameter("speed_kp", speed_kp_);
    get_parameter("turn_kp", turn_kp_);
}

void
Local_Controller::publish_motordata()
{
    geometry_msgs::msg::TwistStamped msg;
    msg.header.stamp = now();
    msg.header.frame_id = "frame";

    if (!last_handle_ || last_range_ > 50 || last_range_ < 2 || ende)   // Wenn Handgriffe nicht gehalten werden / Abstand zu groß / zu klein
    {
        msg.twist.linear.x = 0;
        msg.twist.angular.z = 0;
        pub_motor_->publish(msg);
        return;
    }

    // Regler
    if (distance(curr_x_, curr_y_, ref_x_, ref_y_) < 0.8)  // setze neue Zielposition
    {
        if (!poses_.empty())
        {
            ref_x_ = poses_.front();
            poses_.pop_front();
            ref_y_ = poses_.front();
            poses_.pop_front();
            ref_w_ = poses_.front();
            poses_.pop_front();
            RCLCPP_INFO(get_logger(), "Ziel: x: %f, y: %f, w: %f", ref_x_, ref_y_, ref_w_);
        } else if (distance(curr_x_, curr_y_, ref_x_, ref_y_) < 0.3) {
            ende = true;
            RCLCPP_WARN(get_logger(), "Ende gesetzt!");
            msg.twist.linear.x = 0;
            msg.twist.angular.z = 0;
            pub_motor_->publish(msg);
            return;
        }
    }

    speed_controller_->set_reference(distance(curr_x_, curr_y_, ref_x_, ref_y_));
    
    double winkel = atan2(ref_y_-curr_y_, ref_x_-curr_x_);
    double diff_winkel = winkel - curr_w_;

    RCLCPP_INFO(get_logger(), "Goal: %f, actu_x: %f, actu_y:%f, w: %f",
            distance(curr_x_, curr_y_, ref_x_, ref_y_), curr_x_, curr_y_, diff_winkel/(3.14159)*180);
    turn_controller_->set_reference(winkel);


    // Regler Sollgeschwindigkeit & Regler Sollwinkel
    double set_speed = controller_speed();      // TODO Speed controller & distance to person
    set_speed = set_speed * (1 - 1/50 * last_range_);   // distance to person

    // publish data
    msg.twist.linear.x = set_speed;
    double new_angle = turn_controller_->next(curr_w_);

    msg.twist.angular.z = new_angle;
    RCLCPP_INFO(get_logger(), "Set new winkel to %f", new_angle);
    msg.twist.angular.z = new_angle;
    pub_motor_->publish(msg);
}

void
Local_Controller::on_motor_received(const geometry_msgs::msg::TwistStamped::SharedPtr msg_)
{
    // gleitender Durchschnitt
    double sum_r = 0.0, sum_l = 0.0;
    for (int i = 0; i < filter_i_; i++)
    {
        sum_r += filter_speed[i];
        filter_speed[i] = i+1 < filter_i_ ? filter_speed[i+1] : msg_->twist.linear.x;

        sum_l += filter_steer[i];
        filter_steer[i] = i+1 < filter_i_ ? filter_steer[i+1] : msg_->twist.angular.z;
    }

    double meas_speed_ = sum_r/filter_i_;
    double meas_steer_ = sum_l/filter_i_;
    
    double t_s = (msg_->header.stamp.nanosec - last_nsec) / 1000;   // ACHTUNG: Keine nanosec sondern millisec
    last_nsec = msg_->header.stamp.nanosec;
    
    curr_w_ += meas_steer_ * t_s;
    //RCLCPP_INFO(get_logger(), "Winkel: %f", curr_w_);
    curr_x_ += cos(curr_w_) * meas_speed_ * t_s;
    curr_y_ += sin(curr_w_) * meas_speed_ * t_s;
}

void
Local_Controller::on_laser_received(const sensor_msgs::msg::Range::SharedPtr msg_)
{
    last_range_ = msg_->range;
}

void
Local_Controller::on_touch_received(const sgd_msgs::msg::Touch::SharedPtr msg_)
{
    last_handle_ = msg_->has_detected;
}

double
Local_Controller::controller_speed()
{   
    double r = distance(curr_x_, curr_y_, ref_x_, ref_y_) * this->get_parameter("speed_kp").as_double();    // Abweichung * Kp

    return r > 0.4 ? 0.4 : r;   // MAX_SPEED = 0.8 m/s -> konfigurierbar machen!
}

inline double
Local_Controller::distance(float x1,float y1,float x2,float y2)
{
    return sqrt(pow(x1-x2,2) + pow(y1-y2,2));
}

inline int
Local_Controller::sig(double x)
{
    if(x > 0) return 1;
    if(x < 0) return -1;
    return 0;
}

}   // namespace sgd_lc

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<nav2_util::LifecycleNode> node = std::make_shared<nav_sgd::Local_Controller>();

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Motor driver startup completed.");

    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}
