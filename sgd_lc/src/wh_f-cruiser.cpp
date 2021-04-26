/*
 *  Some text
 */

# include "sgd_lc/wh_f-cruiser.hpp"

namespace sgd_lc
{

using namespace std::chrono_literals;

WH_Fcruiser::WH_Fcruiser() : Node("wh_cruiser")
{
    this->declare_parameter("port", "/dev/novalue");
    this->declare_parameter("msg_regex", "HS:(\\d+),Rm:(-?\\d+),Lm:(-?\\d+),V:(\\d+),T:(\\d+),S:(\\d?)");
    this->declare_parameter("motor_kp", 0.3);
    this->declare_parameter("max_speed", 200);

    std::string port = this->get_parameter("port").as_string();
    std::string topic_pub = "write_" + port.substr(port.find_last_of("/")+1);
    std::string topic_sub = "serial_" + port.substr(port.find_last_of("/")+1);


    pub_motor_ = this->create_publisher<sgd_msgs::msg::Serial>(topic_pub,default_qos);
    sub_motor_ = this->create_subscription<sgd_msgs::msg::Serial>(topic_sub, default_qos,
        std::bind(&WH_Fcruiser::on_motor_received, this, std::placeholders::_1));
    pub_data_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("motordata", default_qos);
    sub_data_ = this->create_subscription<geometry_msgs::msg::TwistStamped>("set_motor", default_qos,
        std::bind(&WH_Fcruiser::on_data_received, this, std::placeholders::_1));


    double moto_kp = this->get_parameter("motor_kp").as_double();
    double max_speed = this->get_parameter("max_speed").as_int();
    wheel_r_controller_ = std::shared_ptr<PID_Controller>(new PID_Controller(moto_kp));
    wheel_l_controller_ = std::shared_ptr<PID_Controller>(new PID_Controller(moto_kp));
    wheel_r_controller_->set_max(max_speed);
    wheel_r_controller_->set_min(-max_speed);
    wheel_l_controller_->set_max(max_speed);
    wheel_l_controller_->set_min(-max_speed);

    regex_ = std::regex(this->get_parameter("msg_regex").as_string());

    timer_ = this->create_wall_timer(100ms, std::bind(&WH_Fcruiser::publish_motordata, this));
}

WH_Fcruiser::~WH_Fcruiser()
{
    // Destroy
    sgd_msgs::msg::Serial msg;
    msg.header.stamp = now();
    msg.msg = "0,0";
    pub_motor_->publish(msg);
}

void
WH_Fcruiser::publish_motordata()
{   
    set_r_speed_ += wheel_r_controller_->next(meas_r_);
    set_l_speed_ += wheel_l_controller_->next(meas_l_);

    //RCLCPP_INFO(get_logger(), "Set r: %f, l: %f", set_r_speed_, set_l_speed_);

    int steer = round(set_l_speed_ - set_r_speed_);
    int speed = round((set_r_speed_ + set_l_speed_) / 2);

    sgd_msgs::msg::Serial msg;
    msg.header.stamp = now();
    std::string m = std::to_string(steer) + "," + std::to_string(speed);
    //std::cout << "Steering command: " << m << std::endl;
    msg.msg = m;
    pub_motor_->publish(msg);
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
        time = std::stoi(matches[1]);
        meas_r_ = std::stod(matches[2]);
        meas_l_ = std::stod(matches[3]);
        v = std::stoi(matches[4]);
    } else { return; }

    // -> Winkelgeschwindigkeit
    double w_r_ = (meas_r_ - sig(meas_r_) * 50) / (173 * 0.68);
    double w_l_ = (meas_l_ - sig(meas_l_) * 50) / (175 * 0.68);

    geometry_msgs::msg::TwistStamped m;
    m.header.stamp.nanosec = time;  // TODO stimmt so nicht
    m.header.frame_id = "frame";
    m.twist.linear.x = ((w_r_ + w_l_) * 0.68) / 2;
    m.twist.angular.z = ((w_r_ - w_l_) * 0.68) / 0.71;
    pub_data_->publish(m);

    // check if voltage is okay
    if (v/100.0 < 34.0)
    {
        RCLCPP_WARN(get_logger(), "Battery voltage low!");
    }
}

void
WH_Fcruiser::on_data_received(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
    // speed and turn speed to rotations
    double w_r = (msg->twist.linear.x + (0.355 * msg->twist.angular.z)) / 0.68;
    double w_l = (2 * msg->twist.linear.x) / 0.68 - w_r;
    
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
    std::shared_ptr<rclcpp::Node> node = std::make_shared<sgd_lc::WH_Fcruiser>();

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Motor driver startup completed.");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
