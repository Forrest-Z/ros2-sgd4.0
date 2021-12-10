
#include "sgd_safety/lidar_obstacle_checker.hpp"

namespace sgd_safety
{

using std::placeholders::_1;

Lidar_Obstacle_Checker::Lidar_Obstacle_Checker() 
        : nav2_util::LifecycleNode("lidar_obstacle_checker", "", true)
{
    RCLCPP_DEBUG(get_logger(), "Creating");

    // Add parameters
    declare_parameter("scan_topic", rclcpp::ParameterValue("scan"));
    declare_parameter("cmd_vel_contr_topic", rclcpp::ParameterValue("cmd_vel"));
    declare_parameter("sgd_move_topic", rclcpp::ParameterValue("cmd_vel_lidar"));

}

Lidar_Obstacle_Checker::~Lidar_Obstacle_Checker()
{
    // Destroy
}

nav2_util::CallbackReturn
Lidar_Obstacle_Checker::on_configure(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "Configure");

    // init
    get_parameter("scan_topic", scan_topic_);
    get_parameter("cmd_vel_contr_topic", cmd_vel_contr_topic_);
    get_parameter("sgd_move_topic", cmd_vel_topic_);

    init_pub_sub();

    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
Lidar_Obstacle_Checker::on_activate(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "Activate");

    pub_cmd_vel->on_activate();

    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
Lidar_Obstacle_Checker::on_deactivate(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "Deactivate");
    pub_cmd_vel->on_deactivate();
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
Lidar_Obstacle_Checker::on_cleanup(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "Cleanup");
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
Lidar_Obstacle_Checker::on_shutdown(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "Shutdown");
    return nav2_util::CallbackReturn::SUCCESS;
}

void
Lidar_Obstacle_Checker::init_pub_sub()
{
    pub_cmd_vel = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, default_qos);

    sub_scan_ = create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic_, default_qos, std::bind(&Lidar_Obstacle_Checker::on_scan_received, this, _1));
    sub_cmd_vel_contr_ = create_subscription<geometry_msgs::msg::Twist>(
        cmd_vel_contr_topic_, default_qos, std::bind(&Lidar_Obstacle_Checker::on_cmd_vel_contr_received, this, _1));
}

void
Lidar_Obstacle_Checker::on_cmd_vel_contr_received(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    //last_cmd_vel_ = *msg;
}

void
Lidar_Obstacle_Checker::on_scan_received(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    // check scan for point with distance < 0.5m
    // publish new cmd_vel
    float angle_ = msg->angle_min - msg->angle_increment;
    float angle_max = msg->angle_max;
    float incr_ = msg->angle_increment;

    int last_dist_ = 0;

    int i = 0;
    for (float dist : msg->ranges)
    {
        angle_ += incr_;
        // ignore all distances > maximum distance and < minimum distance
        if (dist > msg->range_max || dist < msg->range_min || abs(angle_) > M_PI/6)
        {
            last_dist_ = 0;
            continue;
        }

        //if (abs(sin(angle_) * dist) < 1.0)
        //{
            i++;
            // relevant point
            //double x_dist_ = cos(angle_) * dist;
            if (dist < 0.8)
            {
                last_dist_ = last_dist_ > 8 ? last_dist_ : last_dist_ + 1;
                if (last_dist_ >= 8)
                {
                    //RCLCPP_INFO(get_logger(), "Publish command to stop.");
                    double speed = 0.0;
                    geometry_msgs::msg::Twist cmd_vel;
                    cmd_vel.linear.x = 0.0;
                    cmd_vel.angular.z = 0.0;
                    pub_cmd_vel->publish(cmd_vel);
                    return;
                    // publish and return
                }
                // stop if last dist < 0.5
            }
            //else if (x_dist_ > 2.5)
            //{
            //    continue; 
            //}
            else
            {
                double speed_ = 0.8 * dist - 0.25;
                last_dist_ = last_dist_ < 2 ? 0 : last_dist_-2;
            }

            
       // }
    }
    //RCLCPP_INFO(get_logger(), "Found %i relevant scan points.", i);
}

}   // namespace sgd_lc

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<nav2_util::LifecycleNode> node = std::make_shared<sgd_safety::Lidar_Obstacle_Checker>();

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Lidar obstacle checker startup completed.");

    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}
