#include "sgd_safety/lidar_obstacle_checker.hpp"

#define deg2rad(ang) (ang * M_PI / 180)
#define rad2deg(ang) (ang * 180 / M_PI)

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
    Lidar_Obstacle_Checker::on_configure(const rclcpp_lifecycle::State & state __attribute__((unused)))
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
    Lidar_Obstacle_Checker::on_activate(const rclcpp_lifecycle::State & state __attribute__((unused)))
    {
        RCLCPP_DEBUG(get_logger(), "Activate");

        pub_cmd_vel->on_activate();

        return nav2_util::CallbackReturn::SUCCESS;
    }

    nav2_util::CallbackReturn
    Lidar_Obstacle_Checker::on_deactivate(const rclcpp_lifecycle::State & state __attribute__((unused)))
    {
        RCLCPP_DEBUG(get_logger(), "Deactivate");
        pub_cmd_vel->on_deactivate();
        return nav2_util::CallbackReturn::SUCCESS;
    }

    nav2_util::CallbackReturn
    Lidar_Obstacle_Checker::on_cleanup(const rclcpp_lifecycle::State & state __attribute__((unused)))
    {
        RCLCPP_DEBUG(get_logger(), "Cleanup");
        return nav2_util::CallbackReturn::SUCCESS;
    }

    nav2_util::CallbackReturn
    Lidar_Obstacle_Checker::on_shutdown(const rclcpp_lifecycle::State & state __attribute__((unused)))
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
        last_cmd_vel_ = *msg;
        cmd_vel_time = now().seconds();
    }

    /**
     * Uses Lidar measurements to determine the speed of the robot according to the obstacles of the environment.
     * It also determines if there is enough space for the robot to continue or if it should stop to avoid collision.
     * <p>
     * @param LaserScan    Lidar sensor measurements
     * @param return       none
     */

    void
    Lidar_Obstacle_Checker::on_scan_received(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        /******* Variables Declaration *******/

        // Angles
        float angle_ = msg->angle_min;
        float incr_ = msg->angle_increment;

        // Robot geometry
        const float robot_width = 0.73;         // Width of the robot in m including the wheels  //FIXME Approximate measurement. Verify!
        const float width_tolerance = 0.1;      // A tolerance on the robot width in m for an extra safety factor.
        float half_width = (robot_width / 2) + width_tolerance;  //Distance from the center of the robot to the wheel plus a tolerance.

        // Distances
        float safe_distance = half_width;   // Initialized temporarily as the robot's half width, but it will change according to the angle.
        float distance_nearest_obstacle = 3.1;// This variable will store the distance to the closest object. It is initialized at a distance larger than distance_max.
        const float distance_min = 0.5;           // Minimal distance the robot has to keep with respect to obstacles
        const float distance_max = 1.5;             // At any distance smaller than this, the robot should start decreasing its speed.

        // Speeds
        const float speed_max = 1;                // Maximum speed in m/s
        double speed = 0.0;

        // Obstacles
        uint8_t obstacle_ocurrences = 0;    // a counter to verify obstacle occurrences
        const uint8_t min_ocurrences = 6;   // minimum number of ocurrences before it can be considered an obstacle

        // Workspace
        float workspace_front_angle = atan2(half_width, distance_min); // This is the angle where objects only modify the speed of the robot, they don't stop it.
        const float workspace_lower_limit = 2;  //Vaiable to disregard the robot's body as an obstacle


        /********** Main loop **********/

        /* Evaluates the distances the Lidar measures at every angle */
        for (float current_distance : msg->ranges)
        {
            /* If the angle is too big, t is detecting its own body. If the distance is too small, it is a false measurement regarding an issue with the lidar sensor.*/
            if ((abs(angle_) > workspace_lower_limit) || (current_distance < 0.009))
                {} // Discard this measurements because it's detecting itself.
            /* If there is an object outside the workspace angle, the distance of such objects must be verified to avoid collisions*/
            else if(abs(angle_) > workspace_front_angle)
            {
                /* This equation determines if there is enough room for the robot to pass without colliding */
                safe_distance = abs(half_width / sin(angle_));
                if (current_distance < safe_distance)
                {
                    obstacle_ocurrences++;
                    if(obstacle_ocurrences > min_ocurrences)        // If certain number of measurements confirmed an obstacle.
                    {
                        distance_nearest_obstacle = current_distance;
                        //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Distance: %f is smaller than SD %f at %f.", current_distance, safe_distance, degrees);
                        speed = 0.0;
                    }
                }
                else
                {
                    obstacle_ocurrences = 0;                // Reset counter
                }
            }
            else    /* If there are no obstacles on the sides that can lead to a collision, then the speed is determined by the nearest obstacle. */
            {
                if(current_distance < distance_nearest_obstacle)
                {
                    distance_nearest_obstacle = current_distance;
                }
                /* This equation calculates the speed at which the robot should move as a function of the distance between the robot and the object. */
                speed = ((distance_nearest_obstacle - distance_min) / (distance_max - distance_min)) * speed_max;
                if(speed < 0)
                    speed = 0;
                else if (speed > speed_max)
                    speed = speed_max;
            }
            angle_ += incr_;
        }

        /* After it has been verified that no obstacles interfere with the robot's trajectory, the calculated speed is set  */
        last_cmd_vel_.linear.x *= speed;
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel = last_cmd_vel_;
        if(speed == 0)
        {
            cmd_vel.angular.z = 0.0;
            cmd_vel.linear.x = 0.0;
            pub_cmd_vel->publish(cmd_vel);
        }
        else if ((now().seconds() - cmd_vel_time) < 1)
        {
            pub_cmd_vel->publish(cmd_vel);
            /*RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Nearest obstacle at: %f m and %f degrees.", distance_nearest_obstacle, degrees);
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Speed: %f", speed);*/
        }



    }

}   // namespace sgd_safety

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<nav2_util::LifecycleNode> node = std::make_shared<sgd_safety::Lidar_Obstacle_Checker>();

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Lidar obstacle checker startup completed.");

    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}
