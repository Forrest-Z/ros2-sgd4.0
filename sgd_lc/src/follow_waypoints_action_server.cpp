#include "sgd_lc/follow_waypoints_action_server.hpp"

namespace nav_sgd
{

using namespace std::chrono_literals;

Follow_Waypoints_Action_Server::~Follow_Waypoints_Action_Server()
{
    // Destroy
}

nav2_util::CallbackReturn
Follow_Waypoints_Action_Server::on_configure(const rclcpp_lifecycle::State & state)
{
    RCLCPP_INFO(get_logger(), "Configuring");

    // Initialize parameters, pub/sub, services, etc.
    init_parameters();
    init_pub_sub();
    init_transforms();

    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
Follow_Waypoints_Action_Server::on_activate(const rclcpp_lifecycle::State & state)
{
    RCLCPP_INFO(get_logger(), "Activating");

    pub_motor_->on_activate();

    return wait_for_transform();
}

nav2_util::CallbackReturn
Follow_Waypoints_Action_Server::on_deactivate(const rclcpp_lifecycle::State & state)
{
    RCLCPP_INFO(get_logger(), "Deactivating");

    pub_motor_->on_deactivate();

    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
Follow_Waypoints_Action_Server::on_cleanup(const rclcpp_lifecycle::State & state)
{
    RCLCPP_INFO(get_logger(), "Cleanup");

    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
Follow_Waypoints_Action_Server::on_shutdown(const rclcpp_lifecycle::State & state)
{
    RCLCPP_INFO(get_logger(), "Shutdown");
    pub_motor_.reset();
    return nav2_util::CallbackReturn::SUCCESS;
}

// Action server methods
rclcpp_action::GoalResponse
Follow_Waypoints_Action_Server::handle_goal(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const WaypointFollower::Goal> goal)
{
    RCLCPP_INFO(get_logger(), "Received goal with %d waypoints", goal->poses.size());
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
Follow_Waypoints_Action_Server::handle_cancel(const std::shared_ptr<GoalHandleWaypointFollower> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void
Follow_Waypoints_Action_Server::handle_accepted(const std::shared_ptr<GoalHandleWaypointFollower> goal_handle)
{
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&Follow_Waypoints_Action_Server::execute, this, _1), goal_handle}.detach();
}

void
Follow_Waypoints_Action_Server::execute(const std::shared_ptr<GoalHandleWaypointFollower> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(rate_);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<WaypointFollower::Feedback>();
    std::vector<int> missed_waypoints_;
    auto result = std::make_shared<WaypointFollower::Result>();

    if (goal->poses.size() == 0)
    {
        result->missed_waypoints = missed_waypoints_;
        goal_handle->succeed(result);
        RCLCPP_INFO(get_logger(), "Goal reached.");
        return;
    }

    // initialize variables
    uint i = 0;
    double ref_x = goal->poses[0].pose.position.x;
    double ref_y = goal->poses[0].pose.position.y;
    double distance_ = 0.0, diff_angle_ = 0.0;
    double speed = 0.0, rot_speed = 0.0;
    double speed_integral_ = 0.0;
    double turn_integral_ = 0.0;
    bool goal_reached = false;
    bool skip_speed_calc = true;
    while (rclcpp::ok() && !goal_reached)
    {
        // if goal is cancelled
        if (goal_handle->is_canceling()) {
            publish_motordata(0,0);
            result->missed_waypoints = missed_waypoints_;
            goal_handle->canceled(result);
            RCLCPP_INFO(this->get_logger(), "Goal canceled");
            return;
        }

        // get current position
        try
        {
            geometry_msgs::msg::TransformStamped tf_base_map = tf_buffer_->lookupTransform("map", "base_footprint",
                        rclcpp::Time(0), rclcpp::Duration(5,0));
            
            // calculate angle from quaternion
            tf2::Quaternion c, r;
            tf2::fromMsg(tf_base_map.transform.rotation, c);
            r.setRPY(0,0,atan2(ref_y - tf_base_map.transform.translation.y,
                                ref_x - tf_base_map.transform.translation.x));

            double tmp_diff_angle = r.angleShortestPath(c);
            if (!skip_speed_calc)  rot_speed = (diff_angle_ - tmp_diff_angle) * rate_;  // rad/s
            diff_angle_ = tmp_diff_angle;

            double tmp_distance_ = sqrt(pow(tf_base_map.transform.translation.x - ref_x, 2)
                            +pow(tf_base_map.transform.translation.y - ref_y, 2));
            if (!skip_speed_calc)  speed = (distance_ - tmp_distance_) * rate_; // m/s
            distance_ = tmp_distance_;
            skip_speed_calc = false;
            //RCLCPP_INFO(get_logger(), "Diff_Angle/Distance (ist): %f, %f", diff_angle_, distance_);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "%s", ex.what());
            return;
        }

        // calculate desired speed
        // Wenn distance < (Bremsweg + goal_tolerance), dann runterregeln
        // Bremsweg bei konstanter Beschleunigung von -max_accel_: s=v^2/(2*a)
        double soll_speed_ = (distance_ > (pow(speed, 2)/(2*max_accel_)+xy_goal_tolerance_) ? max_speed_ : speed-max_accel_*1/rate_);

        double abw = soll_speed_ - speed;   // Abweichung
        speed_integral_ += abw * 1/rate_;
        double next_speed_ = speed + abw * speed_kp_ + speed_integral_ * speed_ki_;
        if (next_speed_ > speed+max_accel_ * 1/rate_)
        {
            next_speed_ = speed + max_accel_ * 1/rate_;
        }
        //RCLCPP_INFO(get_logger(), "Speed (i,s,n): %f, %f, %f", speed, soll_speed_, next_speed_);

        // calculate desired turn rate
        double soll_turn_ = diff_angle_;
        if (soll_turn_ > max_rot_speed_)
        {
            soll_turn_ = max_rot_speed_;
        }
        else if (soll_turn_ < -max_rot_speed_)
        {
            soll_turn_ = -max_rot_speed_;
        }
        double abwt = soll_turn_ - rot_speed;
        turn_integral_ += abwt * 1/rate_;
        double next_turn = abwt * turn_kp_ + turn_integral_ * turn_ki_;

        //RCLCPP_INFO(get_logger(), "Turn (i,s,n): %f, %f, %f", rot_speed, soll_turn_, next_turn);
        //publish_motordata(next_speed_, diff_angle_);
        publish_motordata(0.1, 0.1);

        if (distance_ < xy_goal_tolerance_)
        {
            // get next goal pose
            i++;
            if (i >= goal->poses.size())
            {
                // Ende
                RCLCPP_INFO(get_logger(), "Ende!");
                publish_motordata(0,0);
                goal_reached = true;
                break;
            }

            // set new reference position
            ref_x = goal->poses[i].pose.position.x;
            ref_y = goal->poses[i].pose.position.y;

            skip_speed_calc = true;
            feedback->current_waypoint = i;
            goal_handle->publish_feedback(feedback);
        }

        loop_rate.sleep();
    }

    // Check if goal is done
    if (rclcpp::ok()) {
        result->missed_waypoints = missed_waypoints_;
        publish_motordata(0,0);
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
}

void
Follow_Waypoints_Action_Server::init_parameters()
{
    get_parameter("rate", rate_);
    get_parameter("out_topic", out_topic_);
    get_parameter("speed_kp", speed_kp_);
    get_parameter("speed_ki", speed_ki_);
    get_parameter("turn_kp", turn_kp_);
    get_parameter("turn_ki", turn_ki_);
    get_parameter("max_speed", max_speed_);
    get_parameter("max_rot_speed", max_rot_speed_);
    get_parameter("max_accel", max_accel_);
    get_parameter("xy_goal_tolerance", xy_goal_tolerance_);
}

void
Follow_Waypoints_Action_Server::init_pub_sub()
{
    pub_motor_ = this->create_publisher<geometry_msgs::msg::Twist>(out_topic_, default_qos);
    sub_position = create_subscription<sgd_msgs::msg::SpeedAccel>("speed_accel", default_qos,
                    std::bind(&Follow_Waypoints_Action_Server::on_position_received, this, std::placeholders::_1));
    //sub_touch_ = this->create_subscription<sgd_msgs::msg::Touch>("handle_touch", default_qos,
    //    std::bind(&Local_Controller::on_touch_received, this, std::placeholders::_1));
    //sub_laser_ = this->create_subscription<sensor_msgs::msg::Range>("laser_1d", default_qos,
    //    std::bind(&Local_Controller::on_laser_received, this, std::placeholders::_1));
}

void
Follow_Waypoints_Action_Server::init_transforms()
{
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(rclcpp_node_->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        rclcpp_node_->get_node_base_interface(),
        rclcpp_node_->get_node_timers_interface());
    tf_buffer_->setCreateTimerInterface(timer_interface);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

nav2_util::CallbackReturn
Follow_Waypoints_Action_Server::wait_for_transform()
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
Follow_Waypoints_Action_Server::publish_motordata(double vel, double turn)
{
    geometry_msgs::msg::Twist msg;
    msg.linear.x = vel;
    msg.angular.z = turn;
    pub_motor_->publish(msg);
}

void
Follow_Waypoints_Action_Server::on_position_received(const sgd_msgs::msg::SpeedAccel::SharedPtr msg)
{
    last_spdacc_ = *msg;
    RCLCPP_INFO(get_logger(), "Position (xp,yp,xpp,ypp): %f, %f, %f, %f",
            last_spdacc_.twist.linear.x, last_spdacc_.twist.linear.y,
            last_spdacc_.accel.linear.x, last_spdacc_.accel.linear.y);
    
    RCLCPP_INFO(get_logger(), "Orientation (wp,wpp): %f, %f",
            last_spdacc_.twist.angular.x, last_spdacc_.accel.angular.x);
}

}   // namespace

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<nav_sgd::Follow_Waypoints_Action_Server>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
}
