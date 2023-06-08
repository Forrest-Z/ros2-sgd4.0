
#include "sgd_global_planner/navigator.hpp"


namespace sgd_global_planner
{

using namespace std::chrono_literals;
using std::placeholders::_1;

Navigator::Navigator() : Node("navigator")
{
    // Constructor

    // Create Subscriber for published points -> /clicked_point (PointStamped)
    // Create subscriber for gps node -> /gpspos (PointStamped)
    // Create service client to compute path

    auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
    sub_gps_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "gps", default_qos, std::bind(&Navigator::sub_gps_handler, this, _1));

    sub_clicked_point_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "clicked_point", default_qos, std::bind(&Navigator::sub_clicked_point_handler, this, _1));

    node_ = std::make_shared<rclcpp::Node>("navigator_");
    compute_path_client_ = node_->create_client<sgd_msgs::srv::ComputePath>("compute_path");

    gps_lat_ = 0.0;
    gps_lon_ = 0.0;
}

Navigator::~Navigator()
{
    // Destructor
}

void
Navigator::sub_gps_handler(const sensor_msgs::msg::NavSatFix::SharedPtr msg_)
{
    // Store gps pos
    gps_lon_ = msg_->longitude;
    gps_lat_ = msg_->latitude;
}

void
Navigator::sub_clicked_point_handler(const geometry_msgs::msg::PointStamped::SharedPtr msg_)
{
    auto latlon = sgd_util::local_to_WGS84(msg_->point.x, msg_->point.y);
    clicked_point_lat_ = latlon.first;
    clicked_point_lon_ = latlon.second;

    if (gps_lon_ == 0.0 || gps_lat_ == 0.0)
    {
        RCLCPP_ERROR(this->get_logger(), "No valid gps position received. Wait for gps fix and try again!");
        return;
    }

    auto request = std::make_shared<sgd_msgs::srv::ComputePath::Request>();
    request->lata = gps_lat_;
    request->lona = gps_lon_;
    request->latb = clicked_point_lat_;
    request->lonb = clicked_point_lon_;

    RCLCPP_INFO(this->get_logger(), "Sending compute path request with parameters lata: %.8f, lona: %.8f, latb: %.8f, lonb: %.8f.",
        request->lata, request->lona, request->latb, request->lonb);

    while (!compute_path_client_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
    }

    auto result = compute_path_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(this->get_logger(), "Compute path completed successfully.");
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to call service compute_path.");
    }
}

}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = std::make_shared<nav_sgd::Navigator>();

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Navigator startup completed.");

    rclcpp::spin(node);
    rclcpp::shutdown();
}
