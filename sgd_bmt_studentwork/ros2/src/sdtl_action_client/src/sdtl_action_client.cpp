//  HAW Hamburg
//  BMT6-Studienarbeit Shared Dog Project
//  Prof. Dr. Henner Gärtner
//  created by Helmer Barcos
//  helmer@barcos.co - https://barcos.co
//  Sommer Semester 2023

#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>
#include <random>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "./visibility_control.h"
#include "sdtl_interfaces/msg/sdtl_point.hpp"
#include "sdtl_interfaces/action/sdtl_pedestrian_traffic_light.hpp"


namespace sdtl_action_client
{
class ActionClient : public rclcpp::Node
{
public:
  using SDTLPoint = sdtl_interfaces::msg::SDTLPoint;
  using SDTLPedestrianTrafficLight = sdtl_interfaces::action::SDTLPedestrianTrafficLight;
  using GoalHandleSDTLPedestrianTrafficLight = rclcpp_action::ClientGoalHandle<SDTLPedestrianTrafficLight>;

  SDTL_ACTION_CLIENT_PUBLIC
  explicit ActionClient(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
  : Node("sdtl_action_client", node_options)
  {
    this->client_ptr_ = rclcpp_action::create_client<SDTLPedestrianTrafficLight>(
      this->get_node_base_interface(),
      this->get_node_graph_interface(),
      this->get_node_logging_interface(),
      this->get_node_waitables_interface(),
      "pedestrian_traffic_light");

    this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&ActionClient::send_goal, this));
  }

  SDTL_ACTION_CLIENT_PUBLIC
  void send_goal()
  {
    using namespace std::placeholders;

    this->timer_->cancel();

    if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
      return;
    }

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dis(-90.0, 90.0);
    auto goal_msg = SDTLPedestrianTrafficLight::Goal();
    std::vector<SDTLPoint> points; // Vector to store the SDTLPoint objects

     for (int i = 0; i < 1; ++i) {
        SDTLPoint point;
        point.latitude = dis(gen);
        point.longitude = dis(gen);
        points.push_back(point); // Add the point to the vector
    }
    
    goal_msg.route = points;

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<SDTLPedestrianTrafficLight>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&ActionClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
      std::bind(&ActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
      std::bind(&ActionClient::result_callback, this, _1);
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<SDTLPedestrianTrafficLight>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;

  SDTL_ACTION_CLIENT_LOCAL
  void goal_response_callback(std::shared_future<GoalHandleSDTLPedestrianTrafficLight::SharedPtr> future)
  {
    auto goal_handle = future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  SDTL_ACTION_CLIENT_LOCAL
  void feedback_callback(
    GoalHandleSDTLPedestrianTrafficLight::SharedPtr,
    const std::shared_ptr<const SDTLPedestrianTrafficLight::Feedback> feedback)
  {
    std::stringstream ss;
    ss << "next_change_at: ";
    for (auto number : feedback->next_change_at) {
      ss << number << " ";
    }
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
  }

  SDTL_ACTION_CLIENT_LOCAL
  void result_callback(const GoalHandleSDTLPedestrianTrafficLight::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }
    std::stringstream ss;
    ss << "Result received: ";
    // for (auto number : result.result->sequence) {
    //   ss << number << " ";
    // }
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    rclcpp::shutdown();
  }
};  // class ActionClient

}  // namespace sdtl_action_client

RCLCPP_COMPONENTS_REGISTER_NODE(sdtl_action_client::ActionClient)