// Copyright 2021 HAW Hamburg
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "sgd_lifecycle_manager/lifecycle_manager.hpp"

namespace sgd_lifecycle
{

using namespace std::chrono_literals;   // if a timer is used

Lifecycle_Manager::Lifecycle_Manager():
    Node("lifecycle_manager")
{
    launch_file = declare_parameter<std::string>("launch_file", "/config/launch.xml");
    get_parameter("use_sim_time", is_sim_);

    LF_Node_Factory lf(launch_file, is_sim_);
    lf.import_launch_file();

    // init subscriber to node_exit publisher
    std::function<void(std::shared_ptr<lifecycle_msgs::msg::TransitionEvent>)> fnc1 = std::bind(
            &Lifecycle_Manager::on_node_transition_received, this, std::placeholders::_1, (std::string)"#");
    sub_node_transition.push_back(this->create_subscription<lifecycle_msgs::msg::TransitionEvent>("node_exit",
                rclcpp::QoS(rclcpp::SystemDefaultsQoS()), fnc1));

    // change node state to configure
    lifecycle_node * nd;
    while ((nd = lf.next_node()) != nullptr)
    {
        // create client, change state
        service_clients_.insert({
            nd->get_node_name(),
            this->create_client<lifecycle_msgs::srv::ChangeState>(nd->get_node_name() + "/change_state")
        });

        // create subscription
        std::function<void(std::shared_ptr<lifecycle_msgs::msg::TransitionEvent>)> fnc = std::bind(
            &Lifecycle_Manager::on_node_transition_received, this, std::placeholders::_1, (std::string)nd->get_node_name());

        sub_node_transition.push_back(this->create_subscription<lifecycle_msgs::msg::TransitionEvent>(nd->get_node_name() + "/transition_event",
                    rclcpp::QoS(rclcpp::SystemDefaultsQoS()), fnc));

        RCLCPP_INFO(get_logger(), "Configure node %s", nd->get_node_name().c_str());
        if(change_state(Transition::TRANSITION_CONFIGURE, nd->get_node_name()))
        {
            nd->state = Transition::TRANSITION_CONFIGURE;
        }
    }

    int num_nodes = service_clients_.size();
    int node = 1;

    int retries = 0;
    rclcpp::WallRate loop_rate(10);
    while (lf.get_lowest_state() < Transition::TRANSITION_ACTIVATE && retries < 10)
    {
        retries++;
        lifecycle_node * nd;
        while ((nd = lf.next_node()) != nullptr)
        {
            if (nd->state == Transition::TRANSITION_ACTIVATE)
            {
                continue;
            }
            RCLCPP_INFO(get_logger(), "Activate node %d/%d: %s", node++, num_nodes, nd->get_node_name().c_str());
            if(change_state(Transition::TRANSITION_ACTIVATE, nd->get_node_name()))
            {
                nd->state = Transition::TRANSITION_ACTIVATE;
            }
            else
            {
                RCLCPP_WARN(get_logger(), "Could not activate node %s", nd->get_node_name().c_str());
                return;
            }
        }

        if (!loop_rate.sleep())
        {
            RCLCPP_INFO(get_logger(), "Lifecycle Manager missed frequency.");
        }
    }

    RCLCPP_INFO(get_logger(), "All nodes active.");

    // create timer and check node state every 5? seconds
    // timer_ = this->create_wall_timer(10ms, std::bind(&Lifecycle_Manager::check_state, this));
}

Lifecycle_Manager::~Lifecycle_Manager()
{
    // Destroy
}

bool
Lifecycle_Manager::change_state(uint8_t transition_id, std::string node_name)
{
    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id = transition_id;

    auto srv_client = service_clients_.at(node_name);

    try
    {
        uint8_t retries = 0;
        while (!srv_client->wait_for_service(1s)) {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                return false;
            }
            else if (retries > 5)
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Maximum number of retries reached. Exiting.");
                return false;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
            retries++;
        }
    }
    catch(const std::runtime_error& e)
    {
        std::cerr << e.what() << '\n';
    }

    auto result = srv_client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS)
    {
        return result.get()->success;
    }

    return false;
}

void
Lifecycle_Manager::on_node_transition_received(const lifecycle_msgs::msg::TransitionEvent::SharedPtr msg, std::string node_name)
{
    // register node transition event
    // get current state from unordered_map and compare to received state
    // if received state > current state -> alles gut
    // if received state < current state -> schlecht

    if (node_name == "#")
    {
        // process of node has died
        RCLCPP_INFO(get_logger(), "Received transition: node %s from %d to %d", msg->transition.label, msg->start_state.id, msg->goal_state.id);    
    }
    else
    {
        RCLCPP_INFO(get_logger(), "Received transition: node %s from %d to %d", node_name.c_str(), msg->start_state.id, msg->goal_state.id);
    }
    lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE;
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
}

}   // namespace sgd_lifecycle

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<sgd_lifecycle::Lifecycle_Manager>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
}
