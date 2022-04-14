#include "sgd_global_planner/global_planner.hpp"

namespace nav_sgd
{

using std::placeholders::_1;
using namespace std::chrono_literals;

Global_Planner_OSM::Global_Planner_OSM():
        rclcpp_lifecycle::LifecycleNode("global_planner_osm") 
{
    RCLCPP_DEBUG(get_logger(), "Creating");

    // declare parameters and default values
    declare_parameter("waypoints_topic", rclcpp::ParameterValue("global_plan"));
    declare_parameter("clicked_point_topic", rclcpp::ParameterValue("clicked_point"));
    declare_parameter("yaml_filename", rclcpp::ParameterValue("map.yaml"));
}

Global_Planner_OSM::~Global_Planner_OSM()
{
    // Destroy
}

CallbackReturn
Global_Planner_OSM::on_configure(const rclcpp_lifecycle::State & state __attribute__((unused)))
{
    RCLCPP_INFO(get_logger(), "Configuring");

    // Initialize parameters, pub/sub, services, etc.
    get_parameter("waypoints_topic", waypoints_topic_);
    get_parameter("clicked_point_topic", clicked_point_topic_);
    get_parameter("yaml_filename", yaml_filename_);

    init_transforms();
    init_yaml();

    // check if file exists
    FILE * mFile, * uFile;
    mFile = fopen(map_filename.c_str(), "r");
    uFile = fopen(users_filename.c_str(), "r");
    if (mFile == NULL || uFile == NULL)
    {
        RCLCPP_ERROR(get_logger(), "Map file %s or users file %s does not exist!", map_filename.c_str(), users_filename.c_str());
        return CallbackReturn::FAILURE;
    }

    init_pub_sub();
    
    RCLCPP_INFO(get_logger(), "Configuring complete");

    return CallbackReturn::SUCCESS;
}

CallbackReturn
Global_Planner_OSM::on_activate(const rclcpp_lifecycle::State & state __attribute__((unused)))
{
    RCLCPP_INFO(get_logger(), "Activating");

    a_star_users = std::make_shared<A_Star_Users>(users_filename);
    a_star = std::make_unique<A_Star>(map_filename, a_star_users);

    publisher_path_->on_activate();

    return wait_for_transform();
}

CallbackReturn
Global_Planner_OSM::on_deactivate(const rclcpp_lifecycle::State & state __attribute__((unused)))
{
    RCLCPP_DEBUG(get_logger(), "Deactivating");
    publisher_path_->on_deactivate();
    
    return CallbackReturn::SUCCESS;
}

CallbackReturn
Global_Planner_OSM::on_cleanup(const rclcpp_lifecycle::State & state __attribute__((unused)))
{
    RCLCPP_DEBUG(get_logger(), "Cleanup");
    publisher_path_.reset();
    map_info_srv.reset();
    compute_path_srv.reset();

    return CallbackReturn::SUCCESS;
}

CallbackReturn
Global_Planner_OSM::on_shutdown(const rclcpp_lifecycle::State & state __attribute__((unused)))
{
    RCLCPP_DEBUG(get_logger(), "Shutdown");
    return CallbackReturn::SUCCESS;
}

void
Global_Planner_OSM::init_pub_sub()
{
    publisher_path_ = this->create_publisher<nav_msgs::msg::Path>(waypoints_topic_, default_qos);
    
    //sub_clicked_point_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    //    clicked_point_topic_, default_qos, std::bind(&Global_Planner_OSM::computePath, this, _1));

    RCLCPP_INFO(get_logger(), "Initialised publisher on topic %s",
            waypoints_topic_.c_str());

    // create compute path service
    map_info_srv = create_service<sgd_msgs::srv::GetMapInfo>("get_map_info",
        std::bind(&Global_Planner_OSM::getMapInfo, this, _1, _2));
    compute_path_srv = create_service<sgd_msgs::srv::GetGlobalPlan>("get_global_plan",
        std::bind(&Global_Planner_OSM::computePath, this, _1, _2));
    
    // create waypoint follower action
    // auto options = rclcpp::NodeOptions().arguments(
    //     {"--ros-args --remap __node:=navigation_dialog_action_client"});
    // client_node_ = std::make_shared<rclcpp::Node>("_follow", options);
    // waypoint_follower_action_client_ = 
    //     rclcpp_action::create_client<nav2_msgs::action::FollowWaypoints>(
    //     client_node_,
    //     "FollowWaypoints");
    // waypoint_follower_goal_ = nav2_msgs::action::FollowWaypoints::Goal();

    server_timeout_ = std::chrono::milliseconds(10);
    RCLCPP_INFO(get_logger(), "Publisher and subscriber initialized.");
}

void
Global_Planner_OSM::init_transforms()
{
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        get_node_base_interface(),
        get_node_timers_interface());
    tf_buffer_->setCreateTimerInterface(timer_interface);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

CallbackReturn
Global_Planner_OSM::init_yaml()
{
    YAML::Node doc = YAML::LoadFile(yaml_filename_);

    // get directory from yaml filename
    std::string base_path = yaml_filename_.substr(0, yaml_filename_.find_last_of("/")+1);

    try
    {
        map_filename = yaml_get_value<std::string>(doc, "osm_map_file");
        adr_filename = yaml_get_value<std::string>(doc, "address_file");
        users_filename = yaml_get_value<std::string>(doc, "users_file");
    }
    catch(const YAML::Exception& e)
    {
        RCLCPP_ERROR(get_logger(), "Error reading yaml file: %s", e.what());
        return CallbackReturn::FAILURE;
    }

    // if all filenames were parsed successfully add base path to get an absolute path
    map_filename = base_path + map_filename;
    adr_filename = base_path + adr_filename;
    users_filename = base_path + users_filename;

    return CallbackReturn::SUCCESS;
}

CallbackReturn
Global_Planner_OSM::wait_for_transform()
{
    // Wait for transform to be available
    RCLCPP_INFO(get_logger(), "Wait for transform");

    std::string err;
    int retries = 0;
    while (rclcpp::ok() && !tf_buffer_->canTransform("earth", "map", tf2::TimePointZero, tf2::durationFromSec(0.1), &err)
        && retries < 10)
    {
        RCLCPP_INFO(this->get_logger(), "Timeout waiting for transform. Tf error: %s", err);
        err.clear();
        rclcpp::sleep_for(500000000ns);
        retries++;
    }

    if (retries > 9)
    {
        return CallbackReturn::FAILURE;
    }

    // transformation from earth -> map in WGS84 coordinates
    // according to REP-105 the x-axis points east (lon) and the y-axis north (lat)
    auto tf_ = tf_buffer_->lookupTransform("earth", "map", rclcpp::Time(0), rclcpp::Duration(5,0));
    map_origin.set_global_coordinates(tf_.transform.translation.y, tf_.transform.translation.x);

    RCLCPP_INFO(get_logger(), "Set map origin to %s", map_origin.to_string().c_str());

    return CallbackReturn::SUCCESS;
}

void
Global_Planner_OSM::getMapInfo(const std::shared_ptr<sgd_msgs::srv::GetMapInfo::Request> request,
                               std::shared_ptr<sgd_msgs::srv::GetMapInfo::Response> response)
{
    // read address file and user file
    RCLCPP_INFO(get_logger(), "Service get_map_info callback");

    // read address file
    std::string line, json = "";
    std::ifstream infile(adr_filename);
    while (std::getline(infile, line))
    {
        json.append(line);
    }
    infile.close();

    response->address_json = json;
    response->usernames = a_star_users->get_user_list();
    RCLCPP_INFO(get_logger(), "Service response filled.");
}

void
Global_Planner_OSM::computePath(const std::shared_ptr<sgd_msgs::srv::GetGlobalPlan::Request> request,
                                std::shared_ptr<sgd_msgs::srv::GetGlobalPlan::Response> response)
{
    // get current position in WGS84 coordinates
    RCLCPP_INFO(get_logger(), "Get current position");
    sgd_util::LatLon position;
    try
    {
        // transformation from map -> base_link in local cooridinates
        geometry_msgs::msg::TransformStamped tf_ = tf_buffer_->lookupTransform("map", "base_link",
                    rclcpp::Time(0), rclcpp::Duration(5,0));
        position.set_local_coordinates(map_origin, tf_.transform.translation.x, tf_.transform.translation.y);        
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_WARN(this->get_logger(), "%s", ex.what());
        return;
    }

    // get destination from message. If multiple IDs are specified, multiple paths are calculated and the best one is returned.
    ROUTE best_route_;
    best_route_.cost = 1E9;

    for (int dest_id : request->dest_ids)
    {
        RCLCPP_INFO(get_logger(), "Compute path to node %d", dest_id);
        try
        {
            // set start and destination
            a_star->set_start(position);
            a_star->set_dest(dest_id);

            auto route = a_star->compute_path();
            if (route.cost < best_route_.cost)
            {
                best_route_ = route;
            }
            RCLCPP_INFO(get_logger(), "Route has length of %.3f m", route.length_m);
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
            RCLCPP_WARN(get_logger(), "Path computation error: %s", e.what());
        }
    }

    // we need at least two waypoints to get a valid path
    if (best_route_.waypoints.size() < 2)
    {
        RCLCPP_WARN(get_logger(), "No route available.");
        response->length = -1;
        return;
    }    

    RCLCPP_INFO(get_logger(), "Route has %d waypoints", best_route_.waypoints.size());
    response->length = best_route_.length_m;
    
    //Smoothen path
    path_smoothing path_object;
    auto smoothened_path = path_object.smoothen_path(best_route_.waypoints);

    
    // publish path
    auto poses = create_poses_from_waypoints(smoothened_path);
    publish_path(poses);
}

void
Global_Planner_OSM::publish_path(std::vector<geometry_msgs::msg::Pose> data)
{
    nav_msgs::msg::Path path_;

    path_.header.frame_id = "map";
    path_.header.stamp = this->get_clock()->now();

    for (auto p : data)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path_.header;

        pose.pose = p;
        path_.poses.push_back(pose);
    }
    publisher_path_->publish(path_);
}

std::vector<geometry_msgs::msg::Pose>
Global_Planner_OSM::create_poses_from_waypoints(std::vector<sgd_util::LatLon> waypoints)
{
    std::pair<double, double> last_wp{-1.0,-1.0};
    std::vector<geometry_msgs::msg::Pose> poses_list;
    for (std::size_t i = 0; i < waypoints.size(); i++)
    {
        auto xy = waypoints.at(i).to_local(map_origin);

        geometry_msgs::msg::Pose pose;
        pose.position.x = xy.first;
        pose.position.y = xy.second;
        pose.position.z = 0.0;

        // calculate angle
        double ang = 0.0;
        auto next_wp = (i < waypoints.size()-1) ? waypoints.at(i+1).to_local(map_origin) : waypoints.at(i-1).to_local(map_origin);
        if (last_wp.first < 0)  // local coordinates should always be positive
        {
            // no last waypoint available -> take angle to next waypoint
            ang = atan2(next_wp.second - xy.second, next_wp.first - xy.first);
        }
        else if (i >= (waypoints.size() - 1))
        {
            // no next waypoint available
            ang = atan2(xy.second - last_wp.second, xy.first - last_wp.first);
        }
        else
        {
            // last and next waypoint available
            ang = (atan2(next_wp.second - xy.second, next_wp.first - xy.first) + 
                   atan2(xy.second - last_wp.second, xy.first - last_wp.first)) / 2;
        }

        tf2::Quaternion q;
        q.setRPY(0,0,ang);
        pose.orientation = tf2::toMsg(q);

        poses_list.push_back(pose);
        last_wp = xy;
    }
    return poses_list;
}

}   // namespace nav_sgd

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<nav_sgd::Global_Planner_OSM>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
}

