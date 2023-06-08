#include "sgd_global_planner/global_planner.hpp"

namespace sgd_global_planner
{

using std::placeholders::_1;
using namespace std::chrono_literals;

Global_Planner_OSM::Global_Planner_OSM():
        rclcpp_lifecycle::LifecycleNode("global_planner_osm") 
{
    // declare parameters and default values
    declare_parameter("log_dir", rclcpp::ParameterValue(".ros/log/"));
    declare_parameter("log_severity", rclcpp::ParameterValue("I"));

    declare_parameter("waypoints_topic", rclcpp::ParameterValue("global_plan"));
    declare_parameter("clicked_point_topic", rclcpp::ParameterValue("clicked_point"));
    declare_parameter("yaml_filename", rclcpp::ParameterValue("map.yaml"));
    declare_parameter("map_info_srv", rclcpp::ParameterValue("get_map_info"));
    declare_parameter("global_plan_srv", rclcpp::ParameterValue("get_global_plan"));

    declare_parameter("global_frame", rclcpp::ParameterValue("earth"));
    declare_parameter("map_frame", rclcpp::ParameterValue("map"));
    declare_parameter("robot_base_frame", rclcpp::ParameterValue("base_link"));

    declare_parameter("visual_topic", rclcpp::ParameterValue("map_visualization"));

    // init logging
    std::string log_dir_, log_sev_;
    get_parameter("log_dir", log_dir_);
    get_parameter("log_severity", log_sev_);

    std::string log_file(log_dir_ + "/global_planner_osm.log");
    plog::init(plog::severityFromString(log_sev_.c_str()), log_file.c_str());
    PLOGI.printf("Created Global_Planner_OSM node. PLOG logging severity is %s", log_sev_.c_str());
    RCLCPP_INFO(get_logger(), "Created Global_Planner_OSM node. Save log file to %s", log_file.c_str());
}

Global_Planner_OSM::~Global_Planner_OSM()
{
    // Destroy
}

CallbackReturn
Global_Planner_OSM::on_configure(const rclcpp_lifecycle::State & state __attribute__((unused)))
{
    PLOGD << "Configuring...";

    init_yaml();

    // init parameters
    get_parameter("map_frame", map_frame_);
    get_parameter("robot_base_frame", robot_base_frame_);

    // check if file exists
    FILE * mFile, * uFile;
    mFile = fopen(map_filename.c_str(), "r");
    uFile = fopen(users_filename.c_str(), "r");
    if (mFile == NULL || uFile == NULL)
    {
        RCLCPP_ERROR(get_logger(), "Map file %s or users file %s does not exist!", map_filename.c_str(), users_filename.c_str());
        PLOGE.printf("Map file %s or users file %s does not exist!", map_filename.c_str(), users_filename.c_str());
        return CallbackReturn::FAILURE;
    }

    init_pub_sub();

    return CallbackReturn::SUCCESS;
}

CallbackReturn
Global_Planner_OSM::on_activate(const rclcpp_lifecycle::State & state __attribute__((unused)))
{
    PLOGD << "Activating...";

    a_star_users = std::make_shared<A_Star_Users>(users_filename);
    a_star = std::make_unique<A_Star>(map_filename, a_star_users);

    // read address file
    std::string line, json = "";
    std::ifstream infile(adr_filename);
    infile >> js;

    publisher_path_->on_activate();
    pub_map_visual_->on_activate();

    auto ret_code = wait_for_transform();

    pub_map_visual_->publish(create_map_visual());
    return ret_code;
}

CallbackReturn
Global_Planner_OSM::on_deactivate(const rclcpp_lifecycle::State & state __attribute__((unused)))
{
    PLOGD << "Deactivating...";
    publisher_path_->on_deactivate();
    pub_map_visual_->on_deactivate();
    
    return CallbackReturn::SUCCESS;
}

CallbackReturn
Global_Planner_OSM::on_cleanup(const rclcpp_lifecycle::State & state __attribute__((unused)))
{
    PLOGD << "Cleaning up...";
    publisher_path_.reset();
    pub_map_visual_.reset();
    map_info_srv.reset();
    compute_path_srv.reset();

    return CallbackReturn::SUCCESS;
}

CallbackReturn
Global_Planner_OSM::on_shutdown(const rclcpp_lifecycle::State & state __attribute__((unused)))
{
    PLOGD << "Shutting down...";
    return CallbackReturn::SUCCESS;
}

void
Global_Planner_OSM::init_pub_sub()
{
    std::string clicked_point_topic_ = get_parameter("clicked_point_topic").as_string();
    PLOGI.printf("Subscribe to %s", clicked_point_topic_.c_str());
    sub_clicked_pnt_ = this->create_subscription<geometry_msgs::msg::PointStamped>(clicked_point_topic_,
            default_qos, std::bind(&Global_Planner_OSM::on_clicked_pnt, this, _1));

    std::string waypoints_topic_ = get_parameter("waypoints_topic").as_string();
    PLOGI.printf("Create publisher on topic %s", waypoints_topic_.c_str());
    publisher_path_ = this->create_publisher<nav_msgs::msg::Path>(waypoints_topic_, default_qos);

    // create compute path service
    std::string map_info_srv_ = get_parameter("map_info_srv").as_string();
    PLOGI.printf("Create service on %s", map_info_srv_.c_str());
    map_info_srv = create_service<sgd_msgs::srv::GetMapInfo>(map_info_srv_,
        std::bind(&Global_Planner_OSM::getMapInfo, this, _1, _2));

    std::string global_plan_srv_ = get_parameter("global_plan_srv").as_string();
    PLOGI.printf("Create service on %s", global_plan_srv_.c_str());
    compute_path_srv = create_service<sgd_msgs::srv::GetGlobalPlan>(global_plan_srv_,
        std::bind(&Global_Planner_OSM::computePath, this, _1, _2));

    std::string visual_topic_ = get_parameter("visual_topic").as_string();
    PLOGI.printf("Create publisher on topic %s", visual_topic_.c_str());
    pub_map_visual_ = create_publisher<visualization_msgs::msg::Marker>(visual_topic_, default_qos);
}

CallbackReturn
Global_Planner_OSM::init_yaml()
{
    std::string yaml_filename_ = get_parameter("yaml_filename").as_string();
    PLOGD.printf("Init yaml file %s", yaml_filename_.c_str());
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
        PLOGE.printf("Error reading yaml file %s: %s", yaml_filename_.c_str(), e.what());
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
    std::string global_frame_ = get_parameter("global_frame").as_string();

    PLOGD.printf("Lookup transform from %s to %s", global_frame_.c_str(), map_frame_.c_str());
    
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        get_node_base_interface(),
        get_node_timers_interface());
    tf_buffer_->setCreateTimerInterface(timer_interface);
    tf_buffer_->setUsingDedicatedThread(true);
    // Warum wird der Listener hier benötigt?? Ohne den kann der Transform nicht bestimmt werden
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    std::string err;
    int retries = 0;
    while (rclcpp::ok() && !tf_buffer_->canTransform(global_frame_, map_frame_, tf2::TimePointZero, tf2::durationFromSec(0.1), &err)
        && retries < 10)
    {
        RCLCPP_WARN(this->get_logger(), "Timeout waiting for transform. Tf error: %s", err.c_str());
        PLOGW.printf("Timeout waiting for transform. Tf error: %s", err.c_str());
        err.clear();
        rclcpp::sleep_for(500000000ns);
        retries++;
    }

    if (retries > 9)
    {
        RCLCPP_ERROR(get_logger(), "Failed to get transform from %s to %s", global_frame_.c_str(), map_frame_.c_str());
        PLOGE.printf("Failed to get transform from %s to %s", global_frame_.c_str(), map_frame_.c_str());
        return CallbackReturn::FAILURE;
    }

    // transformation from earth -> map in WGS84 coordinates
    // according to REP-105 the x-axis points east (lon) and the y-axis north (lat)
    auto tf_ = tf_buffer_->lookupTransform(global_frame_, map_frame_, rclcpp::Time(0), rclcpp::Duration(5,0));
    map_origin.set_global_coordinates(tf_.transform.translation.y, tf_.transform.translation.x);

    PLOGI.printf("Set map origin to %s", map_origin.to_string().c_str());

    return CallbackReturn::SUCCESS;
}

void
Global_Planner_OSM::on_clicked_pnt(std::shared_ptr<geometry_msgs::msg::PointStamped> msg)
{
    PLOGI.printf("Received a clicked point at %.3f, %.3f", msg->point.x, msg->point.y);
    RCLCPP_WARN(get_logger(), "Received a clicked point at %.3f, %.3f", msg->point.x, msg->point.y);

    sgd_util::LatLon ll;
    ll.set_local_coordinates(map_origin, msg->point.x, msg->point.y);

    //std::shared_ptr<sgd_msgs::srv::GetGlobalPlan::Request> request;

    auto request = std::make_shared<sgd_msgs::srv::GetGlobalPlan::Request>();
    request->dest_address = "#" + std::to_string(a_star->node_near_lat_lon(ll));
    auto response = std::make_shared<sgd_msgs::srv::GetGlobalPlan::Response>();

    computePath(request, response);
}

void
Global_Planner_OSM::getMapInfo(const std::shared_ptr<sgd_msgs::srv::GetMapInfo::Request> request,
                               std::shared_ptr<sgd_msgs::srv::GetMapInfo::Response> response)
{
    // send json and list with usernames
    response->address_json = js.dump();
    response->usernames = a_star_users->get_user_list();
}

void
Global_Planner_OSM::computePath(const std::shared_ptr<sgd_msgs::srv::GetGlobalPlan::Request> request,
                                std::shared_ptr<sgd_msgs::srv::GetGlobalPlan::Response> response)
{
    PLOGD.printf("Compute path to %s", request->dest_address.c_str());
    // get current position in local coordinate frame
    sgd_util::LatLon position;
    try
    {
        // transformation from map -> base_link in local cooridinates
        geometry_msgs::msg::TransformStamped tf_ = tf_buffer_->lookupTransform(map_frame_, robot_base_frame_,
                    rclcpp::Time(0), rclcpp::Duration(5,0));

        PLOGD.printf("Set current position to %.3f, %.3f", tf_.transform.translation.x, tf_.transform.translation.y);
        position.set_local_coordinates(map_origin, tf_.transform.translation.x, tf_.transform.translation.y);
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_WARN(this->get_logger(), "Could not get transform from '%s' to '%s': %s",
                map_frame_.c_str(), robot_base_frame_.c_str(), ex.what());
        PLOGW.printf("Could not get transform from '%s' to '%s': %s", map_frame_.c_str(), robot_base_frame_.c_str(), ex.what());
        return;
    }

    // get destination from message. If multiple IDs are specified, multiple paths are calculated and the best one is returned.
    ROUTE best_route_;
    best_route_.cost = 1E9;

    // get destination ids from json
    std::vector<int64_t> dest_ids;
    for (auto& adr : js["addresslist"])
    {
        if (adr["address"] == request->dest_address)
        {
            for (auto& nd : adr["nodes"])
            {
                try
                {
                    dest_ids.push_back(std::stoll(nd.get<std::string>()));
                }
                catch(const std::exception& e)
                {
                    RCLCPP_ERROR(get_logger(), "Could not parse %s", nd.get<std::string>().c_str());
                    PLOGE.printf("Could not parse %s", nd.get<std::string>().c_str());
                }
            }
        }
    }

    if (dest_ids.empty() && request->dest_address.at(0) == '#')
    {
        // do not use address
        PLOGD.printf("Direct mode id: %s", request->dest_address.substr(1).c_str());
        auto id = std::stoll(request->dest_address.substr(1));
        dest_ids.push_back(id);
    }

    for (auto dest_id : dest_ids)
    {
        RCLCPP_INFO(get_logger(), "Compute path from %s to node %d", position.to_string().c_str(), dest_id);
        try
        {
            RCLCPP_INFO(get_logger(), "Start path computation");
            auto route = a_star->compute_path(position, dest_id);
            //auto route = a_star->compute_path();
            if (route.waypoints.size() > 2 && route.cost < best_route_.cost)
            {
               best_route_ = route;
            }
        }
        catch(const std::exception& e)
        {
            RCLCPP_WARN(get_logger(), "Path computation error: %s", e.what());
        }
    }

    if (request->dest_address == "Testfahrt1")
    {
        // temporary path for BSVH demonstration -> Testfahrt1
        std::vector<sgd_util::LatLon> tmp_ll_;
        sgd_util::LatLon ll0(53.5554353, 10.0219323);
        tmp_ll_.push_back(ll0);
        sgd_util::LatLon ll1(53.5554552, 10.0219729);
        tmp_ll_.push_back(ll1);
        sgd_util::LatLon ll2(53.5554757, 10.0220057);
        tmp_ll_.push_back(ll2);
        sgd_util::LatLon ll3(53.5554877, 10.0220434);
        tmp_ll_.push_back(ll3);
        sgd_util::LatLon ll4(53.5555123, 10.022077);
        tmp_ll_.push_back(ll4);
        sgd_util::LatLon ll5(53.5555369, 10.0221107);
        tmp_ll_.push_back(ll5);
        sgd_util::LatLon ll6(53.5555562, 10.0221566);
        tmp_ll_.push_back(ll6);
        sgd_util::LatLon ll7(53.5555944, 10.0221975);
        tmp_ll_.push_back(ll7);
        sgd_util::LatLon ll8(53.555633, 10.0222388);
        tmp_ll_.push_back(ll8);
        sgd_util::LatLon ll9(53.5556698, 10.0222595);
        tmp_ll_.push_back(ll9);
        sgd_util::LatLon ll10(53.5557055, 10.0222795);
        tmp_ll_.push_back(ll10);
        sgd_util::LatLon ll11(53.5557346, 10.022296);
        tmp_ll_.push_back(ll11);
        best_route_.waypoints = tmp_ll_;
    }
    else if (request->dest_address == "Testfahrt2")
    {
        // temporary path for BSVH demonstration -> Testfahrt2
        std::vector<sgd_util::LatLon> tmp_ll_;
        sgd_util::LatLon ll1(53.5557375, 10.0222817);
        tmp_ll_.push_back(ll1);
        sgd_util::LatLon ll2(53.5557083, 10.0222652);
        tmp_ll_.push_back(ll2);
        sgd_util::LatLon ll3(53.5556726, 10.0222451);
        tmp_ll_.push_back(ll3);
        sgd_util::LatLon ll4(53.5556408, 10.0222114);
        tmp_ll_.push_back(ll4);
        sgd_util::LatLon ll5(53.555604, 10.0221719);
        tmp_ll_.push_back(ll5);
        sgd_util::LatLon ll6(53.5555668, 10.022132);
        tmp_ll_.push_back(ll6);
        sgd_util::LatLon ll7(53.5555426, 10.0220989);
        tmp_ll_.push_back(ll7);
        sgd_util::LatLon ll8(53.555518, 10.0220653);
        tmp_ll_.push_back(ll8);
        sgd_util::LatLon ll9(53.5554946, 10.0220338);
        tmp_ll_.push_back(ll9);
        sgd_util::LatLon ll10(53.5554836, 10.0219986);
        tmp_ll_.push_back(ll10);
        sgd_util::LatLon ll11(53.5554722, 10.0219631);
        tmp_ll_.push_back(ll11);
        sgd_util::LatLon ll12(53.5554591, 10.0219233);
        tmp_ll_.push_back(ll12);
        sgd_util::LatLon ll13(53.5554267, 10.0219118);
        tmp_ll_.push_back(ll13);
        best_route_.waypoints = tmp_ll_;
    }

    // we need at least two waypoints to get a valid path
    if (best_route_.waypoints.size() < 2)
    {
        RCLCPP_WARN(get_logger(), "No route available.");
        response->length = -1;
        return;
    }

    std::vector<geometry_msgs::msg::Point> points;
    for (auto ll : best_route_.waypoints)
    {
        geometry_msgs::msg::Point pnt;
        auto xy = ll.to_local(map_origin);
        pnt.x = xy.first;
        pnt.y = xy.second;
        pnt.z = 0.0;
        points.push_back(pnt);
    }

    response->length = best_route_.length_m;
    response->waypoints = points;
    
    RCLCPP_INFO(get_logger(), "Route consists of %d waypoints and has a length of %.3f m",
                    best_route_.waypoints.size(), best_route_.length_m);
    // publish path
    auto poses = create_poses_from_waypoints(best_route_.waypoints);
    publish_path(poses);
}

void
Global_Planner_OSM::publish_path(std::vector<geometry_msgs::msg::Pose> data)
{
    nav_msgs::msg::Path path_;

    path_.header.frame_id = map_frame_;
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

visualization_msgs::msg::Marker
Global_Planner_OSM::create_map_visual()
{
    // go through all nodes
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = now();
    marker.header.frame_id = "map";
    marker.ns = "sgd";
    marker.id = now().nanoseconds();
    marker.type = visualization_msgs::msg::Marker::POINTS;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    //marker.scale.z = 10.05;
    marker.lifetime.sec = 0;
    marker.color.a = 1.0;
    marker.color.r = 0;
    marker.color.g = 0;
    marker.color.b = 1.0;

    for (auto node : a_star->get_nodelist())
    {
        // get local coordinates
        auto xy = node.get_latlon().to_local(map_origin);

        geometry_msgs::msg::Point pnt;
        pnt.x = xy.first;
        pnt.y = xy.second;
        pnt.z = 1.0;
        marker.points.push_back(pnt);
        marker.colors.push_back(marker.color);
    }
    return marker;    
}

}   // namespace sgd_global_planner

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<sgd_global_planner::Global_Planner_OSM>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
}

