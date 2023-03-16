// Copyright 2022 HAW Hamburg
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

#include "sgd_local_costmap/obstacle_detector.hpp"

namespace sgd_local_costmap
{

ObstacleDetector::ObstacleDetector() :
    rclcpp_lifecycle::LifecycleNode("obstacle_detector")
{
    RCLCPP_DEBUG(get_logger(), "Creating");

    declare_parameter("log_dir", rclcpp::ParameterValue(".ros/log/"));
    declare_parameter("log_severity", rclcpp::ParameterValue("I"));

    declare_parameter("scan_topic", rclcpp::ParameterValue("/scan"));

    // initialize logging
    std::string log_dir_, log_sev_;
    get_parameter("log_dir", log_dir_);
    get_parameter("log_severity", log_sev_);

    time_t now = time(0);
    tm *ltm = localtime(&now);

    char buf[24];
    std::sprintf(&buf[0], "%4d-%02d-%02d_%02d-%02d-%02d.csv", 1900+ltm->tm_year, 1+ltm->tm_mon, ltm->tm_mday,
                            ltm->tm_hour, ltm->tm_min, ltm->tm_sec);
    std::string log_file_(buf);
    std::string log_file(log_dir_ + "/costmap_" + log_file_);

    plog::init(plog::severityFromString(log_sev_.c_str()), log_file.c_str());
}

ObstacleDetector::~ObstacleDetector()
{
    // Destroy
}

CallbackReturn
ObstacleDetector::on_configure(const rclcpp_lifecycle::State & state __attribute__((unused)))
{
    RCLCPP_INFO(get_logger(), "Configuring");
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Initialize parameters, pub/sub, services, etc.
    init_parameters();
    
    return CallbackReturn::SUCCESS;
}

CallbackReturn
ObstacleDetector::on_activate(const rclcpp_lifecycle::State & state __attribute__((unused)))
{
    RCLCPP_INFO(get_logger(), "Activating");

    sub_scan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(scan_topic, default_qos,
            std::bind(&ObstacleDetector::on_scan_received, this, std::placeholders::_1));
    pub_obstacles = this->create_publisher<sgd_msgs::msg::VecObstacleArray>("/obstacles/vec", default_qos);
    pub_obstacles->on_activate();

    obstacle_id = 0;
    last_msg = now();
    return CallbackReturn::SUCCESS;
}

CallbackReturn
ObstacleDetector::on_deactivate(const rclcpp_lifecycle::State & state __attribute__((unused)))
{
    RCLCPP_INFO(get_logger(), "Deactivating");
    pub_obstacles->on_deactivate();
    return CallbackReturn::SUCCESS;
}

CallbackReturn
ObstacleDetector::on_cleanup(const rclcpp_lifecycle::State & state __attribute__((unused)))
{
    RCLCPP_INFO(get_logger(), "Cleanup");
    return CallbackReturn::SUCCESS;
}

CallbackReturn
ObstacleDetector::on_shutdown(const rclcpp_lifecycle::State & state __attribute__((unused)))
{
    RCLCPP_INFO(get_logger(), "Shutdown");
    return CallbackReturn::SUCCESS;
}

void
ObstacleDetector::init_parameters()
{
    get_parameter("scan_topic", scan_topic);
}

void
ObstacleDetector::on_scan_received(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    // process message every 10 seconds
    if ((now()-last_msg).nanoseconds() < 5E9)  return;
    last_msg = now();

    // get current position of robo
    try
    {
        tf_map_scan = tf_buffer_->lookupTransform("map", "base_scan", rclcpp::Time(0));
    }
    catch(const std::exception& e)
    {
        RCLCPP_WARN(get_logger(), "Could not get transform from map to base_scan");
    }

    printf("New scan: -------------\n");
    PLOGI << "New scan: --------------\n";
    print_debug_ = true;

    obstacle_message.header.stamp = now();
    obstacle_message.header.frame_id = "map";
    obstacle_message.pose.position.x = tf_map_scan.transform.translation.x;
    obstacle_message.pose.position.y = tf_map_scan.transform.translation.y;
    obstacle_message.pose.orientation = tf_map_scan.transform.rotation;
    obstacle_message.obstacles.clear();

    float yaw = tf2::getYaw(tf_map_scan.transform.rotation);

    int subset_start = 0;  // set possible start point for subset
    int num_inv_pnts = 0;   // number of consecutive invalid scan ranges
    for (int i = 1; i < msg->ranges.size(); i++)
    {
        // check if point is in the valid range
        if (msg->ranges[i-1] > msg->range_max || msg->ranges[i-1] < msg->range_min)
        {
            num_inv_pnts += 1;
            if (num_inv_pnts > 3)
            {
                // start new subset
                subset_start = i;
            }
        }
        else if (num_inv_pnts > 0)
        {
            num_inv_pnts -= 1;
        }

        // jump in data points detected
        if (abs(msg->ranges[i] - msg->ranges[i-1]) > 1.0 || i+1 >= msg->ranges.size())
        {
            if (i - subset_start < 5)   // subset should have at least 5 data points
            {
                // forget this subset
                subset_start = i;
                num_inv_pnts = 0;
                continue;
            }

            // create new vector with subset in x-y-coordinates
            PLOGI << "New subset detected (size=" << i-subset_start << "):";
            std::vector<Point> subset;
            for (int k = subset_start; k < i; k++)
            {
                if (is_valid(msg->ranges[k]))
                {
                    // Achtung: Lidar-Sensor ist kopfüber angebracht
                    float ang = -k*msg->angle_increment - msg->angle_min;
                    Point p(cos(ang)*msg->ranges[k], sin(ang)*msg->ranges[k]);
                    subset.push_back(p);
                    PLOGI << subset.back().x << "; " << subset.back().y;
                }
            }
            // compute ausgleichsgerade
            auto points = split_and_merge(subset, 0, subset.size(), 0);

            Obstacle obst;
            // add points to obstacle
            PLOGI << "Computed points:";
            for (auto p : points)
            {
                // transform points from base_link -> map coordinate frame
                obst.add_vertex(cos(yaw)*p.x - sin(yaw)*p.y + tf_map_scan.transform.translation.x,
                                cos(yaw)*p.y + sin(yaw)*p.x + tf_map_scan.transform.translation.y);
                PLOGI << cos(yaw)*p.x - sin(yaw)*p.y + tf_map_scan.transform.translation.x << "; "
                      << cos(yaw)*p.y + sin(yaw)*p.x + tf_map_scan.transform.translation.y;
            }

            if (obstacles.size() < 5)
            {
                obstacles.push_back(obst);
            }
            subset_start = i;
        }
    }

    // compute expected scan
    for (auto obstacle : obstacles)
    {
        auto points = obstacle.expected_scan(tf_map_scan.transform.translation.x, tf_map_scan.transform.translation.y, yaw);
        // first entry is min_phi
        

        obstacle_message.obstacles.push_back(obstacle.to_msg());
    }

    pub_obstacles->publish(obstacle_message);
}

std::vector<Point>
ObstacleDetector::split_and_merge(std::vector<Point> subset, int start, int end, int depth)
{
    // implementation of a split-and-merge algorithm by Douglas and Peucker

    // if end-start < 2 -> two or less points in subset
    PLOGI << "Split and merge(start=" << start << ", end=" << end << ", depth=" << depth << ")";
    // compute average x and y coordinate
    float x_a = 0.0, y_a = 0.0;
    for (int m = start; m < end; m++)
    {
        x_a += subset.at(m).x;
        y_a += subset.at(m).y;
    }
    x_a /= (end-start);
    y_a /= (end-start);

    float sum_xiyi = 0.0, sum_xi2 = 0.0;
    float sroot = sqrt(pow(subset[end-1].x-subset[start].x,2) + pow(subset[end-1].y-subset[start].y,2));
    int max_dist_index = 0; // index of point with maximum distance to line
    float max_dist = 0.0;   // maximum distance to line
    for (int n = start; n < end; n++)
    {
        sum_xiyi += (subset[n].x-x_a)*(subset[n].y-y_a);
        sum_xi2 += std::pow(subset[n].x-x_a, 2);

        // get data point with maximum distance to line connecting first and last data point
        float dist = abs((subset[end-1].x-subset[start].x)*(subset[start].y-subset[n].y) 
                        -(subset[start].x-subset[n].x)*(subset[end-1].y-subset[start].y)) / sroot;
        if (dist > max_dist)
        {
            max_dist = dist;
            max_dist_index = n;
        }
    }

    if (max_dist > 0.5 && depth < 4)
    {
        // split line and do it again
        depth++;
        auto line1 = split_and_merge(subset, start, max_dist_index, depth); // first line
        auto line2 = split_and_merge(subset, max_dist_index, end, depth);   // second line

        // calculate intersection
        // replace last point of line 1 with intersection point
        auto p_intersect = calc_intersection(line1[line1.size() - 2], line1[line1.size() - 1], line2[0], line2[1]);
        line1.pop_back();
        line1.push_back(p_intersect);
        // add line2 to line1
        for (int l = 1; l<line2.size(); l++)
        {
            line1.push_back(line2[l]);
        }

        return line1;
    }

    std::vector<Point> regression;
    float m = sum_xiyi / sum_xi2;
    float b = y_a - m*x_a;

    Point p1(subset.at(start).x, m*subset.at(start).x+b);
    Point p2(subset.at(end-1).x, m*subset.at(end-1).x+b);

    regression.push_back(p1);
    regression.push_back(p2);

    // print points and regression
    PLOGI << "Return Regression: " << p1.x << ", " << p1.y << "; " << p2.x << ", " << p2.y;
    print_debug_ = false;

    return regression;
}

Point
ObstacleDetector::calc_intersection(Point p1, Point p2, Point p3, Point p4)
{
    // TODO check if steigung is identical -> kann logisch eigentlich nicht passieren
    float u = (p1.x*(p3.y-p2.y) + p2.x*(p1.y-p3.y) + p3.x*(p2.y-p1.y))
            / (p1.x*(p3.y-p4.y) + p2.x*(p4.y-p3.y) + p3.x*(p2.y-p1.y) + p4.x*(p1.y-p2.y));
    Point ip(p3.x+u*(p4.x-p3.x), p3.y+u*(p4.y-p3.y));   // intersection point

    return ip;
}

float
ObstacleDetector::calc_distance_to_line(Point pl1, Point pl2, Point p)
{
    return abs((pl2.x-pl1.x)*(pl1.y-p.y)-(pl1.x-p.x)*(pl2.y-pl1.y)) / sqrt(pow(pl2.x-pl1.x,2) + pow(pl2.y-pl1.y,2));
}

}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<sgd_local_costmap::ObstacleDetector>(); 
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}