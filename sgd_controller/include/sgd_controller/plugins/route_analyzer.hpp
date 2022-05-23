#include <string>
#include <list>
#include <vector>
#include <cmath>
#include <map>
#include <iostream>

#include "sgd_util/angle_utils.hpp"

namespace sgd_ctrl
{

class RouteAnalyzer
{

struct RouteInfo
{
    int next_maneuver;
    std::string text;
};

struct Pos2D
{
    double x, y;
    double angle;
};

private:
    std::list<Pos2D> global_plan_;
    double last_dist = 10.0;    // last distance to next waypoint
    std::map<double, std::string> maneuvers_;
public:
    RouteAnalyzer(/* args */);
    ~RouteAnalyzer();

    void add_waypoints(std::vector<std::pair<double, double>> waypoints);
    RouteInfo next_step(double pos_x, double pos_y);
};

RouteAnalyzer::RouteAnalyzer(/* args */)
{
    maneuvers_.insert({-2.618, "Nach rechts umkehren in "});
    maneuvers_.insert({-2.094, "Stark rechts abbiegen in "});
    maneuvers_.insert({-1.047, "Rechts abbiegen in "});
    maneuvers_.insert({-0.349, "Leicht rechts in "});
    maneuvers_.insert({0.349, "Geradeaus in "});
    maneuvers_.insert({1.047, "Leicht links in "});
    maneuvers_.insert({2.094, "Links abbiegen in "});
    maneuvers_.insert({2.618, "Stark links abbiegen in "});
    maneuvers_.insert({3.142, "Nach links umkehren in "});
    maneuvers_.insert({4.0, "Ziel erreicht in "});
}

RouteAnalyzer::~RouteAnalyzer()
{
}

void
RouteAnalyzer::add_waypoints(std::vector<std::pair<double, double>> waypoints)
{
    if (waypoints.size() < 2)   // minimum size is 2
    {
        return;
    }

    // Angle for first waypoint is 0
    Pos2D pos;
    pos.x = waypoints.front().first;
    pos.y = waypoints.front().second;
    pos.angle = 0.0;
    global_plan_.push_back(pos);

    for (std::size_t i = 1; i < (waypoints.size()-1); i++)
    {
        pos.x = waypoints.at(i).first;
        pos.y = waypoints.at(i).second;
        
        // calculate angle
        pos.angle = sgd_util::delta_angle(waypoints.at(i-1).first, waypoints.at(i-1).second,
                                          waypoints.at(i).first  , waypoints.at(i).first,
                                          waypoints.at(i+1).first, waypoints.at(i+1).first);
        global_plan_.push_back(pos);
    }

    Pos2D pos_end;
    pos_end.x = waypoints.back().first;
    pos_end.y = waypoints.back().second;
    pos_end.angle = 3.5;
    global_plan_.push_back(pos_end);
}

RouteAnalyzer::RouteInfo
RouteAnalyzer::next_step(double pos_x, double pos_y)
{
    // if distance < 1.0m and distance > last_dist -> remove current waypoint
    auto wp = global_plan_.front();
    double distance = std::hypot(wp.x - pos_x, wp.y - pos_y);
    if (distance < 1.0 && distance > last_dist)
    {
        // remove first waypoint
        global_plan_.pop_front();
        wp = global_plan_.front();
    }

    auto it = maneuvers_.begin();
    int i = 0;
    while (wp.angle > it->first && it != maneuvers_.end())
    {
        i++;
        it++;     
    }

    last_dist = std::hypot(wp.x - pos_x, wp.y - pos_y);

    RouteInfo info;
    info.next_maneuver = 1;
    info.text = it->second + std::to_string(round(distance)) + "m";
    return info;
}

} // namespace sgd_ctrl
