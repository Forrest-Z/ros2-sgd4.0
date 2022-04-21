#include "sgd_global_planner/path_smoothing.hpp"


using std::vector;
using std::string;
using namespace sgd_util;


path_smoothing::path_smoothing(double weight_data_, double weight_smooth_, double tolerance_, int path_increase_)
{
    weight_data = weight_data_;
    weight_smooth = weight_smooth_;
    tolerance = tolerance_;
    path_increase = path_increase_;

}


path_smoothing::path_smoothing()
{
    weight = 9;
	weight_data = 1 - (weight / 10);
	weight_smooth = (weight / 10);
    tolerance = 0.000001;
    path_increase = 10;
}

path_smoothing::~path_smoothing()
{
    
}

vector<sgd_util::LatLon> 
path_smoothing::extend_path(vector<sgd_util::LatLon> original_path)
{
    vector<LatLon> new_path; 
    LatLon path_point;
    LatLon next_point;

    for(int i = 0; i < original_path.size(); i++)
    {
        path_point.set_global_coordinates(original_path.at(i).lat(), original_path.at(i).lon());
        new_path.push_back(path_point);
        
        if(i != (original_path.size() - 1))
        {
            next_point.set_global_coordinates(original_path.at(i + 1).lat(), original_path.at(i + 1).lon());
            vector<LatLon> interpolated_pts = path_point.interpolate(next_point, path_increase);                    
            new_path.insert(end(new_path), begin(interpolated_pts), end(interpolated_pts));
        }
        
    }
    return new_path;
}

vector<sgd_util::LatLon> 
path_smoothing::smoothen_path(vector<sgd_util::LatLon> path)
{
    // Increase the path points
    vector<LatLon> extended_path = extend_path(path);

    // Create a copy of the extended path 
    vector<LatLon> smooth_path(extended_path);
    double change[] = {tolerance, tolerance};
    LatLon x_i, y_i, y_prev, y_next, y_i_saved;

        while((change[0] >= tolerance) || (change[1] >= tolerance))
        {
            change[0] = 0; change[1] = 0;
            for(int i = 1; i < ((smooth_path.size()) - 1); i++)
            {
                x_i.set_global_coordinates(extended_path.at(i).lat(), extended_path.at(i).lon());
                y_i.set_global_coordinates(smooth_path.at(i).lat(), smooth_path.at(i).lon());
                y_prev.set_global_coordinates(smooth_path.at(i - 1).lat(), smooth_path.at(i - 1).lon());
                y_next.set_global_coordinates(smooth_path.at(i + 1).lat(), smooth_path.at(i + 1).lon());
                y_i_saved.set_global_coordinates(y_i.lat(), y_i.lon());
                y_i.set_global_coordinates(
                    y_i.lat() + weight_data * (x_i.lat() - y_i.lat()) + weight_smooth * (y_next.lat() + y_prev.lat() - (2 * y_i.lat())),
                    y_i.lon() + weight_data * (x_i.lon() - y_i.lon()) + weight_smooth * (y_next.lon() + y_prev.lon() - (2 * y_i.lon()))
                );
                smooth_path.at(i).set_global_coordinates(y_i.lat(), y_i.lon());
                change[0] += std::abs(y_i.lat() - y_i_saved.lat());
                change[1] += std::abs(y_i.lon() - y_i_saved.lon());
            }
                
        }

    return smooth_path;
}

void 
path_smoothing::set_weights(double weight)
{
    weight_data = 1 - (weight / 10);
    weight_smooth = (weight / 10);
}

void 
path_smoothing::set_path_increase(int path_increase_)
{
    path_increase_ = path_increase_;
}

std::tuple<double, double> 
path_smoothing::get_weights()
{
    return std::make_tuple(weight_data, weight_smooth);
}

int 
path_smoothing::get_path_increase()
{
    return path_increase;
}