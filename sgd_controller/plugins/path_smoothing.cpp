#include "sgd_controller/plugins/path_smoothing.hpp"

namespace sgd_ctrl
{

PathSmoothing::PathSmoothing(double weight_data_, double weight_smooth_, double tolerance_, double interpolation_resolution)
{
    weight_data = weight_data_;
    weight_smooth = weight_smooth_;
    tolerance = tolerance_;
    interpolation_resolution_ = interpolation_resolution;
}

PathSmoothing::PathSmoothing()
{
    weight = 9;
	weight_data = 1 - (weight / 10);
	weight_smooth = (weight / 10);
    tolerance = 0.000001;
    interpolation_resolution_ = 0.10;
}

PathSmoothing::~PathSmoothing() {}

std::vector<xy_pnt> 
PathSmoothing::extend_path(std::vector<xy_pnt> original_path)
{
    std::vector<xy_pnt> new_path_xy;

    // std::vector<sgd_util::LatLon> new_path; 
    // sgd_util::LatLon path_point;
    // sgd_util::LatLon next_point;

    for(std::size_t i = 0; i < original_path.size(); i++)
    {
        new_path_xy.push_back(original_path.at(i));

        // path_point.set_global_coordinates(original_path.at(i).lat(), original_path.at(i).lon());
        // new_path.push_back(path_point);
        
        if(i != (original_path.size() - 1))
        {
            auto interpolated_pts_xy = interpolate(original_path.at(i), original_path.at(i + 1), interpolation_resolution_);
            new_path_xy.insert(end(new_path_xy), begin(interpolated_pts_xy), end(interpolated_pts_xy));

            // next_point.set_global_coordinates(original_path.at(i + 1).lat(), original_path.at(i + 1).lon());
            // std::vector<sgd_util::LatLon> interpolated_pts = path_point.interpolate(next_point, path_increase);
            // new_path.insert(end(new_path), begin(interpolated_pts), end(interpolated_pts));
        }
    }
    return new_path_xy;
}

std::vector<xy_pnt> 
PathSmoothing::smoothen_path(std::vector<xy_pnt> path)
{
    // Increase the path points
    std::vector<xy_pnt> extended_path = extend_path(path);

    // Create a copy of the extended path 
    std::vector<xy_pnt> smooth_path(extended_path);


    // Create a copy of the extended path 
    //std::vector<sgd_util::LatLon> smooth_path(extended_path);
    double change[] = {tolerance, tolerance};
    //sgd_util::LatLon x_i, y_i, y_prev, y_next, y_i_saved;
    xy_pnt x_i, y_i, y_prev, y_next, y_i_saved;

        while((change[0] >= tolerance) || (change[1] >= tolerance))
        {
            change[0] = 0; change[1] = 0;
            for(std::size_t i = 1; i < (smooth_path.size() - 1); i++)
            {
                x_i = extended_path.at(i);
                y_i = smooth_path.at(i);
                y_prev = smooth_path.at(i-1);
                y_next = smooth_path.at(i+1);
                y_i_saved = y_i;

                // x_i.set_global_coordinates(extended_path.at(i).lat(), extended_path.at(i).lon());
                // y_i.set_global_coordinates(smooth_path.at(i).lat(), smooth_path.at(i).lon());
                // y_prev.set_global_coordinates(smooth_path.at(i - 1).lat(), smooth_path.at(i - 1).lon());
                // y_next.set_global_coordinates(smooth_path.at(i + 1).lat(), smooth_path.at(i + 1).lon());
                // y_i_saved.set_global_coordinates(y_i.lat(), y_i.lon());
                // y_i.set_global_coordinates(
                //     y_i.lat() + weight_data * (x_i.lat() - y_i.lat()) + weight_smooth * (y_next.lat() + y_prev.lat() - (2 * y_i.lat())),
                //     y_i.lon() + weight_data * (x_i.lon() - y_i.lon()) + weight_smooth * (y_next.lon() + y_prev.lon() - (2 * y_i.lon()))
                // );

                y_i = xy_pnt(y_i.first + weight_data * (x_i.first - y_i.first) + weight_smooth * (y_next.first + y_prev.first - (2 * y_i.first)),
                             y_i.second + weight_data * (x_i.second - y_i.second) + weight_smooth * (y_next.second + y_prev.second - (2 * y_i.second)));


                smooth_path.at(i) = y_i;

                change[0] += std::abs(y_i.first - y_i_saved.first);
                change[1] += std::abs(y_i.second - y_i_saved.second);

                // change[0] += std::abs(y_i.lat() - y_i_saved.lat());
                // change[1] += std::abs(y_i.lon() - y_i_saved.lon());
            }
        }

    return smooth_path;
}

void 
PathSmoothing::set_weights(double weight)
{
    weight_data = 1 - (weight / 10);
    weight_smooth = (weight / 10);
}

void 
PathSmoothing::set_interpolation_resolution(double interpolation_resolution)
{
    interpolation_resolution_ = interpolation_resolution;
}

xy_pnt
PathSmoothing::get_weights()
{
    return std::make_pair(weight_data, weight_smooth);
}

double 
PathSmoothing::get_interpolation_resolution()
{
    return interpolation_resolution_;
}

std::vector<xy_pnt>
PathSmoothing::interpolate(xy_pnt pt1, xy_pnt pt2, double max_distance)
{
    std::vector<xy_pnt> vec_xy;
    double delta_x = pt2.first - pt1.first;
    double delta_y = pt2.second - pt1.second;

    // calculate distance between points
    int pts_ = floor(std::hypot(delta_x, delta_y) / max_distance);
    delta_x /= (pts_+1);
    delta_y /= (pts_+1);
    for (int i = 0; i < pts_; i++)
    {
        xy_pnt p(pt1.first + (i+1) * delta_x, pt1.second + (i+1) * delta_y);
        vec_xy.push_back(p);
    }
    return vec_xy;
}

} // namespace nav_sgd